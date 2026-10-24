// Copyright 2019, Collabora, Ltd.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Wayland direct mode code.
 * @author Drew DeVault <sir@cmpwn.com>
 * @author Lubosz Sarnecki <lubosz.sarnecki@collabora.com>
 * @author Simon Zeni <simon@bl4ckb0ne.ca>
 * @ingroup comp
 */

#include <errno.h>
#include <linux/input.h>
#include <poll.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <wayland-client.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "drm-lease-v1-client-protocol.h"
#include "xrt/xrt_compiler.h"
#include "main/comp_window.h"
#include "main/comp_window_direct.h"
#include "util/u_debug.h"
#include "util/u_notice.h"

#ifndef VK_EXT_acquire_drm_display
#error "Wayland direct requires the Vulkan extension VK_EXT_acquire_drm_display"
#endif

DEBUG_GET_ONCE_OPTION(requested_connector, "XRT_COMPOSITOR_WAYLAND_CONNECTOR", NULL)

struct direct_wayland_lease
{
	struct comp_window_direct_wayland *w;

	int leased_fd;
	bool finished;
	struct wp_drm_lease_v1 *lease;
};

struct direct_wayland_lease_connector
{
	struct comp_window_direct_wayland *w;

	uint32_t id;
	char *name, *description;
	//! Taken back by the desktop (leased, or the display gone) (RiftLinux).
	bool withdrawn;
	struct wp_drm_lease_connector_v1 *connector;
	struct direct_wayland_lease_device *dev;
	struct direct_wayland_lease_connector *next;
};

struct direct_wayland_lease_device
{
	struct comp_window_direct_wayland *w;

	int drm_fd;
	char *path;
	bool done;
	struct wp_drm_lease_device_v1 *device;
	struct direct_wayland_lease_connector *connectors;
	struct direct_wayland_lease_device *next;
};

struct comp_window_direct_wayland
{
	struct comp_target_swapchain base;

	struct wl_display *display;
	struct direct_wayland_lease_device *devices;
	struct direct_wayland_lease *lease;

	// The device and connector to use
	struct direct_wayland_lease_device *selected_device;
	struct direct_wayland_lease_connector *selected_connector;

	VkDisplayKHR vk_display;

	/*
	 * The headset taken again after its lease ended (RiftLinux): the CV1 unplugs itself from HDMI when its display is
	 * switched off (not worn), the desktop then ends the lease, and Monado had no way back ("TODO handle graceful
	 * shutdown"): the compositor lost the headset and SteamVR failed. Now the output is given up, and when
	 * the desktop offers the headset again a new lease is taken and the output made on it.
	 */
	bool lost;
	bool released;
	uint32_t width, height;
	bool (*swapchain_check_ready)(struct comp_target *ct);
	VkResult (*swapchain_present)(struct comp_target *ct,
	                              struct vk_bundle_queue *present_queue,
	                              uint32_t index,
	                              uint64_t timeline_semaphore_value,
	                              int64_t desired_present_time_ns,
	                              int64_t present_slop_ns);
};

static void
direct_wayland_lease_device_destroy(struct direct_wayland_lease_device *dev)
{
	struct direct_wayland_lease_connector *conn = dev->connectors, *next_conn;
	while (conn) {
		next_conn = conn->next;

		wp_drm_lease_connector_v1_destroy(conn->connector);

		free(conn->name);
		free(conn->description);
		free(conn);

		conn = next_conn;
	}

	wp_drm_lease_device_v1_destroy(dev->device);
	close(dev->drm_fd);

	free(dev->path);
	free(dev);
}

static inline struct vk_bundle *
get_vk(struct comp_window_direct_wayland *cww)
{
	return &cww->base.base.c->base.vk;
}

static void
comp_window_direct_wayland_destroy(struct comp_target *w)
{
	struct comp_window_direct_wayland *w_wayland = (struct comp_window_direct_wayland *)w;

	comp_target_swapchain_cleanup(&w_wayland->base);

	if (w_wayland->vk_display != VK_NULL_HANDLE) {
		struct vk_bundle *vk = get_vk(w_wayland);
		vk->vkReleaseDisplayEXT(vk->physical_device, w_wayland->vk_display);
		w_wayland->vk_display = VK_NULL_HANDLE;
	}

	struct direct_wayland_lease_device *dev = w_wayland->devices, *next_dev;
	while (dev) {
		next_dev = dev->next;
		direct_wayland_lease_device_destroy(dev);
		dev = next_dev;
	}

	if (w_wayland->lease) {
		close(w_wayland->lease->leased_fd);
		wp_drm_lease_v1_destroy(w_wayland->lease->lease);
		free(w_wayland->lease);
	}

	if (w_wayland->display) {
		wl_display_disconnect(w_wayland->display);
	}
	free(w_wayland);
}

static void
_lease_fd(void *data, struct wp_drm_lease_v1 *wp_drm_lease_v1, int32_t leased_fd)
{
	struct direct_wayland_lease *lease = data;
	COMP_DEBUG(lease->w->base.base.c, "Lease granted");

	lease->leased_fd = leased_fd;
}

static void
_lease_finished(void *data, struct wp_drm_lease_v1 *wp_drm_lease_v1)
{
	struct direct_wayland_lease *lease = data;
	if (lease->leased_fd >= 0) {
		close(lease->leased_fd);
		lease->leased_fd = -1;
	}

	lease->finished = true;
	// The lease in use: the output goes, and the headset is taken again when the desktop offers it (RiftLinux).
	if (lease == lease->w->lease && lease->w->vk_display != VK_NULL_HANDLE) {
		COMP_WARN(lease->w->base.base.c, "The desktop ended the headset's lease (its display switched off or gone)");
		lease->w->lost = true;
	}
}

static const struct wp_drm_lease_v1_listener lease_listener = {
    .lease_fd = _lease_fd,
    .finished = _lease_finished,
};

static VkResult
comp_window_direct_wayland_create_surface(struct comp_window_direct_wayland *w,
                                          VkSurfaceKHR *surface,
                                          uint32_t width,
                                          uint32_t height)
{
	struct vk_bundle *vk = get_vk(w);
	w->vk_display = VK_NULL_HANDLE;
	VkResult ret = VK_ERROR_INCOMPATIBLE_DISPLAY_KHR;

	if (w->selected_device == NULL || w->selected_connector == NULL || w->lease == NULL) {
		COMP_ERROR(w->base.base.c, "Connector was disconnected before it could be acquired");
		return VK_ERROR_INITIALIZATION_FAILED;
	}

	ret = vk->vkGetDrmDisplayEXT(vk->physical_device, w->selected_device->drm_fd, w->selected_connector->id,
	                             &w->vk_display);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "vkGetDrmDisplayEXT failed: %s", vk_result_string(ret));
		return ret;
	}

	ret = vk->vkAcquireDrmDisplayEXT(vk->physical_device, w->lease->leased_fd, w->vk_display);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "vkAcquireDrmDisplayEXT failed: %s", vk_result_string(ret));
		return ret;
	}

	ret = comp_window_direct_create_surface(&w->base, w->vk_display, width, height);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->base.base.c, "Failed to create surface: %s", vk_result_string(ret));
	}

	return ret;
}

static bool
comp_window_direct_wayland_init_swapchain(struct comp_target *w, uint32_t width, uint32_t height)
{
	struct comp_window_direct_wayland *w_wayland = (struct comp_window_direct_wayland *)w;
	VkResult ret;

	ret = comp_window_direct_wayland_create_surface(w_wayland, &w_wayland->base.surface.handle, width, height);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(w->c, "Failed to create surface!");
		return false;
	}

	struct comp_target_swapchain *cts = (struct comp_target_swapchain *)w_wayland;
	cts->display = w_wayland->vk_display;
	w_wayland->width = width;
	w_wayland->height = height;

	return true;
}

static void
comp_window_direct_wayland_flush(struct comp_target *w)
{
	struct comp_window_direct_wayland *w_wayland = (struct comp_window_direct_wayland *)w;

	while (wl_display_prepare_read(w_wayland->display) != 0)
		wl_display_dispatch_pending(w_wayland->display);

	if (wl_display_flush(w_wayland->display) < 0 && errno != EAGAIN) {
		wl_display_cancel_read(w_wayland->display);
		return;
	}

	struct pollfd fds[] = {
	    {
	        .fd = wl_display_get_fd(w_wayland->display),
	        .events = POLLIN,
	    },
	};

	if (poll(fds, 1, 0) > 0) {
		wl_display_read_events(w_wayland->display);
		wl_display_dispatch_pending(w_wayland->display);
	} else {
		wl_display_cancel_read(w_wayland->display);
	}
}

static void
_lease_connector_name(void *data, struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1, const char *name)
{
	struct direct_wayland_lease_connector *conn = data;
	conn->name = strdup(name);
}

static void
_lease_connector_description(void *data,
                             struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1,
                             const char *description)
{
	struct direct_wayland_lease_connector *conn = data;
	if (conn->description) {
		free(conn->description);
	}
	conn->description = strdup(description);
}

static void
_lease_connector_connector_id(void *data,
                              struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1,
                              uint32_t connector_id)
{
	struct direct_wayland_lease_connector *conn = data;
	conn->id = connector_id;
}

static void
_lease_connector_done(void *data, struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1)
{
	struct direct_wayland_lease_connector *conn = data;
	COMP_INFO(conn->w->base.base.c, "[%s] connector %s (%s) id: %d", conn->dev->path, conn->name, conn->description,
	          conn->id);
}

static void
_lease_connector_withdrawn(void *data, struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1)
{
	struct direct_wayland_lease_connector *conn = data;
	COMP_INFO(conn->w->base.base.c, "Connector %s has been withdrawn by the compositor", conn->name);
	conn->withdrawn = true;
	if (conn == conn->w->selected_connector && (conn->w->lease == NULL || !conn->w->lease->lease)) {
		conn->w->selected_connector = NULL;
	}
}

static const struct wp_drm_lease_connector_v1_listener lease_connector_listener = {
    .name = _lease_connector_name,
    .description = _lease_connector_description,
    .connector_id = _lease_connector_connector_id,
    .done = _lease_connector_done,
    .withdrawn = _lease_connector_withdrawn,
};

static void
_drm_lease_device_drm_fd(void *data, struct wp_drm_lease_device_v1 *drm_lease_device, int fd)
{
	struct direct_wayland_lease_device *dev = data;
	dev->drm_fd = fd;
	dev->path = drmGetDeviceNameFromFd2(fd);
	COMP_INFO(dev->w->base.base.c, "Available DRM lease device: %s", dev->path);
}

static void
_drm_lease_device_connector(void *data,
                            struct wp_drm_lease_device_v1 *wp_drm_lease_device_v1,
                            struct wp_drm_lease_connector_v1 *wp_drm_lease_connector_v1)
{
	struct direct_wayland_lease_device *dev = data;

	struct direct_wayland_lease_connector *conn = calloc(1, sizeof(struct direct_wayland_lease_connector));
	conn->connector = wp_drm_lease_connector_v1;
	conn->dev = dev;
	conn->w = dev->w;
	wp_drm_lease_connector_v1_add_listener(conn->connector, &lease_connector_listener, conn);

	conn->next = dev->connectors;
	dev->connectors = conn;
}

static void
_drm_lease_device_done(void *data, struct wp_drm_lease_device_v1 *wp_drm_lease_device_v1)
{
	struct direct_wayland_lease_device *dev = data;
	dev->done = true;
}

static void
_drm_lease_device_released(void *data, struct wp_drm_lease_device_v1 *wp_drm_lease_device_v1)
{
	struct direct_wayland_lease_device *dev = data;
	COMP_ERROR(dev->w->base.base.c, "Releasing lease device %s", dev->path);
	if (dev == dev->w->selected_device) {
		dev->w->selected_device = NULL;
		dev->w->selected_connector = NULL;
	}
	direct_wayland_lease_device_destroy(dev);
}

static const struct wp_drm_lease_device_v1_listener drm_lease_device_listener = {
    .drm_fd = _drm_lease_device_drm_fd,
    .connector = _drm_lease_device_connector,
    .done = _drm_lease_device_done,
    .released = _drm_lease_device_released,
};

static void
_registry_global_remove_cb(void *data, struct wl_registry *registry, uint32_t name)
{}

static void
_registry_global_cb(void *data, struct wl_registry *registry, uint32_t name, const char *interface, uint32_t version)
{
	struct comp_window_direct_wayland *w = data;
	if (strcmp(interface, wp_drm_lease_device_v1_interface.name) == 0) {
		struct direct_wayland_lease_device *dev = calloc(1, sizeof(struct direct_wayland_lease_device));
		dev->device = (struct wp_drm_lease_device_v1 *)wl_registry_bind(registry, name,
		                                                                &wp_drm_lease_device_v1_interface, 1);
		dev->w = w;
		dev->drm_fd = -1;
		wp_drm_lease_device_v1_add_listener(dev->device, &drm_lease_device_listener, dev);

		dev->next = w->devices;
		w->devices = dev;
	}
}

static const struct wl_registry_listener registry_listener = {
    .global = _registry_global_cb,
    .global_remove = _registry_global_remove_cb,
};

//! A connector description naming the Rift (the desktop describes the display by its EDID) (RiftLinux).
static bool
is_rift_description(const char *description)
{
	if (description == NULL) {
		return false;
	}
	static const char *const names[] = {"rift", "oculus", "ovr"};
	for (size_t n = 0; n < sizeof(names) / sizeof(names[0]); n++) {
		const size_t len = strlen(names[n]);
		for (const char *p = description; *p != '\0'; p++) {
			size_t i = 0;
			while (i < len && p[i] != '\0' && (p[i] | 0x20) == names[n][i]) {
				i++;
			}
			if (i == len) {
				return true;
			}
		}
	}
	return false;
}

// Find the connector matching name, return the first available if no match
static bool
get_named_connector_or_first(struct comp_window_direct_wayland *w, const char *name)
{
	// No name asked for: the Rift's connector where the desktop offers several (RiftLinux).
	if (name == NULL) {
		for (struct direct_wayland_lease_device *d = w->devices; d != NULL; d = d->next) {
			while (!d->done) {
				if (wl_display_dispatch(w->display) == -1) {
					break;
				}
			}
			for (struct direct_wayland_lease_connector *cn = d->connectors; cn != NULL; cn = cn->next) {
				if (is_rift_description(cn->description) || is_rift_description(cn->name)) {
					w->selected_device = d;
					w->selected_connector = cn;
					return false;
				}
			}
		}
	}

	struct direct_wayland_lease_device *dev = w->devices;
	struct direct_wayland_lease_connector *conn = NULL;

	while (dev) {
		if (!dev->done) {
			wl_display_dispatch(w->display);
			continue;
		}
		if (dev->connectors) {
			/* Use the first connector available as default */
			if (!w->selected_connector) {
				w->selected_device = dev;
				w->selected_connector = dev->connectors;
				if (!name) {
					return false;
				}
			}

			// If no connector is chosen, we're done.
			conn = dev->connectors;
			while (conn) {
				if (!strcmp(name, conn->name)) {
					w->selected_device = dev;
					w->selected_connector = conn;
					return true;
				}
				conn = conn->next;
			}
		}
		dev = dev->next;
	}
	return false;
}

/*!
 * NVIDIA flips a leased display without vsync while any connected monitor reports adaptive sync (RiftLinux;
 * NVIDIA open-gpu-kernel-modules #992): the lower part of the headset's picture tears and lags when the
 * head turns. Said in the log and in the headset, once; the fix is the user's choice, as it takes the monitors' VRR
 * away (riftlinux-setup nvidia-vrr conceal, then a reboot; the README has the commands).
 *
 * Seen with drivers 570 to 580 (580.95.05 on GNOME 48); not with 615.78.08 on KDE Plasma 6.8 with the same monitor
 * offering VRR. NVIDIA names no fixed version: from 615 on only the log says it.
 */
#define NVIDIA_VRR_TEAR_FIXED_MAJOR 615

//! The NVIDIA kernel module's major version (615 for 615.78.08), 0 if unknown.
static int
nvidia_driver_major(void)
{
	FILE *f = fopen("/sys/module/nvidia/version", "r");
	if (f == NULL) {
		return 0;
	}
	int major = 0;
	if (fscanf(f, "%d", &major) != 1) {
		major = 0;
	}
	fclose(f);
	return major;
}

static void
warn_if_nvidia_vrr(struct comp_window_direct_wayland *w, int drm_fd)
{
	static bool warned = false;
	if (warned || drm_fd < 0) {
		return;
	}
	drmVersionPtr version = drmGetVersion(drm_fd);
	const bool nvidia = version != NULL && version->name != NULL && strcmp(version->name, "nvidia-drm") == 0;
	drmFreeVersion(version);
	if (!nvidia) {
		return;
	}
	drmModeResPtr res = drmModeGetResources(drm_fd);
	if (res == NULL) {
		return;
	}
	char names[128] = "";
	for (int i = 0; i < res->count_connectors; i++) {
		// The current state, without probing the monitors again.
		drmModeConnectorPtr conn = drmModeGetConnectorCurrent(drm_fd, res->connectors[i]);
		if (conn == NULL) {
			continue;
		}
		drmModeObjectPropertiesPtr props = conn->connection == DRM_MODE_CONNECTED
		                                       ? drmModeObjectGetProperties(drm_fd, conn->connector_id,
		                                                                    DRM_MODE_OBJECT_CONNECTOR)
		                                       : NULL;
		for (uint32_t j = 0; props != NULL && j < props->count_props; j++) {
			drmModePropertyPtr prop = drmModeGetProperty(drm_fd, props->props[j]);
			if (prop != NULL && strcmp(prop->name, "vrr_capable") == 0 && props->prop_values[j] == 1) {
				const char *type = conn->connector_type == DRM_MODE_CONNECTOR_DisplayPort ? "DP"
				                   : conn->connector_type == DRM_MODE_CONNECTOR_HDMIA     ? "HDMI-A"
				                   : conn->connector_type == DRM_MODE_CONNECTOR_eDP       ? "eDP"
				                                                                          : "connector";
				size_t len = strlen(names);
				snprintf(names + len, sizeof(names) - len, "%s%s-%u", len ? ", " : "", type,
				         conn->connector_type_id);
			}
			drmModeFreeProperty(prop);
		}
		drmModeFreeObjectProperties(props);
		drmModeFreeConnector(conn);
	}
	drmModeFreeResources(res);
	if (names[0] == '\0') {
		return;
	}
	warned = true;
	const int major = nvidia_driver_major();
	if (major >= NVIDIA_VRR_TEAR_FIXED_MAJOR) {
		COMP_INFO(w->base.base.c,
		          "NVIDIA %d with a monitor offering adaptive sync (%s): the headset tore with drivers 570-580 on "
		          "Wayland, not with 615 and later; if it tears, riftlinux-setup nvidia-vrr conceal.",
		          major, names);
		return;
	}
	COMP_WARN(w->base.base.c,
	          "NVIDIA with a monitor offering adaptive sync (%s): on Wayland NVIDIA shows the headset's frames without "
	          "vsync, the lower part of the picture tears and lags. Fix (turns the monitors' VRR off): riftlinux-setup "
	          "nvidia-vrr conceal, then reboot; see the README.",
	          names);
	u_notice_post(U_NOTICE_DISPLAY, "display-vrr", 0);
}

static bool
comp_window_direct_wayland_init(struct comp_target *w)
{
	struct comp_window_direct_wayland *w_wayland = (struct comp_window_direct_wayland *)w;

	w_wayland->display = wl_display_connect(NULL);
	if (!w_wayland->display) {
		COMP_ERROR(w->c, "Failed to connect to Wayland display");
		return false;
	}

	struct wl_registry *registry = wl_display_get_registry(w_wayland->display);
	wl_registry_add_listener(registry, &registry_listener, w_wayland);
	wl_display_roundtrip(w_wayland->display);
	wl_registry_destroy(registry);

	if (!w_wayland->devices) {
		COMP_ERROR(w->c, "Compositor is missing drm-lease support");
		return false;
	}

	const char *requested_connector = debug_get_option_requested_connector();
	if (requested_connector) {
		COMP_INFO(w->c, "Requesting connector %s", requested_connector);
	} else {
		COMP_INFO(w->c, "No connector was chosen, will use first available connector");
	}

	bool found = get_named_connector_or_first(w_wayland, requested_connector);

	if (!w_wayland->selected_connector) {
		COMP_INFO(w->c, "Found no connectors available for direct mode");
		return false;
	}

	// Inform when chosen connector was not found
	if (requested_connector && !found) {
		COMP_WARN(w->c, "Could not find requested connector %s, selected first available connector %s",
		          requested_connector, w_wayland->selected_connector->name);
	}

	COMP_INFO(w->c, "Using DRM node %s", w_wayland->selected_device->path);
	COMP_INFO(w->c, "Connector id %d %s (%s)", w_wayland->selected_connector->id,
	          w_wayland->selected_connector->name, w_wayland->selected_connector->description);

	struct wp_drm_lease_request_v1 *request =
	    wp_drm_lease_device_v1_create_lease_request(w_wayland->selected_device->device);
	if (!request) {
		COMP_ERROR(w->c, "Failed to create lease request");
		return false;
	}

	wp_drm_lease_request_v1_request_connector(request, w_wayland->selected_connector->connector);

	struct direct_wayland_lease *lease = calloc(1, sizeof(struct direct_wayland_lease));
	lease->w = w_wayland;
	lease->leased_fd = -1;
	lease->finished = false;
	lease->lease = wp_drm_lease_request_v1_submit(request);

	w_wayland->lease = lease;

	wp_drm_lease_v1_add_listener(lease->lease, &lease_listener, lease);

	while (lease->leased_fd < 0 && !lease->finished) {
		if (wl_display_dispatch(w_wayland->display) == -1) {
			COMP_ERROR(w->c, "wl_display roundtrip failed");
			return false;
		}
	}

	if (lease->finished) {
		COMP_ERROR(w->c, "Failed to lease connector");
		return false;
	}

	warn_if_nvidia_vrr(w_wayland, w_wayland->selected_device->drm_fd);

	return true;
}

static void
_update_window_title(struct comp_target *ct, const char *title)
{
	/* Not required in direct mode */
}

/*
 *
 * The headset taken again after its lease ended (RiftLinux).
 *
 */

static void
forget_lease(struct comp_window_direct_wayland *w)
{
	if (w->lease == NULL) {
		return;
	}
	if (w->lease->leased_fd >= 0) {
		close(w->lease->leased_fd);
	}
	if (w->lease->lease != NULL) {
		wp_drm_lease_v1_destroy(w->lease->lease);
	}
	free(w->lease);
	w->lease = NULL;
}

//! A connector the desktop offers now: the Rift's, else the first.
static bool
pick_offered_connector(struct comp_window_direct_wayland *w,
                       struct direct_wayland_lease_device **out_dev,
                       struct direct_wayland_lease_connector **out_conn)
{
	*out_dev = NULL;
	*out_conn = NULL;
	for (struct direct_wayland_lease_device *d = w->devices; d != NULL; d = d->next) {
		if (!d->done) {
			continue;
		}
		for (struct direct_wayland_lease_connector *cn = d->connectors; cn != NULL; cn = cn->next) {
			if (cn->withdrawn || cn->name == NULL) {
				continue;
			}
			if (*out_conn == NULL || is_rift_description(cn->description) || is_rift_description(cn->name)) {
				*out_dev = d;
				*out_conn = cn;
			}
			if (is_rift_description(cn->description) || is_rift_description(cn->name)) {
				return true;
			}
		}
	}
	return *out_conn != NULL;
}

/*!
 * Called while the headset is lost, from the compositor thread (check_ready): gives up the output once, then asks the
 * desktop for the headset as soon as it offers it again (the CV1 back on HDMI when worn), and makes the output on the
 * new lease. Returns true when the output is back.
 */
static bool
take_headset_again(struct comp_window_direct_wayland *w)
{
	struct comp_target *ct = &w->base.base;
	struct vk_bundle *vk = get_vk(w);

	// The desktop's news (lease ended or granted, connectors offered).
	comp_window_direct_wayland_flush(ct);

	if (!w->released) {
		comp_target_swapchain_release_output(&w->base);
		if (w->vk_display != VK_NULL_HANDLE) {
			vk->vkReleaseDisplayEXT(vk->physical_device, w->vk_display);
			w->vk_display = VK_NULL_HANDLE;
		}
		forget_lease(w);
		w->selected_connector = NULL;
		w->released = true;
		COMP_INFO(ct->c, "Headset output given up; it is taken again when the desktop offers it");
	}

	if (w->lease != NULL) {
		if (w->lease->finished) {
			// Refused, or ended at once: asked again when offered.
			forget_lease(w);
			return false;
		}
		if (w->lease->leased_fd < 0) {
			return false; // Not granted yet.
		}
		VkResult ret = comp_window_direct_wayland_create_surface(w, &w->base.surface.handle, w->width, w->height);
		if (ret != VK_SUCCESS) {
			COMP_ERROR(ct->c, "The headset's display could not be taken again: %s", vk_result_string(ret));
			if (w->vk_display != VK_NULL_HANDLE) {
				vk->vkReleaseDisplayEXT(vk->physical_device, w->vk_display);
				w->vk_display = VK_NULL_HANDLE;
			}
			forget_lease(w);
			return false;
		}
		w->base.display = w->vk_display;
		w->lost = false;
		w->released = false;
		COMP_INFO(ct->c, "Headset taken again: connector %s (%s)", w->selected_connector->name,
		          w->selected_connector->description);
		return true;
	}

	struct direct_wayland_lease_device *dev = NULL;
	struct direct_wayland_lease_connector *conn = NULL;
	if (!pick_offered_connector(w, &dev, &conn)) {
		return false; // Not offered (yet).
	}
	struct wp_drm_lease_request_v1 *request = wp_drm_lease_device_v1_create_lease_request(dev->device);
	if (request == NULL) {
		return false;
	}
	wp_drm_lease_request_v1_request_connector(request, conn->connector);
	struct direct_wayland_lease *lease = calloc(1, sizeof(struct direct_wayland_lease));
	lease->w = w;
	lease->leased_fd = -1;
	lease->lease = wp_drm_lease_request_v1_submit(request);
	wp_drm_lease_v1_add_listener(lease->lease, &lease_listener, lease);
	w->lease = lease;
	w->selected_device = dev;
	w->selected_connector = conn;
	wl_display_flush(w->display);
	COMP_INFO(ct->c, "Headset offered again: asking for connector %s (%s)", conn->name, conn->description);
	return false;
}

static bool
direct_wayland_check_ready(struct comp_target *ct)
{
	struct comp_window_direct_wayland *w = (struct comp_window_direct_wayland *)ct;
	if (w->lost && !take_headset_again(w)) {
		return false;
	}
	return w->swapchain_check_ready(ct);
}

static VkResult
direct_wayland_present(struct comp_target *ct,
                       struct vk_bundle_queue *present_queue,
                       uint32_t index,
                       uint64_t timeline_semaphore_value,
                       int64_t desired_present_time_ns,
                       int64_t present_slop_ns)
{
	struct comp_window_direct_wayland *w = (struct comp_window_direct_wayland *)ct;
	VkResult ret = w->swapchain_present(ct, present_queue, index, timeline_semaphore_value, desired_present_time_ns,
	                                    present_slop_ns);
	/*
	 * The display gone before the desktop's word arrived (VK_ERROR_UNKNOWN on NVIDIA): the same as the
	 * lease ending. Out of date to the renderer, which then lets go of its images and waits for the target.
	 */
	if (ret == VK_ERROR_SURFACE_LOST_KHR || ret == VK_ERROR_UNKNOWN) {
		/*
		 * Expected: the CV1 drops its HDMI link whenever its display is turned off (not worn), and this is how
		 * the compositor hears of it first. Taken again when the desktop offers the headset back.
		 */
		if (!w->lost) {
			COMP_INFO(ct->c, "The headset's display went dark (%s, off when not worn, or unplugged): taken again "
			          "when it is back",
			          vk_result_string(ret));
		}
		w->lost = true;
		return VK_ERROR_OUT_OF_DATE_KHR;
	}
	return ret;
}

struct comp_target *
comp_window_direct_wayland_create(struct comp_compositor *c)
{
	struct comp_window_direct_wayland *w = U_TYPED_CALLOC(struct comp_window_direct_wayland);

	comp_target_swapchain_init_and_set_fnptrs(&w->base, COMP_TARGET_FORCE_FAKE_DISPLAY_TIMING);

	w->base.base.name = "wayland-direct";
	w->base.display = VK_NULL_HANDLE;
	w->base.base.destroy = comp_window_direct_wayland_destroy;
	w->base.base.flush = comp_window_direct_wayland_flush;
	w->base.base.init_pre_vulkan = comp_window_direct_wayland_init;
	w->base.base.init_post_vulkan = comp_window_direct_wayland_init_swapchain;
	w->base.base.set_title = _update_window_title;
	w->base.base.c = c;
	// Through the headset's leases coming and going (RiftLinux).
	w->swapchain_check_ready = w->base.base.check_ready;
	w->base.base.check_ready = direct_wayland_check_ready;
	w->swapchain_present = w->base.base.present;
	w->base.base.present = direct_wayland_present;

	return &w->base.base;
}


/*
 *
 * Factory
 *
 */

static const char *instance_extensions[] = {
    VK_KHR_DISPLAY_EXTENSION_NAME,             //
    VK_KHR_WAYLAND_SURFACE_EXTENSION_NAME,     //
    VK_EXT_DIRECT_MODE_DISPLAY_EXTENSION_NAME, //

#ifdef VK_EXT_acquire_drm_display
    VK_EXT_ACQUIRE_DRM_DISPLAY_EXTENSION_NAME,
#endif
};

/*
 * Detection (RiftLinux): a Wayland desktop that offers a connector for lease (GNOME and KDE offer non-desktop displays,
 * the CV1 among them) gets the headset through its own lease, ahead of Xwayland's X11 emulation of it (taken on GNOME
 * when this target was not detected; there the picture swam at the bottom and colours were off).
 */
struct detect_state
{
	int connectors;
	bool rift;
	struct wp_drm_lease_device_v1 *devices[8];
	int device_count;
	struct wp_drm_lease_connector_v1 *conns[32];
	int conn_count;
};

static void
detect_conn_name(void *data, struct wp_drm_lease_connector_v1 *conn, const char *name)
{
	struct detect_state *st = data;
	st->rift = st->rift || is_rift_description(name);
}

static void
detect_conn_description(void *data, struct wp_drm_lease_connector_v1 *conn, const char *description)
{
	struct detect_state *st = data;
	st->rift = st->rift || is_rift_description(description);
}

static void
detect_conn_id(void *data, struct wp_drm_lease_connector_v1 *conn, uint32_t id)
{}

static void
detect_conn_done(void *data, struct wp_drm_lease_connector_v1 *conn)
{}

static void
detect_conn_withdrawn(void *data, struct wp_drm_lease_connector_v1 *conn)
{
	struct detect_state *st = data;
	st->connectors--;
}

static const struct wp_drm_lease_connector_v1_listener detect_conn_listener = {
    .name = detect_conn_name,
    .description = detect_conn_description,
    .connector_id = detect_conn_id,
    .done = detect_conn_done,
    .withdrawn = detect_conn_withdrawn,
};

static void
detect_dev_drm_fd(void *data, struct wp_drm_lease_device_v1 *dev, int fd)
{
	close(fd);
}

static void
detect_dev_connector(void *data, struct wp_drm_lease_device_v1 *dev, struct wp_drm_lease_connector_v1 *conn)
{
	struct detect_state *st = data;
	st->connectors++;
	if (st->conn_count < (int)(sizeof(st->conns) / sizeof(st->conns[0]))) {
		st->conns[st->conn_count++] = conn;
	}
	wp_drm_lease_connector_v1_add_listener(conn, &detect_conn_listener, st);
}

static void
detect_dev_done(void *data, struct wp_drm_lease_device_v1 *dev)
{}

static void
detect_dev_released(void *data, struct wp_drm_lease_device_v1 *dev)
{}

static const struct wp_drm_lease_device_v1_listener detect_dev_listener = {
    .drm_fd = detect_dev_drm_fd,
    .connector = detect_dev_connector,
    .done = detect_dev_done,
    .released = detect_dev_released,
};

static void
detect_global(void *data, struct wl_registry *registry, uint32_t name, const char *interface, uint32_t version)
{
	struct detect_state *st = data;
	if (strcmp(interface, wp_drm_lease_device_v1_interface.name) != 0 ||
	    st->device_count >= (int)(sizeof(st->devices) / sizeof(st->devices[0]))) {
		return;
	}
	struct wp_drm_lease_device_v1 *dev = wl_registry_bind(registry, name, &wp_drm_lease_device_v1_interface, 1);
	st->devices[st->device_count++] = dev;
	wp_drm_lease_device_v1_add_listener(dev, &detect_dev_listener, st);
}

static void
detect_global_remove(void *data, struct wl_registry *registry, uint32_t name)
{}

static const struct wl_registry_listener detect_registry_listener = {
    .global = detect_global,
    .global_remove = detect_global_remove,
};

static bool
detect(const struct comp_target_factory *ctf, struct comp_compositor *c)
{
	const char *wd = getenv("WAYLAND_DISPLAY");
	if (wd == NULL || wd[0] == '\0') {
		return false;
	}
	struct wl_display *display = wl_display_connect(NULL);
	if (display == NULL) {
		return false;
	}
	struct detect_state st = {0};
	struct wl_registry *registry = wl_display_get_registry(display);
	wl_registry_add_listener(registry, &detect_registry_listener, &st);
	// The globals, then each lease device's connectors, then their names and descriptions.
	for (int i = 0; i < 3; i++) {
		wl_display_roundtrip(display);
	}
	for (int i = 0; i < st.conn_count; i++) {
		wp_drm_lease_connector_v1_destroy(st.conns[i]);
	}
	for (int i = 0; i < st.device_count; i++) {
		wp_drm_lease_device_v1_destroy(st.devices[i]);
	}
	wl_registry_destroy(registry);
	wl_display_disconnect(display);

	if (st.connectors <= 0) {
		COMP_INFO(c, "Wayland desktop offers no display for lease%s",
		          st.device_count == 0 ? " (no drm-lease support)" : "");
		return false;
	}
	COMP_INFO(c, "Wayland desktop offers %d display(s) for lease%s", st.connectors,
	          st.rift ? ", the Rift among them" : "");
	return true;
}

static bool
create_target(const struct comp_target_factory *ctf, struct comp_compositor *c, struct comp_target **out_ct)
{
	struct comp_target *ct = comp_window_direct_wayland_create(c);
	if (ct == NULL) {
		return false;
	}

	*out_ct = ct;

	return true;
}

const struct comp_target_factory comp_target_factory_direct_wayland = {
    .name = "Wayland Direct-Mode",
    .identifier = "direct_wayland",
    .requires_vulkan_for_create = false,
    .is_deferred = false,
    .required_instance_version = 0,
    .required_instance_extensions = instance_extensions,
    .required_instance_extension_count = ARRAY_SIZE(instance_extensions),
    .optional_device_extensions = NULL,
    .optional_device_extension_count = 0,
    .detect = detect,
    .create_target = create_target,
};
