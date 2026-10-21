// Copyright 2020-2024, Collabora, Ltd.
// Copyright 2024-2026, NVIDIA CORPORATION.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Server process functions.
 * @author Pete Black <pblack@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup ipc_server
 */

#include "xrt/xrt_device.h"
#include "xrt/xrt_results.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_instance.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_config_have.h"
#include "xrt/xrt_config_os.h"

#include "os/os_time.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_trace_marker.h"
#include "util/u_verify.h"
#include "util/u_process.h"
#include "util/u_debug_gui.h"
#include "util/u_pretty_print.h"
#include "math/m_mathinclude.h"
#include "math/m_api.h"

#include "util/u_git_tag.h"

#include "shared/ipc_protocol.h"
#include "shared/ipc_shmem.h"
#include "server/ipc_server.h"
#include "server/ipc_server_objects.h"
#include "server/ipc_server_interface.h"
#ifdef RIFTLINUX_HAVE_PANEL
#include "server/ipc_server_rift_panel.h"
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <limits.h>

#if defined(XRT_OS_WINDOWS)
#include <timeapi.h>
#endif


/*
 *
 * Defines and helpers.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(exit_when_idle, "IPC_EXIT_WHEN_IDLE", false)
DEBUG_GET_ONCE_NUM_OPTION(exit_when_idle_delay_ms, "IPC_EXIT_WHEN_IDLE_DELAY_MS", 5000)
DEBUG_GET_ONCE_LOG_OPTION(ipc_log, "IPC_LOG", U_LOGGING_INFO)

/*
 * "XRT_NO_STDIN" option disables stdin and prevents monado-service from terminating.
 * This could be useful for situations where there is no proper or in a non-interactive shell.
 * Two example scenarios are:
 *    * IDE terminals,
 *    * Some scripting environments where monado-service is spawned in the background
 */
DEBUG_GET_ONCE_BOOL_OPTION(no_stdin, "XRT_NO_STDIN", false)
//! Recenter the local spaces whenever the head device detects being put on.
DEBUG_GET_ONCE_BOOL_OPTION(recenter_on_head_detect, "IPC_RECENTER_ON_HEAD_DETECT", false)
DEBUG_GET_ONCE_BOOL_OPTION(recenter_on_system_hold, "IPC_RECENTER_ON_SYSTEM_HOLD", true)


/*
 *
 * Idev functions.
 *
 */

static int32_t
find_xdev_index(struct ipc_server *s, struct xrt_device *xdev)
{
	if (xdev == NULL) {
		return -1;
	}

	for (int32_t i = 0; i < XRT_SYSTEM_MAX_DEVICES; i++) {
		if (s->xsysd->static_xdevs[i] == xdev) {
			return i;
		}
	}

	IPC_WARN(s, "Could not find index for xdev: '%s'", xdev->str);

	return -1;
}


/*
 *
 * Static functions.
 *
 */

XRT_MAYBE_UNUSED static void
print_linux_end_user_failed_information(enum u_logging_level log_level)
{
	struct u_pp_sink_stack_only sink;
	u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);

	// Print Newline
#define PN() u_pp(dg, "\n")
	// Print Newline, Hash, Space
#define PNH() u_pp(dg, "\n#")
	// Print Newline, Hash, Space
#define PNHS(...) u_pp(dg, "\n# "__VA_ARGS__)
	// Print Newline, 80 Hashes
#define PN80H()                                                                                                        \
	do {                                                                                                           \
		PN();                                                                                                  \
		for (uint32_t i = 0; i < 8; i++) {                                                                     \
			u_pp(dg, "##########");                                                                        \
		}                                                                                                      \
	} while (false)

	PN80H();
	PNHS("                                                                             #");
	PNHS("                 The Monado service has failed to start.                     #");
	PNHS("                                                                             #");
	PNHS("If you want to report please upload the logs of the service as a text file.  #");
	PNHS("You can also capture the output the monado-cli info command to provide more  #");
	PNHS("information about your system, that will help diagnosing your problem. The   #");
	PNHS("below commands is how you best capture the information from the commands.    #");
	PNHS("                                                                             #");
	PNHS("    monado-cli info 2>&1 | tee info.txt                                      #");
	PNHS("    monado-service 2>&1 | tee logs.txt                                       #");
	PNHS("                                                                             #");
	PN80H();

	U_LOG_IFL_I(log_level, "%s", sink.buffer);
}

XRT_MAYBE_UNUSED static void
print_linux_end_user_started_information(enum u_logging_level log_level)
{
	struct u_pp_sink_stack_only sink;
	u_pp_delegate_t dg = u_pp_sink_stack_only_init(&sink);


	PN80H();
	PNHS("                                                                             #");
	PNHS("                       The Monado service has started.                       #");
	PNHS("                                                                             #");
	PN80H();

#undef PN
#undef PNH
#undef PNHS
#undef PN80H

	U_LOG_IFL_I(log_level, "%s", sink.buffer);
}

#ifdef XRT_OS_WINDOWS
/*!
 * Cancel the blocking read each client thread is sitting in.
 *
 * The epoll based client loop wakes up on its own timeout and so notices
 * `s->running` going false, but the Windows one blocks in ReadFile() with no
 * timeout and would only notice once the client happens to send something.
 * Cancelling the pending I/O makes that read fail so the thread can run its
 * shutdown and be joined.
 *
 * The client thread closes this handle in common_shutdown() while holding the
 * global state lock, so take it here as well, otherwise we could cancel I/O on
 * a handle that has just been closed and possibly reused.
 */
static void
cancel_all_client_io(struct ipc_server *s)
{
	os_mutex_lock(&s->global_state.lock);

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		struct ipc_thread *it = &s->threads[i];
		if (it->state == IPC_THREAD_READY) {
			continue;
		}

		xrt_ipc_handle_t ipc_handle = it->ics.imc.ipc_handle;
		if (!xrt_ipc_handle_is_valid(ipc_handle)) {
			continue;
		}

		CancelIoEx(ipc_handle, NULL);
	}

	os_mutex_unlock(&s->global_state.lock);
}
#endif // XRT_OS_WINDOWS

/*!
 * Join any still-running per-client threads.
 *
 * On shutdown @ref main_loop returns as soon as `s->running` is cleared, but the
 * per-client threads may still be inside common_shutdown(), which destroys each
 * client's compositor and dereferences `s->xsysd` / `s->xso`. Waiting for them
 * here, before teardown_all() destroys those resources, avoids a race where a
 * client thread locks a mutex the teardown below has already freed (observed as
 * an `om->initialized` assertion in multi_compositor_destroy). `s->running` is
 * already false by this point, so the epoll based client loop exits on its next
 * timeout; the Windows one is woken by @ref cancel_all_client_io.
 */
static void
join_all_client_threads(struct ipc_server *s)
{
#ifdef XRT_OS_WINDOWS
	cancel_all_client_io(s);
#endif

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		struct ipc_thread *it = &s->threads[i];
		if (it->state == IPC_THREAD_READY) {
			continue;
		}
		os_thread_join(&it->thread);
		os_thread_destroy(&it->thread);
		it->state = IPC_THREAD_READY;
	}
}

static void
teardown_all(struct ipc_server *s)
{
	u_var_remove_root(s);

	// Client threads reference the resources destroyed below; wait them out first.
	join_all_client_threads(s);

	xrt_syscomp_destroy(&s->xsysc);

	xrt_space_overseer_destroy(&s->xso);
	xrt_system_devices_destroy(&s->xsysd);
	xrt_system_destroy(&s->xsys);

	xrt_instance_destroy(&s->xinst);

	ipc_server_mainloop_deinit(&s->ml);

	u_process_destroy(s->process);

	// Destroyed last.
	os_mutex_destroy(&s->global_state.lock);
}

XRT_CHECK_RESULT static xrt_result_t
init_shm_and_instance_state(struct ipc_server *s, volatile struct ipc_client_state *ics)
{
	const size_t size = sizeof(struct ipc_shared_memory);
	xrt_shmem_handle_t handle;

	xrt_result_t xret = ipc_shmem_create(size, &handle, (void **)&s->isms[ics->server_thread_index]);
	IPC_CHK_AND_RET(s, xret, "ipc_shmem_create");

	// we have a filehandle, we will pass this to our client
	ics->ism_handle = handle;

	// Convenience
	struct ipc_shared_memory *ism = s->isms[ics->server_thread_index];

	// Clients expect git version info and timestamp available upon connect.
	snprintf(ism->u_git_tag, IPC_VERSION_NAME_LEN, "%s", u_git_tag);

	/*
	 * Used to synchronize all client's xrt_instance::startup_timestamp.
	 * Add random variation of ±5 minutes around the global runtime startup timestamp
	 * so clients are close to each other but not exactly the same.
	 */
	const int64_t variation_range_ns = (int64_t)(10 * 60) * U_TIME_1S_IN_NS; // 10 minutes total range
	const int64_t half_range_ns = (int64_t)(5 * 60) * U_TIME_1S_IN_NS;       // 5 minutes
	int64_t random_offset_ns = ((int64_t)rand() % variation_range_ns) - half_range_ns;
	ism->startup_timestamp = s->start_of_time_timestamp_ns + random_offset_ns;

	return XRT_SUCCESS;
}

static void
init_system_shm_state(struct ipc_server *s, volatile struct ipc_client_state *ics)
{
	struct ipc_shared_memory *ism = get_ism(ics);
	xrt_result_t xret = XRT_SUCCESS;

	/*
	 * Loop over all of the devices to pre-populate the device IDs,
	 * this also populates the tracking origins.
	 */
	for (size_t i = 0; i < XRT_SYSTEM_MAX_DEVICES; i++) {
		struct xrt_device *xdev = s->xsysd->static_xdevs[i];
		if (xdev == NULL) {
			continue;
		}

		// Populate the tracking origin.
		uint32_t tracking_origin_id = 0;
		xret = ipc_server_objects_get_xtrack_id_or_add(ics, xdev->tracking_origin, &tracking_origin_id);
		if (xret != XRT_SUCCESS) {
			IPC_ERROR(s, "Failed to get/add tracking origin ID for: '%s'", xdev->tracking_origin->name);
			continue;
		}

		// Assign a per-client slot index for IPC lookups (not xrt_device::id).
		uint32_t device_id = 0;
		xret = ipc_server_objects_get_xdev_id_or_add(ics, xdev, &device_id);
		if (xret != XRT_SUCCESS) {
			IPC_ERROR(s, "Failed to get/add device ID for: '%s'", xdev->str);
			continue;
		}
	}

	// Setup the HMD
	// set view count
	const struct xrt_device *xhead = s->xsysd->static_roles.head;
	const struct xrt_hmd_parts *xhmd = xhead != NULL ? xhead->hmd : NULL;
	U_ZERO(&ism->hmd);
	if (xhmd != NULL) {
		ism->hmd.view_count = xhmd->view_count;
		for (uint32_t view = 0; view < xhmd->view_count; ++view) {
			ism->hmd.views[view].display.w_pixels = xhmd->views[view].display.w_pixels;
			ism->hmd.views[view].display.h_pixels = xhmd->views[view].display.h_pixels;
		}

		for (uint32_t i = 0; i < xhmd->blend_mode_count; i++) {
			// Not super necessary, we also do this assert in oxr_system.c
			assert(u_verify_blend_mode_valid(xhmd->blend_modes[i]));
			ism->hmd.blend_modes[i] = xhmd->blend_modes[i];
		}
		ism->hmd.blend_mode_count = xhmd->blend_mode_count;
	}

	// Assign all of the roles.
	ism->roles.head = find_xdev_index(s, s->xsysd->static_roles.head);
	ism->roles.eyes = find_xdev_index(s, s->xsysd->static_roles.eyes);
	ism->roles.face = find_xdev_index(s, s->xsysd->static_roles.face);
	ism->roles.body = find_xdev_index(s, s->xsysd->static_roles.body);

#define SET_HT_ROLE(SRC)                                                                                               \
	ism->roles.hand_tracking.SRC.left = find_xdev_index(s, s->xsysd->static_roles.hand_tracking.SRC.left);         \
	ism->roles.hand_tracking.SRC.right = find_xdev_index(s, s->xsysd->static_roles.hand_tracking.SRC.right);
	SET_HT_ROLE(unobstructed)
	SET_HT_ROLE(conforming)
#undef SET_HT_ROLE
}

static void
init_server_state(struct ipc_server *s)
{
	// set up initial state for global vars, and each client state

	s->global_state.active_client_index = -1; // we start off with no active client.
	s->global_state.last_active_client_index = -1;
	s->global_state.connected_client_count = 0; // No clients connected initially
	s->current_slot_index = 0;

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;
		ics->server = s;
		ics->server_thread_index = -1;
	}
}

static xrt_result_t
init_all(struct ipc_server *s,
         enum u_logging_level log_level,
         const struct ipc_server_callbacks *callbacks,
         void *callback_data,
         bool exit_on_disconnect)
{
	xrt_result_t xret = XRT_SUCCESS;
	int ret;

	// First order of business set the log level.
	s->log_level = log_level;

	// Store callbacks and data
	s->callbacks = callbacks;
	s->callback_data = callback_data;

	// This should never fail.
	ret = os_mutex_init(&s->global_state.lock);
	if (ret < 0) {
		IPC_ERROR(s, "Global state lock mutex failed to init!");
		// Do not call teardown_all here, os_mutex_destroy will assert.
		return XRT_ERROR_SYNC_PRIMITIVE_CREATION_FAILED;
	}

	s->process = u_process_create_if_not_running();
	if (!s->process) {
		IPC_ERROR(s, "monado-service is already running! Use XRT_LOG=trace for more information.");
		xret = XRT_ERROR_IPC_SERVICE_ALREADY_RUNNING;
	}
	IPC_CHK_WITH_GOTO(s, xret, "u_process_create_if_not_running", error);

	// Yes we should be running.
	s->running = true;
	s->exit_on_disconnect = exit_on_disconnect;
	s->exit_when_idle = debug_get_bool_option_exit_when_idle();
	s->last_client_disconnect_ns = 0;
	uint64_t delay_ms = debug_get_num_option_exit_when_idle_delay_ms();
	s->exit_when_idle_delay_ns = delay_ms * U_TIME_1MS_IN_NS;

	// See the comment in ipc_server.h for more details.
	const int64_t offset = (int64_t)(42 * 60) * U_TIME_1S_IN_NS;
	s->start_of_time_timestamp_ns = os_monotonic_get_ns() - offset;

	/*
	 * Initialize random number generator for startup timestamp variation.
	 * This not used for any security critical purposes, just to ensure that
	 * the startup timestamps are not exactly the same for all clients.
	 */
	srand((unsigned int)(os_monotonic_get_ns() / U_TIME_1MS_IN_NS));

	xret = xrt_instance_create(NULL, &s->xinst);
	IPC_CHK_WITH_GOTO(s, xret, "xrt_instance_create", error);

	ret = ipc_server_mainloop_init(&s->ml, s->no_stdin);
	if (ret < 0) {
		xret = XRT_ERROR_IPC_MAINLOOP_FAILED_TO_INIT;
	}
	IPC_CHK_WITH_GOTO(s, xret, "ipc_server_mainloop_init", error);

	// Never fails, do this second last.
	init_server_state(s);

	u_var_add_root(s, "IPC Server", false);
	u_var_add_log_level(s, &s->log_level, "Log level");
	u_var_add_bool(s, &s->exit_on_disconnect, "exit_on_disconnect");
	u_var_add_bool(s, &s->exit_when_idle, "exit_when_idle");
	u_var_add_u64(s, &s->exit_when_idle_delay_ns, "exit_when_idle_delay_ns");
	u_var_add_bool(s, (bool *)&s->running, "running");

	return XRT_SUCCESS;

error:
	teardown_all(s);

	return xret;
}

static void
shutdown_clients(struct ipc_server *s)
{
	os_mutex_lock(&s->global_state.lock);
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		struct ipc_thread *it = &s->threads[i];
		volatile struct ipc_client_state *ics = &it->ics;
		if (ics->client_state.info.immediate_disconnect) {
			// instantly disconnect clients with this flag
			// some clients (e.g. libmonado) have no session or event polling to respond to an exit request
			U_LOG_I("Disconnecting client (Client %d): %s", ics->client_state.id,
			        ics->client_state.info.application_name);
			it->state = IPC_THREAD_STOPPING;
			continue;
		}

		if (it->state == IPC_THREAD_READY || it->state == IPC_THREAD_STOPPING || ics->xs == NULL)
			continue;

		U_LOG_I("Requesting application %s exit", ics->client_state.info.application_name);
		xrt_result_t xret = xrt_session_request_exit(ics->xs);
		if (xret != XRT_SUCCESS) {
			U_LOG_E("Failed to request exit for %s!", ics->client_state.info.application_name);
		}
	}
	os_mutex_unlock(&s->global_state.lock);

	uint32_t connected_client_count = 0;
	int64_t end = os_monotonic_get_ns() + (int64_t)3 * U_TIME_1S_IN_NS;
	while (os_monotonic_get_ns() < end) {
		os_mutex_lock(&s->global_state.lock);
		connected_client_count = s->global_state.connected_client_count;
		os_mutex_unlock(&s->global_state.lock);

		if (connected_client_count == 0)
			break;
		os_nanosleep((int64_t)10 * U_TIME_1MS_IN_NS);
	}

	if (connected_client_count != 0) {
		U_LOG_W("%" PRIx32 " clients still connected after shutdown timeout!", connected_client_count);
		for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
			struct ipc_thread *it = &s->threads[i];
			if (it->state == IPC_THREAD_READY)
				continue;
			it->state = IPC_THREAD_STOPPING;
			os_thread_join(&it->thread);
			os_thread_destroy(&it->thread);
		}
	}
}

//! The checks run 20 times a second, see main_loop.
#define HEAD_CHECKS_PER_S 20
//! The head counts as settled when its heading has stayed within this many degrees for 1.5 s of checks.
#define RECENTER_SETTLED_DEG 8.0f
#define RECENTER_SETTLE_CHECKS (HEAD_CHECKS_PER_S * 3 / 2)
//! A put-on recenters after this long even if the head never settles.
#define RECENTER_SETTLE_TIMEOUT_NS (15 * (int64_t)U_TIME_1S_IN_NS)
//! How often the head's place in the app-facing spaces goes to the log, see log_spaces.
#define SPACES_LOG_PERIOD_NS (10 * (int64_t)U_TIME_1S_IN_NS)

//! The right controller, or NULL.
static struct xrt_device *
right_controller(struct ipc_server *s)
{
	for (size_t d = 0; d < s->xsysd->static_xdev_count; d++) {
		struct xrt_device *xdev = s->xsysd->static_xdevs[d];
		if (xdev != NULL && xdev->device_type == XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER) {
			return xdev;
		}
	}
	return NULL;
}

//! Logs where the view is in @p base as an application sees it: heading and pitch in degrees, position in meters.
static void
log_view_in(struct ipc_server *s, struct xrt_space *base, const char *name, int64_t at_ns)
{
	if (base == NULL || s->xso->semantic.view == NULL) {
		return;
	}
	const struct xrt_pose identity = XRT_POSE_IDENTITY;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	xrt_space_overseer_locate_space(s->xso, base, &identity, at_ns, s->xso->semantic.view, &identity, &rel);
	if ((rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		IPC_INFO(s, "View in %s: not located.", name);
		return;
	}
	const struct xrt_vec3 minus_z = {0.0f, 0.0f, -1.0f};
	struct xrt_vec3 forward;
	math_quat_rotate_vec3(&rel.pose.orientation, &minus_z, &forward);
	IPC_INFO(s, "View in %s: heading %.1f deg (positive is left), pitch %.1f deg, at %.2f %.2f %.2f m.", name,
	         atan2f(-forward.x, -forward.z) * (180.0f / (float)M_PI),
	         asinf(CLAMP(forward.y, -1.0f, 1.0f)) * (180.0f / (float)M_PI), rel.pose.position.x,
	         rel.pose.position.y, rel.pose.position.z);
}

/*!
 * Where the view is in LOCAL and STAGE, as an application locates it, and in the tracking root: a scene centered on
 * the user has the view at heading 0 in the space it is drawn in. In testing the scene was often off to the right
 * after recentering, and nothing in the log said where the spaces were.
 */
static void
log_spaces(struct ipc_server *s, const char *why)
{
	if (s->xso == NULL) {
		return;
	}
	const int64_t now_ns = os_monotonic_get_ns();
	IPC_INFO(s, "Spaces %s:", why);
	log_view_in(s, s->xso->semantic.local, "LOCAL", now_ns);
	log_view_in(s, s->xso->semantic.stage, "STAGE", now_ns);
	log_view_in(s, s->xso->semantic.root, "the tracking root", now_ns);
	s->spaces_logged_ns = now_ns;
}

static void
recenter(struct ipc_server *s, const char *why)
{
	xrt_result_t xret = xrt_space_overseer_recenter_local_spaces(s->xso);
	if (xret != XRT_SUCCESS) {
		IPC_WARN(s, "%s, but recentering the local spaces failed: %d", why, xret);
		return;
	}
	IPC_INFO(s, "%s, recentered the local spaces.", why);
	log_spaces(s, "after recentering");

	// A short buzz in the right hand, so the user knows it happened: without it there was no telling.
	struct xrt_device *right = right_controller(s);
	if (right != NULL && right->output_count > 0) {
		const struct xrt_output_value buzz = {
		    .type = XRT_OUTPUT_VALUE_TYPE_VIBRATION,
		    .vibration = {.frequency = XRT_FREQUENCY_UNSPECIFIED,
		                  .amplitude = 0.7f,
		                  .duration_ns = 150 * (int64_t)U_TIME_1MS_IN_NS},
		};
		xrt_device_set_output(right, right->outputs[0].name, &buzz);
	}
}

//! The head's heading and pitch, in degrees, false when it has no orientation.
static bool
head_heading(struct xrt_device *head, int64_t at_ns, float *out_yaw_deg, float *out_pitch_deg)
{
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (xrt_device_get_tracked_pose(head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return false;
	}
	const struct xrt_vec3 minus_z = {0.0f, 0.0f, -1.0f};
	struct xrt_vec3 forward;
	math_quat_rotate_vec3(&rel.pose.orientation, &minus_z, &forward);
	*out_yaw_deg = atan2f(-forward.x, -forward.z) * (180.0f / (float)M_PI);
	*out_pitch_deg = asinf(CLAMP(forward.y, -1.0f, 1.0f)) * (180.0f / (float)M_PI);
	return true;
}

/*!
 * Recenters the local spaces when the head device goes from not worn to worn, so a scene is centered on the user
 * wherever the headset was when the application started, with no button to press.
 *
 * Not at the put-on itself but once the head has settled: putting the headset on, the head is still turning, and the
 * scene ended up off to one side, recentered on where the head pointed for that moment.
 */
static void
check_head_detect(struct ipc_server *s)
{
	if (!debug_get_bool_option_recenter_on_head_detect() || s->xsysd == NULL || s->xso == NULL) {
		return;
	}

	struct xrt_device *head = s->xsysd->static_roles.head;
	if (head == NULL || !head->supported.presence || xrt_device_update_inputs(head) != XRT_SUCCESS) {
		return;
	}

	bool worn = false;
	for (uint32_t i = 0; i < head->input_count; i++) {
		if (head->inputs[i].name == XRT_INPUT_GENERIC_HEAD_DETECT) {
			worn = head->inputs[i].value.boolean;
		}
	}

	const int64_t now_ns = os_monotonic_get_ns();
	if (!worn && s->head_not_worn_since_ns == 0) {
		s->head_not_worn_since_ns = now_ns;
	}

	/*
	 * Only a put-on after a while off counts: the CV1's proximity sensor flickers when something is near it, and
	 * every recenter moves the scene.
	 */
	// Never worn yet counts as off long enough, so a headset worn from the start still gets recentered once.
	const bool off_long_enough =
	    !s->head_ever_worn ||
	    (s->head_not_worn_since_ns != 0 && now_ns - s->head_not_worn_since_ns > 2 * (int64_t)U_TIME_1S_IN_NS);
	if (worn) {
		s->head_not_worn_since_ns = 0;
	}

	if (worn && !s->head_was_worn && off_long_enough) {
		s->head_ever_worn = true;
		s->recenter_pending_since_ns = now_ns;
		s->recenter_yaw_count = 0;
	}
	s->head_was_worn = worn;

	if (s->recenter_pending_since_ns == 0) {
		return;
	}
	if (!worn) {
		s->recenter_pending_since_ns = 0; // Taken off again, the next put-on starts over.
		return;
	}
	if (now_ns - s->recenter_pending_since_ns > RECENTER_SETTLE_TIMEOUT_NS) {
		s->recenter_pending_since_ns = 0;
		recenter(s, "Headset put on, the head never settled");
		return;
	}

	float yaw_deg, pitch_deg;
	if (!head_heading(head, now_ns, &yaw_deg, &pitch_deg)) {
		return;
	}
	const uint32_t n = ARRAY_SIZE(s->recenter_yaw_deg);
	s->recenter_yaw_deg[s->recenter_yaw_count % n] = yaw_deg;
	s->recenter_yaw_count++;
	if (s->recenter_yaw_count < RECENTER_SETTLE_CHECKS || fabsf(pitch_deg) > 35.0f) {
		return;
	}

	// Spread of the heading over the last checks, around the newest so that it is right across +-180 degrees.
	float lo = 0.0f, hi = 0.0f;
	for (uint32_t i = 1; i < RECENTER_SETTLE_CHECKS; i++) {
		float d = s->recenter_yaw_deg[(s->recenter_yaw_count - 1 - i) % n] - yaw_deg;
		d = d > 180.0f ? d - 360.0f : (d < -180.0f ? d + 360.0f : d);
		lo = fminf(lo, d);
		hi = fmaxf(hi, d);
	}
	if (hi - lo > RECENTER_SETTLED_DEG) {
		return;
	}

	s->recenter_pending_since_ns = 0;
	recenter(s, "Headset put on and the head settled");
}

/*!
 * Pressing the right controller's system button (the Touch's Oculus button) recenters, as the Oculus runtime's "reset
 * view" does: the way to put the scene in front of you whenever it is not. On a press rather than a held second: a
 * tap did nothing in testing, and with no menu on the button here a tap means nothing else.
 */
static void
check_system_hold(struct ipc_server *s)
{
	if (!debug_get_bool_option_recenter_on_system_hold() || s->xsysd == NULL || s->xso == NULL) {
		return;
	}

	bool held = false;
	struct xrt_device *right = right_controller(s);
	if (right == NULL) {
		return;
	}
	for (uint32_t i = 0; i < right->input_count; i++) {
		if (right->inputs[i].name != XRT_INPUT_TOUCH_SYSTEM_CLICK) {
			continue;
		}
		if (xrt_device_update_inputs(right) == XRT_SUCCESS) {
			held = right->inputs[i].value.boolean;
		}
		break;
	}

	// Logged: in a recorded session a hold did nothing and it was not known whether the press arrived.
	if (held != (s->system_hold_since_ns != 0)) {
		IPC_INFO(s, "Right controller's system button %s.", held ? "pressed" : "released");
	}

	const int64_t now_ns = os_monotonic_get_ns();
#ifdef RIFTLINUX_HAVE_PANEL
	/*
	 * With the Rift panel, as in SteamVR: a press opens or closes it, held for a second recenters (and closes it).
	 */
	if (s->panel != NULL) {
		if (!held) {
			if (s->system_hold_since_ns != 0 && !s->system_hold_done) {
				rl_panel_toggle(s->panel);
			}
			s->system_hold_since_ns = 0;
			s->system_hold_done = false;
			return;
		}
		if (s->system_hold_since_ns == 0) {
			s->system_hold_since_ns = now_ns;
		}
		if (!s->system_hold_done && now_ns - s->system_hold_since_ns >= U_TIME_1S_IN_NS) {
			s->system_hold_done = true;
			recenter(s, "System button held");
		}
		return;
	}
#endif
	if (!held) {
		s->system_hold_since_ns = 0;
		s->system_hold_done = false;
		return;
	}
	if (s->system_hold_since_ns == 0) {
		s->system_hold_since_ns = now_ns;
	}
	if (!s->system_hold_done) {
		s->system_hold_done = true;
		recenter(s, "System button pressed");
	}
}

static int
main_loop(struct ipc_server *s)
{
	while (s->running) {
		os_nanosleep(U_TIME_1S_IN_NS / 20);

		// Check polling.
		ipc_server_mainloop_poll(s, &s->ml);

		check_head_detect(s);
#ifdef RIFTLINUX_HAVE_PANEL
		// The panel once the system is up (it may come up only with the first client).
		if (s->panel == NULL && !s->panel_failed && s->xsysc != NULL) {
			s->panel = rl_panel_create(s);
			s->panel_failed = s->panel == NULL;
		}
#endif
		check_system_hold(s);

		if (s->xso != NULL && os_monotonic_get_ns() - s->spaces_logged_ns > SPACES_LOG_PERIOD_NS) {
			log_spaces(s, "now");
		}
	}

	shutdown_clients(s);
#ifdef RIFTLINUX_HAVE_PANEL
	rl_panel_destroy(&s->panel);
#endif

	return 0;
}


/*
 *
 * Client management functions.
 *
 */

static void
handle_overlay_client_events(volatile struct ipc_client_state *ics, int active_id, int prev_active_id)
{
	// Is an overlay session?
	if (!ics->client_state.session_overlay) {
		return;
	}

	// Does this client have a compositor yet, if not return?
	if (ics->xc == NULL) {
		return;
	}

	// Switch between main applications
	if (active_id >= 0 && prev_active_id >= 0) {
		xrt_syscomp_set_main_app_visibility(ics->server->xsysc, ics->xc, false);
		xrt_syscomp_set_main_app_visibility(ics->server->xsysc, ics->xc, true);
	}

	// Switch from idle to active application
	if (active_id >= 0 && prev_active_id < 0) {
		xrt_syscomp_set_main_app_visibility(ics->server->xsysc, ics->xc, true);
	}

	// Switch from active application to idle
	if (active_id < 0 && prev_active_id >= 0) {
		xrt_syscomp_set_main_app_visibility(ics->server->xsysc, ics->xc, false);
	}
}

static void
handle_focused_client_events(volatile struct ipc_client_state *ics, int active_id, int prev_active_id)
{
	// Set start z_order at the bottom.
	int64_t z_order = INT64_MIN;

	// Set visibility/focus to false on all applications.
	bool focused = false;
	bool visible = false;

	// Set visible + focused if we are the primary application; not focused while the Rift panel is open.
	if (ics->server_thread_index == active_id) {
		visible = true;
		focused = !ics->server->panel_open;
		z_order = INT64_MIN;
	}

	// Set all overlays to always active and focused.
	if (ics->client_state.session_overlay) {
		visible = true;
		focused = true;
		z_order = ics->client_state.z_order;
	}

	ics->client_state.session_visible = visible;
	ics->client_state.session_focused = focused;
	ics->client_state.z_order = z_order;

	if (ics->xc != NULL) {
		xrt_syscomp_set_state(ics->server->xsysc, ics->xc, visible, focused, os_monotonic_get_ns());
		xrt_syscomp_set_z_order(ics->server->xsysc, ics->xc, z_order);
	}
}

static void
flush_state_to_all_clients_locked(struct ipc_server *s)
{
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;

		// Not running?
		if (ics->server_thread_index < 0) {
			continue;
		}

		handle_focused_client_events(ics, s->global_state.active_client_index,
		                             s->global_state.last_active_client_index);
		handle_overlay_client_events(ics, s->global_state.active_client_index,
		                             s->global_state.last_active_client_index);
	}
}

void
ipc_server_set_panel_focus(struct ipc_server *s, bool panel_open)
{
	os_mutex_lock(&s->global_state.lock);
	s->panel_open = panel_open;
	flush_state_to_all_clients_locked(s);
	os_mutex_unlock(&s->global_state.lock);
}

static void
update_server_state_locked(struct ipc_server *s)
{
	// if our client that is set to active is still active,
	// and it is the same as our last active client, we can
	// early-out, as no events need to be sent

	if (s->global_state.active_client_index >= 0) {

		volatile struct ipc_client_state *ics = &s->threads[s->global_state.active_client_index].ics;

		if (ics->client_state.session_active &&
		    s->global_state.active_client_index == s->global_state.last_active_client_index) {
			return;
		}
	}


	// our active application has changed - this would typically be
	// switched by the monado-ctl application or other app making a
	// 'set active application' ipc call, or it could be a
	// connection loss resulting in us needing to 'fall through' to
	// the first active application
	//, or finally to the idle 'wallpaper' images.


	bool set_idle = true;
	int fallback_active_application = -1;

	// do we have a fallback application?
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;
		if (ics->client_state.session_overlay == false && ics->server_thread_index >= 0 &&
		    ics->client_state.session_active) {
			fallback_active_application = i;
			set_idle = false;
		}
	}

	// if there is a currently-set active primary application and it is not
	// actually active/displayable, use the fallback application
	// instead.
	if (s->global_state.active_client_index >= 0) {
		volatile struct ipc_client_state *ics = &s->threads[s->global_state.active_client_index].ics;
		if (!(ics->client_state.session_overlay == false && ics->client_state.session_active)) {
			s->global_state.active_client_index = fallback_active_application;
		}
	}


	// if we have no applications to fallback to, enable the idle
	// wallpaper.
	if (set_idle) {
		s->global_state.active_client_index = -1;
	}

	flush_state_to_all_clients_locked(s);

	s->global_state.last_active_client_index = s->global_state.active_client_index;
}

static volatile struct ipc_client_state *
find_client_locked(struct ipc_server *s, uint32_t client_id)
{
	// Check for invalid IDs.
	if (client_id == 0 || client_id > INT_MAX) {
		IPC_WARN(s, "Invalid ID '%u', failing operation.", client_id);
		return NULL;
	}

	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *ics = &s->threads[i].ics;

		// Is this the client we are looking for?
		if (ics->client_state.id != client_id) {
			continue;
		}

		// Just in case of state data.
		if (!xrt_ipc_handle_is_valid(ics->imc.ipc_handle)) {
			IPC_WARN(s, "Encountered invalid state while searching for client with ID '%d'", client_id);
			return NULL;
		}

		return ics;
	}

	IPC_WARN(s, "No client with ID '%u', failing operation.", client_id);

	return NULL;
}

static xrt_result_t
get_client_app_state_locked(struct ipc_server *s, uint32_t client_id, struct ipc_app_state *out_ias)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	struct ipc_app_state ias = ics->client_state;

	// @todo: track this data in the ipc_client_state struct
	ias.primary_application = false;

	// The active client is decided by index, so get that from the ics.
	int index = ics->server_thread_index;

	if (s->global_state.active_client_index == index) {
		ias.primary_application = true;
	}

	*out_ias = ias;

	return XRT_SUCCESS;
}

static xrt_result_t
set_active_client_locked(struct ipc_server *s, uint32_t client_id)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	// The active client is decided by index, so get that from the ics.
	int index = ics->server_thread_index;

	if (index != s->global_state.active_client_index) {
		s->global_state.active_client_index = index;
		update_server_state_locked(s);
	}

	return XRT_SUCCESS;
}

static xrt_result_t
toggle_io_client_locked(struct ipc_server *s, uint32_t client_id)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	volatile struct ipc_client_io_blocks *iob = &ics->client_state.io_blocks;
	bool io_active = iob->block_poses && iob->block_hand_tracking && iob->block_inputs && iob->block_outputs;
	iob->block_poses = iob->block_hand_tracking = iob->block_inputs = iob->block_outputs = !io_active;

	return XRT_SUCCESS;
}

static xrt_result_t
set_client_io_blocks_locked(struct ipc_server *s, uint32_t client_id, const struct ipc_client_io_blocks *blocks)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	volatile struct ipc_client_io_blocks *iob = &ics->client_state.io_blocks;
	*iob = *blocks;

	return XRT_SUCCESS;
}

static xrt_result_t
get_client_session_running_state_locked(struct ipc_server *s,
                                        uint32_t client_id,
                                        struct xrt_compositor_session_running_state *out_running_state)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	xrt_result_t xret = xrt_syscomp_session_get_running_state(s->xsysc, ics->xc, out_running_state);
	if (xret != XRT_SUCCESS) {
		IPC_ERROR(s, "Failed to get session running state for client '%u', error: '%s'", client_id,
		          u_str_xrt_result_or_null(xret));
		return xret;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
get_client_view_config_locked(struct ipc_server *s,
                              uint32_t client_id,
                              enum xrt_view_type view_type,
                              struct xrt_view_config *out_default_view_config,
                              struct xrt_recommended_view_config *out_recommended_view_config)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	// The default view config comes from the system compositor, not from the client, so it is always available.
	xrt_result_t xret = xrt_syscomp_get_view_config(s->xsysc, view_type, out_default_view_config);
	if (xret != XRT_SUCCESS) {
		IPC_ERROR(s, "Failed to get default view config for client '%u' and view type '%d', error: '%s'",
		          client_id, view_type, u_str_xrt_result_or_null(xret));
		return xret;
	}

	// A client without any app system has no recommendation.
	(*out_recommended_view_config) = (struct xrt_recommended_view_config){
	    .valid = false,
	};

	for (uint32_t i = 0; i < ARRAY_SIZE(ics->objects.xasys); i++) {
		struct xrt_app_system *xasys = ics->objects.xasys[i];
		if (xasys == NULL) {
			continue;
		}

		xret = xrt_app_system_get_recommended_view_configuration( //
		    xasys,                                                //
		    view_type,                                            //
		    out_recommended_view_config);
		if (xret != XRT_SUCCESS) {
			IPC_ERROR(
			    s, "Failed to get recommended view config for client '%u' and view type '%d', error: '%s'",
			    client_id, view_type, u_str_xrt_result_or_null(xret));
			return xret;
		}

		// Only return the first system's view config
		break;
	}

	return XRT_SUCCESS;
}

static xrt_result_t
set_client_recommended_view_config_locked(struct ipc_server *s,
                                          uint32_t client_id,
                                          enum xrt_view_type view_type,
                                          const struct xrt_recommended_view_config *recommended_view_config)
{
	volatile struct ipc_client_state *ics = find_client_locked(s, client_id);
	if (ics == NULL) {
		return XRT_ERROR_IPC_FAILURE;
	}

	for (uint32_t i = 0; i < ARRAY_SIZE(ics->objects.xasys); i++) {
		struct xrt_app_system *xasys = ics->objects.xasys[i];
		if (xasys == NULL) {
			continue;
		}

		xrt_result_t xret = xrt_app_system_set_recommended_view_configuration( //
		    xasys,                                                             //
		    view_type,                                                         //
		    recommended_view_config);
		if (xret != XRT_SUCCESS) {
			IPC_ERROR(
			    s, "Failed to set recommended view config for client '%u' and view type '%d', error: '%s'",
			    client_id, view_type, u_str_xrt_result_or_null(xret));
			return xret;
		}

		// Only set the first system's view config
		break;
	}

	return XRT_SUCCESS;
}

static uint32_t
allocate_id_locked(struct ipc_server *s)
{
	uint32_t id = 0;
	while (id == 0) {
		// Allocate a new one.
		id = ++s->id_generator;

		for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
			volatile struct ipc_client_state *ics = &s->threads[i].ics;

			// If we find the ID, get a new one by setting to zero.
			if (ics->client_state.id == id) {
				id = 0;
				break;
			}
		}
	}

	// Paranoia.
	if (id == 0) {
		U_LOG_E("Got app(client) id 0, not allowed!");
		assert(id > 0);
	}

	return id;
}


/*
 *
 * Exported functions.
 *
 */

xrt_result_t
ipc_server_init_system_if_available_locked(struct ipc_server *s,
                                           volatile struct ipc_client_state *ics,
                                           bool *out_available)
{
	xrt_result_t xret = XRT_SUCCESS;

	bool available = false;

	if (s->xsys) {
		available = true;
	} else {
		xret = xrt_instance_is_system_available(s->xinst, &available);
		IPC_CHK_WITH_GOTO(s, xret, "xrt_instance_is_system_available", error);

		if (available) {
			xret = xrt_instance_create_system(s->xinst, &s->xsys, &s->xsysd, &s->xso, &s->xsysc);
			IPC_CHK_WITH_GOTO(s, xret, "xrt_instance_create_system", error);
		}
	}

	if (available && ics != NULL && !ics->has_init_shm_system) {
		init_system_shm_state(s, ics);
		ics->has_init_shm_system = true;
	}

	if (out_available) {
		*out_available = available;
	}

	return XRT_SUCCESS;

error:
	return xret;
}

xrt_result_t
ipc_server_get_client_app_state(struct ipc_server *s, uint32_t client_id, struct ipc_app_state *out_ias)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = get_client_app_state_locked(s, client_id, out_ias);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_set_active_client(struct ipc_server *s, uint32_t client_id)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = set_active_client_locked(s, client_id);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_toggle_io_client(struct ipc_server *s, uint32_t client_id)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = toggle_io_client_locked(s, client_id);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_set_client_io_blocks(struct ipc_server *s, uint32_t client_id, const struct ipc_client_io_blocks *blocks)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = set_client_io_blocks_locked(s, client_id, blocks);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_get_client_session_running_state(struct ipc_server *s,
                                            uint32_t client_id,
                                            struct xrt_compositor_session_running_state *out_running_state)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = get_client_session_running_state_locked(s, client_id, out_running_state);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_get_client_view_config(struct ipc_server *s,
                                  uint32_t client_id,
                                  enum xrt_view_type view_type,
                                  struct xrt_view_config *out_default_view_config,
                                  struct xrt_recommended_view_config *out_recommended_view_config)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = get_client_view_config_locked(s, client_id, view_type, out_default_view_config,
	                                                  out_recommended_view_config);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

xrt_result_t
ipc_server_set_client_recommended_view_config(struct ipc_server *s,
                                              uint32_t client_id,
                                              enum xrt_view_type view_type,
                                              const struct xrt_recommended_view_config *recommended_view_config)
{
	os_mutex_lock(&s->global_state.lock);
	xrt_result_t xret = set_client_recommended_view_config_locked(s, client_id, view_type, recommended_view_config);
	os_mutex_unlock(&s->global_state.lock);

	return xret;
}

void
ipc_server_activate_session(volatile struct ipc_client_state *ics)
{
	struct ipc_server *s = ics->server;

	// Already active, noop.
	if (ics->client_state.session_active) {
		return;
	}

	assert(ics->server_thread_index >= 0);

	// Multiple threads could call this at the same time.
	os_mutex_lock(&s->global_state.lock);

	ics->client_state.session_active = true;

	if (ics->client_state.session_overlay) {
		// For new active overlay sessions only update this session.
		handle_focused_client_events(ics, s->global_state.active_client_index,
		                             s->global_state.last_active_client_index);
		handle_overlay_client_events(ics, s->global_state.active_client_index,
		                             s->global_state.last_active_client_index);
	} else {
		// Update active client
		set_active_client_locked(s, ics->client_state.id);
	}

	os_mutex_unlock(&s->global_state.lock);
}

void
ipc_server_deactivate_session(volatile struct ipc_client_state *ics)
{
	struct ipc_server *s = ics->server;

	// Multiple threads could call this at the same time.
	os_mutex_lock(&s->global_state.lock);

	ics->client_state.session_active = false;

	update_server_state_locked(s);

	os_mutex_unlock(&s->global_state.lock);
}

void
ipc_server_update_state(struct ipc_server *s)
{
	// Multiple threads could call this at the same time.
	os_mutex_lock(&s->global_state.lock);

	update_server_state_locked(s);

	os_mutex_unlock(&s->global_state.lock);
}

void
ipc_server_handle_failure(struct ipc_server *vs)
{
	// Right now handled just the same as a graceful shutdown.
	vs->running = false;
}

void
ipc_server_handle_shutdown_signal(struct ipc_server *vs)
{
	vs->running = false;
}

void
ipc_server_handle_client_connected(struct ipc_server *vs, xrt_ipc_handle_t ipc_handle)
{
	volatile struct ipc_client_state *ics = NULL;
	int32_t cs_index = -1;

	os_mutex_lock(&vs->global_state.lock);

	// Increment the connected client counter
	vs->global_state.connected_client_count++;

	// A client connected, so we're no longer in a delayed exit state
	// (The delay thread will still check the client count before exiting)
	vs->last_client_disconnect_ns = 0;

	// find the next free thread in our array (server_thread_index is -1)
	// and have it handle this connection
	for (uint32_t i = 0; i < IPC_MAX_CLIENTS; i++) {
		volatile struct ipc_client_state *_cs = &vs->threads[i].ics;
		if (_cs->server_thread_index < 0) {
			ics = _cs;
			cs_index = i;
			break;
		}
	}
	if (ics == NULL) {
		xrt_ipc_handle_close(ipc_handle);

		// Unlock when we are done.
		os_mutex_unlock(&vs->global_state.lock);

		U_LOG_E("Max client count reached!");
		return;
	}

	struct ipc_thread *it = &vs->threads[cs_index];
	if (it->state != IPC_THREAD_READY && it->state != IPC_THREAD_STOPPING) {
		// we should not get here
		xrt_ipc_handle_close(ipc_handle);

		// Unlock when we are done.
		os_mutex_unlock(&vs->global_state.lock);

		U_LOG_E("Client state management error!");
		return;
	}

	if (it->state != IPC_THREAD_READY) {
		os_thread_join(&it->thread);
		os_thread_destroy(&it->thread);
		it->state = IPC_THREAD_READY;
	}

	it->state = IPC_THREAD_STARTING;

	// Allocate a new ID, avoid zero.
	uint32_t id = allocate_id_locked(vs);

	// Reset everything.
	U_ZERO((struct ipc_client_state *)ics);

	// Set state.
	ics->local_space_overseer_index = UINT32_MAX;
	ics->client_state.id = id;
	ics->imc.ipc_handle = ipc_handle;
	ics->server = vs;
	ics->server_thread_index = cs_index;

	ics->plane_detection_size = 0;
	ics->plane_detection_count = 0;
	ics->plane_detection_ids = NULL;
	ics->plane_detection_xdev = NULL;

	xrt_result_t xret = init_shm_and_instance_state(vs, ics);
	if (xret != XRT_SUCCESS) {

		// Unlock when we are done.
		os_mutex_unlock(&vs->global_state.lock);

		U_LOG_E("Failed to allocate shared memory!");
		return;
	}

	os_thread_start(&it->thread, ipc_server_client_thread, (void *)ics);

	// Unlock when we are done.
	os_mutex_unlock(&vs->global_state.lock);
}

xrt_result_t
ipc_server_get_system_properties(struct ipc_server *vs, struct xrt_system_properties *out_properties)
{
	memcpy(out_properties, &vs->xsys->properties, sizeof(*out_properties));
	return XRT_SUCCESS;
}

int
ipc_server_main_common(const struct ipc_server_main_info *ismi,
                       const struct ipc_server_callbacks *callbacks,
                       void *data)
{
	xrt_result_t xret = XRT_SUCCESS;
	int ret = -1;

	// Get log level first.
	enum u_logging_level log_level = debug_get_log_option_ipc_log();

	// Log very early who we are.
	U_LOG_IFL_I(log_level, "%s '%s' starting up...", u_runtime_description, u_git_tag);

	// Allocate the server itself.
	struct ipc_server *s = U_TYPED_CALLOC(struct ipc_server);

	// Can be set by either.
	s->no_stdin = ismi->no_stdin || debug_get_bool_option_no_stdin();

#ifdef XRT_OS_WINDOWS
	timeBeginPeriod(1);
#endif

	/*
	 * Need to create early before any vars are added. Not created in
	 * init_all since that function is shared with Android and the debug
	 * GUI isn't supported on Android.
	 */
	u_debug_gui_create(&ismi->udgci, &s->debug_gui);

	xret = init_all(s, log_level, callbacks, data, ismi->exit_on_disconnect);
	U_LOG_CHK_ONLY_PRINT(log_level, xret, "init_all");
	if (xret != XRT_SUCCESS) {
		// Propagate the failure.
		callbacks->init_failed(xret, data);
		u_debug_gui_stop(&s->debug_gui);
		free(s);
		return -1;
	}

	// Tell the callbacks we are entering the main-loop.
	callbacks->mainloop_entering(s, s->xinst, data);

	// Early init the system. If not available now, will try again per client request.
	xret = ipc_server_init_system_if_available_locked( //
	    s,                                             //
	    NULL,                                          // optional - ics
	    NULL);                                         // optional - out_available
	if (xret != XRT_SUCCESS) {
		U_LOG_CHK_ONLY_PRINT(log_level, xret, "ipc_server_init_system_if_available_locked");
	}

	// Start the debug UI now (if enabled).
	u_debug_gui_start(s->debug_gui, s->xinst, s->xsysd);

	// Main loop.
	ret = main_loop(s);

	// Tell the callbacks we are leaving the main-loop.
	callbacks->mainloop_leaving(s, s->xinst, data);

	// Stop the UI before tearing everything down.
	u_debug_gui_stop(&s->debug_gui);

	// Done after UI stopped.
	teardown_all(s);
	free(s);

#ifdef XRT_OS_WINDOWS
	timeEndPeriod(1);
#endif

	U_LOG_IFL_I(log_level, "Server exiting: '%i'", ret);

	return ret;
}

int
ipc_server_stop(struct ipc_server *s)
{
	s->running = false;
	return 0;
}

#ifndef XRT_OS_ANDROID

static void
init_failed(xrt_result_t xret, void *data)
{
#ifdef XRT_OS_LINUX
	// Print information how to debug issues.
	print_linux_end_user_failed_information(debug_get_log_option_ipc_log());
#endif
}

static void
mainloop_entering(struct ipc_server *s, struct xrt_instance *xinst, void *data)
{
#ifdef XRT_OS_LINUX
	// Print a very clear service started message.
	print_linux_end_user_started_information(s->log_level);
#endif
}

static void
mainloop_leaving(struct ipc_server *s, struct xrt_instance *xinst, void *data)
{
	// No-op
}

void
client_connected(struct ipc_server *s, uint32_t client_id, void *data)
{
	IPC_INFO(s, "Client %u connected", client_id);
}

void
client_disconnected(struct ipc_server *s, uint32_t client_id, void *data)
{
	IPC_INFO(s, "Client %u disconnected", client_id);
}

int
ipc_server_main(int argc, char **argv, const struct ipc_server_main_info *ismi)
{
	const struct ipc_server_callbacks callbacks = {
	    .init_failed = init_failed,
	    .mainloop_entering = mainloop_entering,
	    .mainloop_leaving = mainloop_leaving,
	    .client_connected = client_connected,
	    .client_disconnected = client_disconnected,
	};

	return ipc_server_main_common(ismi, &callbacks, NULL);
}

#endif // !XRT_OS_ANDROID
