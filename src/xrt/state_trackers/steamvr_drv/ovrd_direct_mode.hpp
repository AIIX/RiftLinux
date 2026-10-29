// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  SteamVR driver direct mode: SteamVR hands the driver its finished eye images, and Monado's own compositor
 *         distorts and shows them on the headset, as Monado shows native OpenXR applications.
 *         Monado's compositor is where the picture processing goes (Auto-HDR, HDR, dithering, motion smoothing);
 *         SteamVR's own compositor can't take it.
 *
 * On by default ("redirectsDisplay" in the driver's manifest); settings.json {"display": {"direct_mode": false}} or
 * `riftlinux-setup direct-mode off` switches it off, and SteamVR's compositor then owns the display.
 *
 * On Linux, SteamVR (2.5 and later) shares the images through its IPC resource manager: the driver asks for shared
 * Vulkan images (NewSharedVulkanImage) and gets a file descriptor for each (RefResource, ReceiveSharedFd), which Monado
 * imports as a swapchain. ALVR's Linux driver does the same.
 */

#pragma once

#include <atomic>

#include "openvr_driver.h"

#include "xrt/xrt_compositor.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_defines.h"

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class OvrdDirectMode final : public vr::IVRDriverDirectModeComponent
{
public:
	//! @p driver_from_world turns a pose in SteamVR's world into the driver's (Monado's) space.
	OvrdDirectMode(struct xrt_compositor_native *xcn,
	               struct xrt_device *head,
	               std::function<void(struct xrt_pose *)> driver_from_world,
	               float guard_ms,
	               bool compositor_unorm,
	               bool app_unorm);
	~OvrdDirectMode();

	void
	CreateSwapTextureSet(uint32_t unPid,
	                     const SwapTextureSetDesc_t *pSwapTextureSetDesc,
	                     SwapTextureSet_t *pOutSwapTextureSet) override;
	void
	DestroySwapTextureSet(vr::SharedTextureHandle_t sharedTextureHandle) override;
	void
	DestroyAllSwapTextureSets(uint32_t unPid) override;
	void
	GetNextSwapTextureSetIndex(vr::SharedTextureHandle_t sharedTextureHandles[2], uint32_t (*pIndices)[2]) override;
	void
	SubmitLayer(const SubmitLayerPerEye_t (&perEye)[2]) override;
	void
	Present(vr::SharedTextureHandle_t syncTexture) override;
	void
	PostPresent(const Throttling_t *pThrottling) override;
	void
	GetFrameTiming(vr::DriverDirectMode_FrameTiming *pFrameTiming) override;

private:
	//! One swap texture set: three shared images, imported into Monado as one swapchain.
	struct Set
	{
		uint32_t pid = 0;
		SwapTextureSetDesc_t desc = {};
		vr::SharedTextureHandle_t handles[3] = {};
		int fds[3] = {-1, -1, -1};
		struct xrt_swapchain *xsc = nullptr;
		//! The image SteamVR renders into now, acquired from Monado's swapchain.
		int32_t acquired = -1;
		//! SteamVR's compositor's set, else the application's.
		bool is_compositor = false;
		int number = 0;
		//! Float images (an HDR game's linear light, white at 1.0): the compositor shows them as HDR.
		bool is_float = false;
	};

	void
	destroy_set_locked(Set *set);
	void
	submit_frame(std::vector<std::array<SubmitLayerPerEye_t, 2>> layers, int fence, int64_t entered_ns, bool sync_given);

	//! The frame Present took, handed to Monado in PostPresent (m_present_split).
	struct PendingFrame
	{
		std::vector<std::array<SubmitLayerPerEye_t, 2>> layers;
		int fence = -1;
		int64_t entered_ns = 0;
		bool sync_given = false;
		bool valid = false;
	};
	PendingFrame m_pending;
	int64_t m_no_post_present = 0;
	void
	acquire_locked(Set *set);

	struct xrt_compositor_native *m_xcn;
	struct xrt_device *m_head;
	std::function<void(struct xrt_pose *)> m_driver_from_world;

	std::mutex m_lock;
	std::vector<std::unique_ptr<Set>> m_sets;
	//! Each shared handle to its set and its index in the set.
	std::map<vr::SharedTextureHandle_t, std::pair<Set *, uint32_t>> m_handles;
	//! HDR descriptions by process (set_hdr), and how many layers carried one since the last log.
	std::map<uint32_t, struct xrt_layer_hdr_data> m_pid_hdr;
	int64_t m_layers_described = 0;
	//! This frame's layers, as SubmitLayer gave them.
	std::vector<std::array<SubmitLayerPerEye_t, 2>> m_layers;

	int64_t m_frames = 0;
	int64_t m_layers_total = 0;
	int64_t m_layers_shown = 0;
	int64_t m_next_calls = 0;
	//! Wait before handing a frame to Monado, for SteamVR's GPU to finish it.
	float m_guard_ms = 0.0f;
	//! SteamVR's sRGB images read as plain (UNORM) values: its compositor's, the application's.
	bool m_compositor_unorm = false;
	bool m_app_unorm = false;
	int64_t m_from_compositor = 0;
	int64_t m_from_app = 0;
	int64_t m_upper_from_compositor = 0;
	int64_t m_upper_from_app = 0;
	int m_set_count = 0;
	std::map<int, int64_t> m_first_layer_set;

public:
	//! Layers shown at most a frame (display.direct_mode_max_layers), for finding which one flashes.
	int m_max_layers = 16;
	/*!
	 * Sends SteamVR a vsync event at each of Monado's refreshes (Prop_DriverDirectModeSendsVsyncEvents_Bool), so its
	 * pacing and its throttling (half rate for motion smoothing, as ASW) follow the headset's display.
	 * Started by the driver once the headset is there.
	 */
	void
	start_vsync_events();
	/*!
	 * A process's images described (a video player's HDR10, HDR10+ or Dolby Vision values, over the driver's
	 * socket, ovrd_socket.hpp), or (nullptr) no longer: its layers carry it to the compositor.
	 */
	void
	set_hdr(uint32_t pid, const struct xrt_layer_hdr_data *hdr);
	//! Frames to copy out (display.direct_mode_capture), left to copy, when the next.
	int m_capture_count = 0;
	int m_capture_left = 0;
	int64_t m_capture_next_ns = 0;
	//! Seconds after the first frame before capturing (display.direct_mode_capture_delay_s).
	float m_capture_delay_s = 20.0f;
	/*!
	 * An HDR test chart in front of the view (display.hdr_test): HDR10-like, PQ and BT.2020. Its metadata:
	 * 1 static (MaxCLL 4000 nits), 2 none (the compositor measures), 3 a scene peak of 1000 nits (HDR10+-like); 0 off.
	 */
	int m_hdr_test = 0;
	//! Players' HDR descriptions used (display.hdr_players); off: kept but not put on their layers.
	std::atomic<bool> m_players_hdr{true};
	/*!
	 * Monado waits for the GPU fences of SteamVR's images before it reads them, where the images carry any (they
	 * are dma-bufs: AMD, Intel); display.direct_mode_fence. On every GPU the guard (m_guard_ms) also holds a frame
	 * back from Monado that long after SteamVR presented it: NVIDIA's shared memory carries no fences, and SteamVR's
	 * compositor's own drawing only placeholder ones.
	 */
	bool m_fence = true;
	//! Present only takes the frame, PostPresent hands it to Monado (display.direct_mode_present_split); false: both
	//! in Present.
	bool m_present_split = true;
	//! SteamVR entered standby (it presents about ten times a second without layers, or not at all); the next frame
	//! with layers ends it.
	void
	enter_standby()
	{
		m_standby.store(true);
	}
	/*!
	 * Whether SteamVR's compositor stopped presenting outside standby: it presented once and not for @p after_ns, or
	 * its own set was made and nothing came for that long. Its GPU device lost (a GPU fault) leaves it running
	 * without presenting, and SteamVR never starts it again.
	 */
	bool
	compositor_stopped(int64_t now_ns, int64_t after_ns) const
	{
		if (m_standby.load()) {
			return false;
		}
		const int64_t last = m_present_seen_ns.load();
		const int64_t since = last != 0 ? last : m_compositor_set_ns.load();
		return since != 0 && now_ns - since > after_ns;
	}

private:
	void
	vsync_loop();

	float m_pose_deg_max = 0.0f;
	float m_pose_cm_max = 0.0f;
	float m_pose_deg_sum = 0.0f;
	int m_pose_n = 0;
	int64_t m_last_log_ns = 0;
	// Stalls (judder, a glitch at the edge while an old frame is turned with the head): presents later than 1.5
	// frames, the longest gap, and the longest run of the same application image shown again.
	int64_t m_last_present_ns = 0;
	// Vsync events: the grid (a vsync time on it and the period, from Monado's frames), the thread.
	std::atomic<int64_t> m_vsync_anchor_ns{0};
	std::atomic<int64_t> m_vsync_period_ns{0};
	std::atomic<int64_t> m_vsync_last_tick_ns{0};
	// Monado's frame the last present went into (its refresh), and how long before a refresh Monado wakes a present:
	// SteamVR's throttling holds from these.
	int64_t m_present_display_ns = 0;
	int64_t m_wake_lead_ns = 0;
	int64_t m_lock_held_sum = 0, m_lock_frames = 0, m_lock_missed = 0;
	// SteamVR's throttling of the frame before; when SteamVR's compositor started its frame (first image request
	// after PostPresent), and its waits from there to Present over the last frames (under m_lock).
	uint32_t m_last_throttle = 0;
	int64_t m_compositor_start_ns = 0;
	static constexpr int kCompositorWaits = 8;
	int64_t m_compositor_waits[kCompositorWaits] = {};
	int m_compositor_wait_at = 0;
	int64_t m_compositor_wait_max = 0;
	int64_t m_shown_late = 0, m_shown_early = 0, m_guarded = 0;
	std::atomic<bool> m_vsync_exit{false};
	std::thread m_vsync_thread;
	std::atomic<int64_t> m_vsync_events{0};
	// Refreshes held for SteamVR's throttling, per number of frames (0-3).
	int64_t m_throttle_frames[4] = {};
	int64_t m_late = 0;
	int64_t m_gap_max_ns = 0;
	vr::SharedTextureHandle_t m_last_first_texture = 0;
	int m_repeat_run = 0;
	int m_repeat_max = 0;
	// Motion smoothing: new application images, depth, SteamVR's flags and throttling, per 10 s.
	vr::SharedTextureHandle_t m_last_new_texture = 0;
	int64_t m_new_app_frames = 0;
	int64_t m_first_layers = 0;
	int64_t m_first_with_depth = 0;
	uint32_t m_reprojection_flags = 0;
	int64_t m_timing_calls = 0;
	int64_t m_post_presents = 0;
	int64_t m_throttled = 0;
	uint32_t m_throttle_max = 0;
	uint32_t m_predict_max = 0;
	//! When SteamVR last presented, entered standby, made its compositor's set (compositor_stopped).
	std::atomic<int64_t> m_present_seen_ns{0};
	std::atomic<bool> m_standby{false};
	std::atomic<int64_t> m_compositor_set_ns{0};
	//! Per 10 s: the longest Present, wait for Monado's frame, wait for an image to render into, present to commit.
	int64_t m_present_max_ns = 0, m_submit_max_ns = 0, m_wait_max_ns = 0, m_acquire_max_ns = 0,
	        m_commit_after_max_ns = 0;
	//! Whether SteamVR's images give GPU fences (-1 not tried yet); frames handed over with a fence, held by the guard.
	int m_fences_work = -1;
	int64_t m_fenced = 0, m_guard_held = 0;
	//! The HDR test chart's swapchain, made at the first frame.
	struct xrt_swapchain *m_hdr_test_xsc = nullptr;
	uint32_t m_hdr_test_w = 0, m_hdr_test_h = 0;
	bool m_hdr_test_tried = false;
};
