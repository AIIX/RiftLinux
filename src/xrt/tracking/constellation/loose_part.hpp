// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  LEDs on an adjustable part of a device, such as the Rift CV1's rear headband: where the part sits on the
 *         device, measured from camera frames in which the rest of the device is tracked.
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "tracking/t_constellation.h"

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace xrt::tracking::constellation {

/*!
 * One camera frame for measuring the part: the device's pose from its rigid LEDs, and the blobs this camera saw that
 * the rigid LEDs do not explain (unclaimed, or claimed by the part's own LEDs once it is in the model).
 */
struct LoosePartFrame
{
	//! Camera from device, OpenCV axes.
	xrt_pose Tcv_cam_device{{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	//! Pinhole intrinsics: the blobs are in undistorted pixels.
	float fx{0}, fy{0}, cx{0}, cy{0};
	std::vector<xrt_vec2> blobs;
	//! Which camera: a frame is only kept when the device has moved since that camera's last kept one.
	uint32_t camera{0};
	/*!
	 * The pose is this camera's own solve of the same image (the part seen from the side with the rigid LEDs):
	 * exact. Otherwise it comes from another camera's solve through the cameras' placement, and carries its error.
	 */
	bool own{false};
};

//! Where the part sits: its LEDs at R * factory + t, in the device's OpenCV axes; and how well that fits.
struct LoosePartFit
{
	xrt_pose offset{{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	size_t frames{0};
	//! Predicted LEDs with a blob within 4 px, the frames with any, and how far those blobs were.
	size_t matches{0};
	size_t frames_matched{0};
	float p50_px{0};
	float p90_px{0};
};

/*!
 * Fits the part's offset to @p frames: unless @p start is given, a grid over the part's main freedoms (tilt about the
 * device's side axis, up/down, forward/back: a headband sits symmetric, so sideways, yaw and roll start at zero),
 * scored by how many predicted LEDs land on a blob; then a robust refinement of all six. @p part_leds are the part's
 * factory LEDs in the device's OpenCV axes.
 */
bool
loose_part_fit(const std::vector<t_constellation_tracker_led> &part_leds,
               const std::vector<const LoosePartFrame *> &frames,
               const xrt_pose *start,
               LoosePartFit &out);

//! For an offset: how many of the part's LEDs are predicted visible in @p frames, and how many have a blob nearby.
void
loose_part_check(const std::vector<t_constellation_tracker_led> &part_leds,
                 const std::vector<const LoosePartFrame *> &frames,
                 const xrt_pose &offset,
                 float gate_px,
                 size_t &out_predicted,
                 size_t &out_matched);

/*!
 * Measures a device's loose part from frames as they come, and keeps checking it once it is in use.
 *
 * Until an offset is in use: when enough frames are in, the two halves of them (every other frame) are fitted
 * separately, and the offset is accepted only when both fit tightly and agree; the final offset is then fitted on all
 * of them. Once in use: the newest frames are checked against it, and when its LEDs stop landing on blobs (the strap
 * was changed) it is measured again, or given up when that does not succeed.
 *
 * Fitting runs on a worker thread, or in the caller's thread in deterministic mode (replays). The outcome is handed
 * to @ref on_result, from that thread: an offset to use, or none (stop using the part).
 */
class LoosePartEstimator
{
public:
	using ResultFn = std::function<void(const std::optional<LoosePartFit> &fit, const char *why)>;

	LoosePartEstimator(std::vector<t_constellation_tracker_led> part_leds_cv, bool deterministic, ResultFn on_result);
	~LoosePartEstimator();

	LoosePartEstimator(const LoosePartEstimator &) = delete;
	LoosePartEstimator &
	operator=(const LoosePartEstimator &) = delete;

	//! The offset already in use (from a saved measurement): it is checked from here on instead of measured.
	void
	setInUse(const xrt_pose &offset);

	void
	addFrame(LoosePartFrame &&frame);

	//! For the status: frames collected, and whether an offset is in use.
	size_t
	frameCount();
	bool
	inUse();

private:
	void
	run();
	//! One measuring or checking pass over a copy of the frames; called without @ref lock held.
	void
	evaluate(std::vector<LoosePartFrame> frames, std::optional<xrt_pose> in_use);
	bool
	measure(const std::vector<const LoosePartFrame *> &frames, LoosePartFit &out, const char *&why);
	bool
	measureOnce(const std::vector<const LoosePartFrame *> &frames,
	            size_t min_matches_per_half,
	            LoosePartFit &out,
	            const char *&why);

	const std::vector<t_constellation_tracker_led> part_leds;
	const bool deterministic;
	ResultFn on_result;

	std::mutex lock;
	std::condition_variable cond;
	std::deque<LoosePartFrame> frames;
	//! Per camera, the device's pose in its last kept frame.
	std::map<uint32_t, xrt_pose> last_kept;
	size_t frames_since_evaluation{0};
	std::optional<xrt_pose> in_use;
	bool evaluate_requested{false};
	bool stop{false};
	std::thread worker;
};

} // namespace xrt::tracking::constellation
