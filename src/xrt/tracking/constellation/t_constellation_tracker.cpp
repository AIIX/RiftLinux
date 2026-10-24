// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the main logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "math/m_vec3.h"
#include "xrt/xrt_config_build.h"
#include "xrt/xrt_defines.h"

#include "util/u_time.h"

#include "os/os_time.h"

#include "util/u_file.h"
#include "util/u_json.h"
#include "util/u_notice.h"

#include "riftlinux/rl_room.hpp"

#include "math/m_predict.h"

#include "constellation/correspondence_search.h"

#include "t_constellation_tracker_internal.hpp"
#include "t_constellation_tracker_dataset.hpp"

#ifdef XRT_FEATURE_RERUN
#include "constellation_tracker_rerun.hpp"
#endif

#include <atomic>
#include <algorithm>
#include <limits>
#include <set>
#include <string>
#include <cstring>
#include <climits>
#include <ctime>


namespace xrt::tracking::constellation {

DEBUG_GET_ONCE_LOG_OPTION(constellation_tracker_log, "CONSTELLATION_TRACKER_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_OPTION(constellation_tracker_data_recorder_output, "CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT", "")
//! Cameras not placed are placed from a few seconds of the headset in view (ConstellationTracker::quick); false: only
//! by the room's solve, from one to two minutes of movement, as before.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_quick_place, "CONSTELLATION_TRACKER_QUICK_PLACE", true)
//! Measured again on request, the new placements put back where the room was (run_room_calibration_func); false:
//! the solve's own tracking space, as before.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_recalibration_keep_room,
                           "CONSTELLATION_TRACKER_RECALIBRATION_KEEP_ROOM",
                           true)
/*
 * Rig consistency: placed cameras that solve a device in the same frame should put it in the same place. A camera
 * that moved shifts them apart the same way every time, a wrong solve shifts them anywhere, so the offset that counts
 * is the per-axis median of the offsets over 200 such frames (solves of 8 or more LEDs). With good CV1 rigs it stayed
 * at 1-10 mm, with an occasional window of wrong solves; with a sensor 2.8 cm and 1.5 degrees off it was 21-25 mm,
 * which shows as the world bobbing. The median distance instead tripped on a good rig while the HMD was taken off.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_rig_moved_m, "CONSTELLATION_TRACKER_RIG_MOVED_M", 0.020f)
#define CT_RIG_CHECK_WINDOW 200
#define CT_RIG_CHECK_STRIKES 3
#define CT_RIG_CHECK_MIN_LEDS 8

/*
 * The moved-camera watch (watchRig), in place of the check above, which missed a moved sensor (in a recording:
 * its solves were turned away by the gravity and prior checks before it saw them, and those that got through pulled the
 * fusion back and forth) and once tripped on a sensor that had not moved, which switches the controllers off while
 * every camera is placed again.
 *
 * It compares directions, not positions: at 3.6 m a camera puts a device's depth a few cm out, so two cameras put the
 * HMD 13-17 mm apart typically and up to 100 mm in a window of 20 pairs, but each sees the direction to it within a
 * fraction of a degree. Through the rig, the larger of each camera's direction to the HMD against where the other
 * camera's simultaneous solve puts it: median over 20 such pairs 0.05-0.45 degrees in every recording, never over
 * 0.70 (replays of 16 sessions, gravity- and prior-rejected solves included). A camera turned
 * by 1 degree, or moved 6 cm across, shows as 1 degree. Two windows in a row over the threshold, 0.7 s with the HMD in
 * view of both, mean a moved camera; no session had a false alarm. A camera moved straight along its view doesn't
 * change directions, the distance check below catches it.
 *
 * Replays (monado-constellation-replay track --move-camera) of two recordings with either camera turned 5-10 degrees,
 * tilted 3, moved 10-20 cm across or 15 cm along its view, while tracking or before the tracker started: found every
 * time, the right camera blamed, back in use within about 5 s of a move while tracking, and placed again 0.16-0.27
 * degrees (9-17 mm) from where it really was. With nothing moved, every solve was the same as without the watch.
 * 0 turns the watch off and the old check on.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_rig_watch, "CONSTELLATION_TRACKER_RIG_WATCH", true)
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_rig_watch_deg, "CONSTELLATION_TRACKER_RIG_WATCH_DEG", 1.0f)
/*!
 * A device's adjustable LED part (the Rift CV1's rear headband, when its driver hands it over): measured where it
 * sits from frames in which the rest of the device is tracked, then tracked with the device (loose_part.hpp). 0 leaves
 * its LEDs unused. _SAVED 0 ignores where it was measured before on the same device.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_loose_part, "CONSTELLATION_TRACKER_LOOSE_PART", true)
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_loose_part_saved, "CONSTELLATION_TRACKER_LOOSE_PART_SAVED", true)
#define CT_LOOSE_PART_FILENAME "loose_parts.json"
/*!
 * The room's level (LevelWatch): levelled when two windows in a row (300 good headset solves each) find every sensor
 * tilted alike (within CT_LEVEL_AGREE_DEG of their common tilt) by CT_LEVEL_MIN_DEG or more, the headset having
 * turned at least CT_LEVEL_MIN_YAW_DEG about the vertical in each (so a sensor's tilt is told from the IMU's seat in
 * the headset), the fit's leftover small, and the two windows agreeing. A correctly placed room read up to 4.5
 * degrees in single windows of a 360 degree session (the accelerometer's down while the head moves), so less is left
 * alone; the gravity checks reject solves from 8 degrees. Sensors tilted apart are moved sensors, the moved-sensor
 * watch's case, never levelled. CONSTELLATION_TRACKER_LEVEL=0 leaves the room as placed.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_level, "CONSTELLATION_TRACKER_LEVEL", true)
/*!
 * Automatic measuring (see ConstellationTracker::remeasure): when every camera that sees the headset tilts its solves
 * more than CT_REMEASURE_TILT_DEG from its IMU's down (a quarter of them at most less), with CT_REMEASURE_MIN_SOLVES
 * of them each over the last CT_REMEASURE_WINDOW_NS, for CT_REMEASURE_HOLD_NS, and none of the headset's solves taken
 * for CT_REMEASURE_UNTRACKED_NS, while the headset is worn. Correctly placed sensors read 1 to 4 degrees (the IMU's
 * seat in the headset included). CONSTELLATION_TRACKER_REMEASURE=0 leaves it to the Measure button.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_remeasure, "CONSTELLATION_TRACKER_REMEASURE", true)
#define CT_REMEASURE_TILT_DEG 8.0f
#define CT_REMEASURE_MIN_SOLVES 30
#define CT_REMEASURE_WINDOW_NS (3LL * U_TIME_1S_IN_NS)
#define CT_REMEASURE_HOLD_NS (3LL * U_TIME_1S_IN_NS)
#define CT_REMEASURE_UNTRACKED_NS (2LL * U_TIME_1S_IN_NS)
#define CT_LEVEL_MIN_DEG 5.0f
#define CT_LEVEL_MIN_YAW_DEG 60.0f
#define CT_LEVEL_MAX_RESIDUAL_DEG 3.0f
#define CT_LEVEL_AGREE_DEG 1.5f
/*!
 * The headset's down correction (down_learn) is saved once it is learned from CT_DOWN_SAVE_SOLVES solves while the
 * room reads level (its last window within CT_DOWN_SAVE_LEVEL_DEG, CT_DOWN_SAVE_LEVEL_NS ago at most) and the headset is
 * worn, and read at the
 * next start, so a first measuring can use quick placement. CONSTELLATION_TRACKER_DOWN_SAVED=0 learns it anew.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_down_saved, "CONSTELLATION_TRACKER_DOWN_SAVED", true)
#define CT_DOWN_SAVE_SOLVES 4000
#define CT_DOWN_SAVE_LEVEL_DEG 3.0f
#define CT_DOWN_SAVE_LEVEL_NS (30LL * U_TIME_1S_IN_NS)
#define CT_DOWN_FILENAME "headset_down.json"
#define CT_RIG_WATCH_WINDOW 20
#define CT_RIG_WATCH_STRIKES 2
/*!
 * With three or more cameras, a pair's disagreement on the direction counts as a move only when one of the two also
 * disagrees with a third camera that agrees with the other. Two sensors in front agreed within 0.4 degrees; with a
 * third, low and behind, a pair's window median reached 1.4 degrees with nothing moved (a recorded session), while a
 * moved camera disagrees with every other. The third's windows may be this old (a low rear sensor and a front one
 * seldom see the same device at once; sensors seldom move): the windows are forgotten whenever a camera is placed.
 * CONSTELLATION_TRACKER_RIG_WATCH_VOTE false: the pair alone decides, as with two cameras.
 */
#define CT_RIG_WATCH_VOTE_NS (60LL * U_TIME_1S_IN_NS)
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_rig_watch_vote, "CONSTELLATION_TRACKER_RIG_WATCH_VOTE", true)
/*!
 * With three or more cameras and no third one that has seen the device recently together with both of a disagreeing
 * pair, the pair alone used to decide which of them moved (tilt, fusion, else the lower numbered one stays) and could
 * blame the wrong one: a replay where a front sensor was moved had the other front one placed again, and then the rear
 * one against it. Now the verdict waits for a third camera's view, the pair asked again at its next window, this long;
 * only then the pair alone decides.
 */
#define CT_RIG_WATCH_DEFER_NS (60LL * U_TIME_1S_IN_NS)
/*
 * A camera moved along its view moves where it puts a device's depth, not its direction: for that, how far apart two
 * cameras put the device, per axis median over 200 pairs (about 4 s): 4-26 mm typically, never over 43 in recordings.
 * Two windows over 70 mm, about 8 s, catch a camera moved 8 cm or more along its view.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_rig_watch_mm, "CONSTELLATION_TRACKER_RIG_WATCH_MM", 70.0f)
#define CT_RIG_WATCH_APART_WINDOW 200
#define CT_RIG_WATCH_MIN_LEDS 8
#define CT_RIG_WATCH_MAX_RMS_PX 0.6f
/*
 * Pairs with a trusted camera a moved one is placed again from: 150 first, about 3 s with the HMD in view of both, so
 * that it tracks again, then once more from the next 1500, about 30 s of use. Each pair puts the camera where the
 * trusted camera's solve of the device and its own say; the median of those, over the rest of each recording, tracked
 * as well as the rig it replaced from 600 pairs (two better; the third: device 23 against 16 mm apart), from 150 up to
 * twice as far apart (replays). Fitting the camera to its directions alone slid it along
 * its view by up to a metre: the HMD is always about as far away.
 */
#define CT_RIG_WATCH_PLACE_PAIRS 150
/*
 * Only new views of a device, and windows over which it moved around: a headset lying still in view of two sensors
 * gave the watch the same view frame after frame, with the same depth error (two front sensors put a headset lying
 * still 85 mm apart, worn 4-26 mm), and the watch took that for a sensor moved along its view: it placed one again
 * 269 mm off and saved that, and the next start placed it back. A frame is compared when the device's fused pose moved
 * CT_RIG_WATCH_NEW_VIEW_M or turned CT_RIG_WATCH_NEW_VIEW_DEG since the frame the watch last compared: a camera's own
 * solves of a headset lying still wander by up to 15 mm and 4 degrees, its fused pose stays put (a recording lying on a
 * bed: 3 new views in 53 s; worn sessions 4-17 per second). A window counts when the device also moved
 * CT_RIG_WATCH_REACH_M or turned CT_RIG_WATCH_REACH_DEG from its first view in it (every 200-pair window of worn
 * sessions did, 72-99 % of the 20-pair ones); one that didn't is dropped and changes no strikes. Placing a camera found
 * moved takes every pair: from new views only, a moved sensor of a two-sensor session never settled (spread 38-59 mm)
 * and stayed left out.
 */
#define CT_RIG_WATCH_NEW_VIEW_M 0.01f
#define CT_RIG_WATCH_NEW_VIEW_DEG 1.0f
#define CT_RIG_WATCH_REACH_M 0.05f
#define CT_RIG_WATCH_REACH_DEG 10.0f
#define CT_RIG_WATCH_REFINE_PAIRS 1500
//! Placed only when its candidate poses agree: median absolute deviation per axis, and of the rotation. Over the
//! 1500 pairs of the second placement the device moves around more, and so do the candidates.
// The rotation spread is mostly the HMD's orientation from 10-15 LEDs at 3.6 m: 0.2-0.6 degrees over 150 pairs,
// 0.8-1.4 over 1500 in replays of moved cameras, where the second placement then landed 0.2 degrees from the truth.
#define CT_RIG_WATCH_PLACE_MAD_M 0.03f
#define CT_RIG_WATCH_PLACE_MAD_DEG 0.75f
#define CT_RIG_WATCH_REFINE_MAD_M 0.06f
#define CT_RIG_WATCH_REFINE_MAD_DEG 1.5f
/*
 * Recent solves per device and camera that tell which camera of a pair moved, compared for the device they disagree
 * on. First the tilt against the IMU's down: a camera tilted since it was placed tilts its solves, whatever the fusion
 * followed (a camera tilted while the tracker was off: at start the fusion can follow it). It needs a clear margin,
 * the cameras' views of the HMD differ. Then the fusion: a camera moved while tracking has its solves kept out by the
 * prior binding, so the fusion follows the other camera and the IMU, and the moved camera's directions are the ones
 * off it (replays: 3-5 times the other's). Compared across devices, a Touch's noisier solves blamed the wrong camera.
 */
#define CT_RIG_WATCH_JUDGE_SOLVES 40
#define CT_RIG_WATCH_FUSION_RATIO 3.0f
// In replays of two recordings with one camera moved the moved one's tilt was the larger every time, by 1.1-2.6 degrees
// when it was tilted; unmoved cameras' differed by up to 0.5.
#define CT_RIG_WATCH_TILT_MARGIN_DEG 1.0f
//! The probe: for a device with more LEDs than a Touch, a camera that hasn't found it for this long, now and then.
#define CT_RIG_WATCH_PROBE_MIN_LEDS 24
#define CT_RIG_WATCH_PROBE_BLIND_NS (500 * U_TIME_1MS_IN_NS)
#define CT_RIG_WATCH_PROBE_PERIOD_NS (200 * U_TIME_1MS_IN_NS)
//! While another camera found it this recently.
#define CT_RIG_WATCH_PROBE_TRACKED_NS (100 * U_TIME_1MS_IN_NS)
//! The probe's last find is where its next look starts for this long.
#define CT_RIG_WATCH_PROBE_FOUND_NS (100 * U_TIME_1MS_IN_NS)
//! Nobody tracked the device for this long (or never): every camera probes for it, see probeForRigWatch.
#define CT_RIG_WATCH_PROBE_ROOM_BLIND_NS (1000 * U_TIME_1MS_IN_NS)

//! Whether to restore camera placements saved by an earlier rig calibration, false forces a new calibration.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_load_rig, "CONSTELLATION_TRACKER_LOAD_RIG", true)

/*
 * How far a solve may tilt a device away from what its IMU says, see @ref ImuTilt. Right solves of a CV1 stayed within
 * 10 degrees (p90) of it while turning faster than 3 rad/s, the HMD's within 4 (p99); the wrong Touch solves that
 * still fit their blobs were 25-38 degrees off, or flipped by 130-180. 0 turns the check off.
 */
// 8 against what Oculus tracked in the recorded day and night sessions: the right controller's time more than 10 cm
// off went from 2.7-4.2 % at 20 degrees to 0-0.2 %, the left's at night from 16.6 to 4.1 %.
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_gravity_tolerance_deg,
                            "CONSTELLATION_TRACKER_GRAVITY_TOLERANCE_DEG",
                            8.0f)

/*
 * A device solved within the last CONSTELLATION_TRACKER_PRIOR_FRESH_MS has a prior good to a degree or two, and a solve
 * must then stay within these of it, per axis. The CV1 Touch ring is near symmetric about its axis too: about 3.6 m
 * from the sensors its LEDs are some 7 pixels apart, a prior a few pixels off paired blobs with the neighbouring LEDs,
 * and the refined pose came out turned by one LED spacing, 37 degrees about the vertical and 20 cm off, which the IMU's
 * gravity can't tell. 0 turns the check off.
 */
// 8, as the gravity check, see there: a ring LED's neighbour is about 10 degrees round at 3.6 m.
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_prior_rot_tolerance_deg,
                            "CONSTELLATION_TRACKER_PRIOR_ROT_TOLERANCE_DEG",
                            8.0f)
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_prior_pos_tolerance_m,
                            "CONSTELLATION_TRACKER_PRIOR_POS_TOLERANCE_M",
                            0.10f)
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_prior_fresh_ms, "CONSTELLATION_TRACKER_PRIOR_FRESH_MS", 500)

/*
 * A device the cameras lost may hand over a prior whose orientation its IMU still tracks, and whose position is held
 * where it was last seen. For CONSTELLATION_TRACKER_HELD_PRIOR_MS after the last solve a solve must keep to that
 * orientation, and stay within CONSTELLATION_TRACKER_HELD_PRIOR_POS_TOLERANCE_M per axis of the position. A CV1 Touch
 * 3.6 m from the sensors shows its ring as 2-3 pixel blobs some 7 pixels apart, and searched from scratch it was found
 * in a fraction of the frames it was in, or turned and flipped; with the orientation given, only its position is left
 * to find.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_held_prior_ms, "CONSTELLATION_TRACKER_HELD_PRIOR_MS", 5000)

/*
 * A prior can be wrong too: a Touch's fused heading drifted some 25 degrees, and the prior then turned away every
 * right solve of it from both sensors, for seconds at a time. When solves have kept leaving it for
 * CONSTELLATION_TRACKER_PRIOR_DISTRUST_MS, at least 3 and none agreeing, the next is taken as it is.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_prior_distrust_ms, "CONSTELLATION_TRACKER_PRIOR_DISTRUST_MS", 300)
#define CT_PRIOR_DISTRUST_MIN_SOLVES 3

/*!
 * An accepted solve this far from the device's fusion at the same moment, and so many in a row, and the fusion is
 * what is wrong: the solves passed every check on their own. See Camera::checkFusionAgrees.
 */
#define CT_FUSION_DISAGREE_ROT_DEG 15.0f
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_fusion_disagree_pos_m,
                            "CONSTELLATION_TRACKER_FUSION_DISAGREE_POS_M",
                            0.10f)
#define CT_FUSION_DISAGREE_POS_M debug_get_float_option_constellation_tracker_fusion_disagree_pos_m()
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_fusion_reset, "CONSTELLATION_TRACKER_FUSION_RESET", true)
/*!
 * A blob that has stayed within CT_STATIC_SPOT_PX of where it was for this long, with no device's label, is a static
 * spot, which searches with no prior leave out. 0 turns it off.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_static_spot_ms, "CONSTELLATION_TRACKER_STATIC_SPOT_MS", 2000)
#define CT_STATIC_SPOT_PX 1.5f
//! A blob unseen for longer starts over: the blobwatch may have reused its ID.
#define CT_STATIC_SPOT_GAP_NS (200 * U_TIME_1MS_IN_NS)
/*!
 * A device lost and lying still has its own blobs become static spots: a search with no prior takes every blob once in
 * this long, so it is still found, only later.
 */
#define CT_FULL_BLIND_SEARCH_NS (1000 * U_TIME_1MS_IN_NS)
/*!
 * A pose the slow search found from scratch fits its blobs within this many pixels (rms) or is not taken. Against
 * what Oculus tracked in a recorded Windows session, 0.25 kept 92 % of the right ones and 35 % of those 12 cm or more
 * off, mostly 20-70 cm: the other LEDs of the ring line up nearly as well, just not quite. The fast
 * paths start from the prior and are not held to it, 0.5 % of theirs are that far off.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_slow_max_rms_px, "CONSTELLATION_TRACKER_SLOW_MAX_RMS_PX", 0.25f)
/*!
 * A pose the slow search found has to match at least this many blobs. With a prior the search may go down to 4, and
 * against what Oculus tracked in the recorded day and night sessions most wrong slow-search poses, 20-70 cm off, had
 * matched just 4: 5 kept 86-100 % of the right ones and dropped 40-67 % of the wrong.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_slow_min_leds, "CONSTELLATION_TRACKER_SLOW_MIN_LEDS", 5)
//! A pose from a fast path has to match at least this many blobs.
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_fast_min_leds, "CONSTELLATION_TRACKER_FAST_MIN_LEDS", 4)
/*!
 * The fast path that checks the last solved pose against the blobs. Against what Oculus tracked, half of the left
 * controller's poses from it were more than 10 cm off in daylight and 6-9 % at night; the held prior
 * covers what it was for. Off unless set.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_fast_last_known, "CONSTELLATION_TRACKER_FAST_LAST_KNOWN", false)
/*!
 * A slow-search pose that passed the fit, LED and gravity checks wins over a prior it leaves: against what Oculus
 * tracked, 58-100 % of the slow-search poses the prior turned away were right, the prior was what was stale.
 * The device's fusion is then started over from it.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_slow_overrides_prior,
                           "CONSTELLATION_TRACKER_SLOW_OVERRIDES_PRIOR",
                           true)
//! Whether a binding prior also binds the slow search itself (CS_FLAG_PRIOR_MUST_MATCH), or is only a hint there.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_slow_prior_binds, "CONSTELLATION_TRACKER_SLOW_PRIOR_BINDS", true)

//! Recorded solves a check turned away carry one of these in their match flags.
#define CT_SOLVE_REJECTED_PRIOR (1u << 31)
#define CT_SOLVE_REJECTED_GRAVITY (1u << 30)
#define CT_SOLVE_REJECTED_FIT (1u << 29)
//! The camera is left out by the moved-camera watch until it is placed again.
#define CT_SOLVE_REJECTED_RIG (1u << 28)
#define CT_FUSION_DISAGREE_MIN_SOLVES 4
#define CT_FUSION_DISAGREE_MIN_NS (150 * U_TIME_1MS_IN_NS)
//! How much the fusion's offset to the solves may vary over that, a burst of wrong solves jumps about far more.
#define CT_FUSION_DISAGREE_CONSISTENT_ROT_DEG 10.0f
#define CT_FUSION_DISAGREE_CONSISTENT_POS_M 0.05f
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_held_prior_pos_tolerance_m,
                            "CONSTELLATION_TRACKER_HELD_PRIOR_POS_TOLERANCE_M",
                            0.75f)

//! Where the rig is saved, in the config directory.
#define CT_RIG_SUBDIR "constellation"
#define CT_RIG_FILENAME "rig.json"
#define CT_RIG_VERSION 1

// Unconditionally present to allow warning that the feature is not enabled.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_enable_rerun, "CONSTELLATION_TRACKER_RERUN_ENABLE", false)
#ifdef XRT_FEATURE_RERUN
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_rerun_spawn, "CONSTELLATION_TRACKER_RERUN_SPAWN", true)
#endif

/*
 *
 * Helper functions
 *
 */

//! Gets the gravity vector of a pose, in the pose's local frame.
static void
get_pose_gravity_vector(xrt_pose &T_world_pose, xrt_vec3 &gravity)
{
	// Extract the gravity vector from the pose's orientation
	gravity = XRT_VEC3_UNIT_Y;

	xrt_quat T_pose_world_orientation;
	math_quat_invert(&T_world_pose.orientation, &T_pose_world_orientation);

	math_quat_rotate_vec3(&T_pose_world_orientation, &gravity, &gravity);
}

static uint32_t
num_blobs_for_device(CameraSample &sample, t_constellation_device_id_t device_id)
{
	uint32_t out_num_blobs = 0;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		t_blob &b = sample.blobs[i];
		if (b.matched_device_id == device_id) {
			out_num_blobs++;
		}
	}
	return out_num_blobs;
}

/*!
 * Sort key used to search for the devices closest to the camera first, since a near device covers more of the image and
 * is the one most likely to explain the blobs. Devices we have no prediction for sort last.
 *
 * @note The prediction is stored on the sample, so this is usable from the slow thread as well, where the fast thread's
 *       predictions are no longer being recomputed.
 */
static float
device_state_camera_distance_sqrd(const DeviceState *device_state)
{
	if (device_state == nullptr || !device_state->Tcv_cam_device_predicted.has_value()) {
		return std::numeric_limits<float>::infinity();
	}

	return m_vec3_len_sqrd(device_state->Tcv_cam_device_predicted->pose.position);
}

/*!
 * Collects the tracker's devices into @p out_devices, ordered nearest-camera-first for @p sample.
 *
 * The caller must hold @ref ConstellationTracker::device_lock for as long as it uses the result, since these are
 * borrowed pointers into @ref ConstellationTracker::devices.
 */
static uint32_t
collect_devices_nearest_first(ConstellationTracker *tracker,
                              CameraSample &sample,
                              std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> &out_devices)
{
	uint32_t device_count = 0;
	for (std::unique_ptr<Device> &device : tracker->devices) {
		assert(device_count < out_devices.max_size());
		out_devices[device_count++] = device.get();
	}

	std::sort(out_devices.data(), out_devices.data() + device_count, [&sample](Device *x, Device *y) {
		return device_state_camera_distance_sqrd(sample.getDeviceState(x->id).value_or(nullptr)) <
		       device_state_camera_distance_sqrd(sample.getDeviceState(y->id).value_or(nullptr));
	});

	return device_count;
}

/*!
 * Quick placement decided (ConstellationTracker::quick): its placements in use, from this thread as the room's solve
 * does it (it holds no camera's lock).
 */
extern "C" void *
run_quick_place_func(void *ptr)
{
	ConstellationTracker *tracker = static_cast<ConstellationTracker *>(ptr);
	std::vector<ConstellationTracker::Placement> placements;
	double seconds = 0.0;
	double frames_s = 0.0;
	{
		std::unique_lock<os::Mutex> lock(tracker->quick.lock);
		for (const auto &[key, pose] : tracker->quick.placed) {
			placements.push_back({key.first, key.second, pose});
		}
		seconds = (double)(os_monotonic_get_ns() - tracker->quick.started_ns) / 1e9;
		frames_s = (double)(tracker->quick.last_frame_ns - tracker->quick.first_frame_ns) / 1e9;
		tracker->quick.active = false;
		tracker->quick.ready = false;
	}
	CT_WARN(tracker,
	        "Sensors placed quickly: %zu camera(s) in %.1f s (%.1f s of the sensors' frames), from the headset seen "
	        "in the same frames.",
	        placements.size(), seconds, frames_s);
	{
		// The room's solve is not needed any more: its collection ends.
		std::unique_lock<os::Mutex> lock(tracker->rig_calibration_lock);
		tracker->rig_calibration->endCollectingData();
	}
	tracker->applyPlacements(placements, 0.0, "quick placement");
	return nullptr;
}

void
ConstellationTracker::applyPlacements(const std::vector<Placement> &placements,
                                      double camera_rms_sigma,
                                      const char *how)
{
	ConstellationTracker *tracker = this;
	auto &calib = this->rig_calibration;

	/*
	 * Measured again on request: the new tracking space has whichever camera was placed first as its origin (one
	 * came out turned 21.5 degrees and 1.07 m away, no sensor moved). The cameras that kept their place put the
	 * new placements back where the room was (rl_room.hpp): the room, the floor and the user's place in it stay.
	 * When none can tell, whoever shows the room places it on the user again (u_notice_recalibration::kept).
	 */
	/*
	 * No camera placed before these (a start with no saved places: after riftlinux-setup recalibrate, every sensor
	 * moved, a fresh install): the tracking space is new, as after a re-measure that kept nothing. Reported the same
	 * way (u_notice_recalibration, nothing kept), so whoever shows the room places it on the user once the head has
	 * settled; the head was held where it was while nothing was placed.
	 */
	bool from_nothing = true;
	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
			from_nothing = from_nothing && !camera->locked_data.has_concrete_pose;
		}
	}

	rl::room::Result room;
	{
		std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
		if (this->recalibration.pending) {
			std::vector<rl::room::Sensor> sensors;
			for (const Placement &placement : placements) {
				const auto &camera = this->mosaics[placement.mosaic_idx]->cameras[placement.camera_idx];
				const auto before = this->recalibration.pose_before.find(camera->serial);
				if (before != this->recalibration.pose_before.end()) {
					sensors.push_back({before->second, placement.Txr_origin_cam});
				}
			}
			room = rl::room::keep(sensors);
			if (!debug_get_bool_option_constellation_tracker_recalibration_keep_room()) {
				room.kept = false;
				room.how += " (not used: CONSTELLATION_TRACKER_RECALIBRATION_KEEP_ROOM=false)";
			}
			CT_WARN(tracker, "Measured every sensor again (%s): %s.", how, room.how.c_str());
		}
	}

	for (const Placement &placement : placements) {
		auto &camera = this->mosaics[placement.mosaic_idx]->cameras[placement.camera_idx];

		// Where the room was, and in OpenCV for the solve (the conversion is its own inverse).
		struct xrt_pose Txr_origin_cam = placement.Txr_origin_cam;
		if (room.kept) {
			Txr_origin_cam = rl::room::apply(room.turn, Txr_origin_cam);
		}
		struct xrt_pose Tcv_world_cam;
		math_pose_convert_from_opencv(&Txr_origin_cam, &Tcv_world_cam);

		std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
		camera->locked_data.has_concrete_pose = true;
		camera->locked_data.Txr_origin_cam = Txr_origin_cam;

		// Later rounds, for cameras this one left unplaced, hold this camera where it is.
		calib->setCameraPose(placement.mosaic_idx, placement.camera_idx, Tcv_world_cam);
	}

	{
		int placed = 0;
		for (auto &mosaic : this->mosaics) {
			for (auto &camera : mosaic->cameras) {
				std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
				placed += camera->locked_data.has_concrete_pose ? 1 : 0;
			}
		}
		this->cameras_placed.store(placed);
	}

	// Next time the same cameras are used, they are placed right away.
	this->saveRig(camera_rms_sigma);

	// Asked for: whether the room was kept, and how far each camera moved up or down, before "calibration-done".
	// Placed from nothing: nothing kept, no camera's change known.
	{
		std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
		const bool asked = this->recalibration.pending;
		if (!asked && from_nothing) {
			CT_WARN(tracker, "Sensors placed (%s) with none placed before: a new tracking space, the view is placed on "
			                 "the user once the head settles.",
			        how);
		}
		if (asked || from_nothing) {
			this->recalibration.pending = false;
			struct u_notice_recalibration change = {};
			change.sequence = ++this->recalibration.sequence;
			change.kept = room.kept;
			change.anchors = room.anchors;
			for (auto &mosaic : this->mosaics) {
				for (auto &camera : mosaic->cameras) {
					const int n = change.count;
					if (n >= U_NOTICE_MAX_CAMERAS) {
						break;
					}
					change.count++;
					std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
					auto before = this->recalibration.pose_before.find(camera->serial);
					if (asked && camera->locked_data.has_concrete_pose &&
					    before != this->recalibration.pose_before.end()) {
						change.known[n] = true;
						change.height_change_m[n] =
						    camera->locked_data.Txr_origin_cam.position.y - before->second.position.y;
						CT_INFO(tracker, "Camera %zu/%zu measured again: %+.0f mm up.", mosaic->index,
						        camera->index, change.height_change_m[n] * 1000.0f);
					}
				}
			}
			u_notice_set_recalibration(&change);
		}
	}
	u_notice_post(U_NOTICE_CALIBRATION, "calibration-done", 0);

	// Every device tracked again now that the cameras are placed.
	{
		std::shared_lock devices_lock(this->device_lock);
		for (auto &device : this->devices) {
			device->enabled.store(true);
		}
	}

	// Last: a new measuring may start (and join this thread) once this is false.
	std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
	this->running_calibration = false;
}

extern "C" void *
run_room_calibration_func(void *ptr)
{
	ConstellationTracker *tracker = static_cast<ConstellationTracker *>(ptr);
	auto &calib = tracker->rig_calibration;

	rig_calibration::CalibrationResult result;
	if (!calib->optimize(result)) {
		std::unique_lock<os::Mutex> lock(tracker->rig_calibration_lock);

		CT_ERROR(tracker, "Offline sensor calibration failed!");
		tracker->running_calibration = false;
		u_notice_post(U_NOTICE_CALIBRATION, "calibration-failed", 0);

		if (tracker->data_recorder) {
			tracker->data_recorder->recordCalibrationEvent(
			    DatasetCalibrationEvent::failed(os_monotonic_get_ns()));
		}
		return nullptr;
	}

	if (tracker->data_recorder) {
		tracker->data_recorder->recordCalibrationEvent(
		    DatasetCalibrationEvent::solved(os_monotonic_get_ns(), result));
	}

	std::vector<ConstellationTracker::Placement> placements;
	for (const auto &calibration_camera : result.cameras) {
		if (calibration_camera.optimized) {
			struct xrt_pose Txr_origin_cam;
			math_pose_convert_from_opencv(&calibration_camera.Tcv_world_cam, &Txr_origin_cam);
			placements.push_back({calibration_camera.mosaic_idx, calibration_camera.camera_idx, Txr_origin_cam});
		}
	}
	{
		// Done before quick placement: it stops.
		std::unique_lock<os::Mutex> lock(tracker->quick.lock);
		tracker->quick.active = false;
		tracker->quick.ready = false;
	}
	tracker->applyPlacements(placements, result.camera_fitness.rmsSigma(), "the room's solve");

	std::shared_lock devices_lock(tracker->device_lock);
	for (auto &device : tracker->devices) {
		// Re-enable all devices now that setup is done.
		device->enabled.store(true);

		if (device->id == result.device_id) {
			device->sensor_fusion->setDeviceImuCalibration(sensor_fusion::DeviceImuCalibration{
			    .accel_bias = result.imu.accel_bias,
			    .gyro_bias = result.imu.gyro_bias,
			    .accel_scale = result.imu.accel_scale,
			    .Qcv_imu_model = result.imu.Qcv_imu_model,
			});
		}
	}

	return nullptr;
}

/*
 *
 * CameraSample implementations
 *
 */

std::optional<DeviceState *>
CameraSample::getDeviceState(t_constellation_device_id_t device_id)
{
	for (uint32_t i = 0; i < this->device_count; i++) {
		if (this->device_states[i].device_id == device_id) {
			return &this->device_states[i];
		}
	}

	return std::nullopt;
}

DeviceState &
CameraSample::putDeviceState(t_constellation_device_id_t device_id)
{
	assert(this->device_count < XRT_CONSTELLATION_MAX_DEVICES);

	DeviceState &new_device_state = this->device_states[this->device_count++];
	new_device_state = {
	    .device_id = device_id,
	    .Txr_world_device_prior = std::nullopt,
	    .found_pose = std::nullopt,
	    .needs_slow_processing = false,
	};

	return new_device_state;
}


CameraSample::CameraSample(t_blob_observation &blobservation, Camera *camera)
{
	// Copy the blob observation into this sample, since we need the data to be safe.
	this->source = blobservation.source;
	this->id = blobservation.id;
	this->sequence_id = blobservation.sequence_id;
	this->timestamp_ns = blobservation.timestamp_ns;
	memcpy(blobs, blobservation.blobs, sizeof(t_blob) * blobservation.num_blobs);
	this->blob_count = blobservation.num_blobs;

	// Get the camera pose
	this->Txr_world_cam = camera->getWorldPose(blobservation.timestamp_ns);

	this->device_states = {};
	this->device_count = 0;

	auto mosaic = camera->mosaic.lock();
	U_ASSERT_WEAK_PTR_THROW(mosaic, "Camera's mosaic was destroyed while we still had a sample referencing it");

	this->mosaic_index = mosaic->index;
	this->camera_index = camera->index;
}

void
CameraSample::markMatchingBlobs(ConstellationTracker *ct,
                                t_constellation_tracker_led_model &led_model,
                                t_constellation_device_id_t device_id,
                                pose_metrics_blob_match_info &blob_match_info)
{
	// First clear existing blob labels for this device
	for (uint32_t i = 0; i < this->blob_count; i++) {
		t_blob &b = this->blobs[i];

		// Skip blobs which already have an ID not belonging to this device
		if (b.matched_device_id != device_id) {
			continue;
		}

		if (b.matched_device_led_id != XRT_CONSTELLATION_INVALID_LED_ID) {
			// @todo is this needed?
			// b.prev_led_id = b.led_id;
		}

		b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
	}

	// Iterate the visible LEDs and mark matching blobs with this device ID and LED ID
	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		pose_metrics_visible_led_info &led_info = blob_match_info.visible_leds[i];
		t_constellation_tracker_led *led = led_info.led;

		if (led_info.matched_blob != NULL) {
			t_blob *b = led_info.matched_blob;

			b->matched_device_led_id = led->id;
			b->matched_device_id = device_id;

			CT_DEBUG(ct, "Marking LED %d/%d at %f,%f angle %f now %d (was %d)", device_id, led->id,
			         b->center_distorted.x, b->center_distorted.y, RAD_TO_DEG(acosf(led_info.facing_dot)),
			         b->matched_device_led_id, /* b->prev_led_id */ -1);
		} else {
			CT_DEBUG(ct, "No blob for device %d LED %d @ %f,%f size %f px angle %f", device_id, led->id,
			         led_info.pos_px_undistorted.x, led_info.pos_px_undistorted.y,
			         2 * led_info.led_radius_px_undistorted, RAD_TO_DEG(acosf(led_info.facing_dot)));
		}
	}
}

/*
 *
 * Camera implementations
 *
 */

Camera::Camera(ConstellationTracker *tracker,
               std::weak_ptr<CameraMosaic> mosaic,
               const t_constellation_tracker_camera &camera_params,
               enum u_logging_level *log_level_ptr,
               size_t index)
    : tracker(tracker), mosaic(mosaic), calibration(camera_params.calibration), index(index)
{
	this->tracker = tracker;
	this->mosaic = mosaic;
	this->calibration = camera_params.calibration;
	this->model = {
	    .width = calibration.image_size_pixels.w,
	    .height = calibration.image_size_pixels.h,
	    .calib_true = {},
	    .calib_pinhole = {},
	};
	t_camera_model_params_from_t_camera_calibration(&this->calibration, &this->model.calib_true);
	this->model.calib_pinhole = this->model.calib_true;
	this->model.calib_pinhole.model = T_DISTORTION_PINHOLE;

	this->serial = std::string(camera_params.serial, strnlen(camera_params.serial, sizeof(camera_params.serial)));

	this->locked_data = {
	    .Txr_origin_cam = camera_params.pose_in_origin,
	    .has_concrete_pose = camera_params.has_concrete_pose,
	};

	this->slow_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);
	this->slow_processing_thread_data.cs_blind = correspondence_search_new(log_level_ptr, &this->model);
	this->fast_processing_thread_data.cs = correspondence_search_new(log_level_ptr, &this->model);

	if (!this->tracker->single_threaded) {
		if (os_thread_helper_init(&this->slow_processing_thread) < 0) {
			throw std::runtime_error("Slow processing thread failed to init");
		}
		if (os_thread_helper_init(&this->fast_processing_thread) < 0) {
			throw std::runtime_error("Fast processing thread failed to init");
		}

		if (os_thread_helper_start(&this->slow_processing_thread, constellation_tracker_camera_slow_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting slow processing thread failed");
		}
		if (os_thread_helper_start(&this->fast_processing_thread, constellation_tracker_camera_fast_thread,
		                           this) < 0) {
			throw std::runtime_error("Starting fast processing thread failed");
		}
	}

	u_sink_debug_init(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_init(&this->fast_processing_thread_data.debug_sink);
}

Camera::~Camera()
{
	if (this->slow_processing_thread.initialized) {
		os_thread_helper_destroy(&this->slow_processing_thread);
	}

	if (this->fast_processing_thread.initialized) {
		os_thread_helper_destroy(&this->fast_processing_thread);
	}

	if (this->slow_processing_thread_data.cs) {
		correspondence_search_free(this->slow_processing_thread_data.cs);
		this->slow_processing_thread_data.cs = nullptr;
	}

	if (this->slow_processing_thread_data.cs_blind) {
		correspondence_search_free(this->slow_processing_thread_data.cs_blind);
		this->slow_processing_thread_data.cs_blind = nullptr;
	}

	if (this->fast_processing_thread_data.cs) {
		correspondence_search_free(this->fast_processing_thread_data.cs);
		this->fast_processing_thread_data.cs = nullptr;
	}

	u_sink_debug_destroy(&this->slow_processing_thread_data.debug_sink);
	u_sink_debug_destroy(&this->fast_processing_thread_data.debug_sink);
}

std::optional<xrt_pose>
Camera::getWorldPose(timepoint_ns when_ns)
{
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(
	    mosaic, "Camera's mosaic was destroyed while we still had a pointer to it, this should never happen",
	    std::nullopt);

	auto Txr_world_origin = mosaic->getTrackingOriginPose(when_ns);
	if (!Txr_world_origin.has_value()) {
		return std::nullopt;
	}

	std::unique_lock<os::Mutex> lock(this->processing_lock);

	if (!this->locked_data.has_concrete_pose) {
		return std::nullopt;
	}

	xrt_pose Txr_world_cam;
	math_pose_transform(&Txr_world_origin.value(), &this->locked_data.Txr_origin_cam, &Txr_world_cam);
	return Txr_world_cam;
}

void
Camera::deferSampleToSlowThread(CameraSample &sample)
{
	uint64_t dropped = 0;

	os_thread_helper_lock(&this->slow_processing_thread);
	{
		if (this->slow_processing_thread_data.sample.has_value()) {
			dropped = this->dropped_slow_samples.tick(os_monotonic_get_ns());
			this->slow_skipped.fetch_add(1);
		}

		this->slow_processing_thread_data.sample = sample;
		os_thread_helper_signal_locked(&this->slow_processing_thread);
	}
	os_thread_helper_unlock(&this->slow_processing_thread);

	// Normal while a device is out of view, the slow search then runs on every frame it can get.
	if (dropped > 0) {
		CT_DEBUG(this->tracker, "Camera %zu: the slow thread skipped %" PRIu64 " sample(s) in the last second.",
		         this->index, dropped);
	}
}

bool
Camera::gravityForSample(const Device *device,
                         const CameraSample &sample,
                         xrt_vec3 &out_camera_gravity,
                         xrt_vec3 &out_device_gravity) const
{
	if (this->tracker->gravity_tolerance_rad <= 0.0f || !sample.Txr_world_cam.has_value()) {
		return false;
	}

	std::optional<xrt_vec3> device_gravity = device->gravityAt(sample.timestamp_ns);
	if (!device_gravity.has_value()) {
		return false;
	}

	// The tracker's world is gravity aligned, down is +Y in OpenCV coordinates.
	xrt_pose Tcv_world_cam;
	math_pose_convert_from_opencv(&sample.Txr_world_cam.value(), &Tcv_world_cam);
	get_pose_gravity_vector(Tcv_world_cam, out_camera_gravity);

	out_device_gravity = device_gravity.value();
	return true;
}

bool
Camera::poseMatchesGravity(Device *device,
                           const CameraSample &sample,
                           const xrt_pose &Tcv_cam_device,
                           SolveSource source)
{
	xrt_vec3 camera_gravity;
	xrt_vec3 device_gravity;
	if (!this->gravityForSample(device, sample, camera_gravity, device_gravity)) {
		return true;
	}

	const float error_rad = pose_metrics_gravity_error_rad(&Tcv_cam_device, &camera_gravity, &device_gravity);
	if (error_rad <= this->tracker->gravity_tolerance_rad) {
		return true;
	}

	device->gravity_rejections_total.fetch_add(1);
	const uint64_t rejected = device->gravity_rejections.tick(os_monotonic_get_ns());
	if (rejected > 0) {
		CT_INFO(this->tracker,
		        "Device %d: rejected %" PRIu64
		        " solve(s) in the last second that disagree with its IMU on which "
		        "way is down, the latest by %.0f degrees (camera %zu, source %d).",
		        device->id, rejected, RAD_TO_DEG(error_rad), this->index, static_cast<int>(source));
	}

	return false;
}

bool
Camera::bindingPrior(Device *device,
                     const CameraSample &sample,
                     const DeviceState &device_state,
                     const xrt_vec3 *&out_pos_tolerance,
                     const xrt_vec3 *&out_rot_tolerance) const
{
	const ConstellationTracker *tracker = this->tracker;
	if (!device_state.Tcv_cam_device_predicted.has_value()) {
		return false;
	}

	timepoint_ns since_solve_ns;
	{
		std::unique_lock<os::Mutex> lock(device->data_lock);
		const auto &last = device->locked_data.last_known_pose;
		if (!last.has_value()) {
			return false;
		}
		since_solve_ns = sample.timestamp_ns - last->timestamp_ns;
	}

	if (device_state.prior_position_tracked) {
		if (tracker->prior_fresh_ns <= 0 || since_solve_ns > tracker->prior_fresh_ns) {
			return false;
		}
		out_pos_tolerance = &tracker->prior_tolerance_pos;
	} else {
		/*
		 * A headset's held prior (see SensorFusion::getTrackedPose) binds no longer than a tracked one: the head it
		 * sits on walks away from where it was lost.
		 */
		const timepoint_ns binds_ns = device->params.head ? tracker->prior_fresh_ns : tracker->held_prior_ns;
		if (binds_ns <= 0 || since_solve_ns > binds_ns) {
			return false;
		}
		out_pos_tolerance = &tracker->held_prior_tolerance_pos;
	}

	out_rot_tolerance = &tracker->prior_tolerance_rot;
	return true;
}

void
Camera::checkFusionAgrees(Device *device, timepoint_ns timestamp_ns, const xrt_pose &Txr_world_device)
{
	/*
	 * The fusion takes a solve far from its state as a small pull, so a window that settled on a wrong pose (a
	 * wrong solve it started from, a gyro bias it has not learned) crawls back over many seconds while the solves
	 * are right all along: 30 degrees for 15 s on the left Touch of a recorded Windows session, the solves within
	 * 2 degrees of what Oculus tracked. The prior binding does not catch it, a prior held from the
	 * last solve lets the solves through in between.
	 */
	if (!debug_get_bool_option_constellation_tracker_fusion_reset()) {
		return;
	}

	xrt_space_relation fused = XRT_SPACE_RELATION_ZERO;
	timepoint_ns fused_ns = 0;
	device->sensor_fusion->getTrackedPose(timestamp_ns, fused_ns, fused);
	if ((fused.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0 ||
	    (fused.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) == 0) {
		std::unique_lock<os::Mutex> lock(device->fusion_disagree.lock);
		device->fusion_disagree.count = 0;
		return;
	}

	xrt_quat delta;
	math_quat_unrotate(&fused.pose.orientation, &Txr_world_device.orientation, &delta);
	const float angle_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(delta.w), 0.0f, 1.0f)));
	xrt_quat fused_inverse;
	math_quat_invert(&fused.pose.orientation, &fused_inverse);
	xrt_quat world_delta;
	math_quat_rotate(&Txr_world_device.orientation, &fused_inverse, &world_delta);
	const xrt_vec3 offset = Txr_world_device.position - fused.pose.position;
	const float offset_m = m_vec3_len(offset);

	{
		auto &d = device->fusion_disagree;
		std::unique_lock<os::Mutex> lock(d.lock);

		if (angle_deg <= CT_FUSION_DISAGREE_ROT_DEG && offset_m <= CT_FUSION_DISAGREE_POS_M) {
			d.count = 0;
			return;
		}

		bool consistent = false;
		if (d.count > 0 && timestamp_ns >= d.since_ns) {
			xrt_quat change;
			math_quat_unrotate(&d.rotation_offset, &delta, &change);
			const float change_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(change.w), 0.0f, 1.0f)));
			math_quat_unrotate(&d.world_rotation_offset, &world_delta, &change);
			const float world_change_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(change.w), 0.0f, 1.0f)));
			// A fusion turned the wrong way drifts in position too, so either one staying put is enough.
			consistent = std::min(change_deg, world_change_deg) <= CT_FUSION_DISAGREE_CONSISTENT_ROT_DEG ||
			             m_vec3_len(offset - d.position_offset) <= CT_FUSION_DISAGREE_CONSISTENT_POS_M;
		}
		if (!consistent) {
			d.count = 1;
			d.since_ns = timestamp_ns;
			d.rotation_offset = delta;
			d.world_rotation_offset = world_delta;
			d.position_offset = offset;
			return;
		}

		d.count++;
		if (d.count < CT_FUSION_DISAGREE_MIN_SOLVES || timestamp_ns - d.since_ns < CT_FUSION_DISAGREE_MIN_NS) {
			return;
		}
		d.count = 0;
	}

	device->sensor_fusion->requestReset();
	device->fusion_resets_total.fetch_add(1);
	const uint64_t resets = device->fusion_resets.tick(os_monotonic_get_ns());
	if (resets > 0) {
		CT_INFO(this->tracker,
		        "Device %d: its solves kept disagreeing with its fusion alike, the latest by %.0f degrees and "
		        "%.0f mm, starting the fusion over (%" PRIu64 " time(s) in the last second).",
		        device->id, angle_deg, offset_m * 1000.0f, resets);
	}
}

bool
Camera::poseMatchesPrior(Device *device,
                         const CameraSample &sample,
                         const DeviceState &device_state,
                         const xrt_pose &Tcv_cam_device,
                         SolveSource source)
{
	const xrt_vec3 *pos_tolerance = nullptr;
	const xrt_vec3 *rot_tolerance = nullptr;
	if (!this->bindingPrior(device, sample, device_state, pos_tolerance, rot_tolerance)) {
		return true;
	}

	const xrt_pose &Tcv_cam_device_prior = device_state.Tcv_cam_device_predicted->pose;
	if (pose_metrics_pose_matches_prior(&Tcv_cam_device, &Tcv_cam_device_prior, pos_tolerance, rot_tolerance)) {
		return true;
	}

	if ((source == SolveSource::SlowShallowSearch || source == SolveSource::SlowDeepSearch) &&
	    debug_get_bool_option_constellation_tracker_slow_overrides_prior()) {
		device->sensor_fusion->requestReset();
		device->fusion_resets_total.fetch_add(1);
		return true;
	}

	// Solves have kept leaving the prior and none agreed: distrust it, take this one.
	int64_t since_ns = device->prior_disagree_since_ns.load();
	if (since_ns == 0 && device->prior_disagree_since_ns.compare_exchange_strong(since_ns, sample.timestamp_ns)) {
		since_ns = sample.timestamp_ns;
	}
	const uint32_t count = device->prior_disagree_count.fetch_add(1) + 1;
	if (this->tracker->prior_distrust_ns > 0 && count >= CT_PRIOR_DISTRUST_MIN_SOLVES &&
	    sample.timestamp_ns - since_ns >= this->tracker->prior_distrust_ns) {
		// The prior is the device's fusion, or follows from it: start that over from the solves too.
		if (debug_get_bool_option_constellation_tracker_fusion_reset()) {
			device->sensor_fusion->requestReset();
			device->fusion_resets_total.fetch_add(1);
		}
		const uint64_t distrusted = device->prior_distrusts.tick(os_monotonic_get_ns());
		if (distrusted > 0) {
			CT_INFO(
			    this->tracker,
			    "Device %d: its prior was left by %u solves over %.0f ms and agreed with by none, taking "
			    "them as they are (%" PRIu64 " time(s) in the last second).",
			    device->id, count, time_ns_to_ms_f(sample.timestamp_ns - since_ns), distrusted);
		}
		return true;
	}

	device->prior_rejections_total.fetch_add(1);
	const uint64_t rejected = device->prior_rejections.tick(os_monotonic_get_ns());
	if (rejected > 0) {
		xrt_quat delta;
		math_quat_unrotate(&Tcv_cam_device.orientation, &Tcv_cam_device_prior.orientation, &delta);
		const float angle_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(delta.w), 0.0f, 1.0f)));
		const float offset_m = m_vec3_len(Tcv_cam_device.position - Tcv_cam_device_prior.position);
		CT_INFO(this->tracker,
		        "Device %d: rejected %" PRIu64
		        " solve(s) in the last second that left its prior, the latest by "
		        "%.0f degrees and %.0f mm (camera %zu, source %d).",
		        device->id, rejected, angle_deg, offset_m * 1000.0f, this->index, static_cast<int>(source));
	}

	return false;
}

bool
Camera::tryDevicePose(Device *device,
                      CameraSample &sample,
                      DeviceState &device_state,
                      const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
                      const xrt_pose &Tcv_cam_device_candidate,
                      SolveSource source)
{
	pose_metrics score;
	if (Tcv_cam_device_prior.has_value()) {
		pose_metrics_evaluate_pose_with_prior(  //
		    &score,                             //
		    &Tcv_cam_device_candidate,          //
		    false,                              //
		    &Tcv_cam_device_prior.value().pose, //
		    &device->prior_pos_error,           //
		    &device->prior_rot_error,           //
		    sample.blobs,                       //
		    sample.blob_count,                  //
		    &device->params.led_model,          //
		    device->id,                         //
		    &this->model,                       //
		    NULL);                              //
	} else {
		pose_metrics_evaluate_pose(    //
		    &score,                    //
		    &Tcv_cam_device_candidate, //
		    sample.blobs,              //
		    sample.blob_count,         //
		    &device->params.led_model, //
		    device->id,                //
		    &this->model,              //
		    NULL);                     //
	}

	// A degenerate pose had fewer LEDs than the device's model asks for, see its match parameters.
	if (POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD | POSE_MATCH_LED_IDS) &&
	    !POSE_HAS_FLAGS(&score, POSE_MATCH_DEGENERATE) &&
	    this->poseMatchesGravity(device, sample, Tcv_cam_device_candidate, source)) {
		return this->pushPose(        //
		    sample,                   //
		    device_state,             //
		    device,                   //
		    score,                    //
		    Tcv_cam_device_candidate, //
		    std::nullopt,             //
		    source);                  //
	}

	return false;
}

/*
 * A CV1 Touch's prior was 14-19 mm from the pose then found (median, 38-63 at p90), against the HMD's 5: at 3.4 m that
 * is 3-4 pixels, while an LED only matches a blob within about 2. The prior's projected LEDs were then a few pixels off
 * the controller's blobs, all the same way, and the fast path gave the frame to the slow search.
 */
// On: against Oculus' poses it halved the left controller's jumps and its time more than 10 cm off at night.
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_shift_match, "CONSTELLATION_TRACKER_SHIFT_MATCH", true)
#define CT_SHIFT_SEARCH_RADIUS_PX 12.0f
#define CT_SHIFT_AGREE_PX 2.0f
#define CT_SHIFT_MIN_LEDS 3

bool
Camera::tryDevicePoseShifted(Device *device,
                             CameraSample &sample,
                             DeviceState &device_state,
                             const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
                             SolveSource source)
{
	/*
	 * Off by default: replaying two worn sessions, it got more of the controllers in one (left 68 -> 84 %, right 29
	 * -> 73 %) and less of the right in the other (moving 90-100 -> 45-66 %), where it latched onto other blobs
	 * nearby. To be judged against ground truth, CONSTELLATION_TRACKER_SHIFT_MATCH=1 turns it on.
	 */
	if (!debug_get_bool_option_constellation_tracker_shift_match()) {
		return false;
	}

	const xrt_pose &prior = Tcv_cam_device_prior->pose;
	if (prior.position.z <= 0.1f) {
		return false;
	}

	pose_metrics_blob_match_info info;
	pose_metrics_match_pose_to_blobs(&prior, sample.blobs, sample.blob_count, &device->params.led_model, device->id,
	                                 &this->model, &info);

	// The offset from each projected LED to the nearest blob no other device has.
	std::array<float, MAX_OBJECT_LEDS> dx{};
	std::array<float, MAX_OBJECT_LEDS> dy{};
	int n = 0;
	for (int i = 0; i < info.num_visible_leds && n < MAX_OBJECT_LEDS; i++) {
		const xrt_vec2 &p = info.visible_leds[i].pos_px_undistorted;
		float best = CT_SHIFT_SEARCH_RADIUS_PX * CT_SHIFT_SEARCH_RADIUS_PX;
		int best_blob = -1;
		for (uint32_t b = 0; b < sample.blob_count; b++) {
			const t_blob &blob = sample.blobs[b];
			if (blob.matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID &&
			    blob.matched_device_id != device->id) {
				continue;
			}
			const float ex = blob.center_undistorted.x - p.x;
			const float ey = blob.center_undistorted.y - p.y;
			if (ex * ex + ey * ey < best) {
				best = ex * ex + ey * ey;
				best_blob = static_cast<int>(b);
			}
		}
		if (best_blob >= 0) {
			dx[n] = sample.blobs[best_blob].center_undistorted.x - p.x;
			dy[n] = sample.blobs[best_blob].center_undistorted.y - p.y;
			n++;
		}
	}
	if (n < CT_SHIFT_MIN_LEDS) {
		return false;
	}

	std::array<float, MAX_OBJECT_LEDS> sx = dx;
	std::array<float, MAX_OBJECT_LEDS> sy = dy;
	std::nth_element(sx.begin(), sx.begin() + n / 2, sx.begin() + n);
	std::nth_element(sy.begin(), sy.begin() + n / 2, sy.begin() + n);
	const float mx = sx[n / 2];
	const float my = sy[n / 2];

	// Enough of them have to agree on it, the rest may be other LEDs' blobs.
	int agree = 0;
	for (int i = 0; i < n; i++) {
		if (fabsf(dx[i] - mx) < CT_SHIFT_AGREE_PX && fabsf(dy[i] - my) < CT_SHIFT_AGREE_PX) {
			agree++;
		}
	}
	if (agree < CT_SHIFT_MIN_LEDS || (fabsf(mx) < 1.0f && fabsf(my) < 1.0f)) {
		return false;
	}

	// Move the prior across the image: OpenCV camera frame, x right, y down.
	xrt_pose shifted = prior;
	shifted.position.x += mx * prior.position.z / static_cast<float>(this->model.calib_pinhole.fx);
	shifted.position.y += my * prior.position.z / static_cast<float>(this->model.calib_pinhole.fy);

	return this->tryDevicePose(device, sample, device_state, Tcv_cam_device_prior, shifted, source);
}

/*
 * A headset no sensor sees is held turning about the neck (SensorFusion::getTrackedPose), and that is where the fast
 * paths look for it. Turned in the hands, or about itself as it is put on, it stays where it was instead: a 50 degree
 * turn carried the held pose 13 cm off the headset, whose 5-6 LEDs in view are too few for a search without a prior,
 * and it was not found again for as long as it stayed so. Where it was last seen, turned as it is held, is tried too.
 */
#define CT_HELD_STILL_MIN_APART_M 0.02f

bool
Camera::tryDevicePoseHeldStill(Device *device,
                               CameraSample &sample,
                               DeviceState &device_state,
                               const std::optional<xrt_pose> &Tcv_cam_world)
{
	if (!device->params.head || device_state.prior_position_tracked || !Tcv_cam_world.has_value() ||
	    !device_state.Tcv_cam_device_predicted.has_value() || !device_state.Txr_world_device_prior.has_value()) {
		return false;
	}

	xrt_pose Txr_world_device = device_state.Txr_world_device_prior.value();
	{
		std::unique_lock<os::Mutex> lock(device->data_lock);
		const auto &last = device->locked_data.last_known_pose;
		if (!last.has_value()) {
			return false;
		}
		Txr_world_device.position = last->Txr_world_device.position;
	}
	if (m_vec3_len(Txr_world_device.position - device_state.Txr_world_device_prior->position) <
	    CT_HELD_STILL_MIN_APART_M) {
		return false;
	}

	xrt_pose Tcv_world_device;
	math_pose_convert_from_opencv(&Txr_world_device, &Tcv_world_device);

	std::optional<xrt_space_relation> Tcv_cam_device_held = device_state.Tcv_cam_device_predicted;
	math_pose_transform(&Tcv_cam_world.value(), &Tcv_world_device, &Tcv_cam_device_held->pose);

	return this->tryDevicePose(device, sample, device_state, Tcv_cam_device_held, Tcv_cam_device_held->pose,
	                           SolveSource::FastHeldStill) ||
	       this->tryDevicePoseShifted(device, sample, device_state, Tcv_cam_device_held,
	                                  SolveSource::FastHeldStill);
}

/*
 * The Oculus runtime keeps a controller with 2 or 3 LEDs in view: with a recent pose the orientation is known from
 * the IMU, and only the position is left to solve (Meta's "P2P" proximity matching, 2 matches with the prior
 * orientation). A full pose needs 4-5 LEDs, and a Touch turned at the wrist often shows a sensor fewer: the left
 * controller was tracked 81-90 % of the time while the wrists turned where Oculus had it 98 %.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_few_leds, "CONSTELLATION_TRACKER_FEW_LEDS", true)
//! How far a projected LED's blob may be, and how closely the matches have to agree on the shift.
#define CT_FEW_SEARCH_PX 10.0f
#define CT_FEW_AGREE_PX 1.5f
//! The next nearest blob has to be this much further than the nearest, or the match is ambiguous.
#define CT_FEW_UNIQUE_PX 3.0f
//! The solved position may be at most this far from the prior's, and has to fit its blobs this well.
#define CT_FEW_MAX_MOVE_M 0.06f
#define CT_FEW_MAX_RMS_PX 1.0f
//! Pulls the position towards the prior's: with 2 LEDs the depth is weakly held. 1 cm costs about what 1 px does.
#define CT_FEW_PRIOR_WEIGHT 0.25
/*!
 * Few-LED poses only bridge a short gap after a full pose, as a wrist turns: chained on their own they followed a
 * drifting prior, 12 % of them 10 cm or more off.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_few_leds_bridge_ms, "CONSTELLATION_TRACKER_FEW_LEDS_BRIDGE_MS", 500)
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_few_leds_min, "CONSTELLATION_TRACKER_FEW_LEDS_MIN", 3)
/*!
 * Also with up to this many LEDs, once the full solves from the prior failed: a Touch seen nearly edge-on shows 4-5
 * LEDs in a short row, which the shift match then finds where the full refinement did not. pushPose refines the pose
 * found with the IMU's tilt weighed in.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_few_leds_max, "CONSTELLATION_TRACKER_FEW_LEDS_MAX", 6)
/*!
 * The fast paths' refinement weighs the IMU's tilt in, see pose_optimize.cpp. Held upright, a pose from the wrong LEDs
 * no longer tilts away but fits its blobs worse: such a pose has to fit within this RMS, in pixels.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_gravity_prior_fast, "CONSTELLATION_TRACKER_GRAVITY_PRIOR_FAST", true)
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_gravity_prior_max_rms_px,
                            "CONSTELLATION_TRACKER_GRAVITY_PRIOR_MAX_RMS_PX",
                            0.4f)
//! The same as pose_optimize.cpp's: poses from more LEDs are refined without the IMU's tilt.
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_gravity_prior_max_leds,
                          "CONSTELLATION_TRACKER_GRAVITY_PRIOR_MAX_LEDS",
                          8)

static bool
is_slow_search(SolveSource source)
{
	return source == SolveSource::SlowShallowSearch || source == SolveSource::SlowDeepSearch;
}

bool
Camera::tryDevicePoseFewLeds(Device *device,
                             CameraSample &sample,
                             DeviceState &device_state,
                             const std::optional<xrt_space_relation> &Tcv_cam_device_prior)
{
	if (!debug_get_bool_option_constellation_tracker_few_leds() || !device_state.prior_position_tracked) {
		return false; // Only a fresh prior's orientation is good enough to keep.
	}
	const int64_t last_full_ns = device->last_full_solve_ns.load();
	if (last_full_ns == 0 ||
	    sample.timestamp_ns - last_full_ns >
	        debug_get_num_option_constellation_tracker_few_leds_bridge_ms() * (int64_t)U_TIME_1MS_IN_NS) {
		return false;
	}

	const xrt_pose &prior = Tcv_cam_device_prior->pose;
	if (prior.position.z <= 0.1f) {
		return false;
	}

	pose_metrics_blob_match_info info;
	pose_metrics_match_pose_to_blobs(&prior, sample.blobs, sample.blob_count, &device->params.led_model, device->id,
	                                 &this->model, &info);

	// Each visible LED's nearest blob that no other device has, if it is clearly the nearest.
	struct Pair
	{
		int led;
		int blob;
		float dx, dy;
	};
	std::array<Pair, MAX_OBJECT_LEDS> pairs{};
	int n = 0;
	for (int i = 0; i < info.num_visible_leds && n < MAX_OBJECT_LEDS; i++) {
		const xrt_vec2 &p = info.visible_leds[i].pos_px_undistorted;
		float best = CT_FEW_SEARCH_PX * CT_FEW_SEARCH_PX;
		float second = 1e9f;
		int best_blob = -1;
		for (uint32_t b = 0; b < sample.blob_count; b++) {
			const t_blob &blob = sample.blobs[b];
			if (blob.matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID &&
			    blob.matched_device_id != device->id) {
				continue;
			}
			const float ex = blob.center_undistorted.x - p.x;
			const float ey = blob.center_undistorted.y - p.y;
			const float d2 = ex * ex + ey * ey;
			if (d2 < best) {
				second = best;
				best = d2;
				best_blob = static_cast<int>(b);
			} else if (d2 < second) {
				second = d2;
			}
		}
		if (best_blob < 0 || sqrtf(second) - sqrtf(best) < CT_FEW_UNIQUE_PX) {
			continue;
		}
		const int led = static_cast<int>(info.visible_leds[i].led - device->params.led_model.leds);
		pairs[n++] = {led, best_blob, sample.blobs[best_blob].center_undistorted.x - p.x,
		              sample.blobs[best_blob].center_undistorted.y - p.y};
	}
	if (n < 2) {
		return false;
	}

	// The shift most of them agree on: the prior's position is a few pixels off, its orientation is not.
	int best_support = 0;
	int best_i = -1;
	for (int i = 0; i < n; i++) {
		int support = 0;
		for (int j = 0; j < n; j++) {
			if (fabsf(pairs[j].dx - pairs[i].dx) < CT_FEW_AGREE_PX &&
			    fabsf(pairs[j].dy - pairs[i].dy) < CT_FEW_AGREE_PX) {
				support++;
			}
		}
		if (support > best_support) {
			best_support = support;
			best_i = i;
		}
	}
	// With more a full pose is better found by the other paths, which ran first.
	if (best_support < debug_get_num_option_constellation_tracker_few_leds_min() ||
	    best_support > debug_get_num_option_constellation_tracker_few_leds_max()) {
		return false;
	}

	/*
	 * Position only, the orientation kept: each blob ray x, y and rotated model point X give
	 * t_x - x t_z = x X_z - X_x and t_y - y t_z = y X_z - X_y, plus a pull towards the prior's position.
	 */
	const xrt_vec3 tp = prior.position;
	double ata[3][3] = {{CT_FEW_PRIOR_WEIGHT, 0, 0}, {0, CT_FEW_PRIOR_WEIGHT, 0}, {0, 0, CT_FEW_PRIOR_WEIGHT}};
	double atb[3] = {CT_FEW_PRIOR_WEIGHT * tp.x, CT_FEW_PRIOR_WEIGHT * tp.y, CT_FEW_PRIOR_WEIGHT * tp.z};
	std::array<Pair, MAX_OBJECT_LEDS> used{};
	int m = 0;
	for (int j = 0; j < n; j++) {
		if (fabsf(pairs[j].dx - pairs[best_i].dx) >= CT_FEW_AGREE_PX ||
		    fabsf(pairs[j].dy - pairs[best_i].dy) >= CT_FEW_AGREE_PX) {
			continue;
		}
		used[m++] = pairs[j];
		xrt_vec3 X;
		math_quat_rotate_vec3(&prior.orientation, &device->params.led_model.leds[pairs[j].led].position, &X);
		const t_blob &blob = sample.blobs[pairs[j].blob];
		const double x = blob.center_homogenized.x;
		const double y = blob.center_homogenized.y;
		const double rows[2][4] = {{1, 0, -x, x * X.z - X.x}, {0, 1, -y, y * X.z - X.y}};
		for (const auto &r : rows) {
			for (int a = 0; a < 3; a++) {
				for (int b = 0; b < 3; b++) {
					ata[a][b] += r[a] * r[b];
				}
				atb[a] += r[a] * r[3];
			}
		}
	}

	auto det3 = [](const double mtx[3][3]) {
		return mtx[0][0] * (mtx[1][1] * mtx[2][2] - mtx[1][2] * mtx[2][1]) -
		       mtx[0][1] * (mtx[1][0] * mtx[2][2] - mtx[1][2] * mtx[2][0]) +
		       mtx[0][2] * (mtx[1][0] * mtx[2][1] - mtx[1][1] * mtx[2][0]);
	};
	const double det = det3(ata);
	if (fabs(det) < 1e-12) {
		return false;
	}
	double t[3];
	for (int c = 0; c < 3; c++) {
		double mtx[3][3];
		for (int a = 0; a < 3; a++) {
			for (int b = 0; b < 3; b++) {
				mtx[a][b] = b == c ? atb[a] : ata[a][b];
			}
		}
		t[c] = det3(mtx) / det;
	}

	xrt_pose solved = prior;
	solved.position = {static_cast<float>(t[0]), static_cast<float>(t[1]), static_cast<float>(t[2])};
	if (m_vec3_len(solved.position - tp) > CT_FEW_MAX_MOVE_M || solved.position.z <= 0.1f) {
		return false;
	}

	// How well it fits the blobs it was solved from, in pixels.
	double sq = 0.0;
	for (int j = 0; j < m; j++) {
		xrt_vec3 X;
		math_quat_rotate_vec3(&solved.orientation, &device->params.led_model.leds[used[j].led].position, &X);
		X = X + solved.position;
		const t_blob &blob = sample.blobs[used[j].blob];
		const double ex = (X.x / X.z - blob.center_homogenized.x) * this->model.calib_pinhole.fx;
		const double ey = (X.y / X.z - blob.center_homogenized.y) * this->model.calib_pinhole.fy;
		sq += ex * ex + ey * ey;
	}
	if (sqrt(sq / m) > CT_FEW_MAX_RMS_PX) {
		return false;
	}

	pose_metrics score;
	pose_metrics_evaluate_pose(&score, &solved, sample.blobs, sample.blob_count, &device->params.led_model,
	                           device->id, &this->model, NULL);
	return this->pushPose(sample, device_state, device, score, solved, std::nullopt, SolveSource::FastFewLeds);
}

bool
Camera::tryDeviceBlobRecovery(Device *device,
                              CameraSample &sample,
                              DeviceState &device_state,
                              const std::optional<xrt_space_relation> &Tcv_cam_device_prior)
{
	auto tracker = this->tracker;

	const uint32_t needed_blobs = 4;
	uint32_t num_blobs = 0;
	for (uint32_t index = 0; index < sample.blob_count; index++) {
		t_blob &b = sample.blobs[index];
		if (b.matched_device_id == device->id) {
			num_blobs++;

			if (num_blobs >= needed_blobs) {
				break;
			}
		}
	}
	if (num_blobs < needed_blobs) {
		return false;
	}

	OldOptimizationData old_optimization_data;
	U_ZERO(&old_optimization_data);

	// RANSAC-PnP with the matched blobs
	// The optimization should still be able to run with an identity pose.
	xrt_pose Tcv_cam_device =
	    Tcv_cam_device_prior
	        .value_or(xrt_space_relation{
	            .relation_flags = static_cast<xrt_space_relation_flags>(0),
	            .pose =
	                {
	                    .orientation = {.x = 0, .y = 1, .z = 0, .w = 0}, // 180 around Y, facing the camera
	                    .position = {0, 0, 1},
	                },
	            .linear_velocity = {},
	            .angular_velocity = {},
	        })
	        .pose;
	xrt_pose Tcv_cam_device_optimized;
	xrt_vec3 camera_gravity;
	xrt_vec3 device_gravity;
	const bool have_gravity = debug_get_bool_option_constellation_tracker_gravity_prior_fast() &&
	                          this->gravityForSample(device, sample, camera_gravity, device_gravity);
	bool success = optimizePose(                   //
	    tracker->log_level,                        //
	    tracker->deterministic,                    //
	    this->model,                               //
	    Tcv_cam_device,                            //
	    sample.blobs,                              //
	    sample.blob_count,                         //
	    &device->params.led_model,                 //
	    device->id,                                //
	    Tcv_cam_device_optimized,                  //
	    old_optimization_data.covariance,          //
	    have_gravity ? &camera_gravity : nullptr,  //
	    have_gravity ? &device_gravity : nullptr); //
	if (!success) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs failed", (void *)this,
		         device->id, sample.blob_count);
		return false;
	}

	CT_DEBUG(tracker, "Camera %p RANSAC-PnP blob recovery for device %d from %u blobs succeeded", (void *)this,
	         device->id, sample.blob_count);
	Tcv_cam_device = Tcv_cam_device_optimized;

	pose_metrics score;
	// Evaluate the pose, using the prior if available
	if (Tcv_cam_device_prior.has_value()) {
		pose_metrics_evaluate_pose_with_prior(  //
		    &score,                             //
		    &Tcv_cam_device,                    //
		    true,                               //
		    &Tcv_cam_device_prior.value().pose, //
		    &device->prior_pos_error,           //
		    &device->prior_rot_error,           //
		    sample.blobs,                       //
		    sample.blob_count,                  //
		    &device->params.led_model,          //
		    device->id,                         //
		    &this->model,                       //
		    NULL);
	} else {
		pose_metrics_evaluate_pose(&score, &Tcv_cam_device, sample.blobs, sample.blob_count,
		                           &device->params.led_model, device->id, &this->model, NULL);
	}

	/*
	 * @note I saw some cases where an otherwise valid solve was dropped because we didn't trust the RANSAC solve.
	 *       I don't think this will make things *worse*, but it seems to at least get some more valid solves
	 *       through, so this is fine for now, set as a constexpr bool for easy tweaking purposes.
	 *       Since a RANSAC has already made an assessment over the validity of the pose, and we already trust that
	 *       assessment, this shouldn't cause many problems. RANSAC is also only meaningful if 4 or more blobs were
	 *       matched in the RANSAC run, otherwise RANSAC did not even run.
	 *
	 * Valid solves failed the check beyond about 2 meters because the projected LED radius had no floor, see
	 * pose_metrics.c. With that fixed, trusting RANSAC let through the worst solves of a worn CV1 session: against
	 * the same sensor's previous solve, 22 arcmin and 5 mm apart at the median and 281 arcmin and 74 mm at p95,
	 * where the checked predicted-pose solves were 11 arcmin and 1.5 mm, and 103 arcmin and 24 mm.
	 */
	constexpr bool kAlwaysTrustRansac = false;

	if (((kAlwaysTrustRansac && score.matched_blobs > 4) || POSE_HAS_FLAGS(&score, POSE_MATCH_GOOD)) &&
	    !POSE_HAS_FLAGS(&score, POSE_MATCH_DEGENERATE) &&
	    this->poseMatchesGravity(device, sample, Tcv_cam_device, SolveSource::FastBlobRecovery)) {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose was good enough for device %d from %u blobs",
		         (void *)this, device->id, sample.blob_count);

		return this->pushPose(              //
		    sample,                         //
		    device_state,                   //
		    device,                         //
		    score,                          //
		    Tcv_cam_device,                 //
		    &old_optimization_data,         //
		    SolveSource::FastBlobRecovery); //
	} else {
		CT_DEBUG(tracker, "Camera %p RANSAC-PnP recovered pose for device %d from %u blobs was not good enough",
		         (void *)this, device->id, sample.blob_count);
	}

	return false;
}

void
Camera::handleProcessingComplete(CameraSample &sample)
{
	auto tracker = this->tracker;

#ifdef XRT_FEATURE_RERUN
	if (tracker->rerun_stream) {
		tracker->rerun_stream->logSample(*tracker, sample);
	}
#endif

	// Both callers hold `tracker->device_lock`, which is what lets this resolve the sample's devices.
	tracker->rig_calibration->pushCameraSample(tracker, sample);

	if (!tracker->checkSensorCalibration()) {
		CT_ERROR(tracker, "Sensor tried to do calibration but it failed.");
	}

	for (uint32_t i = 0; i < sample.device_count; i++) {
		auto &sample_device = sample.device_states[i];
		auto device_iter = std::find_if(
		    tracker->devices.begin(), tracker->devices.end(),
		    [&sample_device](const auto &device) { return sample_device.device_id == device->id; });
		// Skip removed devices
		if (device_iter == tracker->devices.end()) {
			continue;
		}
		auto &device = *device_iter;

		// Mark the device's found pose as handled.
		if (auto &found_pose = sample_device.found_pose) {
			if (!found_pose->handled) {
				device->sensor_fusion->pushCameraSample(tracker, sample);
			}

			found_pose->handled = true;
		}
	}
}

void
Camera::processSampleSlow(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	auto &data = this->slow_processing_thread_data;

	CT_TRACE(tracker, "Starting slow processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	correspondence_search_set_blobs(data.cs, sample.blobs, sample.blob_count);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera");

	if (tracker->devices_changing.load()) {
		return;
	}

	std::shared_lock lock(tracker->device_lock);

	auto Txr_world_cam = sample.Txr_world_cam;

	// Borrowed pointers, only valid while we hold the device lock taken above.
	std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> devices{};
	const uint32_t device_count = collect_devices_nearest_first(tracker, sample, devices);

	bool any_shallow_failed = false;

	for (int pass = 0; pass < 2; pass++) {
		for (uint32_t device_iter = 0; device_iter < device_count; device_iter++) {
			Device *device = devices[device_iter];

			// A device is being added or removed, let it have the lock.
			if (tracker->devices_changing.load()) {
				return;
			}

			// Device is not enabled.
			if (!device->enabled.load()) {
				continue;
			}

			// Do a shallow search first go around
			correspondence_search_flags search_flags =
			    pass == 0 ? CS_FLAG_SHALLOW_SEARCH : CS_FLAG_DEEP_SEARCH;

			{
				std::lock_guard<os::Mutex> lock(device->data_lock);

				bool enable_slow_skip =
				    device->locked_data.last_known_pose.has_value() && // device has been seen
				    Txr_world_cam.has_value();                         // we have a pose

				if (enable_slow_skip) {
					const auto &last_known_pose = *device->locked_data.last_known_pose;

					time_duration_ns time_since_solve_ns =
					    sample.timestamp_ns - last_known_pose.timestamp_ns;

					if (time_since_solve_ns < (U_TIME_1MS_IN_NS * 150LL)) {
						// If the device has been seen in the last 150ms, don't try to brute
						// force search it, hope that it will be seen by a fast solve in that
						// time period.
						continue;
					} else if ((time_since_solve_ns < (U_TIME_1MS_IN_NS * 1000LL) ||
					            this->shallow_searches_failed.load() < 10) &&
					           (search_flags & CS_FLAG_DEEP_SEARCH)) {
						// Don't do a deep search until a shallow search has failed for one full
						// second, or we've failed less than 10 shallow searches for any device.
						continue;
					}
				}
			}

			search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_STOP_FOR_STRONG_MATCH);

			DeviceState *device_state_ptr = sample.getDeviceState(device->id).value_or(nullptr);
			// If there was no device state in the sample, that means this device appeared after the
			// constellation tracker started this sample, so we need to fill out the device state here.
			if (device_state_ptr == nullptr) {
				device_state_ptr = &sample.putDeviceState(device->id);

				// we need to do a slow process for this device since it wasn't present in the fast
				// processing
				device_state_ptr->needs_slow_processing = true;
			}
			DeviceState &device_state = *device_state_ptr;

			if (!device_state.needs_slow_processing) {
				// we already did a fast process for this device and it succeeded, so there is no need
				// to do a slow search
				continue;
			}

			xrt_pose Tcv_cam_device = XRT_POSE_IDENTITY;
			if (device_state.Txr_world_device_prior.has_value() && Txr_world_cam.has_value()) {
				xrt_pose Txr_cam_world;
				math_pose_invert(&Txr_world_cam.value(), &Txr_cam_world);

				xrt_pose Txr_cam_device;
				math_pose_transform(&Txr_cam_world, &device_state.Txr_world_device_prior.value(),
				                    &Txr_cam_device);

				math_pose_convert_from_opencv(&Txr_cam_device, &Tcv_cam_device);

				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_HAVE_POSE_PRIOR);
			}

			/*
			 * Candidates that tilt the device away from what its IMU says are skipped inside the search, so
			 * that it goes on to the right one rather than stopping at a wrong strong match, see @ref
			 * ImuTilt.
			 */
			xrt_vec3 camera_gravity = {0.0f, 1.0f, 0.0f};
			xrt_vec3 device_gravity = {0.0f, 1.0f, 0.0f};
			if (this->gravityForSample(device, sample, camera_gravity, device_gravity)) {
				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_MATCH_GRAVITY);
			}

			// A binding prior binds the search as it binds the fast paths, see bindingPrior.
			const xrt_vec3 *pos_error_thresh = &device->prior_pos_error;
			const xrt_vec3 *rot_error_thresh = &device->prior_rot_error;
			if ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) != 0 &&
			    debug_get_bool_option_constellation_tracker_slow_prior_binds() &&
			    this->bindingPrior(device, sample, device_state, pos_error_thresh, rot_error_thresh)) {
				search_flags = (correspondence_search_flags)(search_flags | CS_FLAG_PRIOR_MUST_MATCH);
			}

			/*
			 * With no prior the search tries the blobs against each other, combinatorially: the static
			 * spots, daylight's, are left out of it, see Camera::classifyStatic. In a recording
			 * every such search over 60 blobs ran out of time and a lost controller was not found again;
			 * Meta's "holiday lights" fix keeps such spots out of matching the same way. Once a second it
			 * takes every blob, for a lost device lying still, whose own blobs are static too. With a prior
			 * the search keeps them all.
			 */
			correspondence_search *cs = data.cs;
			if ((search_flags & CS_FLAG_HAVE_POSE_PRIOR) == 0) {
				timepoint_ns &last_full_ns =
				    data.last_full_blind_ns[device->id % XRT_CONSTELLATION_MAX_DEVICES];
				uint32_t left_out = 0;
				for (uint32_t i = 0; i < sample.blob_count; i++) {
					left_out += sample.blob_static[i] ? 1 : 0;
				}
				if (left_out > 0 && sample.timestamp_ns - last_full_ns < CT_FULL_BLIND_SEARCH_NS) {
					data.blind_blobs.clear();
					for (uint32_t i = 0; i < sample.blob_count; i++) {
						if (!sample.blob_static[i]) {
							data.blind_blobs.push_back(sample.blobs[i]);
						}
					}
					correspondence_search_set_blobs(data.cs_blind, data.blind_blobs.data(),
					                                static_cast<int>(data.blind_blobs.size()));
					cs = data.cs_blind;
					data.blind_searches++;
					data.blind_spots_left_out += left_out;
				} else {
					last_full_ns = sample.timestamp_ns;
				}
			}

			pose_metrics score;
			bool found_pose = correspondence_search_find_one_pose( //
			    cs,                                                //
			    device->search_model,                              //
			    search_flags,                                      //
			    &Tcv_cam_device,                                   //
			    pos_error_thresh,                                  //
			    rot_error_thresh,                                  //
			    &camera_gravity,                                   //
			    &device_gravity,                                   //
			    tracker->gravity_tolerance_rad,                    //
			    &score);                                           //
			if (found_pose) {
				const SolveSource source = (search_flags & CS_FLAG_DEEP_SEARCH) != 0
				                               ? SolveSource::SlowDeepSearch
				                               : SolveSource::SlowShallowSearch;

				found_pose = this->pushPose(sample,         //
				                            device_state,   //
				                            device,         //
				                            score,          //
				                            Tcv_cam_device, //
				                            std::nullopt,   //
				                            source);        //
			}

			if (found_pose) {
				// We found a pose for this device in this sample
				device_state.needs_slow_processing = false;
			} else {
				CT_TRACE(tracker, "Camera %p slow processing for device %d failed to find a pose",
				         (void *)this, device->id);

				if (search_flags & CS_FLAG_SHALLOW_SEARCH) {
					any_shallow_failed = true;
				}
			}
		}
	}

	if (any_shallow_failed) {
		this->shallow_searches_failed.fetch_add(1);
	} else {
		this->shallow_searches_failed.store(0);
	}

	// The moved-camera watch's own look at the HMD, see probeForRigWatch; the Touch rings are too symmetric for it.
	if (debug_get_bool_option_constellation_tracker_rig_watch() && Txr_world_cam.has_value()) {
		for (uint32_t device_iter = 0; device_iter < device_count; device_iter++) {
			Device *device = devices[device_iter];
			if (tracker->devices_changing.load()) {
				break;
			}
			if (device->enabled.load() && device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS) {
				this->probeForRigWatch(sample, device);
			}
		}
	}

	for (uint32_t i = 0; i < sample.device_count; i++) {
		auto &device = sample.device_states[i];

		device.needs_slow_processing = false;
	}

	this->debugScribbleSample(sample, false);

	this->handleProcessingComplete(sample);
}

bool
Camera::processSampleFast(CameraSample &sample)
{
	ConstellationTracker *tracker = this->tracker;

	CT_TRACE(tracker, "Starting fast processing for camera %p with %u blobs", (void *)this, sample.blob_count);

	if (tracker->devices_changing.load()) {
		return false;
	}

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	auto Txr_world_cam = sample.Txr_world_cam;
	auto Tcv_world_cam = Txr_world_cam;
	auto Tcv_cam_world = Txr_world_cam;
	if (Txr_world_cam.has_value()) {
		math_pose_convert_from_opencv(&Txr_world_cam.value(), &Tcv_world_cam.value());
		math_pose_invert(&Tcv_world_cam.value(), &Tcv_cam_world.value());
	}

	// A loose part measured or given up since: into the model before this sample, with no lock held yet.
	tracker->applyLooseParts();
	// The room asked to be measured again (remeasure), also with no lock held.
	tracker->applyRemeasure();

	std::shared_lock lock(tracker->device_lock);

	// Figure out where all the devices are first, so we can then sort it.
	for (std::unique_ptr<Device> &device : tracker->devices) {
		xrt_space_relation device_predicted_relation = XRT_SPACE_RELATION_ZERO; //< AKA "the prior"

		if (device->params.tracking_source != nullptr) {
			t_constellation_tracker_tracking_source_get_tracked_pose(
			    device->params.tracking_source, sample.timestamp_ns, &device_predicted_relation);
		}

		bool prior_pose_valid =
		    (device_predicted_relation.relation_flags &
		     (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT)) ==
		    (XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT);

		std::optional<xrt_space_relation> Tcv_cam_device_predicted = std::nullopt; //< AKA "the prior"
		// Whether the prior pose is actually valid and we have a pose for the camera
		if (Tcv_cam_world.has_value() && prior_pose_valid) {
			xrt_space_relation Tcv_world_device_predicted = device_predicted_relation;
			math_space_relation_convert_from_opencv(&device_predicted_relation,
			                                        &Tcv_world_device_predicted);

			Tcv_cam_device_predicted.emplace(xrt_space_relation{}); // initialize it to a value
			math_pose_transform(&Tcv_cam_world.value(), &Tcv_world_device_predicted.pose,
			                    &Tcv_cam_device_predicted.value().pose);
		}

		auto &device_state = sample.putDeviceState(device->id);
		device_state.Txr_world_device_prior =
		    prior_pose_valid ? std::optional<xrt_pose>(device_predicted_relation.pose) : std::nullopt;
		device_state.Tcv_cam_device_predicted = Tcv_cam_device_predicted;
		device_state.prior_position_tracked =
		    prior_pose_valid &&
		    (device_predicted_relation.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
	}

	// Borrowed pointers, only valid while we hold the device lock taken above.
	std::array<Device *, XRT_CONSTELLATION_MAX_DEVICES> devices{};
	const uint32_t device_count = collect_devices_nearest_first(tracker, sample, devices);

	bool need_slow_search = false;
	for (uint32_t device_iter = 0; device_iter < device_count; device_iter++) {
		Device *device = devices[device_iter];

		// Device is not enabled.
		if (!device->enabled.load()) {
			continue;
		}

		// The first pass above put a state in the sample for every device we are about to walk.
		DeviceState &device_state = *sample.getDeviceState(device->id).value();

		const auto Tcv_cam_device_predicted = device_state.Tcv_cam_device_predicted; //< AKA "the prior"

		bool do_blob_recovery = true;
		if (Tcv_cam_device_predicted.has_value()) {
			// If the device is moving more than 5 meters per second, or it's rotating at more than one
			// rotation every second, then skip blob recovery, and also wipe associations, that's too fast
			// to keep sane blob associations.
			if (m_vec3_len(Tcv_cam_device_predicted->linear_velocity) > 5 ||
			    m_vec3_len(Tcv_cam_device_predicted->angular_velocity) > (M_PI * 2)) {
				do_blob_recovery = true;
			}
		}

		// if we have a valid prior pose, try to use it for fast matching
		if (Tcv_cam_device_predicted.has_value() &&                    //
		    this->tryDevicePose(device,                                //
		                        sample,                                //
		                        device_state,                          //
		                        Tcv_cam_device_predicted,              //
		                        Tcv_cam_device_predicted.value().pose, //
		                        SolveSource::FastPredicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with predicted pose", device->id);
			continue; // try the next device, we found a pose!
		}

		if (Tcv_cam_device_predicted.has_value() &&
		    this->tryDevicePoseShifted(device, sample, device_state, Tcv_cam_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with the shifted predicted pose",
			         device->id);
			continue;
		}

		if (this->tryDevicePoseHeldStill(device, sample, device_state, Tcv_cam_world)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded where it was last seen", device->id);
			continue;
		}

		if (Tcv_cam_device_predicted.has_value() &&
		    this->tryDevicePoseFewLeds(device, sample, device_state, Tcv_cam_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded from a few LEDs", device->id);
			continue;
		}

		// Try to use blobs after we've checked the prior itself
		bool wipe_blob_associations = false;
		if (do_blob_recovery &&                       //
		    this->tryDeviceBlobRecovery(device,       //
		                                sample,       //
		                                device_state, //
		                                Tcv_cam_device_predicted)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with blob recovery", device->id);
			continue; // try the next device, we found a pose!
		} else {
			wipe_blob_associations = true;
		}

		// Try to get a last known pose
		std::optional<xrt_pose> Tcv_cam_device_last_known = std::nullopt;
		{
			std::unique_lock<os::Mutex> lock(device->data_lock);

			// If we know where the camera is, we found any pose, and that pose had a valid world-space
			// pose.
			if (device->locked_data.last_known_pose.has_value()) {
				Tcv_cam_device_last_known = device->locked_data.last_known_pose->Tcv_cam_device;
			}
		}

		if (debug_get_bool_option_constellation_tracker_fast_last_known() &&
		    Tcv_cam_device_last_known.has_value() &&               //
		    this->tryDevicePose(device,                            //
		                        sample,                            //
		                        device_state,                      //
		                        Tcv_cam_device_predicted,          //
		                        Tcv_cam_device_last_known.value(), //
		                        SolveSource::FastLastKnown)) {
			CT_DEBUG(tracker, "Fast processing for device %d succeeded with last known pose", device->id);
			continue; // try the next device, we found a pose!
		}

		if (wipe_blob_associations) {
			// Blob-based recovery failed, unmark all blobs for this device.
			for (uint32_t i = 0; i < sample.blob_count; i++) {
				t_blob &b = sample.blobs[i];
				if (b.matched_device_id == device->id) {
					b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
					b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				}
			}
		}

		device_state.needs_slow_processing = true;

		need_slow_search = true;
	}

	this->debugScribbleSample(sample, true);

	// A device's loose part, when this camera sees little of its rigid side: what else it sees may be the part.
	for (std::unique_ptr<Device> &device : tracker->devices) {
		if (device->loose_part && device->enabled.load() && device->rigidBlobs(sample) <= 2) {
			device->loosePartView(static_cast<uint32_t>(mosaic->index * XRT_TRACKING_MAX_CAMS + this->index),
			                      sample, this->model);
		}
	}

	// Only save samples on the fast processing thread, since the slow processing thread is *triggered* by the fast
	// processing thread.
	if (tracker->data_recorder) {
		tracker->data_recorder->recordSample(sample);
	}

	/*
	 * Handle any poses that this sample may now have, it's okay if this is called again in the slow thread, that's
	 * what @ref FoundDevicePose::handled is for.
	 */
	this->handleProcessingComplete(sample);

	return need_slow_search;
}

void
Camera::feedRigWatch(Device *device,
                     const CameraSample &sample,
                     const xrt_pose &Tcv_cam_device,
                     uint32_t matched_blobs,
                     float rms_px)
{
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	if (device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS) {
		this->tracker->loadDownLearn(device);
	}
	// While cameras wait to be placed: the headset's good solves place them quickly (ConstellationTracker::quick).
	if (mosaic && matched_blobs >= CT_RIG_WATCH_MIN_LEDS && rms_px <= CT_RIG_WATCH_MAX_RMS_PX &&
	    device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS) {
		this->tracker->quickPlace(device, mosaic->index, this->index, sample, Tcv_cam_device, matched_blobs);
	}
	if (!mosaic || !sample.Txr_world_cam.has_value() || matched_blobs < CT_RIG_WATCH_MIN_LEDS ||
	    rms_px > CT_RIG_WATCH_MAX_RMS_PX) {
		return;
	}
	const xrt_pose &Txr_world_cam = sample.Txr_world_cam.value();
	xrt_pose Txr_cam_device;
	math_pose_convert_from_opencv(&Tcv_cam_device, &Txr_cam_device);

	float tilt_deg = -1.0f;
	xrt_vec3 camera_gravity;
	xrt_vec3 device_gravity;
	if (this->gravityForSample(device, sample, camera_gravity, device_gravity)) {
		tilt_deg = RAD_TO_DEG(pose_metrics_gravity_error_rad(&Tcv_cam_device, &camera_gravity, &device_gravity));
		// The room's level, from the headset's solves: its orientation in the room against its IMU's down.
		if (device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS &&
		    debug_get_bool_option_constellation_tracker_level()) {
			xrt_pose Tcv_world_cam;
			math_pose_convert_from_opencv(&sample.Txr_world_cam.value(), &Tcv_world_cam);
			xrt_quat Rcv_world_device;
			math_quat_rotate(&Tcv_world_cam.orientation, &Tcv_cam_device.orientation, &Rcv_world_device);
			this->tracker->feedLevel(sample.timestamp_ns,
			                         static_cast<uint32_t>(mosaic->index * XRT_TRACKING_MAX_CAMS + this->index),
			                         Rcv_world_device, m_vec3_normalize(device_gravity));
		}
		// Every camera's solves far off gravity: the room is measured again (remeasure).
		if (device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS) {
			this->tracker->feedRemeasure(sample.timestamp_ns, device,
			                             static_cast<uint32_t>(mosaic->index * XRT_TRACKING_MAX_CAMS + this->index),
			                             tilt_deg);
		}
		// The headset's: the room's down in its frame against its IMU's, for quick placement (down_learn).
		if (device->params.led_model.led_count > CT_RIG_WATCH_PROBE_MIN_LEDS && tilt_deg < 8.0f) {
			xrt_quat inv;
			math_quat_invert(&Tcv_cam_device.orientation, &inv);
			xrt_vec3 room_down;
			math_quat_rotate_vec3(&inv, &camera_gravity, &room_down);
			// Saved once learned in a level room: read before taking the learning's lock (never both held).
			bool room_level = false;
			{
				std::unique_lock<os::Mutex> level_lock(this->tracker->level.lock);
				room_level = this->tracker->level.last_tilt_deg >= 0.0f &&
				             this->tracker->level.last_tilt_deg <= CT_DOWN_SAVE_LEVEL_DEG &&
				             sample.timestamp_ns - this->tracker->level.last_ns <= CT_DOWN_SAVE_LEVEL_NS;
			}
			std::optional<std::pair<xrt_vec3, xrt_vec3>> save;
			size_t save_count = 0;
			{
				std::unique_lock<os::Mutex> lock(this->tracker->down_learn.lock);
				auto &l = this->tracker->down_learn;
				// The last few thousand solves weigh in: the IMU's seat in the headset does not change.
				const float w = 1.0f / (float)std::min<size_t>(l.count + 1, 4000);
				l.imu_down = l.imu_down * (1.0f - w) + m_vec3_normalize(device_gravity) * w;
				l.room_down = l.room_down * (1.0f - w) + m_vec3_normalize(room_down) * w;
				l.count++;
				// Worn: learned while lying still in one way it would not be the seat alone.
				if (!l.saved && l.count >= CT_DOWN_SAVE_SOLVES && room_level && device->worn.load()) {
					l.saved = true;
					save = std::make_pair(l.imu_down, l.room_down);
					save_count = l.count;
				}
			}
			const std::string serial(device->params.serial,
			                         strnlen(device->params.serial, sizeof(device->params.serial)));
			if (save.has_value() && !serial.empty() && debug_get_bool_option_constellation_tracker_down_saved()) {
				this->tracker->saveDownLearn(serial, save->first, save->second, save_count);
			}
		}
	}

	float fusion_deg = -1.0f;
	xrt_space_relation fused = XRT_SPACE_RELATION_ZERO;
	timepoint_ns fused_ns = 0;
	device->sensor_fusion->getTrackedPose(sample.timestamp_ns, fused_ns, fused);
	if ((fused.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) {
		xrt_pose Txr_cam_world;
		math_pose_invert(&Txr_world_cam, &Txr_cam_world);
		xrt_vec3 fused_in_cam;
		math_pose_transform_point(&Txr_cam_world, &fused.pose.position, &fused_in_cam);
		const float lengths = m_vec3_len(fused_in_cam) * m_vec3_len(Txr_cam_device.position);
		if (lengths > 0.0f) {
			fusion_deg = RAD_TO_DEG(
			    acosf(CLAMP(m_vec3_dot(fused_in_cam, Txr_cam_device.position) / lengths, -1.0f, 1.0f)));
		}
	}

	const bool fused_valid = (fused.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
	                         (fused.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;
	this->tracker->watchRig(device->id, mosaic->index, this->index, sample.timestamp_ns, Txr_world_cam,
	                        Txr_cam_device, tilt_deg, fusion_deg,
	                        fused_valid ? std::optional<xrt_pose>(fused.pose) : std::nullopt);
}

/*
 * The moved-camera watch's own look at a device: a camera moved more than a few cm (at 3.6 m) no longer
 * finds the device at all, the prior binding and the gravity check inside the search keep it to where the device
 * should be from its old placement, so the watch would never see it (replays with a camera moved by 3-10 degrees or
 * 10-20 cm: no solve of the moved camera reached the watch). When a camera has not found the HMD for a while that
 * another camera tracks, and that should be in its view, a search held to nothing runs now and then, refined on a copy
 * of the sample, and what it finds goes to the watch only: no blob is labelled, no pose is pushed. A camera the watch
 * leaves out looks every sample, its own solves are not used anyway.
 */
void
Camera::probeForRigWatch(CameraSample &sample, Device *device)
{
	ConstellationTracker *tracker = this->tracker;
	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	if (!mosaic || !sample.Txr_world_cam.has_value()) {
		return;
	}

	const bool left_out = tracker->rigWatchLeavesOut(mosaic->index, this->index);
	// Searches from nothing now and then; from a recent find of its own every sample, they're quick: the watch has
	// its 40 pairs in a second or two rather than the 12 s it took at 5 a second in a replay of a turned camera.
	const bool found_recently = this->rig_watch_probe_found.has_value() &&
	                            sample.timestamp_ns - this->rig_watch_probe_found_ns < CT_RIG_WATCH_PROBE_FOUND_NS;
	if (!left_out) {
		const size_t slot = device->id % XRT_CONSTELLATION_MAX_DEVICES;
		if (sample.timestamp_ns - this->rig_watch_seen_ns[slot].load() < CT_RIG_WATCH_PROBE_BLIND_NS ||
		    (!found_recently && sample.timestamp_ns - this->rig_watch_probe_ns < CT_RIG_WATCH_PROBE_PERIOD_NS)) {
			return;
		}
		/*
		 * Another camera tracks it: this one looks where it should be. No camera does, for a while (or ever since
		 * the start): every camera's solves may be rejected because the room was placed tilted (sensors tilted alike
		 * while the driver was off), which only these looks can show (to the room's level and the watch), so every
		 * camera looks, from nothing or its own last find.
		 */
		bool room_blind = false;
		{
			std::lock_guard<os::Mutex> lock(device->data_lock);
			const auto &last = device->locked_data.last_known_pose;
			if (!last.has_value() || sample.timestamp_ns - last->timestamp_ns > CT_RIG_WATCH_PROBE_ROOM_BLIND_NS) {
				room_blind = true;
			} else if (sample.timestamp_ns - last->timestamp_ns > CT_RIG_WATCH_PROBE_TRACKED_NS) {
				return;
			}
		}

		// In view of this camera, where it was placed: in front of it and inside its field of view.
		if (!room_blind) {
			DeviceState *state = sample.getDeviceState(device->id).value_or(nullptr);
			if (state == nullptr || !state->Txr_world_device_prior.has_value()) {
				return;
			}
			xrt_pose Txr_cam_world;
			math_pose_invert(&sample.Txr_world_cam.value(), &Txr_cam_world);
			xrt_vec3 in_cam;
			math_pose_transform_point(&Txr_cam_world, &state->Txr_world_device_prior->position, &in_cam);
			const float ahead = -in_cam.z; // OpenXR cameras look down -Z
			if (ahead < 0.3f || fabsf(in_cam.x) > 1.2f * ahead || fabsf(in_cam.y) > 1.0f * ahead) {
				return;
			}
		}
	}
	this->rig_watch_probe_ns = sample.timestamp_ns;

	/*
	 * Its last find, when recent, is where the next look starts (not binding): a camera left out is tracked in its own
	 * view, so placing it again needs seconds rather than the many a search from nothing takes (20 s in a replay of a
	 * camera turned 5 degrees).
	 */
	xrt_pose Tcv_cam_device = XRT_POSE_IDENTITY;
	xrt_vec3 no_gravity = {0.0f, 1.0f, 0.0f};
	pose_metrics score;
	correspondence_search_flags flags =
	    (correspondence_search_flags)(CS_FLAG_SHALLOW_SEARCH | CS_FLAG_STOP_FOR_STRONG_MATCH);
	if (found_recently) {
		Tcv_cam_device = this->rig_watch_probe_found.value();
		flags = (correspondence_search_flags)(flags | CS_FLAG_HAVE_POSE_PRIOR);
	}
	if (!correspondence_search_find_one_pose(this->slow_processing_thread_data.cs, device->search_model, flags,
	                                         &Tcv_cam_device, &device->prior_pos_error, &device->prior_rot_error,
	                                         &no_gravity, &no_gravity, 0.0f, &score)) {
		return;
	}

	// Refined as pushPose does, on a copy, so the sample's own labels stay as they are.
	CameraSample probe = sample;
	pose_metrics_blob_match_info match_info;
	pose_metrics_match_pose_to_blobs(&Tcv_cam_device, probe.blobs, probe.blob_count, &device->params.led_model,
	                                 device->id, &this->model, &match_info);
	probe.markMatchingBlobs(tracker, device->params.led_model, device->id, match_info);
	RawPoseCovarianceMatrix covariance;
	xrt_pose refined;
	if (optimizePose(tracker->log_level, tracker->deterministic, this->model, Tcv_cam_device, probe.blobs,
	                 probe.blob_count, &device->params.led_model, device->id, refined, covariance)) {
		Tcv_cam_device = refined;
	}
	pose_metrics_evaluate_pose(&score, &Tcv_cam_device, probe.blobs, probe.blob_count, &device->params.led_model,
	                           device->id, &this->model, NULL);
	float rms_px = score.matched_blobs > 0 ? sqrtf(score.reprojection_error / score.matched_blobs) : 0.0f;
	uint32_t matched = score.matched_blobs;
	// As a solve's: the watch learns from the rigid LEDs alone.
	xrt_pose Tcv_cam_rigid = Tcv_cam_device;
	if (device->partBlobs(probe.blobs, probe.blob_count) > 0 &&
	    !device->rigidPose(*tracker, this->model, probe.blobs, probe.blob_count, Tcv_cam_device, Tcv_cam_rigid,
	                       matched, rms_px)) {
		return;
	}
	if (matched >= CT_RIG_WATCH_MIN_LEDS && rms_px <= CT_RIG_WATCH_MAX_RMS_PX) {
		this->rig_watch_probe_found = Tcv_cam_rigid;
		this->rig_watch_probe_found_ns = sample.timestamp_ns;
	}
	this->feedRigWatch(device, sample, Tcv_cam_rigid, matched, rms_px);
}

bool
Camera::pushPose(CameraSample &camera_sample,
                 DeviceState &device_state,
                 Device *device,
                 pose_metrics &score,
                 const xrt_pose &Tcv_cam_device_initial,
                 std::optional<OldOptimizationData *> was_optimized,
                 SolveSource source)
{
	// We should never find two poses for the same device in a single frame
	assert(device_state.found_pose.has_value() == false);

	ConstellationTracker *tracker = this->tracker;

	uint32_t blobs_marked_before_update = num_blobs_for_device(camera_sample, device->id);

	// Match visible blobs to the pose we found
	pose_metrics_blob_match_info blob_match_info;
	pose_metrics_match_pose_to_blobs(&Tcv_cam_device_initial, camera_sample.blobs, camera_sample.blob_count,
	                                 &device->params.led_model, device->id, &this->model, &blob_match_info);

	// Mark all the new blobs using the match info
	camera_sample.markMatchingBlobs(tracker, device->params.led_model, device->id, blob_match_info);

	RawPoseCovarianceMatrix covariance;
	// If the pose was already optimized once, pull the covariance matrix from that
	if (was_optimized) {
		std::memcpy(covariance, (*was_optimized)->covariance, sizeof(RawPoseCovarianceMatrix));
	}

	xrt_pose Tcv_cam_device = Tcv_cam_device_initial;
	bool gravity_offered = false;

	// Only do an optimization if we haven't already optimized, or we marked new blobs.
	// This prevents us from optimizing a pose multiple times in a single frame.
	if (!was_optimized || num_blobs_for_device(camera_sample, device->id) > blobs_marked_before_update) {
		// Try to optimize the pose again, unmarking outliers
		xrt_pose Tcv_cam_device_optimized;
		/*
		 * Held to the IMU's tilt only from a prior: the slow search tries many wrong LED sets, and its gravity
		 * check is what turns them away, which a refinement held upright would get past (night jumps vs
		 * Oculus).
		 */
		xrt_vec3 camera_gravity;
		xrt_vec3 device_gravity;
		const bool have_gravity = !is_slow_search(source) &&
		                          debug_get_bool_option_constellation_tracker_gravity_prior_fast() &&
		                          this->gravityForSample(device, camera_sample, camera_gravity, device_gravity);
		gravity_offered = have_gravity;
		bool success = optimizePose(                   //
		    tracker->log_level,                        //
		    tracker->deterministic,                    //
		    this->model,                               //
		    Tcv_cam_device,                            //
		    camera_sample.blobs,                       //
		    camera_sample.blob_count,                  //
		    &device->params.led_model,                 //
		    device->id,                                //
		    Tcv_cam_device_optimized,                  //
		    covariance,                                //
		    have_gravity ? &camera_gravity : nullptr,  //
		    have_gravity ? &device_gravity : nullptr); //

		if (!success) {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs failed",
			         0, 0, device->id, camera_sample.blob_count);

			if (!was_optimized) {
				// If we didn't do a pose optimization, and this one failed, just compute the covariance
				// directly.
				computePoseCovariance(         //
				    tracker->log_level,        //
				    this->model,               //
				    Tcv_cam_device,            //
				    camera_sample.blobs,       //
				    camera_sample.blob_count,  //
				    &device->params.led_model, //
				    device->id,                //
				    covariance);               //
			}
		} else {
			CT_DEBUG(tracker,
			         "Camera %d (group %d) RANSAC-PnP refinement for device %d from %u "
			         "blobs. Produced pose %f,%f,%f,%f pos %f,%f,%f",
			         0, 0, device->id, camera_sample.blob_count, Tcv_cam_device.orientation.x,
			         Tcv_cam_device.orientation.y, Tcv_cam_device.orientation.z,
			         Tcv_cam_device.orientation.w, Tcv_cam_device.position.x, Tcv_cam_device.position.y,
			         Tcv_cam_device.position.z);

			Tcv_cam_device = Tcv_cam_device_optimized;

			// Now that we've optimized the pose, we gotta update the match info
			pose_metrics_match_pose_to_blobs(&Tcv_cam_device, camera_sample.blobs, camera_sample.blob_count,
			                                 &device->params.led_model, device->id, &this->model,
			                                 &blob_match_info);

			// Unmark any other blobs
			camera_sample.markMatchingBlobs(tracker, device->params.led_model, device->id, blob_match_info);
		}

		// If after RANSAC and optimization no blobs remain, skip, so that we aren't putting an invalid pose
		// into the fusion/latest field.
		if (num_blobs_for_device(camera_sample, device->id) == 0) {
			return false;
		}

		// We need to re-evaluate the pose after optimization, since the reprojection error may have changed.
		pose_metrics_evaluate_pose(    //
		    &score,                    //
		    &Tcv_cam_device,           //
		    camera_sample.blobs,       //
		    camera_sample.blob_count,  //
		    &device->params.led_model, //
		    device->id,                //
		    &this->model,              //
		    NULL);                     //
	}

	const bool slow_search = is_slow_search(source);
	const float rms_px = score.matched_blobs > 0 ? sqrtf(score.reprojection_error / score.matched_blobs) : 0.0f;
	const float max_rms_px = debug_get_float_option_constellation_tracker_slow_max_rms_px();
	// Held to the IMU's tilt, wrong LEDs fit worse instead of tilting, see pose_optimize.cpp.
	const float gravity_max_rms_px = debug_get_float_option_constellation_tracker_gravity_prior_max_rms_px();
	const bool gravity_held =
	    gravity_offered &&
	    score.matched_blobs <= (uint32_t)debug_get_num_option_constellation_tracker_gravity_prior_max_leds();
	const bool fits =
	    slow_search ? ((max_rms_px <= 0.0f || rms_px <= max_rms_px) &&
	                   score.matched_blobs >= debug_get_num_option_constellation_tracker_slow_min_leds())
	                : score.matched_blobs >= (source == SolveSource::FastFewLeds
	                                              ? 2
	                                              : debug_get_num_option_constellation_tracker_fast_min_leds()) &&
	                      (!gravity_held || gravity_max_rms_px <= 0.0f || rms_px <= gravity_max_rms_px);
	if (!fits) {
		device->rms_rejections_total.fetch_add(1);
	}

	/*
	 * What the cameras' placement and the room learn from a solve comes from the rigid LEDs alone: a loose part's
	 * LEDs sit where the part was last measured, and a solve that used them bends by as much as that is off. A
	 * measurement from another day had a camera placed again 6 cm from where it stood.
	 */
	xrt_pose Tcv_cam_rigid = Tcv_cam_device;
	uint32_t rigid_matched = score.matched_blobs;
	float rigid_rms_px = rms_px;
	const bool used_part = fits && device->partBlobs(camera_sample.blobs, camera_sample.blob_count) > 0;
	const bool rigid_ok = !used_part || device->rigidPose(*tracker, this->model, camera_sample.blobs,
	                                                      camera_sample.blob_count, Tcv_cam_device, Tcv_cam_rigid,
	                                                      rigid_matched, rigid_rms_px);

	/*
	 * The moved-camera watch sees every solve that fits, before the gravity and prior checks: a moved camera's solves
	 * are the ones they turn away. It only reads (the IMU's down, the fusion's pose) until a camera is found moved.
	 */
	bool left_out = false;
	if (fits && debug_get_bool_option_constellation_tracker_rig_watch()) {
		if (rigid_ok) {
			this->feedRigWatch(device, camera_sample, Tcv_cam_rigid, rigid_matched, rigid_rms_px);
		}
		std::shared_ptr<CameraMosaic> watch_mosaic = this->mosaic.lock();
		left_out = watch_mosaic && tracker->rigWatchLeavesOut(watch_mosaic->index, this->index);
	}

	// The refinement may have slipped to other LEDs, check the pose as it now is.
	uint32_t rejected = 0;
	if (!fits) {
		rejected = CT_SOLVE_REJECTED_FIT;
	} else if (left_out) {
		rejected = CT_SOLVE_REJECTED_RIG;
	} else if (!this->poseMatchesGravity(device, camera_sample, Tcv_cam_device, source)) {
		rejected = CT_SOLVE_REJECTED_GRAVITY;
	} else if (!this->poseMatchesPrior(device, camera_sample, device_state, Tcv_cam_device, source)) {
		rejected = CT_SOLVE_REJECTED_PRIOR;
	}
	if (rejected != 0) {
		// Recorded too, flagged, so that what each check turns away can be scored against ground truth.
		std::shared_ptr<CameraMosaic> rejected_mosaic = this->mosaic.lock();
		if (tracker->data_recorder && camera_sample.Txr_world_cam.has_value() && rejected_mosaic) {
			xrt_pose Txr_cam_rejected;
			math_pose_convert_from_opencv(&Tcv_cam_device, &Txr_cam_rejected);
			xrt_pose Txr_world_rejected;
			math_pose_transform(&camera_sample.Txr_world_cam.value(), &Txr_cam_rejected,
			                    &Txr_world_rejected);
			DatasetSolve solve = {
			    .device_id = device->id,
			    .sample_id = camera_sample.id,
			    .timestamp_ns = camera_sample.timestamp_ns,
			    .mosaic_index = static_cast<uint32_t>(rejected_mosaic->index),
			    .camera_index = static_cast<uint32_t>(this->index),
			    .source = source,
			    .match_flags = static_cast<uint32_t>(score.match_flags) | rejected,
			    .Tcv_cam_device = Tcv_cam_device,
			    .Txr_world_device = Txr_world_rejected,
			    .matched_blobs = score.matched_blobs,
			    .unmatched_blobs = score.unmatched_blobs,
			    .visible_leds = score.visible_leds,
			    .reprojection_error_px = rms_px,
			    .led_ids = {},
			};
			tracker->data_recorder->recordSolve(solve);
		}

		for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
			t_blob &b = camera_sample.blobs[i];
			if (b.matched_device_id == device->id) {
				b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			}
		}
		return false;
	}

	// For the moved-camera watch's probe: this camera still finds this device.
	this->rig_watch_seen_ns[device->id % XRT_CONSTELLATION_MAX_DEVICES].store(camera_sample.timestamp_ns);

	/*
	 * For a loose part: where the device is, from its rigid LEDs. Never from a solve that used the part's: one bent
	 * to fit them where they were measured always finds them there, and a part that moved would pass every check.
	 */
	if (device->loose_part && rigid_ok && rigid_rms_px <= CT_RIG_WATCH_MAX_RMS_PX &&
	    (used_part ? rigid_matched : device->rigidBlobs(camera_sample)) >= CT_RIG_WATCH_MIN_LEDS) {
		if (std::shared_ptr<CameraMosaic> part_mosaic = this->mosaic.lock()) {
			device->loosePartSolve(static_cast<uint32_t>(part_mosaic->index * XRT_TRACKING_MAX_CAMS + this->index),
			                       camera_sample, this->model, Tcv_cam_rigid);
		}
	}

	// Move to OpenXR space
	xrt_pose Txr_cam_device;
	math_pose_convert_from_opencv(&Tcv_cam_device, &Txr_cam_device);

	CT_DEBUG(tracker, "Pose: orient %f %f %f %f pos %f %f %f", Txr_cam_device.orientation.x,
	         Txr_cam_device.orientation.y, Txr_cam_device.orientation.z, Txr_cam_device.orientation.w,
	         Txr_cam_device.position.x, Txr_cam_device.position.y, Txr_cam_device.position.z);

	std::shared_ptr<CameraMosaic> mosaic = this->mosaic.lock();
	U_ASSERT_WEAK_PTR_RET(mosaic,
	                      "Camera mosaic was destroyed while processing a sample, this should never happen since "
	                      "the mosaic owns the camera",
	                      false);

	float average_brightness = 0.0f;
	uint32_t used_blobs = 0;
	for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
		struct t_blob &b = camera_sample.blobs[i];
		if (b.matched_device_id == device->id) {
			average_brightness += b.brightness;
			used_blobs++;
		}
	}
	if (used_blobs > 0) {
		average_brightness /= used_blobs;
	}

	// Mark that we found a pose
	device_state.found_pose = {
	    .Tcv_cam_device = Tcv_cam_device,
	    .average_blob_brightness = average_brightness,
	    .covariance = {},
	};
	std::memcpy(device_state.found_pose->covariance, covariance, sizeof(RawPoseCovarianceMatrix));

	auto Txr_world_cam = camera_sample.Txr_world_cam;

	std::optional<xrt_pose> Txr_world_device = Txr_world_cam;
	if (Txr_world_cam.has_value()) {
		math_pose_transform(&Txr_world_cam.value(), &Txr_cam_device, &Txr_world_device.value());
	}

	// Compute the metrics
	t_constellation_tracker_sample_metrics metrics = {
	    .matched_blob_count = score.matched_blobs,
	    .visible_led_count = score.visible_leds,
	    .reprojection_error = score.matched_blobs > 0 ? sqrtf(score.reprojection_error / score.matched_blobs) : 0.0,
	};

	// Push the sample to the device
	t_constellation_tracker_sample sample = {
	    .timestamp_ns = camera_sample.timestamp_ns,
	    .sequence_id = camera_sample.sequence_id,
	    .has_world_pose = Txr_world_device.has_value(),
	    .world_pose = XRT_POSE_IDENTITY,
	    .mosaic_index = mosaic->index,
	    .camera_index = this->index,
	    .average_brightness = average_brightness,
	    .metrics = metrics,
	    .leds = {},
	};
	if (Txr_world_device.has_value()) {
		sample.world_pose = Txr_world_device.value();
	}

	for (int i = 0; i < blob_match_info.num_visible_leds; i++) {
		const auto &visible_led = blob_match_info.visible_leds[i];

		// Skip visible LEDs that have no matched blob
		if (visible_led.matched_blob == nullptr) {
			continue;
		}

		// Assert the IDs are valid indices into the array.
		assert(visible_led.led->id >= 0);
		assert((size_t)visible_led.led->id < ARRAY_SIZE(sample.leds));

		// Mark the LED as observed
		sample.leds[visible_led.led->id] = {
		    .observed = true,
		    .brightness = visible_led.matched_blob->brightness,
		    .facing_dot = static_cast<float>(visible_led.facing_dot),
		};
	}

	if (tracker->data_recorder) {
		DatasetSolve solve = {
		    .device_id = device->id,
		    .sample_id = camera_sample.id,
		    .timestamp_ns = camera_sample.timestamp_ns,
		    .mosaic_index = static_cast<uint32_t>(mosaic->index),
		    .camera_index = static_cast<uint32_t>(this->index),
		    .source = source,
		    .match_flags = static_cast<uint32_t>(score.match_flags),
		    .Tcv_cam_device = Tcv_cam_device,
		    .Txr_world_device = Txr_world_device,
		    .matched_blobs = score.matched_blobs,
		    .unmatched_blobs = score.unmatched_blobs,
		    .visible_leds = score.visible_leds,
		    .reprojection_error_px = metrics.reprojection_error,
		    .led_ids = {},
		};
		for (size_t i = 0; i < ARRAY_SIZE(sample.leds); i++) {
			if (sample.leds[i].observed) {
				solve.led_ids.push_back(static_cast<uint8_t>(i));
			}
		}
		tracker->data_recorder->recordSolve(solve);
	}

	// Push the sample now that the data is filled out.
	t_constellation_tracker_device_push_sample(device->device, &sample);

	// The old moved-camera check, when the watch is off (see watchRig).
	if (Txr_world_device.has_value() && !debug_get_bool_option_constellation_tracker_rig_watch()) {
		tracker->checkRigConsistency(device->id, mosaic->index, this->index, camera_sample.timestamp_ns,
		                             metrics.matched_blob_count, Txr_world_device->position);
	}

	if (Txr_world_device.has_value()) {
		std::unique_lock<os::Mutex> lock(device->data_lock);

		// If we already found a pose in the future, then don't mark blobs, since the device has definitely
		// moved.
		if (!device->locked_data.last_known_pose.has_value() ||
		    device->locked_data.last_known_pose->timestamp_ns <= camera_sample.timestamp_ns) {
			// Call back to the blobwatch to update the blobs for this device. Done after pose optimization
			// since the RANSAC process will unlabel any outliers.
			auto tbo = camera_sample.toBlobObservation();
			t_blobwatch_mark_blob_device(camera_sample.source, &tbo, device->id);
		}

		device->prior_disagree_since_ns.store(0);
		device->prior_disagree_count.store(0);
		if (source != SolveSource::FastFewLeds &&
		    camera_sample.timestamp_ns > device->last_full_solve_ns.load()) {
			device->last_full_solve_ns.store(camera_sample.timestamp_ns);
		}

		device->locked_data.last_known_pose =
		    DeviceLastPose(Txr_world_device.value(), Tcv_cam_device, camera_sample.timestamp_ns);
	}

	if (Txr_world_device.has_value()) {
		this->checkFusionAgrees(device, camera_sample.timestamp_ns, Txr_world_device.value());
	}

	CT_DEBUG(tracker, "Found pose for device %d", device->id);
	return true;
}

/*
 *
 * CameraMosaic implementations
 *
 */

CameraMosaic::CameraMosaic(ConstellationTracker *tracker,
                           const t_constellation_tracker_camera_mosaic &mosaic_params,
                           size_t index)
    : index(index)
{
	this->tracking_origin = mosaic_params.tracking_origin;

	// NOTE: the stability of this vector is important since we're passing to C callbacks and APIs!
	this->cameras.reserve(mosaic_params.num_cameras);
}

std::optional<xrt_pose>
CameraMosaic::getTrackingOriginPose(timepoint_ns when_ns)
{
	if (this->tracking_origin) {
		xrt_space_relation relation;
		t_constellation_tracker_tracking_source_get_tracked_pose(this->tracking_origin, when_ns, &relation);

		// If the tracking source has a valid position, grab it
		if ((relation.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
		    (relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
			return relation.pose;
		} else {
			return std::nullopt;
		}
	}

	return std::optional<xrt_pose>(XRT_POSE_IDENTITY);
}

/*
 *
 * DeviceLastPose implementations
 *
 */

DeviceLastPose::DeviceLastPose(xrt_pose Txr_world_device, xrt_pose Tcv_cam_device, timepoint_ns timestamp_ns)
    : Txr_world_device(Txr_world_device), Tcv_cam_device(Tcv_cam_device), timestamp_ns(timestamp_ns)
{}

/*
 *
 * Device implementations
 *
 */

Device::Device(ConstellationTracker *tracker,
               t_constellation_tracker_device_params *params,
               t_constellation_tracker_device *device,
               t_constellation_device_id_t id)
    : params(*params), device(device), id(id), tracker(tracker), data_lock(),
      locked_data({.last_known_pose = std::nullopt})
{
	this->imu_sink = {
	    .push_imu =
	        [](xrt_imu_sink *ptr, xrt_imu_sample *sample) {
		        auto self = Device::fromXrtImuSink(ptr);
		        self->pushImuSample(*sample);
	        },
	};
	params->imu_sink = &this->imu_sink;

	// flip all LEDs from OpenXR -> OpenCV coordinate space, since the tracker works in OpenCV space
	auto to_cv = [](const t_constellation_tracker_led &src) {
		t_constellation_tracker_led dst = src;
		math_vec3_convert_from_opencv(&src.position, &dst.position);
		math_vec3_convert_from_opencv(&src.normal, &dst.normal);
		return dst;
	};

	// The driver's LEDs are not kept: the model is ours, in safe memory.
	this->params.loose_part = {};
	const bool with_part = params->loose_part.led_count > 0 && params->loose_part.leds != nullptr &&
	                       debug_get_bool_option_constellation_tracker_loose_part();
	if (!with_part) {
		this->params.led_model.leds = new t_constellation_tracker_led[this->params.led_model.led_count];
		for (size_t i = 0; i < this->params.led_model.led_count; i++) {
			this->params.led_model.leds[i] = to_cv(params->led_model.leds[i]);
		}
		this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
		return;
	}

	auto lp = std::make_unique<LoosePartState>();
	for (size_t i = 0; i < params->led_model.led_count; i++) {
		lp->rigid_leds.push_back(to_cv(params->led_model.leds[i]));
	}
	for (size_t i = 0; i < params->loose_part.led_count; i++) {
		const t_constellation_tracker_led &led = params->loose_part.leds[i];
		lp->part_leds.push_back(to_cv(led));
		if (led.id >= 0 && (size_t)led.id < lp->is_part_led.size()) {
			lp->is_part_led[(size_t)led.id] = true;
		}
	}
	lp->serial = std::string(params->serial, strnlen(params->serial, sizeof(params->serial)));
	this->loose_part = std::move(lp);
	this->params.led_model.leds = nullptr;
	this->params.led_model.led_count = 0;

	/*
	 * Measured before on this device: in the model once the first frames show its LEDs there. The strap moves when
	 * the headset is put on, and a part in the model where it no longer sits bends the solves that use it.
	 */
	std::optional<xrt_pose> saved;
	if (!this->loose_part->serial.empty() && debug_get_bool_option_constellation_tracker_loose_part_saved()) {
		saved = tracker->loadLoosePart(this->loose_part->serial);
	}
	this->buildLedModel(std::nullopt);

	LoosePartState *state = this->loose_part.get();
	state->estimator = std::make_unique<LoosePartEstimator>(
	    state->part_leds, tracker->deterministic,
	    [state, tracker](const std::optional<LoosePartFit> &fit, const char *why) {
		    {
			    std::unique_lock<std::mutex> l(state->result_lock);
			    state->result_pending = true;
			    state->result = fit;
			    state->result_why = why;
		    }
		    tracker->loose_part_pending.store(true);
	    });
	if (saved.has_value()) {
		state->estimator->setSaved(saved.value());
	}
	CT_INFO(tracker, "Device %d has %zu LEDs on an adjustable part (%s): %s.", id, state->part_leds.size(),
	        state->serial.empty() ? "no serial" : state->serial.c_str(),
	        saved.has_value() ? "measured before, used once it is seen there again" : "measured once it is seen");
}

void
Device::buildLedModel(const std::optional<xrt_pose> &part_offset)
{
	const LoosePartState &lp = *this->loose_part;
	std::vector<t_constellation_tracker_led> leds(lp.rigid_leds);
	if (part_offset.has_value()) {
		const xrt_pose &off = part_offset.value();
		for (t_constellation_tracker_led led : lp.part_leds) {
			xrt_vec3 p, n;
			math_quat_rotate_vec3(&off.orientation, &led.position, &p);
			math_quat_rotate_vec3(&off.orientation, &led.normal, &n);
			led.position = m_vec3_add(p, off.position);
			led.normal = n;
			leds.push_back(led);
		}
	}

	auto *array = new t_constellation_tracker_led[leds.size()];
	std::copy(leds.begin(), leds.end(), array);
	delete[] this->params.led_model.leds;
	this->params.led_model.leds = array;
	this->params.led_model.led_count = leds.size();

	// The sensor fusion and the rig calibration read the model through params.led_model, which stays put.
	if (this->search_model != nullptr) {
		t_constellation_search_model_free(this->search_model);
	}
	this->search_model = t_constellation_search_model_new(this->id, &this->params.led_model);
	this->loose_part->in_model = part_offset;
}

uint32_t
Device::rigidBlobs(const CameraSample &sample) const
{
	uint32_t n = 0;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		const t_blob &b = sample.blobs[i];
		if (b.matched_device_id != this->id) {
			continue;
		}
		if (this->isPartLed(b.matched_device_led_id)) {
			continue;
		}
		n++;
	}
	return n;
}

bool
Device::isPartLed(t_constellation_led_id_it led_id) const
{
	return this->loose_part && led_id >= 0 && (size_t)led_id < this->loose_part->is_part_led.size() &&
	       this->loose_part->is_part_led[(size_t)led_id];
}

uint32_t
Device::partBlobs(const t_blob *blobs, uint32_t blob_count) const
{
	uint32_t n = 0;
	for (uint32_t i = 0; i < blob_count; i++) {
		if (blobs[i].matched_device_id == this->id && this->isPartLed(blobs[i].matched_device_led_id)) {
			n++;
		}
	}
	return n;
}

bool
Device::rigidPose(const ConstellationTracker &tracker,
                  const camera_model &model,
                  const t_blob *blobs,
                  uint32_t blob_count,
                  const xrt_pose &Tcv_cam_device,
                  xrt_pose &out_pose,
                  uint32_t &out_matched,
                  float &out_rms_px)
{
	std::vector<t_blob> rigid(blobs, blobs + blob_count);
	for (t_blob &b : rigid) {
		if (b.matched_device_id == this->id && this->isPartLed(b.matched_device_led_id)) {
			b.matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
			b.matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
		}
	}
	RawPoseCovarianceMatrix covariance;
	if (!optimizePose(tracker.log_level, tracker.deterministic, model, Tcv_cam_device, rigid.data(),
	                  static_cast<uint32_t>(rigid.size()), &this->params.led_model, this->id, out_pose, covariance)) {
		return false;
	}

	// How well it fits the rigid LEDs it kept, the way a solve's own fit is told.
	double squared_px = 0.0;
	uint32_t matched = 0;
	for (const t_blob &b : rigid) {
		if (b.matched_device_id != this->id) {
			continue;
		}
		const t_constellation_tracker_led *led = nullptr;
		for (size_t i = 0; i < this->params.led_model.led_count; i++) {
			if (this->params.led_model.leds[i].id == b.matched_device_led_id) {
				led = &this->params.led_model.leds[i];
				break;
			}
		}
		if (led == nullptr) {
			continue;
		}
		xrt_vec3 p;
		math_quat_rotate_vec3(&out_pose.orientation, &led->position, &p);
		p = m_vec3_add(p, out_pose.position);
		float x, y;
		if (p.z <= 0.0f || !t_camera_models_project(&model.calib_pinhole, p.x, p.y, p.z, &x, &y)) {
			continue;
		}
		const double dx = x - b.center_undistorted.x;
		const double dy = y - b.center_undistorted.y;
		squared_px += dx * dx + dy * dy;
		matched++;
	}
	if (matched == 0) {
		return false;
	}
	out_matched = matched;
	out_rms_px = sqrtf(static_cast<float>(squared_px / matched));
	return true;
}

namespace {

//! The blobs that may be the loose part's: unclaimed, or claimed by its LEDs.
std::vector<xrt_vec2>
loose_part_blobs(const Device &device, const CameraSample &sample)
{
	std::vector<xrt_vec2> out;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		const t_blob &b = sample.blobs[i];
		const bool unclaimed = b.matched_device_id == XRT_CONSTELLATION_INVALID_DEVICE_ID;
		const bool part = b.matched_device_id == device.id && device.isPartLed(b.matched_device_led_id);
		if (unclaimed || part) {
			out.push_back(b.center_undistorted);
		}
	}
	return out;
}

LoosePartFrame
loose_part_frame(const Device::LoosePartState::View &view, const xrt_pose &Txr_world_device)
{
	xrt_pose Txr_cam_world, Txr_cam_device;
	math_pose_invert(&view.Txr_world_cam, &Txr_cam_world);
	math_pose_transform(&Txr_cam_world, &Txr_world_device, &Txr_cam_device);
	LoosePartFrame f;
	math_pose_convert_from_opencv(&Txr_cam_device, &f.Tcv_cam_device);
	f.fx = view.fx;
	f.fy = view.fy;
	f.cx = view.cx;
	f.cy = view.cy;
	f.blobs = view.blobs;
	f.camera = view.camera;
	return f;
}

//! Views and solves are kept this long for their partner from the same exposure.
constexpr timepoint_ns kLoosePartPairNs = U_TIME_1S_IN_NS / 2;

} // namespace

void
Device::loosePartView(uint32_t camera_key, const CameraSample &sample, const camera_model &model)
{
	if (!this->loose_part || !sample.Txr_world_cam.has_value() || !this->worn.load()) {
		return;
	}
	LoosePartState &lp = *this->loose_part;
	LoosePartState::View view = {
	    .ts = sample.timestamp_ns,
	    .camera = camera_key,
	    .Txr_world_cam = sample.Txr_world_cam.value(),
	    .fx = model.calib_pinhole.fx,
	    .fy = model.calib_pinhole.fy,
	    .cx = model.calib_pinhole.cx,
	    .cy = model.calib_pinhole.cy,
	    .blobs = loose_part_blobs(*this, sample),
	};
	if (view.blobs.empty()) {
		return;
	}

	std::optional<LoosePartFrame> frame;
	{
		std::unique_lock<std::mutex> l(lp.pair_lock);
		for (const auto &s : lp.solves) {
			if (s.ts == view.ts && s.camera != view.camera) {
				frame = loose_part_frame(view, s.Txr_world_device);
				break;
			}
		}
		if (!frame.has_value()) {
			lp.views.push_back(std::move(view));
		}
		while (!lp.views.empty() && lp.views.front().ts + kLoosePartPairNs < sample.timestamp_ns) {
			lp.views.pop_front();
		}
		while (!lp.solves.empty() && lp.solves.front().ts + kLoosePartPairNs < sample.timestamp_ns) {
			lp.solves.pop_front();
		}
	}
	if (frame.has_value()) {
		lp.estimator->addFrame(std::move(frame.value()));
	}
}

void
Device::loosePartSolve(uint32_t camera_key,
                       const CameraSample &sample,
                       const camera_model &model,
                       const xrt_pose &Tcv_cam_device)
{
	if (!this->loose_part || !sample.Txr_world_cam.has_value() || !this->worn.load()) {
		return;
	}
	LoosePartState &lp = *this->loose_part;
	xrt_pose Txr_cam_device, Txr_world_device;
	math_pose_convert_from_opencv(&Tcv_cam_device, &Txr_cam_device);
	math_pose_transform(&sample.Txr_world_cam.value(), &Txr_cam_device, &Txr_world_device);

	std::vector<LoosePartFrame> frames;
	// The part seen from the side, together with the rigid LEDs that gave this pose.
	LoosePartFrame own;
	own.Tcv_cam_device = Tcv_cam_device;
	own.fx = model.calib_pinhole.fx;
	own.fy = model.calib_pinhole.fy;
	own.cx = model.calib_pinhole.cx;
	own.cy = model.calib_pinhole.cy;
	own.blobs = loose_part_blobs(*this, sample);
	own.camera = camera_key;
	own.own = true;
	if (!own.blobs.empty()) {
		frames.push_back(std::move(own));
	}
	{
		std::unique_lock<std::mutex> l(lp.pair_lock);
		lp.solves.push_back({sample.timestamp_ns, camera_key, Txr_world_device});
		for (auto it = lp.views.begin(); it != lp.views.end();) {
			if (it->ts == sample.timestamp_ns && it->camera != camera_key) {
				frames.push_back(loose_part_frame(*it, Txr_world_device));
				it = lp.views.erase(it);
			} else {
				++it;
			}
		}
	}
	for (auto &f : frames) {
		lp.estimator->addFrame(std::move(f));
	}
}

Device::~Device()
{
	// The part's estimator first: its worker hands results to this device.
	if (this->loose_part) {
		this->loose_part->estimator.reset();
	}

	// Clear the sensor fusion so it gets destructed.
	this->sensor_fusion.reset();

	if (this->search_model) {
		t_constellation_search_model_free(this->search_model);
		this->search_model = nullptr;
	}

	if (this->params.led_model.leds) {
		delete[] this->params.led_model.leds;
		this->params.led_model.leds = nullptr;
	}
}

void
Device::pushImuSample(const xrt_imu_sample &sample)
{
	if (this->tracker->data_recorder) {
		this->tracker->data_recorder->recordImuSample(this->id, sample);
	}

	this->tracker->rig_calibration->pushImuSample(this->id, sample);

	this->sensor_fusion->pushImuSample(sample);

	this->imu_tilt.push(sample);
}

std::optional<xrt_vec3>
Device::gravityAt(timepoint_ns when_ns) const
{
	std::optional<xrt_vec3> gravity = this->imu_tilt.gravityAt(when_ns);
	if (gravity.has_value()) {
		math_vec3_convert_from_opencv(&gravity.value(), &gravity.value());
	}
	return gravity;
}

/*
 *
 * ImuTilt implementations
 *
 */

//! Under this the accelerometer read nothing: a CV1 Touch going idle sends zeroes. Or the device is falling.
static constexpr float kTiltMinAccel = 2.0f;
//! How far from its resting reading the accelerometer may be and still say where gravity is.
static constexpr float kTiltAccelBand = 1.5f;
//! Seconds over which the accelerometer pulls the estimate around, at rest.
static constexpr float kTiltTimeConstantS = 1.0f;
//! Turning slower than this counts as resting, rad/s.
static constexpr float kTiltRestingGyro = 0.1f;
//! The gyro can't carry the estimate over a longer gap in the samples.
static constexpr timepoint_ns kTiltGapNs = 250 * U_TIME_1MS_IN_NS;
//! How long a fresh estimate settles before it is trusted.
static constexpr timepoint_ns kTiltSettleNs = 500 * U_TIME_1MS_IN_NS;
//! How long the device must have rested before its IMU stopped for the estimate to stay good.
static constexpr timepoint_ns kTiltIdleRestNs = 500 * U_TIME_1MS_IN_NS;

void
ImuTilt::resetLocked()
{
	this->have_up = false;
	this->history_count = 0;
	this->history_head = 0;
}

void
ImuTilt::recordLocked(timepoint_ns timestamp_ns)
{
	if (this->history_count > 0) {
		const Entry &newest = this->history[(this->history_head + kHistorySize - 1) % kHistorySize];
		if (timestamp_ns - newest.timestamp_ns < kHistoryStepNs) {
			return;
		}
	}

	this->history[this->history_head] = Entry{timestamp_ns, this->up};
	this->history_head = (this->history_head + 1) % kHistorySize;
	this->history_count = std::min(this->history_count + 1, kHistorySize);
}

void
ImuTilt::push(const xrt_imu_sample &sample)
{
	const xrt_vec3 accel = {static_cast<float>(sample.accel_m_s2.x), static_cast<float>(sample.accel_m_s2.y),
	                        static_cast<float>(sample.accel_m_s2.z)};
	const xrt_vec3 gyro = {static_cast<float>(sample.gyro_rad_secs.x), static_cast<float>(sample.gyro_rad_secs.y),
	                       static_cast<float>(sample.gyro_rad_secs.z)};
	const float accel_len = m_vec3_len(accel);
	const float gyro_len = m_vec3_len(gyro);

	if (accel_len < kTiltMinAccel) {
		return;
	}

	std::unique_lock<os::Mutex> lock(this->lock);

	if (this->have_up) {
		// Out of order, which the clock tracking of a Touch still does now and then.
		if (sample.timestamp_ns <= this->last_ns) {
			return;
		}

		// A gap the device was moving into leaves nothing to go on, start over.
		if (sample.timestamp_ns - this->last_ns > kTiltGapNs &&
		    this->last_ns - this->last_moving_ns < kTiltIdleRestNs) {
			this->resetLocked();
		}
	}

	if (!this->have_up) {
		// Start from the accelerometer while it reads gravity alone, near enough.
		if (gyro_len < 5.0f * kTiltRestingGyro && fabsf(accel_len - this->rest_accel_len) < kTiltAccelBand) {
			this->up = m_vec3_mul_scalar(accel, 1.0f / accel_len);
			this->have_up = true;
			this->start_ns = sample.timestamp_ns;
			this->last_ns = sample.timestamp_ns;
			this->last_moving_ns = sample.timestamp_ns;
			this->recordLocked(sample.timestamp_ns);
		}
		return;
	}

	const timepoint_ns dt_ns = sample.timestamp_ns - this->last_ns;
	this->last_ns = sample.timestamp_ns;

	// After a gap at rest (an idle Touch) the device has not turned, so carry nothing over it.
	const float dt_s = dt_ns > kTiltGapNs ? 0.0f : static_cast<float>(time_ns_to_s(dt_ns));

	// A direction fixed in the world turns the other way in the device's frame.
	if (gyro_len > 1e-6f && dt_s > 0.0f) {
		xrt_vec3 axis = m_vec3_mul_scalar(gyro, 1.0f / gyro_len);
		xrt_quat delta;
		math_quat_from_angle_vector(-gyro_len * dt_s, &axis, &delta);
		math_quat_rotate_vec3(&delta, &this->up, &this->up);
	}

	float accel_off = fabsf(accel_len - this->rest_accel_len);
	if (gyro_len > kTiltRestingGyro) {
		this->last_moving_ns = sample.timestamp_ns;
	} else if (accel_off < kTiltAccelBand) {
		this->rest_accel_len += 0.01f * (accel_len - this->rest_accel_len);
		accel_off = fabsf(accel_len - this->rest_accel_len);
	}

	// Pull towards the accelerometer, less the more it reads something else than gravity or the faster it turns.
	if (accel_off < kTiltAccelBand) {
		const float weight = (1.0f - accel_off / kTiltAccelBand) / (1.0f + gyro_len);
		const float gain = std::min(1.0f, weight * dt_s / kTiltTimeConstantS);
		const xrt_vec3 measured = m_vec3_mul_scalar(accel, 1.0f / accel_len);
		this->up = m_vec3_normalize(this->up + (measured - this->up) * gain);
	}

	this->recordLocked(sample.timestamp_ns);
}

std::optional<xrt_vec3>
ImuTilt::gravityAt(timepoint_ns when_ns) const
{
	std::unique_lock<os::Mutex> lock(this->lock);

	if (!this->have_up || when_ns - this->start_ns < kTiltSettleNs) {
		return std::nullopt;
	}

	// Without recent samples it is only good if the device was resting when they stopped, see the struct.
	if (when_ns - this->last_ns > kTiltGapNs && this->last_ns - this->last_moving_ns < kTiltIdleRestNs) {
		return std::nullopt;
	}

	xrt_vec3 up_at = this->up;
	if (when_ns < this->last_ns) {
		// The newest entry at or before the asked time, the IMU usually runs ahead of the cameras.
		bool found = false;
		for (size_t i = 1; i <= this->history_count; i++) {
			const Entry &entry = this->history[(this->history_head + kHistorySize - i) % kHistorySize];
			if (entry.timestamp_ns <= when_ns) {
				up_at = entry.up;
				found = true;
				break;
			}
		}

		// Older than the history reaches.
		if (!found) {
			return std::nullopt;
		}
	}

	return m_vec3_mul_scalar(up_at, -1.0f);
}

/*
 *
 * ConstellationTracker implementations
 *
 */

ConstellationTracker::ConstellationTracker(t_constellation_tracker_params *params)
    : log_level(debug_get_log_option_constellation_tracker_log()), params(*params), rig_calibration_lock()
{
	// If the deterministic flag is set, we enable single-threaded processing, and mark it as deterministic.
	if ((params->flags & T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC) != 0) {
		this->single_threaded = true;
		this->deterministic = true;
	}

	this->gravity_tolerance_rad = static_cast<float>(
	    DEG_TO_RAD(MAX(0.0f, debug_get_float_option_constellation_tracker_gravity_tolerance_deg())));

	const float prior_rot_rad =
	    static_cast<float>(DEG_TO_RAD(debug_get_float_option_constellation_tracker_prior_rot_tolerance_deg()));
	const float prior_pos_m = debug_get_float_option_constellation_tracker_prior_pos_tolerance_m();
	this->prior_tolerance_rot = {prior_rot_rad, prior_rot_rad, prior_rot_rad};
	this->prior_tolerance_pos = {prior_pos_m, prior_pos_m, prior_pos_m};
	if (prior_rot_rad > 0.0f && prior_pos_m > 0.0f) {
		this->prior_fresh_ns = debug_get_num_option_constellation_tracker_prior_fresh_ms() * U_TIME_1MS_IN_NS;
	}

	this->prior_distrust_ns = debug_get_num_option_constellation_tracker_prior_distrust_ms() * U_TIME_1MS_IN_NS;

	const float held_pos_m = debug_get_float_option_constellation_tracker_held_prior_pos_tolerance_m();
	this->held_prior_tolerance_pos = {held_pos_m, held_pos_m, held_pos_m};
	if (prior_rot_rad > 0.0f && held_pos_m > 0.0f) {
		this->held_prior_ns = debug_get_num_option_constellation_tracker_held_prior_ms() * U_TIME_1MS_IN_NS;
	}

	this->mosaics.reserve(params->num_mosaics);

	// Fill in our internal data structures based on the provided params
	for (size_t mosaic_idx = 0; mosaic_idx < params->num_mosaics; mosaic_idx++) {
		const t_constellation_tracker_camera_mosaic &mosaic_params = params->mosaics[mosaic_idx];

		std::shared_ptr<CameraMosaic> mosaic = std::make_shared<CameraMosaic>(this, mosaic_params, mosaic_idx);

		// Assert pointer stability!
		assert(mosaic->cameras.capacity() >= mosaic_params.num_cameras);

		for (size_t cam_idx = 0; cam_idx < mosaic_params.num_cameras; cam_idx++) {
			const t_constellation_tracker_camera &camera_params = mosaic_params.cameras[cam_idx];

			// This can't be in the constructor since the `shared_ptr` of the `mosaic` isn't formed
			// yet, but the camera needs to own a weak_ptr to it's mosaic.
			mosaic->cameras.push_back(
			    std::make_unique<Camera>(this, mosaic, camera_params, &this->log_level, cam_idx));
		}

		this->mosaics.push_back(mosaic);
	}

	// Fill in the blob sinks for each camera
	for (size_t i = 0; i < this->mosaics.size(); i++) {
		std::shared_ptr<CameraMosaic> &mosaic = this->mosaics[i];

		for (size_t j = 0; j < mosaic->cameras.size(); j++) {
			Camera *camera = mosaic->cameras[j].get();

			params->mosaics[i].cameras[j].blob_sink = &camera->base;
		}
	}

	std::string data_recorder_output = debug_get_option_constellation_tracker_data_recorder_output();
	if (!data_recorder_output.empty()) {
		this->data_recorder = std::make_unique<DataRecorder>(this, data_recorder_output);
		CT_INFO(this, "Constellation tracker data recorder enabled, outputting to %s",
		        data_recorder_output.c_str());
	}

	if (debug_get_bool_option_constellation_tracker_enable_rerun()) {
#ifdef XRT_FEATURE_RERUN
		this->rerun_stream = std::make_unique<RerunContext>();
		CT_INFO(this, "Constellation tracker Rerun stream enabled, outputting to constellation_tracker.rerun");

		if (debug_get_bool_option_constellation_tracker_rerun_spawn()) {
			this->rerun_stream->spawnViewer();
		} else {
			this->rerun_stream->stream->connect_grpc().exit_on_failure();
		}
#else
		CT_ERROR(this, "Rerun stream requested but XRT_FEATURE_RERUN is not enabled");
#endif
	}

	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			CT_INFO(this, "Camera %zu/%zu: serial '%s'.", mosaic->index, camera->index,
			        camera->serial.c_str());
		}
	}

	this->loadStoredRig();

	this->rig_calibration = std::make_unique<rig_calibration::RigCalibration>();

	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			// The solve works in OpenCV space, and its world is the tracking origin: that is the frame the
			// solved poses are written back into below in checkSensorCalibration.
			xrt_pose Tcv_origin_cam;
			math_pose_convert_from_opencv(&camera->locked_data.Txr_origin_cam, &Tcv_origin_cam);

			// Add the camera to the sensor calibration job
			this->rig_calibration->addCamera({
			    .mosaic_idx = mosaic->index,
			    .camera_idx = camera->index,
			    .params = camera->model,
			    .Tcv_world_cam = Tcv_origin_cam,
			    .has_concrete_pose = camera->locked_data.has_concrete_pose,
			});
		}
	}

	CT_DEBUG(this, "Created constellation tracker with %zu mosaics", this->mosaics.size());

	// The Rift tab's "Measure the sensors again" (and riftlinux-setup recalibrate) reach the tracker this way.
	u_notice_set_recalibrator(
	    [](void *ptr) { return static_cast<ConstellationTracker *>(ptr)->recalibrate(); }, this);
}

void
ConstellationTracker::stopCameraThreads()
{
	this->running = false;
	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->slow_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->slow_processing_thread);
			}
			if (camera->fast_processing_thread.initialized) {
				os_thread_helper_stop_and_wait(&camera->fast_processing_thread);
			}
		}
	}
}

ConstellationTracker::~ConstellationTracker()
{
	// First: waits for a request in progress, so none reaches a tracker being destroyed.
	u_notice_set_recalibrator(nullptr, nullptr);

	/*
	 * The cameras' threads before the devices and cameras go (members, destroyed after this): a tracker destroyed
	 * without its frame node broken apart first, as the replay tool's, had a camera's search use a freed device's
	 * search model. Stopping again after a break apart does nothing.
	 */
	this->stopCameraThreads();

	if (this->rig_calibration_thread.initialized) {
		os_thread_helper_destroy(&this->rig_calibration_thread);
	}

	CT_DEBUG(this, "Destroying constellation tracker");
}

void
ConstellationTracker::publishStatus(timepoint_ns now_ns)
{
	bool unplaced = false;
	// Every camera read now (none busy): the placed count is current, not left from before.
	bool fresh = true;
	u_notice_sensors status = {};
	bool quarantined = false;
	CameraKey quarantined_key{};
	{
		std::unique_lock<os::Mutex> lock(this->rig_watch.lock, std::try_to_lock);
		if (lock.owns_lock() && this->rig_watch.quarantined.has_value()) {
			quarantined = true;
			quarantined_key = this->rig_watch.quarantined.value();
		}
	}
	size_t n = 0;
	for (size_t m = 0; m < this->mosaics.size(); m++) {
		for (size_t c = 0; c < this->mosaics[m]->cameras.size(); c++, n++) {
			Camera &camera = *this->mosaics[m]->cameras[c];
			if (this->status_placed.size() <= n) {
				this->status_placed.push_back(false);
			}
			{
				std::unique_lock<os::Mutex> lock(camera.processing_lock, std::try_to_lock);
				if (lock.owns_lock()) {
					this->status_placed[n] = camera.locked_data.has_concrete_pose;
				} else {
					fresh = false;
				}
			}
			status.found++;
			const int64_t last = camera.last_push_ns.load();
			status.streaming += last != 0 && now_ns - last < (int64_t)U_TIME_1S_IN_NS;
			const bool left_out = quarantined && quarantined_key == CameraKey{m, c};
			status.placed += this->status_placed[n] && !left_out;
			unplaced = unplaced || !this->status_placed[n];
		}
	}
	// Measuring: a camera being placed again, the room's solve, or a camera not placed yet (collecting for it).
	status.measuring = quarantined || unplaced || this->running_calibration.load();
	if (fresh) {
		this->cameras_placed.store(status.placed);
	}
	u_notice_set_sensors(&status);
}

void
ConstellationTracker::loadStoredRig()
{
	if (!debug_get_bool_option_constellation_tracker_load_rig()) {
		CT_INFO(this, "Not restoring stored camera placements (CONSTELLATION_TRACKER_LOAD_RIG=false).");
		return;
	}

	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_RIG_FILENAME, "r");
	if (file == nullptr) {
		return;
	}
	size_t size = 0;
	char *content = u_file_read_content(file, &size);
	fclose(file);
	if (content == nullptr) {
		return;
	}

	cJSON *root = cJSON_Parse(content);
	free(content);
	if (root == nullptr) {
		CT_WARN(this,
		        "The stored rig in " CT_RIG_SUBDIR "/" CT_RIG_FILENAME " is not valid JSON, ignoring it.");
		return;
	}

	int version = 0;
	u_json_get_int(u_json_get(root, "version"), &version);
	if (version != CT_RIG_VERSION) {
		CT_WARN(this, "The stored rig has version %d, expected %d, ignoring it.", version, CT_RIG_VERSION);
		cJSON_Delete(root);
		return;
	}

	size_t restored = 0;
	const cJSON *entries = u_json_get(root, "cameras");
	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			if (camera->serial.empty() || camera->locked_data.has_concrete_pose) {
				continue;
			}

			const cJSON *entry = nullptr;
			cJSON_ArrayForEach(entry, entries)
			{
				char serial[64] = {0};
				if (!u_json_get_string_into_array(u_json_get(entry, "serial"), serial,
				                                  sizeof(serial)) ||
				    camera->serial != serial) {
					continue;
				}

				xrt_pose pose;
				if (!u_json_get_pose(u_json_get(entry, "pose"), &pose)) {
					CT_WARN(this, "The stored pose of camera %s is incomplete, ignoring it.",
					        serial);
					break;
				}

				camera->locked_data.Txr_origin_cam = pose;
				camera->locked_data.has_concrete_pose = true;
				restored++;
				CT_INFO(this, "Restored camera %zu/%zu (%s) at [%f, %f, %f].", mosaic->index,
				        camera->index, serial, pose.position.x, pose.position.y, pose.position.z);
				break;
			}
		}
	}

	cJSON_Delete(root);

	if (restored > 0) {
		CT_INFO(this, "Restored %zu camera placement(s), no calibration needed for them.", restored);
	}
}

void
ConstellationTracker::saveRig(double camera_rms_sigma)
{
	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "version", CT_RIG_VERSION);
	cJSON_AddNumberToObject(root, "saved_unix_seconds", static_cast<double>(time(nullptr)));
	cJSON_AddNumberToObject(root, "camera_rms_sigma", camera_rms_sigma);
	cJSON *entries = cJSON_AddArrayToObject(root, "cameras");

	size_t saved = 0;
	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			xrt_pose pose;
			{
				std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
				if (!camera->locked_data.has_concrete_pose || camera->serial.empty()) {
					continue;
				}
				pose = camera->locked_data.Txr_origin_cam;
			}

			cJSON *entry = cJSON_CreateObject();
			cJSON_AddStringToObject(entry, "serial", camera->serial.c_str());
			cJSON *json_pose = cJSON_AddObjectToObject(entry, "pose");
			cJSON *position = cJSON_AddObjectToObject(json_pose, "position");
			cJSON_AddNumberToObject(position, "x", pose.position.x);
			cJSON_AddNumberToObject(position, "y", pose.position.y);
			cJSON_AddNumberToObject(position, "z", pose.position.z);
			cJSON *orientation = cJSON_AddObjectToObject(json_pose, "orientation");
			cJSON_AddNumberToObject(orientation, "x", pose.orientation.x);
			cJSON_AddNumberToObject(orientation, "y", pose.orientation.y);
			cJSON_AddNumberToObject(orientation, "z", pose.orientation.z);
			cJSON_AddNumberToObject(orientation, "w", pose.orientation.w);
			cJSON_AddItemToArray(entries, entry);
			saved++;
		}
	}

	char *text = cJSON_Print(root);
	cJSON_Delete(root);
	if (text == nullptr || saved == 0) {
		cJSON_free(text);
		return;
	}

	// Written next to it first and renamed over it, so a crash never leaves half a file.
	char dir[PATH_MAX];
	int len = u_file_get_config_dir(dir, sizeof(dir));
	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_RIG_FILENAME ".tmp", "w");
	if (len <= 0 || len >= (int)sizeof(dir) || file == nullptr) {
		CT_ERROR(this, "Could not open the rig file to save camera placements.");
		if (file != nullptr) {
			fclose(file);
		}
		cJSON_free(text);
		return;
	}
	const bool written = fputs(text, file) >= 0;
	fclose(file);
	cJSON_free(text);

	const std::string tmp_path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_RIG_FILENAME ".tmp";
	const std::string path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_RIG_FILENAME;
	if (!written || rename(tmp_path.c_str(), path.c_str()) != 0) {
		CT_ERROR(this, "Could not save camera placements to %s.", path.c_str());
		return;
	}

	CT_INFO(this, "Saved the placement of %zu camera(s) to %s.", saved, path.c_str());
}

void
ConstellationTracker::applyLooseParts()
{
	if (!this->loose_part_pending.exchange(false)) {
		return;
	}

	this->devices_changing.store(true);
	std::unique_lock lock(this->device_lock);
	this->devices_changing.store(false);

	for (auto &device : this->devices) {
		Device::LoosePartState *lp = device->loose_part.get();
		if (lp == nullptr) {
			continue;
		}
		std::optional<LoosePartFit> fit;
		std::string why;
		{
			std::unique_lock<std::mutex> l(lp->result_lock);
			if (!lp->result_pending) {
				continue;
			}
			lp->result_pending = false;
			fit = lp->result;
			why = lp->result_why;
		}

		if (!fit.has_value()) {
			device->buildLedModel(std::nullopt);
			CT_WARN(this, "Device %d's adjustable part: %s.", device->id, why.c_str());
			continue;
		}

		device->buildLedModel(fit->offset);
		const xrt_pose &off = fit->offset;
		const float moved_mm = m_vec3_len(off.position) * 1000.0f;
		const float turned_deg = RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(off.orientation.w), 0.0f, 1.0f)));
		CT_WARN(this,
		        "Device %d's adjustable part %s: %.1f mm and %.1f degrees from its factory place (%zu of its LED "
		        "views matched, %.2f px, p90 %.2f px, over %zu frames); its %zu LEDs are tracked with the device.",
		        device->id, why.c_str(), moved_mm, turned_deg, fit->matches, (double)fit->p50_px,
		        (double)fit->p90_px, fit->frames, lp->part_leds.size());
		if (!lp->serial.empty()) {
			this->saveLoosePart(lp->serial, fit.value());
		}
	}
}

void
ConstellationTracker::feedLevel(timepoint_ns timestamp_ns,
                                uint32_t camera_key,
                                const xrt_quat &Rcv_world_device,
                                const xrt_vec3 &imu_down_cv)
{
	// Not while a sensor is left out or being placed, or the room measured: the places are changing.
	{
		std::unique_lock<os::Mutex> watch_lock(this->rig_watch.lock);
		if (this->rig_watch.quarantined.has_value() || this->rig_watch.refining.has_value() ||
		    this->running_calibration.load()) {
			std::unique_lock<os::Mutex> lock(this->level.lock);
			this->level.watch.reset();
			this->level.pending.reset();
			return;
		}
	}

	LevelResult result;
	std::optional<xrt_vec3> apply;
	{
		std::unique_lock<os::Mutex> lock(this->level.lock);
		if (!this->level.watch.add(camera_key, Rcv_world_device, imu_down_cv, result)) {
			return;
		}
		this->level.last_tilt_deg = result.tilt_deg;
		this->level.last_ns = timestamp_ns;
		std::string per_camera;
		for (size_t i = 0; i < result.cameras; i++) {
			char part[64];
			snprintf(part, sizeof(part), "%s%u/%u %.1f (%zu)", i == 0 ? "" : ", ",
			         result.camera_key[i] / XRT_TRACKING_MAX_CAMS, result.camera_key[i] % XRT_TRACKING_MAX_CAMS,
			         (double)result.camera_tilt_deg[i], result.camera_solves[i]);
			per_camera += part;
		}
		/*
		 * Sensors tilted alike tilt the room; one tilted on its own is a moved sensor, the watch's case. Only a
		 * window in which the headset turned tells a tilt from the IMU's seat in the headset.
		 */
		const bool tilted = result.cameras > 0 && result.tilt_deg >= CT_LEVEL_MIN_DEG &&
		                    result.spread_deg <= CT_LEVEL_AGREE_DEG &&
		                    result.yaw_spread_deg >= CT_LEVEL_MIN_YAW_DEG &&
		                    result.residual_deg <= CT_LEVEL_MAX_RESIDUAL_DEG;
		CT_INFO(this,
		        "Room level: %.1f degrees from the headset's gravity, the sensors %.1f apart (sensor, degrees, solves: "
		        "%s; its IMU's seat %.1f degrees; turned %.0f degrees about the vertical, leftover %.1f degrees, %zu "
		        "solves)%s.",
		        (double)result.tilt_deg, (double)result.spread_deg, per_camera.c_str(), (double)result.mount_deg,
		        (double)result.yaw_spread_deg, (double)result.residual_deg, result.solves, tilted ? ": tilted" : "");
		if (!tilted) {
			this->level.pending.reset();
			return;
		}
		if (this->level.pending.has_value()) {
			const xrt_vec3 a = this->level.pending.value();
			const xrt_vec3 b = result.level_rotvec;
			if (RAD_TO_DEG(m_vec3_len(a - b)) <= CT_LEVEL_AGREE_DEG) {
				apply = (a + b) * 0.5f;
				this->level.pending.reset();
			} else {
				this->level.pending = b;
			}
		} else {
			this->level.pending = result.level_rotvec;
		}
	}
	if (apply.has_value()) {
		this->applyLevel(timestamp_ns, apply.value(), result);
	}
}

void
ConstellationTracker::applyLevel(timepoint_ns timestamp_ns, const xrt_vec3 &level_rotvec_cv, const LevelResult &result)
{
	// The level turn in OpenXR's axes (the conversion flips y and z, so a rotation's axis does too).
	const xrt_vec3 rotvec_xr = {level_rotvec_cv.x, -level_rotvec_cv.y, -level_rotvec_cv.z};
	const float angle = m_vec3_len(rotvec_xr);
	if (angle <= 0.0f) {
		return;
	}
	const xrt_vec3 axis = m_vec3_mul_scalar(rotvec_xr, 1.0f / angle);

	/*
	 * Turned about the headset, so it stays where it is (and its height above the floor): about the room's origin,
	 * a headset 3 m from it would jump by 3 m times the tilt's sine. Without a position, about the origin.
	 */
	xrt_vec3 pivot = XRT_VEC3_ZERO;
	for (auto &device : this->devices) {
		if (device->params.led_model.led_count <= CT_RIG_WATCH_PROBE_MIN_LEDS) {
			continue;
		}
		timepoint_ns pose_ns = 0;
		xrt_space_relation fused = XRT_SPACE_RELATION_ZERO;
		device->sensor_fusion->getTrackedPose(timestamp_ns, pose_ns, fused);
		if ((fused.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0) {
			pivot = fused.pose.position;
			break;
		}
	}
	xrt_pose L = XRT_POSE_IDENTITY;
	math_quat_from_angle_vector(angle, &axis, &L.orientation);
	xrt_vec3 turned_pivot;
	math_quat_rotate_vec3(&L.orientation, &pivot, &turned_pivot);
	L.position = pivot - turned_pivot;

	size_t turned = 0;
	for (auto &mosaic : this->mosaics) {
		std::optional<xrt_pose> Txr_world_origin = mosaic->getTrackingOriginPose(timestamp_ns);
		if (!Txr_world_origin.has_value()) {
			continue;
		}
		xrt_pose Txr_origin_world;
		math_pose_invert(&Txr_world_origin.value(), &Txr_origin_world);
		for (auto &camera : mosaic->cameras) {
			xrt_pose Txr_origin_cam;
			{
				std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
				if (!camera->locked_data.has_concrete_pose) {
					continue;
				}
				xrt_pose Txr_world_cam, levelled;
				math_pose_transform(&Txr_world_origin.value(), &camera->locked_data.Txr_origin_cam, &Txr_world_cam);
				math_pose_transform(&L, &Txr_world_cam, &levelled);
				math_pose_transform(&Txr_origin_world, &levelled, &Txr_origin_cam);
				camera->locked_data.Txr_origin_cam = Txr_origin_cam;
			}
			xrt_pose Tcv_origin_cam;
			math_pose_convert_from_opencv(&Txr_origin_cam, &Tcv_origin_cam);
			std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
			this->rig_calibration->setCameraPose(mosaic->index, camera->index, Tcv_origin_cam);
			turned++;
		}
	}
	if (turned == 0) {
		return;
	}

	// The devices' fusion carried the tilted room; and the headset's down learned in it is off by as much.
	for (auto &device : this->devices) {
		device->sensor_fusion->requestReset();
	}
	{
		std::unique_lock<os::Mutex> learn_lock(this->down_learn.lock);
		this->down_learn.imu_down = XRT_VEC3_ZERO;
		this->down_learn.room_down = XRT_VEC3_ZERO;
		this->down_learn.count = 0;
	}
	{
		std::unique_lock<os::Mutex> lock(this->level.lock);
		this->level.levelled++;
	}

	CT_WARN(this,
	        "The room was tilted %.1f degrees from the headset's gravity (its sensors placed while tilted otherwise): "
	        "all %zu sensors turned to level it, about the headset at (%.2f, %.2f, %.2f) m (sensors %.1f degrees "
	        "apart, the headset's IMU seat %.1f degrees, turned %.0f degrees about the vertical). Saved.",
	        RAD_TO_DEG(angle), turned, (double)pivot.x, (double)pivot.y, (double)pivot.z, (double)result.spread_deg,
	        (double)result.mount_deg, (double)result.yaw_spread_deg);
	this->saveRig(0.0);
}

std::optional<xrt_pose>
ConstellationTracker::loadLoosePart(const std::string &serial)
{
	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_LOOSE_PART_FILENAME, "r");
	if (file == nullptr) {
		return std::nullopt;
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0) {
		text.append(buf, n);
	}
	fclose(file);

	cJSON *root = cJSON_Parse(text.c_str());
	std::optional<xrt_pose> out;
	const cJSON *parts = cJSON_GetObjectItemCaseSensitive(root, "parts");
	const cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, parts)
	{
		const cJSON *s = cJSON_GetObjectItemCaseSensitive(entry, "serial");
		if (!cJSON_IsString(s) || serial != s->valuestring) {
			continue;
		}
		const cJSON *pos = cJSON_GetObjectItemCaseSensitive(entry, "position");
		const cJSON *ori = cJSON_GetObjectItemCaseSensitive(entry, "orientation");
		auto num = [](const cJSON *o, const char *k, bool &ok) {
			const cJSON *v = cJSON_GetObjectItemCaseSensitive(o, k);
			ok = ok && cJSON_IsNumber(v);
			return cJSON_IsNumber(v) ? (float)v->valuedouble : 0.0f;
		};
		bool ok = pos != nullptr && ori != nullptr;
		xrt_pose p;
		p.position = {num(pos, "x", ok), num(pos, "y", ok), num(pos, "z", ok)};
		p.orientation = {num(ori, "x", ok), num(ori, "y", ok), num(ori, "z", ok), num(ori, "w", ok)};
		if (ok) {
			math_quat_normalize(&p.orientation);
			out = p;
		}
	}
	cJSON_Delete(root);
	return out;
}

void
ConstellationTracker::saveLoosePart(const std::string &serial, const LoosePartFit &fit)
{
	// Other devices' entries are kept.
	cJSON *root = nullptr;
	if (FILE *in = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_LOOSE_PART_FILENAME, "r")) {
		std::string text;
		char buf[4096];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
			text.append(buf, n);
		}
		fclose(in);
		root = cJSON_Parse(text.c_str());
	}
	if (root == nullptr || !cJSON_IsArray(cJSON_GetObjectItemCaseSensitive(root, "parts"))) {
		cJSON_Delete(root);
		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "version", 1);
		cJSON_AddArrayToObject(root, "parts");
	}
	cJSON *parts = cJSON_GetObjectItemCaseSensitive(root, "parts");
	for (int i = cJSON_GetArraySize(parts) - 1; i >= 0; i--) {
		const cJSON *s = cJSON_GetObjectItemCaseSensitive(cJSON_GetArrayItem(parts, i), "serial");
		if (cJSON_IsString(s) && serial == s->valuestring) {
			cJSON_DeleteItemFromArray(parts, i);
		}
	}
	cJSON *entry = cJSON_CreateObject();
	cJSON_AddStringToObject(entry, "serial", serial.c_str());
	cJSON_AddNumberToObject(entry, "saved_unix_seconds", static_cast<double>(time(nullptr)));
	cJSON_AddNumberToObject(entry, "matched", static_cast<double>(fit.matches));
	cJSON_AddNumberToObject(entry, "p50_px", fit.p50_px);
	cJSON *position = cJSON_AddObjectToObject(entry, "position");
	cJSON_AddNumberToObject(position, "x", fit.offset.position.x);
	cJSON_AddNumberToObject(position, "y", fit.offset.position.y);
	cJSON_AddNumberToObject(position, "z", fit.offset.position.z);
	cJSON *orientation = cJSON_AddObjectToObject(entry, "orientation");
	cJSON_AddNumberToObject(orientation, "x", fit.offset.orientation.x);
	cJSON_AddNumberToObject(orientation, "y", fit.offset.orientation.y);
	cJSON_AddNumberToObject(orientation, "z", fit.offset.orientation.z);
	cJSON_AddNumberToObject(orientation, "w", fit.offset.orientation.w);
	cJSON_AddItemToArray(parts, entry);

	char *text = cJSON_Print(root);
	cJSON_Delete(root);
	char dir[PATH_MAX];
	const int len = u_file_get_config_dir(dir, sizeof(dir));
	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_LOOSE_PART_FILENAME ".tmp", "w");
	if (text == nullptr || len <= 0 || len >= (int)sizeof(dir) || file == nullptr) {
		CT_ERROR(this, "Could not save where the adjustable part of %s sits.", serial.c_str());
		if (file != nullptr) {
			fclose(file);
		}
		cJSON_free(text);
		return;
	}
	const bool written = fputs(text, file) >= 0;
	fclose(file);
	cJSON_free(text);
	const std::string tmp_path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_LOOSE_PART_FILENAME ".tmp";
	const std::string path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_LOOSE_PART_FILENAME;
	if (!written || rename(tmp_path.c_str(), path.c_str()) != 0) {
		CT_ERROR(this, "Could not save where the adjustable part of %s sits to %s.", serial.c_str(), path.c_str());
		return;
	}
	CT_INFO(this, "Saved where the adjustable part of %s sits to %s.", serial.c_str(), path.c_str());
}

void
ConstellationTracker::loadDownLearn(const Device *device)
{
	const std::string serial(device->params.serial, strnlen(device->params.serial, sizeof(device->params.serial)));
	{
		std::unique_lock<os::Mutex> lock(this->down_learn.lock);
		if (this->down_learn.loaded) {
			return;
		}
		this->down_learn.loaded = true;
		if (serial.empty() || !debug_get_bool_option_constellation_tracker_down_saved() ||
		    this->down_learn.count > 0) {
			return;
		}
	}
	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_DOWN_FILENAME, "r");
	if (file == nullptr) {
		return;
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0) {
		text.append(buf, n);
	}
	fclose(file);

	cJSON *root = cJSON_Parse(text.c_str());
	const cJSON *headsets = cJSON_GetObjectItemCaseSensitive(root, "headsets");
	const cJSON *entry = nullptr;
	cJSON_ArrayForEach(entry, headsets)
	{
		const cJSON *s = cJSON_GetObjectItemCaseSensitive(entry, "serial");
		if (!cJSON_IsString(s) || serial != s->valuestring) {
			continue;
		}
		auto vec = [&](const char *key, xrt_vec3 &out) {
			const cJSON *a = cJSON_GetObjectItemCaseSensitive(entry, key);
			if (!cJSON_IsArray(a) || cJSON_GetArraySize(a) != 3) {
				return false;
			}
			float v[3];
			for (int i = 0; i < 3; i++) {
				const cJSON *x = cJSON_GetArrayItem(a, i);
				if (!cJSON_IsNumber(x)) {
					return false;
				}
				v[i] = (float)x->valuedouble;
			}
			out = {v[0], v[1], v[2]};
			return m_vec3_len(out) > 0.5f;
		};
		xrt_vec3 imu_down, room_down;
		if (!vec("imu_down", imu_down) || !vec("room_down", room_down)) {
			continue;
		}
		// A seat is a few degrees; anything else is not one.
		const float deg = RAD_TO_DEG(acosf(
		    std::clamp(m_vec3_dot(m_vec3_normalize(imu_down), m_vec3_normalize(room_down)), -1.0f, 1.0f)));
		if (deg > 8.0f) {
			continue;
		}
		std::unique_lock<os::Mutex> lock(this->down_learn.lock);
		if (this->down_learn.count == 0) {
			this->down_learn.imu_down = m_vec3_normalize(imu_down);
			this->down_learn.room_down = m_vec3_normalize(room_down);
			// Weighs like a thousand solves: this session's own soon take over.
			this->down_learn.count = 1000;
			CT_INFO(this, "The headset's down correction of %s: %.1f degrees (saved).", serial.c_str(), deg);
		}
	}
	cJSON_Delete(root);
}

void
ConstellationTracker::saveDownLearn(const std::string &serial,
                                    const xrt_vec3 &imu_down,
                                    const xrt_vec3 &room_down,
                                    size_t count)
{
	// Other headsets' entries are kept.
	cJSON *root = nullptr;
	if (FILE *in = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_DOWN_FILENAME, "r")) {
		std::string text;
		char buf[4096];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), in)) > 0) {
			text.append(buf, n);
		}
		fclose(in);
		root = cJSON_Parse(text.c_str());
	}
	if (root == nullptr || !cJSON_IsArray(cJSON_GetObjectItemCaseSensitive(root, "headsets"))) {
		cJSON_Delete(root);
		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "version", 1);
		cJSON_AddArrayToObject(root, "headsets");
	}
	cJSON *headsets = cJSON_GetObjectItemCaseSensitive(root, "headsets");
	for (int i = cJSON_GetArraySize(headsets) - 1; i >= 0; i--) {
		const cJSON *s = cJSON_GetObjectItemCaseSensitive(cJSON_GetArrayItem(headsets, i), "serial");
		if (cJSON_IsString(s) && serial == s->valuestring) {
			cJSON_DeleteItemFromArray(headsets, i);
		}
	}
	const xrt_vec3 a = m_vec3_normalize(imu_down), b = m_vec3_normalize(room_down);
	const float va[3] = {a.x, a.y, a.z}, vb[3] = {b.x, b.y, b.z};
	cJSON *entry = cJSON_CreateObject();
	cJSON_AddStringToObject(entry, "serial", serial.c_str());
	cJSON_AddNumberToObject(entry, "saved_unix_seconds", static_cast<double>(time(nullptr)));
	cJSON_AddNumberToObject(entry, "solves", static_cast<double>(count));
	cJSON_AddItemToObject(entry, "imu_down", cJSON_CreateFloatArray(va, 3));
	cJSON_AddItemToObject(entry, "room_down", cJSON_CreateFloatArray(vb, 3));
	cJSON_AddItemToArray(headsets, entry);

	char *text = cJSON_Print(root);
	cJSON_Delete(root);
	char dir[PATH_MAX];
	const int len = u_file_get_config_dir(dir, sizeof(dir));
	FILE *file = u_file_open_file_in_config_dir_subpath(CT_RIG_SUBDIR, CT_DOWN_FILENAME ".tmp", "w");
	if (text == nullptr || len <= 0 || len >= (int)sizeof(dir) || file == nullptr) {
		CT_ERROR(this, "Could not save the headset's down correction of %s.", serial.c_str());
		if (file != nullptr) {
			fclose(file);
		}
		cJSON_free(text);
		return;
	}
	const bool written = fputs(text, file) >= 0;
	fclose(file);
	cJSON_free(text);
	const std::string tmp_path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_DOWN_FILENAME ".tmp";
	const std::string path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_DOWN_FILENAME;
	if (!written || rename(tmp_path.c_str(), path.c_str()) != 0) {
		CT_ERROR(this, "Could not save the headset's down correction of %s to %s.", serial.c_str(), path.c_str());
		return;
	}
	const float deg = RAD_TO_DEG(acosf(std::clamp(m_vec3_dot(a, b), -1.0f, 1.0f)));
	CT_INFO(this, "Saved the headset's down correction of %s (%.1f degrees, %zu solves) to %s.", serial.c_str(), deg,
	        count, path.c_str());
}

void
ConstellationTracker::checkRigConsistency(t_constellation_device_id_t device_id,
                                          size_t mosaic_idx,
                                          size_t camera_idx,
                                          timepoint_ns timestamp_ns,
                                          uint32_t matched_blobs,
                                          const xrt_vec3 &world_position)
{
	if (matched_blobs < CT_RIG_CHECK_MIN_LEDS) {
		return;
	}

	float median_m = 0.0f;
	bool moved = false;
	{
		std::unique_lock<os::Mutex> lock(this->rig_check.lock);

		for (const auto &[key, value] : this->rig_check.last) {
			const auto &[other_device, other_mosaic, other_camera] = key;
			if (other_device != device_id || (other_mosaic == mosaic_idx && other_camera == camera_idx) ||
			    value.first != timestamp_ns) {
				continue;
			}
			const bool other_first =
			    std::make_pair(other_mosaic, other_camera) < std::make_pair(mosaic_idx, camera_idx);
			this->rig_check.differences.push_back(other_first ? m_vec3_sub(world_position, value.second)
			                                                  : m_vec3_sub(value.second, world_position));
		}
		this->rig_check.last[{device_id, mosaic_idx, camera_idx}] = {timestamp_ns, world_position};

		if (this->rig_check.differences.size() < CT_RIG_CHECK_WINDOW) {
			return;
		}

		auto &differences = this->rig_check.differences;
		xrt_vec3 median = {};
		float *median_axes[3] = {&median.x, &median.y, &median.z};
		for (int axis = 0; axis < 3; axis++) {
			std::vector<float> values;
			values.reserve(differences.size());
			for (const xrt_vec3 &difference : differences) {
				values.push_back(axis == 0 ? difference.x : (axis == 1 ? difference.y : difference.z));
			}
			std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
			*median_axes[axis] = values[values.size() / 2];
		}
		median_m = m_vec3_len(median);
		differences.clear();

		const float threshold_m = debug_get_float_option_constellation_tracker_rig_moved_m();
		this->rig_check.strikes = median_m > threshold_m ? this->rig_check.strikes + 1 : 0;
		moved = this->rig_check.strikes >= CT_RIG_CHECK_STRIKES;
		if (moved) {
			this->rig_check.strikes = 0;
			this->rig_check.last.clear();
		}
	}

	CT_DEBUG(this, "Cameras are offset on device %d by %.1f mm (median of %d frames).", device_id,
	         median_m * 1000.0f, CT_RIG_CHECK_WINDOW);

	if (!moved) {
		return;
	}

	CT_WARN(this,
	        "The cameras keep disagreeing on where device %d is, by %.1f mm: one has moved since it was placed. "
	        "Placing them again, move the device around in view of every camera.",
	        device_id, median_m * 1000.0f);

	// Keep the first placed camera as the reference, so the world stays where it was.
	bool kept_reference = false;
	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			{
				std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
				if (!camera->locked_data.has_concrete_pose) {
					continue;
				}
				if (!kept_reference) {
					kept_reference = true;
					continue;
				}
				camera->locked_data.has_concrete_pose = false;
			}
			this->rig_calibration->clearCameraPose(mosaic->index, camera->index);
		}
	}
}

//! A camera's number for the user, counted from 1 across the mosaics.
static int
rig_watch_camera_number(const ConstellationTracker &tracker, const ConstellationTracker::CameraKey &key)
{
	int number = 1;
	for (size_t m = 0; m < key.first && m < tracker.mosaics.size(); m++) {
		number += static_cast<int>(tracker.mosaics[m]->cameras.size());
	}
	return number + static_cast<int>(key.second);
}

//! How far a camera's own direction to a device is from the direction to @p world_point, degrees.
static float
rig_watch_direction_deg(const ConstellationTracker::RigWatchSolve &solve, const xrt_vec3 &world_point)
{
	xrt_pose Txr_cam_world;
	math_pose_invert(&solve.Txr_world_cam, &Txr_cam_world);
	xrt_vec3 in_cam;
	math_pose_transform_point(&Txr_cam_world, &world_point, &in_cam);
	const float lengths = m_vec3_len(in_cam) * m_vec3_len(solve.Txr_cam_device.position);
	if (lengths <= 0.0f) {
		return 0.0f;
	}
	return RAD_TO_DEG(acosf(CLAMP(m_vec3_dot(in_cam, solve.Txr_cam_device.position) / lengths, -1.0f, 1.0f)));
}

//! How far @p to is turned from @p from, degrees.
static float
rig_watch_turned_deg(const xrt_pose &from, const xrt_pose &to)
{
	xrt_quat delta;
	math_quat_unrotate(&from.orientation, &to.orientation, &delta);
	return RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(delta.w), 0.0f, 1.0f)));
}

//! Whether @p to moved at least @p min_m or turned at least @p min_deg from @p from.
static bool
rig_watch_moved(const xrt_pose &from, const xrt_pose &to, float min_m, float min_deg)
{
	return m_vec3_len(to.position - from.position) >= min_m || rig_watch_turned_deg(from, to) >= min_deg;
}

static float
rig_watch_median(std::vector<float> values)
{
	std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
	return values[values.size() / 2];
}

/*!
 * The median of camera poses: per axis for the position, per axis of the rotation vector about the first for the
 * orientation. @p out_mad_m and @p out_mad_deg are how far they spread about it (median absolute deviation, the
 * larger axis, and of the angle).
 */
static xrt_pose
rig_watch_median_pose(const std::vector<xrt_pose> &poses, float &out_mad_m, float &out_mad_deg)
{
	std::vector<float> axis[3];
	std::vector<float> rot[3];
	const xrt_quat &first = poses.front().orientation;
	for (const xrt_pose &pose : poses) {
		axis[0].push_back(pose.position.x);
		axis[1].push_back(pose.position.y);
		axis[2].push_back(pose.position.z);
		xrt_quat delta;
		math_quat_unrotate(&first, &pose.orientation, &delta);
		if (delta.w < 0.0f) {
			delta = xrt_quat{-delta.x, -delta.y, -delta.z, -delta.w};
		}
		xrt_vec3 v;
		math_quat_ln_so3(&delta, &v);
		rot[0].push_back(v.x);
		rot[1].push_back(v.y);
		rot[2].push_back(v.z);
	}

	xrt_pose out;
	out.position = {rig_watch_median(axis[0]), rig_watch_median(axis[1]), rig_watch_median(axis[2])};
	const xrt_vec3 median_rot = {rig_watch_median(rot[0]), rig_watch_median(rot[1]), rig_watch_median(rot[2])};
	xrt_quat median_delta;
	math_quat_exp_so3(&median_rot, &median_delta);
	math_quat_rotate(&first, &median_delta, &out.orientation);
	math_quat_normalize(&out.orientation);

	out_mad_m = 0.0f;
	const float centre[3] = {out.position.x, out.position.y, out.position.z};
	for (int k = 0; k < 3; k++) {
		std::vector<float> deviation;
		for (float value : axis[k]) {
			deviation.push_back(fabsf(value - centre[k]));
		}
		out_mad_m = std::max(out_mad_m, rig_watch_median(deviation));
	}
	std::vector<float> angles;
	for (const xrt_pose &pose : poses) {
		xrt_quat delta;
		math_quat_unrotate(&out.orientation, &pose.orientation, &delta);
		angles.push_back(RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(delta.w), 0.0f, 1.0f))));
	}
	out_mad_deg = rig_watch_median(angles);
	return out;
}

void
ConstellationTracker::watchRig(t_constellation_device_id_t device_id,
                               size_t mosaic_idx,
                               size_t camera_idx,
                               timepoint_ns timestamp_ns,
                               const xrt_pose &Txr_world_cam,
                               const xrt_pose &Txr_cam_device,
                               float tilt_deg,
                               float fusion_deg,
                               const std::optional<xrt_pose> &Txr_world_device_fused)
{
	const CameraKey self{mosaic_idx, camera_idx};
	const RigWatchSolve solve{timestamp_ns, Txr_world_cam, Txr_cam_device};

	struct Found
	{
		CameraKey moved;
		CameraKey anchor;
		//! By how much they disagreed, in degrees on the direction or mm on the position.
		float by;
		bool on_direction;
		const char *why;
	};
	std::optional<Found> found;
	struct Placement
	{
		CameraKey camera;
		xrt_pose Txr_world_cam;
		xrt_pose before;
		float mad_m;
		float mad_deg;
		size_t pairs;
		bool refined;
	};
	std::optional<Placement> placement;
	std::optional<CameraKey> spread_too_wide;
	//! Tilt and fusion medians of the two cameras when one was found moved, for the log.
	float judged[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
	float spread_mm = 0.0f;
	float spread_deg = 0.0f;

	{
		std::unique_lock<os::Mutex> lock(this->rig_watch.lock);
		auto &w = this->rig_watch;

		auto keep_recent = [](std::vector<float> &values, float value) {
			if (value < 0.0f) {
				return;
			}
			values.push_back(value);
			if (values.size() > CT_RIG_WATCH_JUDGE_SOLVES) {
				values.erase(values.begin());
			}
		};
		keep_recent(w.tilt[{device_id, mosaic_idx, camera_idx}], tilt_deg);
		keep_recent(w.fusion[{device_id, mosaic_idx, camera_idx}], fusion_deg);

		// A new view (CT_RIG_WATCH_NEW_VIEW_M): the frame last compared, or the fused pose away from it.
		bool new_view = false;
		if (Txr_world_device_fused.has_value()) {
			const auto it = w.view.find(device_id);
			new_view = it == w.view.end() || it->second.first == timestamp_ns ||
			           rig_watch_moved(it->second.second, Txr_world_device_fused.value(),
			                           CT_RIG_WATCH_NEW_VIEW_M, CT_RIG_WATCH_NEW_VIEW_DEG);
		}
		// How far the device moved over a window, from the first view in it (CT_RIG_WATCH_REACH_M).
		auto extend = [&](RigWatchReach &reach, bool starts) {
			const xrt_pose &view = Txr_world_device_fused.value();
			if (starts) {
				reach = RigWatchReach{view, 0.0f, 0.0f};
				return;
			}
			reach.m = std::max(reach.m, m_vec3_len(view.position - reach.first.position));
			reach.deg = std::max(reach.deg, rig_watch_turned_deg(reach.first, view));
		};
		auto moved_around = [](const RigWatchReach &reach) {
			return reach.m >= CT_RIG_WATCH_REACH_M || reach.deg >= CT_RIG_WATCH_REACH_DEG;
		};

		for (const auto &[key, other] : w.last) {
			const auto &[other_device, other_mosaic, other_camera] = key;
			const CameraKey other_key{other_mosaic, other_camera};
			if (other_device != device_id || other_key == self || other.timestamp_ns != timestamp_ns) {
				continue;
			}

			/*
			 * A camera being placed again, left out or (refining) in use: where its solve puts it, with the
			 * anchor's solve of the same frame taken as where the device is.
			 */
			const std::optional<CameraKey> placing = w.quarantined.has_value() ? w.quarantined : w.refining;
			if (placing.has_value()) {
				const CameraKey placed_key = placing.value();
				const bool pair_with_anchor = (self == placed_key && other_key == w.anchor) ||
				                              (other_key == placed_key && self == w.anchor);
				if (pair_with_anchor) {
					const RigWatchSolve &moved = self == placed_key ? solve : other;
					const RigWatchSolve &anchor = self == placed_key ? other : solve;
					xrt_pose Txr_world_device;
					math_pose_transform(&anchor.Txr_world_cam, &anchor.Txr_cam_device, &Txr_world_device);
					xrt_pose Txr_device_cam;
					math_pose_invert(&moved.Txr_cam_device, &Txr_device_cam);
					xrt_pose candidate;
					math_pose_transform(&Txr_world_device, &Txr_device_cam, &candidate);
					w.candidates.push_back(candidate);

					const bool first = w.quarantined.has_value();
					if (w.candidates.size() >= (first ? CT_RIG_WATCH_PLACE_PAIRS : CT_RIG_WATCH_REFINE_PAIRS)) {
						float mad_m = 0.0f;
						float mad_deg = 0.0f;
						const xrt_pose placed = rig_watch_median_pose(w.candidates, mad_m, mad_deg);
						const float max_mad_m = first ? CT_RIG_WATCH_PLACE_MAD_M : CT_RIG_WATCH_REFINE_MAD_M;
						const float max_mad_deg =
						    first ? CT_RIG_WATCH_PLACE_MAD_DEG : CT_RIG_WATCH_REFINE_MAD_DEG;
						if (mad_m <= max_mad_m && mad_deg <= max_mad_deg) {
							placement = Placement{placed_key,
							                      placed,
							                      first ? w.quarantined_from : moved.Txr_world_cam,
							                      mad_m,
							                      mad_deg,
							                      w.candidates.size(),
							                      !first};
							w.quarantined.reset();
							w.refining = first ? placing : std::nullopt;
							w.window.clear();
							w.strikes.clear();
							w.last_median.clear();
							w.suspect_since.clear();
							w.apart.clear();
							w.apart_strikes.clear();
							w.tilt.clear();
							w.fusion.clear();
						} else {
							spread_too_wide = placed_key;
							spread_mm = mad_m * 1000.0f;
							spread_deg = mad_deg;
						}
						w.candidates.clear();
					}
				}
				if (w.quarantined.has_value()) {
					// Nothing else is watched while a camera is left out.
					continue;
				}
			}

			// Only new views are compared; placing a camera again, above, takes every pair.
			if (!new_view) {
				continue;
			}
			w.view[device_id] = {timestamp_ns, Txr_world_device_fused.value()};

			/*
			 * Watching: the larger of each camera's own direction to the device against where the other
			 * camera's solve of the same frame puts it, through the rig.
			 */
			xrt_pose Txr_world_device_self;
			math_pose_transform(&solve.Txr_world_cam, &solve.Txr_cam_device, &Txr_world_device_self);
			xrt_pose Txr_world_device_other;
			math_pose_transform(&other.Txr_world_cam, &other.Txr_cam_device, &Txr_world_device_other);
			const float deg = std::max(rig_watch_direction_deg(solve, Txr_world_device_other.position),
			                           rig_watch_direction_deg(other, Txr_world_device_self.position));

			const auto pair = self < other_key ? std::make_pair(self, other_key) : std::make_pair(other_key, self);
			bool seen = false;
			float by = 0.0f;
			bool on_direction = true;

			std::vector<float> &window = w.window[pair];
			RigWatchReach &reach = w.reach[pair];
			extend(reach, window.empty());
			window.push_back(deg);
			if (window.size() >= CT_RIG_WATCH_WINDOW && !moved_around(reach)) {
				// The same few views of a device lying still: they tell nothing (CT_RIG_WATCH_REACH_M).
				window.clear();
			} else if (window.size() >= CT_RIG_WATCH_WINDOW) {
				const float median_deg = rig_watch_median(window);
				window.clear();
				w.last_median[pair] = {timestamp_ns, median_deg};
				uint32_t &strikes = w.strikes[pair];
				strikes = median_deg > debug_get_float_option_constellation_tracker_rig_watch_deg() ? strikes + 1 : 0;
				if (strikes == 0) {
					w.suspect_since.erase(pair);
				}
				if (strikes >= CT_RIG_WATCH_STRIKES) {
					strikes = 0;
					seen = true;
					by = median_deg;
				}
			}

			// How far apart they put the device, for a camera moved along its view.
			std::vector<xrt_vec3> &apart = w.apart[pair];
			RigWatchReach &apart_reach = w.apart_reach[pair];
			extend(apart_reach, apart.empty());
			apart.push_back(self < other_key ? Txr_world_device_other.position - Txr_world_device_self.position
			                                 : Txr_world_device_self.position - Txr_world_device_other.position);
			if (apart.size() >= CT_RIG_WATCH_APART_WINDOW && !moved_around(apart_reach)) {
				apart.clear();
			} else if (apart.size() >= CT_RIG_WATCH_APART_WINDOW) {
				std::vector<float> axis[3];
				for (const xrt_vec3 &v : apart) {
					axis[0].push_back(v.x);
					axis[1].push_back(v.y);
					axis[2].push_back(v.z);
				}
				const xrt_vec3 median = {rig_watch_median(axis[0]), rig_watch_median(axis[1]),
				                         rig_watch_median(axis[2])};
				const float median_mm = m_vec3_len(median) * 1000.0f;
				apart.clear();
				uint32_t &strikes = w.apart_strikes[pair];
				strikes = median_mm > debug_get_float_option_constellation_tracker_rig_watch_mm() ? strikes + 1 : 0;
				if (strikes >= CT_RIG_WATCH_STRIKES && !seen) {
					strikes = 0;
					seen = true;
					by = median_mm;
					on_direction = false;
				}
			}
			if (!seen) {
				continue;
			}

			/*
			 * More cameras: the vote. The moved one disagrees with a third camera too, which agrees with the
			 * other one. Both agreeing with every third camera seen recently: the pair's own noise, not a move.
			 */
			std::optional<CameraKey> voted;
			bool third_seen = false;
			if (on_direction && debug_get_bool_option_constellation_tracker_rig_watch_vote()) {
				const float thr = debug_get_float_option_constellation_tracker_rig_watch_deg();
				auto recent_median = [&](const CameraKey &x, const CameraKey &y) -> std::optional<float> {
					const auto key = x < y ? std::make_pair(x, y) : std::make_pair(y, x);
					auto it = w.last_median.find(key);
					if (it == w.last_median.end() || timestamp_ns - it->second.first > CT_RIG_WATCH_VOTE_NS) {
						return std::nullopt;
					}
					return it->second.second;
				};
				std::set<CameraKey> thirds;
				for (const auto &[p, _] : w.last_median) {
					for (const CameraKey &c : {p.first, p.second}) {
						if (c != pair.first && c != pair.second) {
							thirds.insert(c);
						}
					}
				}
				for (const CameraKey &c : thirds) {
					const std::optional<float> with_a = recent_median(pair.first, c);
					const std::optional<float> with_b = recent_median(pair.second, c);
					if (!with_a.has_value() || !with_b.has_value()) {
						continue;
					}
					third_seen = true;
					if (with_a.value() > thr && with_b.value() <= thr) {
						voted = pair.first;
						break;
					}
					if (with_b.value() > thr && with_a.value() <= thr) {
						voted = pair.second;
						break;
					}
				}
				if (third_seen && !voted.has_value()) {
					CT_INFO(this,
					        "Cameras %zu/%zu and %zu/%zu disagree on the direction to device %d by %.1f degrees, "
					        "but no third camera sides with either: taken as their noise, not a move.",
					        pair.first.first, pair.first.second, pair.second.first, pair.second.second,
					        device_id, by);
					w.suspect_since.erase(pair);
					continue;
				}
				// Three or more cameras, but no third one has seen the device with both lately: wait for it.
				size_t cameras_seen = 0;
				for (const auto &[key, _] : w.last) {
					cameras_seen += std::get<0>(key) == device_id ? 1 : 0;
				}
				if (!third_seen && cameras_seen >= 3) {
					auto since = w.suspect_since.try_emplace(pair, timestamp_ns);
					if (timestamp_ns - since.first->second < CT_RIG_WATCH_DEFER_NS) {
						w.strikes[pair] = CT_RIG_WATCH_STRIKES - 1;
						if (since.second) {
							CT_INFO(this,
							        "Cameras %zu/%zu and %zu/%zu disagree on the direction to device %d by %.1f "
							        "degrees: waiting for a third camera's view of it to tell which one moved.",
							        pair.first.first, pair.first.second, pair.second.first, pair.second.second,
							        device_id, by);
						}
						continue;
					}
					CT_INFO(this,
					        "Cameras %zu/%zu and %zu/%zu: no third camera saw device %d with both for %.0f s, the two "
					        "alone decide which one moved.",
					        pair.first.first, pair.first.second, pair.second.first, pair.second.second, device_id,
					        time_ns_to_s(timestamp_ns - since.first->second));
				}
			}
			w.suspect_since.erase(pair);

			/*
			 * Which of the two moved: a camera that tilted makes its solves tilt the device away from the
			 * IMU's down; one that turned or slid while the other stayed puts the device away from where the
			 * fusion has it, carried by the other camera and the IMU. Without either, the lower numbered
			 * camera stays where it is, as the first placed camera does in the rig calibration.
			 */
			auto recent = [device_id](const std::map<std::tuple<t_constellation_device_id_t, size_t, size_t>,
			                                         std::vector<float>> &values,
			                          const CameraKey &key) {
				auto it = values.find({device_id, key.first, key.second});
				return it == values.end() || it->second.size() < CT_RIG_WATCH_JUDGE_SOLVES / 4
				           ? -1.0f
				           : rig_watch_median(it->second);
			};
			const CameraKey &a = pair.first;
			const CameraKey &b = pair.second;
			const float tilt_a = recent(w.tilt, a);
			const float tilt_b = recent(w.tilt, b);
			const float fusion_a = recent(w.fusion, a);
			const float fusion_b = recent(w.fusion, b);
			CameraKey moved = b;
			const char *why = "neither its tilt nor the fusion told them apart, the lower numbered one stays";
			if (voted.has_value()) {
				moved = voted.value();
				why = "it disagrees with a third camera as well, which agrees with the other";
			} else if (tilt_a >= 0.0f && tilt_b >= 0.0f && fabsf(tilt_a - tilt_b) > CT_RIG_WATCH_TILT_MARGIN_DEG) {
				moved = tilt_a > tilt_b ? a : b;
				why = "its solves tilt the device away from the IMU's down";
			} else if (fusion_a >= 0.0f && fusion_b >= 0.0f &&
			           std::max(fusion_a, fusion_b) > CT_RIG_WATCH_FUSION_RATIO * std::min(fusion_a, fusion_b) &&
			           std::max(fusion_a, fusion_b) > debug_get_float_option_constellation_tracker_rig_watch_deg()) {
				moved = fusion_a > fusion_b ? a : b;
				why = "it puts the device away from where the fusion has it";
			}
			judged[0] = tilt_a;
			judged[1] = tilt_b;
			judged[2] = fusion_a;
			judged[3] = fusion_b;

			w.quarantined = moved;
			w.refining.reset();
			w.anchor = moved == a ? b : a;
			w.candidates.clear();
			w.quarantined_from = moved == self ? solve.Txr_world_cam : other.Txr_world_cam;
			found = Found{moved, w.anchor, by, on_direction, why};
			break;
		}

		w.last[{device_id, mosaic_idx, camera_idx}] = solve;
	}

	if (found.has_value()) {
		CT_WARN(this,
		        "Camera %zu/%zu has moved: it and camera %zu/%zu disagree on %s device %d by %.1f %s (%s; tilt "
		        "%.1f / %.1f, fusion %.1f / %.1f degrees, lower numbered camera first). Leaving it out while it is "
		        "placed again against camera %zu/%zu.",
		        found->moved.first, found->moved.second, found->anchor.first, found->anchor.second,
		        found->on_direction ? "the direction to" : "where they put", device_id, found->by,
		        found->on_direction ? "degrees" : "mm", found->why, judged[0], judged[1], judged[2], judged[3],
		        found->anchor.first, found->anchor.second);
		u_notice_post(U_NOTICE_SENSORS, "sensor-moved", rig_watch_camera_number(*this, found->moved));
	}
	if (spread_too_wide.has_value()) {
		CT_INFO(this, "Camera %zu/%zu: its new placement isn't settled yet (spread %.0f mm, %.2f degrees), collecting again.",
		        spread_too_wide->first, spread_too_wide->second, spread_mm, spread_deg);
	}
	if (!placement.has_value()) {
		return;
	}

	// The new placement into the camera, in its mosaic's tracking origin, and into the rig calibration.
	const CameraKey &key = placement->camera;
	if (key.first >= this->mosaics.size() || key.second >= this->mosaics[key.first]->cameras.size()) {
		return;
	}
	auto &mosaic = this->mosaics[key.first];
	auto &camera = mosaic->cameras[key.second];
	std::optional<xrt_pose> Txr_world_origin = mosaic->getTrackingOriginPose(timestamp_ns);
	if (!Txr_world_origin.has_value()) {
		CT_WARN(this, "Camera %zu/%zu: no tracking origin to place it in, it stays where it was.", key.first,
		        key.second);
		return;
	}
	xrt_pose Txr_origin_world;
	math_pose_invert(&Txr_world_origin.value(), &Txr_origin_world);
	xrt_pose Txr_origin_cam;
	math_pose_transform(&Txr_origin_world, &placement->Txr_world_cam, &Txr_origin_cam);
	{
		std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
		camera->locked_data.Txr_origin_cam = Txr_origin_cam;
		camera->locked_data.has_concrete_pose = true;
	}
	{
		// The rig calibration works in OpenCV coordinates; the conversion is its own inverse.
		xrt_pose Tcv_origin_cam;
		math_pose_convert_from_opencv(&Txr_origin_cam, &Tcv_origin_cam);
		std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
		this->rig_calibration->setCameraPose(key.first, key.second, Tcv_origin_cam);
	}

	xrt_quat turned;
	math_quat_unrotate(&placement->before.orientation, &placement->Txr_world_cam.orientation, &turned);
	CT_WARN(this,
	        "Camera %zu/%zu %s: %.0f mm and %.2f degrees from where it was (spread %.0f mm, %.2f degrees over %zu "
	        "pairs). Saved.",
	        key.first, key.second, placement->refined ? "placement refined" : "placed again",
	        m_vec3_len(placement->Txr_world_cam.position - placement->before.position) * 1000.0f,
	        RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(turned.w), 0.0f, 1.0f))), placement->mad_m * 1000.0f,
	        placement->mad_deg, placement->pairs);

	// Not from a calibration, so no fitness of its own.
	this->saveRig(0.0);
	u_notice_post(U_NOTICE_SENSORS, placement->refined ? "sensor-saved" : "sensor-placed",
	              rig_watch_camera_number(*this, key));
}

bool
ConstellationTracker::rigWatchLeavesOut(size_t mosaic_idx, size_t camera_idx)
{
	std::unique_lock<os::Mutex> lock(this->rig_watch.lock);
	return this->rig_watch.quarantined.has_value() &&
	       this->rig_watch.quarantined.value() == CameraKey{mosaic_idx, camera_idx};
}

bool
ConstellationTracker::needSensorCalibration()
{
	bool need_sensor_calibration = false;

	// Check if any cameras need calibration.
	for (const auto &mosaic : this->mosaics) {
		for (const auto &camera : mosaic->cameras) {
			if (!camera->locked_data.has_concrete_pose) {
				need_sensor_calibration = true;
			}
		}
	}

	return need_sensor_calibration;
}

bool
ConstellationTracker::recalibrate(bool automatic)
{
	{
		std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);
		/*
		 * Not on top of measuring already under way: the room's solve, a camera not placed yet, or one being placed
		 * again by the moved-camera watch (unless measuring by itself: the watch's placing is relative to a camera
		 * that is off too).
		 */
		bool busy = this->running_calibration || this->needSensorCalibration();
		if (!automatic) {
			std::unique_lock<os::Mutex> watch_lock(this->rig_watch.lock);
			busy = busy || this->rig_watch.quarantined.has_value();
		}
		if (busy) {
			CT_WARN(this, "Measuring the sensors again: measuring is already under way; asked again once it is done.");
			return false;
		}
		// Every camera unplaced, in the tracker and in the solve's job: no saved or partly wrong placement is kept.
		// Their placements first, for the room and the floor (see recalibration).
		this->recalibration.pose_before.clear();
		for (auto &mosaic : this->mosaics) {
			for (auto &camera : mosaic->cameras) {
				{
					std::unique_lock<os::Mutex> cam_lock(camera->processing_lock);
					if (camera->locked_data.has_concrete_pose && !camera->serial.empty()) {
						this->recalibration.pose_before[camera->serial] =
						    camera->locked_data.Txr_origin_cam;
					}
					camera->locked_data.has_concrete_pose = false;
				}
				this->rig_calibration->clearCameraPose(mosaic->index, camera->index);
			}
		}
		// checkSensorCalibration begins the collection again (dropping what an earlier one gathered) and says so.
		this->rig_calibration->endCollectingData();
		this->recalibration.pending = true;
		this->cameras_placed.store(0);
	}
	{
		std::unique_lock<os::Mutex> lock(this->rig_watch.lock);
		this->rig_watch.last.clear();
		this->rig_watch.window.clear();
		this->rig_watch.strikes.clear();
		this->rig_watch.last_median.clear();
		this->rig_watch.suspect_since.clear();
		this->rig_watch.apart.clear();
		this->rig_watch.apart_strikes.clear();
		this->rig_watch.tilt.clear();
		this->rig_watch.fusion.clear();
		this->rig_watch.quarantined.reset();
		this->rig_watch.refining.reset();
		this->rig_watch.candidates.clear();
	}
	{
		std::unique_lock<os::Mutex> lock(this->rig_check.lock);
		this->rig_check.last.clear();
		this->rig_check.differences.clear();
		this->rig_check.strikes = 0;
	}

	// The saved rig set aside (not deleted): stopped while measuring, the next start measures again too.
	char dir[PATH_MAX];
	if (u_file_get_config_dir(dir, sizeof(dir)) > 0) {
		const std::string path = std::string(dir) + "/" CT_RIG_SUBDIR "/" CT_RIG_FILENAME;
		const std::string aside = path + ".before-recalibration";
		if (rename(path.c_str(), aside.c_str()) == 0) {
			CT_INFO(this, "The saved rig is set aside as %s.", aside.c_str());
		}
	}

	{
		std::unique_lock<os::Mutex> lock(this->remeasure.lock);
		this->remeasure.tilts.clear();
		this->remeasure.since_ns = 0;
	}
	{
		std::unique_lock<os::Mutex> lock(this->level.lock);
		this->level.watch.reset();
		this->level.pending.reset();
	}
	// Asked for (or needed), measuring may start even where it was held back (replays of recorded placements).
	this->calibration_held = false;

	CT_WARN(this, "Measuring every sensor again, %s: move the headset around in view of every sensor.",
	        automatic ? "by itself (every sensor's view of the headset disagrees with its gravity)" : "on request");
	return true;
}

void
ConstellationTracker::feedRemeasure(timepoint_ns timestamp_ns, Device *device, uint32_t camera_key, float tilt_deg)
{
	/*
	 * Only while worn: a headset lying face down solves far off its gravity too (its LEDs seen from behind the
	 * lenses), and one left lying about must not drop the placements.
	 */
	if (!debug_get_bool_option_constellation_tracker_remeasure() || tilt_deg < 0.0f || !device->worn.load()) {
		std::unique_lock<os::Mutex> lock(this->remeasure.lock);
		this->remeasure.since_ns = 0;
		return;
	}
	// Tracked lately: the checks take its solves, nothing to measure.
	bool tracked = false;
	{
		std::lock_guard<os::Mutex> lock(device->data_lock);
		const auto &last = device->locked_data.last_known_pose;
		tracked = last.has_value() && timestamp_ns - last->timestamp_ns < CT_REMEASURE_UNTRACKED_NS;
	}

	std::string per_camera;
	{
		std::unique_lock<os::Mutex> lock(this->remeasure.lock);
		auto &mine = this->remeasure.tilts[camera_key];
		mine.emplace_back(timestamp_ns, tilt_deg);
		bool all_off = !tracked;
		size_t cameras = 0;
		for (auto &[key, q] : this->remeasure.tilts) {
			while (!q.empty() && (timestamp_ns - q.front().first > CT_REMEASURE_WINDOW_NS || q.size() > 400)) {
				q.pop_front();
			}
			if (q.empty()) {
				continue;
			}
			// A camera that sees the headset: enough solves, three quarters of them off by more than the checks.
			std::vector<float> v;
			v.reserve(q.size());
			for (const auto &e : q) {
				v.push_back(e.second);
			}
			std::sort(v.begin(), v.end());
			const float p25 = v[v.size() / 4];
			cameras++;
			char part[64];
			snprintf(part, sizeof(part), "%s%u/%u %.1f", per_camera.empty() ? "" : ", ", key / XRT_TRACKING_MAX_CAMS,
			         key % XRT_TRACKING_MAX_CAMS, (double)v[v.size() / 2]);
			per_camera += part;
			if (v.size() < CT_REMEASURE_MIN_SOLVES || p25 <= CT_REMEASURE_TILT_DEG) {
				all_off = false;
			}
		}
		all_off = all_off && cameras > 0;
		if (!all_off) {
			this->remeasure.since_ns = 0;
			return;
		}
		if (this->remeasure.since_ns == 0) {
			this->remeasure.since_ns = timestamp_ns;
			return;
		}
		if (timestamp_ns - this->remeasure.since_ns < CT_REMEASURE_HOLD_NS) {
			return;
		}
		this->remeasure.since_ns = 0;
	}
	if (!this->remeasure_requested.exchange(true)) {
		CT_WARN(this,
		        "Every sensor's view of the headset disagrees with its gravity beyond the checks (sensor and median "
		        "degrees: %s), and none of its solves is taken: the sensors were moved, measuring them again.",
		        per_camera.c_str());
	}
}

void
ConstellationTracker::applyRemeasure()
{
	if (!this->remeasure_requested.load()) {
		return;
	}
	// Another camera's thread may be at it: only one goes on.
	if (!this->remeasure_requested.exchange(false)) {
		return;
	}
	if (!this->recalibrate(true)) {
		// Measuring under way already: nothing more to do.
		std::unique_lock<os::Mutex> lock(this->remeasure.lock);
		this->remeasure.tilts.clear();
	}
}

/*
 * Quick placement: how many of the headset's solves place the first camera level, how many same-frame pairs another
 * one (as the moved-camera watch's first placement), and how far apart they may be.
 */
#define CT_QUICK_ORIGIN_SOLVES 90
#define CT_QUICK_PAIRS CT_RIG_WATCH_PLACE_PAIRS
/*!
 * A solve with fewer LEDs has a larger depth error: not used (the headset has about 20 in view of a camera, at the
 * distance of a seated player often fewer). Replay of a recorded seated session (measured at 20 s): 12 LEDs placed the
 * second camera after 63 s of frames within 4.5 mm, 8 LEDs after 32 s within 10 mm (the room's solve: 12 mm); the
 * head's turn (below) is what keeps a still headset from placing it wrong.
 */
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_quick_min_leds, "CONSTELLATION_TRACKER_QUICK_MIN_LEDS", 8)
//! The headset turned at least this far (degrees) over a camera's samples (see quick.turned_deg).
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_quick_turn_deg, "CONSTELLATION_TRACKER_QUICK_TURN_DEG", 20)
//! Solves kept per camera to find another camera's of the same frame.
#define CT_QUICK_RECENT_NS (300 * U_TIME_1MS_IN_NS)

void
ConstellationTracker::startQuickPlace()
{
	if (!debug_get_bool_option_constellation_tracker_quick_place()) {
		return;
	}
	std::unique_lock<os::Mutex> lock(this->quick.lock);
	this->quick.active = true;
	this->quick.ready = false;
	this->quick.started_ns = os_monotonic_get_ns();
	this->quick.unplaced.clear();
	this->quick.recent.clear();
	this->quick.downs.clear();
	this->quick.candidates.clear();
	this->quick.placed.clear();
	this->quick.first_turn.clear();
	this->quick.turned_deg.clear();
	this->quick.first_frame_ns = 0;
	this->quick.last_frame_ns = 0;
	this->quick.told_no_correction = false;
	for (const auto &mosaic : this->mosaics) {
		for (const auto &camera : mosaic->cameras) {
			if (!camera->locked_data.has_concrete_pose) {
				this->quick.unplaced.push_back({mosaic->index, camera->index});
			}
		}
	}
	CT_WARN(this, "Placing %zu sensor(s) from the headset in view of them.", this->quick.unplaced.size());
}

//! How far @p q is turned from the first orientation kept for @p key, keeping the largest so far.
static float
quick_turned(std::map<std::pair<size_t, size_t>, std::optional<xrt_quat>> &first,
             std::map<std::pair<size_t, size_t>, float> &turned,
             const std::pair<size_t, size_t> &key,
             const xrt_quat &q)
{
	std::optional<xrt_quat> &f = first[key];
	if (!f.has_value()) {
		f = q;
	}
	xrt_quat d;
	math_quat_unrotate(&f.value(), &q, &d);
	const float deg = (float)RAD_TO_DEG(2.0f * acosf(CLAMP(fabsf(d.w), 0.0f, 1.0f)));
	float &max_deg = turned[key];
	max_deg = std::max(max_deg, deg);
	return max_deg;
}

void
ConstellationTracker::quickPlace(const Device *device,
                                 size_t mosaic_idx,
                                 size_t camera_idx,
                                 const CameraSample &sample,
                                 const xrt_pose &Tcv_cam_device,
                                 uint32_t leds)
{
	const CameraKey self{mosaic_idx, camera_idx};
	std::unique_lock<os::Mutex> lock(this->quick.lock);
	auto &q = this->quick;
	if (!q.active || q.ready) {
		return;
	}
	const bool to_place = std::find(q.unplaced.begin(), q.unplaced.end(), self) != q.unplaced.end();
	size_t cameras = 0;
	for (const auto &mosaic : this->mosaics) {
		cameras += mosaic->cameras.size();
	}
	// None placed when it started: the first one is placed level, at the origin.
	const bool all_unplaced = q.unplaced.size() == cameras;

	if (leds < (uint32_t)debug_get_num_option_constellation_tracker_quick_min_leds()) {
		return;
	}
	QuickSolve solve;
	solve.timestamp_ns = sample.timestamp_ns;
	solve.leds = leds;
	if (q.first_frame_ns == 0) {
		q.first_frame_ns = sample.timestamp_ns;
	}
	q.last_frame_ns = sample.timestamp_ns;
	math_pose_convert_from_opencv(&Tcv_cam_device, &solve.Txr_cam_device);
	if (!to_place) {
		solve.Txr_world_cam = sample.Txr_world_cam;
	} else if (q.placed.count(self) != 0) {
		solve.Txr_world_cam = q.placed[self];
	}

	/*
	 * The first camera, when none is placed: level by the headset's down as this camera sees it, from its IMU
	 * (the same down the gravity check compares solves with), at the origin.
	 */
	if (all_unplaced && q.placed.empty()) {
		// The headset's down at this frame, in its own frame (OpenCV), as the gravity check has it, corrected by
		// what the cameras showed of its IMU's seat (down_learn); without that, the room's solve places them.
		std::optional<xrt_vec3> down = device->gravityAt(sample.timestamp_ns);
		{
			std::unique_lock<os::Mutex> learn_lock(this->down_learn.lock);
			const auto &l = this->down_learn;
			if (l.count < 500) {
				if (!q.told_no_correction) {
					q.told_no_correction = true;
					CT_WARN(this, "No headset down correction learned yet (%zu solves): the sensors are placed by "
					              "the room's solve.",
					        l.count);
				}
				down.reset();
			} else if (down.has_value()) {
				xrt_quat correction;
				const xrt_vec3 from = m_vec3_normalize(l.imu_down), to = m_vec3_normalize(l.room_down);
				math_quat_from_vec_a_to_vec_b(&from, &to, &correction);
				math_quat_rotate_vec3(&correction, &down.value(), &down.value());
			}
		}
		if (down.has_value() && m_vec3_len(down.value()) > 0.0f) {
			xrt_vec3 in_cam;
			math_quat_rotate_vec3(&Tcv_cam_device.orientation, &down.value(), &in_cam);
			std::vector<xrt_vec3> &downs = q.downs[self];
			downs.push_back(m_vec3_normalize(in_cam));
			if (downs.size() > 4 * CT_QUICK_ORIGIN_SOLVES) {
				downs.erase(downs.begin());
			}
			// Down is the solve's orientation with the IMU's: a still headset gives it as well as a turning one.
			const float turned =
			    quick_turned(q.first_turn, q.turned_deg, self, solve.Txr_cam_device.orientation);
			if (downs.size() >= CT_QUICK_ORIGIN_SOLVES) {
				xrt_vec3 sum = XRT_VEC3_ZERO;
				for (const xrt_vec3 &d : downs) {
					sum = sum + d;
				}
				const xrt_vec3 mean = m_vec3_normalize(sum);
				float spread_deg = 0.0f;
				for (const xrt_vec3 &d : downs) {
					spread_deg = std::max(
					    spread_deg, (float)RAD_TO_DEG(acosf(CLAMP(m_vec3_dot(d, mean), -1.0f, 1.0f))));
				}
				// The tracker's world is level, down is +Y in OpenCV: turned so the camera's down is it.
				xrt_pose Tcv_world_cam = XRT_POSE_IDENTITY;
				const xrt_vec3 world_down = {0.0f, 1.0f, 0.0f};
				math_quat_from_vec_a_to_vec_b(&mean, &world_down, &Tcv_world_cam.orientation);
				xrt_pose Txr_world_cam;
				math_pose_convert_from_opencv(&Tcv_world_cam, &Txr_world_cam);
				q.placed[self] = Txr_world_cam;
				solve.Txr_world_cam = Txr_world_cam;
				// Against the camera's previous placement, when there was one: the down it implies (OpenCV).
				const auto &camera = this->mosaics[mosaic_idx]->cameras[camera_idx];
				const auto before = this->recalibration.pose_before.find(camera->serial);
				if (before != this->recalibration.pose_before.end()) {
					xrt_pose Tcv_before;
					math_pose_convert_from_opencv(&before->second, &Tcv_before);
					xrt_quat inv;
					math_quat_invert(&Tcv_before.orientation, &inv);
					const xrt_vec3 cv_y = {0.0f, 1.0f, 0.0f};
					xrt_vec3 expected;
					math_quat_rotate_vec3(&inv, &cv_y, &expected);
					CT_WARN(this,
					        "Sensor %zu/%zu: the headset's down in its frame %.3f %.3f %.3f; its previous "
					        "placement's world +y there %.3f %.3f %.3f.",
					        mosaic_idx, camera_idx, mean.x, mean.y, mean.z, expected.x, expected.y, expected.z);
				}
				CT_WARN(this,
				        "Sensor %zu/%zu placed first, level by the headset's down over %zu solves, the headset "
				        "turned %.0f degrees over them (within %.1f degrees).",
				        mosaic_idx, camera_idx, downs.size(), turned, spread_deg);
				// The other cameras' turns counted in their own frames so far: the pairs count in the world's.
				q.first_turn.clear();
				q.turned_deg.clear();
			}
		}
	}

	// Pairs of the same frame with a camera placed (or just placed), for the camera that is not.
	for (const auto &[other, solves] : q.recent) {
		if (other == self) {
			continue;
		}
		for (auto it = solves.rbegin(); it != solves.rend(); ++it) {
			if (it->timestamp_ns != solve.timestamp_ns) {
				continue;
			}
			const QuickSolve *anchor = nullptr;
			const QuickSolve *moving = nullptr;
			CameraKey moving_key;
			std::optional<xrt_pose> anchor_world;
			const bool self_placed = solve.Txr_world_cam.has_value();
			const std::optional<xrt_pose> other_world =
			    it->Txr_world_cam.has_value() ? it->Txr_world_cam
			    : q.placed.count(other) != 0  ? std::optional<xrt_pose>(q.placed.at(other))
			                                  : std::nullopt;
			if (self_placed && !other_world.has_value() && q.placed.count(other) == 0) {
				anchor = &solve;
				anchor_world = solve.Txr_world_cam;
				moving = &*it;
				moving_key = other;
			} else if (!self_placed && other_world.has_value() && q.placed.count(self) == 0) {
				anchor = &*it;
				anchor_world = other_world;
				moving = &solve;
				moving_key = self;
			}
			if (anchor == nullptr ||
			    std::find(q.unplaced.begin(), q.unplaced.end(), moving_key) == q.unplaced.end()) {
				break;
			}
			xrt_pose Txr_world_device;
			math_pose_transform(&anchor_world.value(), &anchor->Txr_cam_device, &Txr_world_device);
			xrt_pose Txr_device_cam;
			math_pose_invert(&moving->Txr_cam_device, &Txr_device_cam);
			xrt_pose candidate;
			math_pose_transform(&Txr_world_device, &Txr_device_cam, &candidate);
			std::vector<xrt_pose> &candidates = q.candidates[moving_key];
			candidates.push_back(candidate);
			// The headset's orientation in the world over the pairs: they need it turned through enough views.
			const float turned =
			    quick_turned(q.first_turn, q.turned_deg, moving_key, Txr_world_device.orientation);
			if (candidates.size() > 4 * CT_QUICK_PAIRS) {
				candidates.erase(candidates.begin());
			}
			if (candidates.size() >= CT_QUICK_PAIRS &&
			    turned >= (float)debug_get_num_option_constellation_tracker_quick_turn_deg()) {
				float mad_m = 0.0f;
				float mad_deg = 0.0f;
				const xrt_pose placed = rig_watch_median_pose(candidates, mad_m, mad_deg);
				if (mad_m <= CT_RIG_WATCH_PLACE_MAD_M && mad_deg <= CT_RIG_WATCH_PLACE_MAD_DEG) {
					q.placed[moving_key] = placed;
					CT_WARN(this,
					        "Sensor %zu/%zu placed from %zu frames shared with a placed one, the headset turned "
					        "%.0f degrees over them (spread %.0f mm, %.2f degrees).",
					        moving_key.first, moving_key.second, candidates.size(), turned, mad_m * 1000.0f,
					        mad_deg);
				} else {
					CT_WARN(this,
					        "Sensor %zu/%zu: %zu frames shared with a placed one disagree (spread %.0f mm, %.2f "
					        "degrees, the headset turned %.0f degrees): collecting again.",
					        moving_key.first, moving_key.second, candidates.size(), mad_m * 1000.0f, mad_deg,
					        turned);
				}
				candidates.clear();
				q.first_turn.erase(moving_key);
				q.turned_deg.erase(moving_key);
			}
			break;
		}
	}

	// Kept for the other cameras' solves of the same frame.
	std::deque<QuickSolve> &mine = q.recent[self];
	mine.push_back(solve);
	while (!mine.empty() && solve.timestamp_ns - mine.front().timestamp_ns > CT_QUICK_RECENT_NS) {
		mine.pop_front();
	}

	bool all = !q.unplaced.empty();
	for (const CameraKey &key : q.unplaced) {
		all = all && q.placed.count(key) != 0;
	}
	q.ready = all;
}

bool
ConstellationTracker::checkSensorCalibration()
{
	std::unique_lock<os::Mutex> lock(this->rig_calibration_lock);

	if (this->running_calibration || this->calibration_held) {
		return true;
	}

	// Quick placement decided: in use from its own thread, as the room's solve.
	{
		std::unique_lock<os::Mutex> quick_lock(this->quick.lock);
		if (this->quick.active && this->quick.ready) {
			quick_lock.unlock();
			this->startCalibrationThread(run_quick_place_func, "quick placement");
			return true;
		}
	}

	size_t num_cameras = 0;
	for (const auto &mosaic : this->mosaics) {
		num_cameras += mosaic->cameras.size();
	}

	// No calibration needed
	if (!this->needSensorCalibration()) {
		return true;
	}

	auto &calib = this->rig_calibration;

	// Start collecting data if we haven't
	if (!calib->isCollectingData()) {
		calib->beginCollectingData();
		u_notice_post(U_NOTICE_CALIBRATION, "calibration-start", 0);
		this->startQuickPlace();

		if (this->data_recorder) {
			this->data_recorder->recordCalibrationEvent(
			    DatasetCalibrationEvent::begin(os_monotonic_get_ns()));
		}

		// Enable the first device, disable all others, for the purposes of clean data.
		bool is_first = true;
		for (auto &device : this->devices) {
			device->enabled.store(is_first);
			is_first = false;
		}
	}

	size_t num_samples;
	if (!calib->numSamplesForBestDevice(num_samples)) {
		// Nothing yet
		return true;
	}

	// @todo Figure out a good number for this, it probably is going to depend on time, "seen by multiple cameras at
	//       once", and other factors rather than pure sample count?
	if (num_samples < (num_cameras * 1500)) {
		return true;
	}

	/*
	 * A solve only places cameras that saw the device, so wait for one that still needs placing to have. Without
	 * this, a camera that can't see the device would have a solve run for nothing every time the count is reached.
	 */
	std::map<std::pair<size_t, size_t>, size_t> camera_samples;
	calib->cameraSampleCountsForBestDevice(camera_samples);
	bool any_unplaced_ready = false;
	for (const auto &mosaic : this->mosaics) {
		for (const auto &camera : mosaic->cameras) {
			if (!camera->locked_data.has_concrete_pose &&
			    camera_samples[{mosaic->index, camera->index}] >= calib->minCameraSamples()) {
				any_unplaced_ready = true;
			}
		}
	}
	if (!any_unplaced_ready) {
		// Start over now and then, so the samples of the cameras that do see the device don't pile up forever.
		if (num_samples > num_cameras * 1500 * 10) {
			calib->endCollectingData();
			calib->beginCollectingData();
		}
		return true;
	}

	/*
	 * Wait for the device to have moved around, as seen by every camera that is ready to be placed. A device
	 * seen in one spot leaves a camera's height and tilt poorly pinned down.
	 */
	{
		std::map<std::pair<size_t, size_t>, std::pair<double, double>> spread;
		calib->poseSpreadForBestDevice(spread);
		double min_position_m = 0.0;
		double min_rotation_deg = 0.0;
		calib->minPoseSpread(min_position_m, min_rotation_deg);

		bool moved_enough = true;
		std::string progress;
		for (const auto &mosaic : this->mosaics) {
			for (const auto &camera : mosaic->cameras) {
				const std::pair<size_t, size_t> key = {mosaic->index, camera->index};
				if (camera->locked_data.has_concrete_pose ||
				    camera_samples[key] < calib->minCameraSamples()) {
					continue;
				}
				const auto [position_m, rotation_deg] = spread[key];
				if (position_m < min_position_m || rotation_deg < min_rotation_deg) {
					moved_enough = false;
				}
				char buf[128];
				snprintf(buf, sizeof(buf), " camera %zu/%zu: %.2f m, %.0f deg;", key.first, key.second,
				         position_m, rotation_deg);
				progress += buf;
			}
		}

		if (!moved_enough) {
			const timepoint_ns now_ns = os_monotonic_get_ns();
			if (now_ns - this->last_calibration_progress_log_ns > 5LL * U_TIME_1S_IN_NS) {
				this->last_calibration_progress_log_ns = now_ns;
				CT_INFO(this,
				        "Sensor calibration waiting for the headset to move around (needs %.2f m and "
				        "%.0f deg "
				        "as seen by each sensor):%s",
				        min_position_m, min_rotation_deg, progress.c_str());
			}

			if (num_samples > num_cameras * 1500 * 20) {
				calib->endCollectingData();
				calib->beginCollectingData();
			}
			return true;
		}
	}

	// Stop collecting data
	calib->endCollectingData();

	if (this->data_recorder) {
		this->data_recorder->recordCalibrationEvent(
		    DatasetCalibrationEvent::end(os_monotonic_get_ns(), num_samples));
	}

	this->startCalibrationThread(run_room_calibration_func, "the room's solve");

	return true;
}

void
ConstellationTracker::startCalibrationThread(void *(*func)(void *), const char *what)
{
	if (!this->rig_calibration_thread.initialized) {
		os_thread_helper_init(&this->rig_calibration_thread);
	} else {
		/*
		 * The last one's thread returned (running_calibration is set false as it ends) but the helper still
		 * counts it as running and refused a second start: the second measuring of a session never finished.
		 * Joined first.
		 */
		os_thread_helper_stop_and_wait(&this->rig_calibration_thread);
	}
	this->running_calibration = true;
	if (os_thread_helper_start(&this->rig_calibration_thread, func, this) != 0) {
		this->running_calibration = false;
		CT_ERROR(this, "Could not start %s: measuring again later.", what);
	}
}

void
ConstellationTracker::setupVariableTracking()
{
	u_var_add_root(this, "Constellation Tracker", true);
	u_var_add_log_level(this, &this->log_level, "Log Level");

	uint32_t mosaic_idx = 0;
	for (auto &mosaic : this->mosaics) {
		uint32_t camera_idx = 0;
		for (auto &camera : mosaic->cameras) {
			auto str =
			    "Camera " + std::to_string(camera_idx) + " (Mosaic " + std::to_string(mosaic_idx) + ")";
			u_var_add_gui_header(this, nullptr, str.c_str());

			u_var_add_sink_debug(this, &camera->fast_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Fast)");
			u_var_add_sink_debug(this, &camera->slow_processing_thread_data.debug_sink,
			                     "Blob Debug Sink (Slow)");

			u_var_add_pose(this, &camera->locked_data.Txr_origin_cam, "Camera Origin Pose");
			u_var_add_bool(this, &camera->locked_data.has_concrete_pose, "Has Concrete Pose");

			camera->scribble_settings.setupDebugTracking(this);

			camera_idx++;
		}
		mosaic_idx++;
	}
}

t_constellation_device_id_t
ConstellationTracker::addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device)
{
	if (params->led_model.led_count > XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE) {
		throw std::runtime_error("Device has too many LEDs, maximum is " +
		                         std::to_string(XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE));
	}

	this->devices_changing.store(true);
	std::unique_lock lock(this->device_lock);
	this->devices_changing.store(false);

	if (this->devices.size() >= XRT_CONSTELLATION_MAX_DEVICES) {
		throw std::runtime_error("Maximum number of devices already added to constellation tracker");
	}

	t_constellation_device_id_t id = this->next_device_id++;

	this->devices.push_back(std::make_unique<Device>(this, params, device, id));

	CT_DEBUG(this, "Added device with ID %d to constellation tracker", id);

	const auto &new_device = this->devices.back();

	// Enable the device if there's no calibration needed
	if (!this->needSensorCalibration()) {
		new_device->enabled.store(true);
	}

	if (this->data_recorder) {
		this->data_recorder->recordDeviceInfo(*new_device);
	}

	new_device->sensor_fusion =
	    std::make_unique<sensor_fusion::SensorFusion>(id, new_device->search_model->led_model);
	new_device->sensor_fusion->deterministic = this->deterministic;
	new_device->sensor_fusion->head = params->head;

	for (auto &mosaic : this->mosaics) {
		for (auto &camera : mosaic->cameras) {
			// Add the camera to the sensor fusion job
			new_device->sensor_fusion->addCamera({
			    .mosaic_idx = mosaic->index,
			    .camera_idx = camera->index,
			    .params = camera->model,
			});
		}
	}

	return id;
}

void
ConstellationTracker::removeDevice(t_constellation_device_id_t device_id)
{
	this->devices_changing.store(true);
	std::unique_lock lock(this->device_lock);
	this->devices_changing.store(false);

	size_t index = 0;
	for (auto &device : this->devices) {
		if (device->id == device_id) {
			break;
		}
		index++;
	}

	if (index == this->devices.size()) {
		throw std::invalid_argument("The device ID is not present in the device list.");
	}

	// Remove the device
	this->devices.erase(this->devices.begin() + index);
}

void
ConstellationTracker::getTrackedPose(t_constellation_device_id_t device_id,
                                     timepoint_ns when_ns,
                                     xrt_space_relation &out_relation,
                                     bool blended)
{
	out_relation = XRT_SPACE_RELATION_ZERO;

	std::shared_lock lock(this->device_lock);
	for (const auto &device : this->devices) {
		if (device->id != device_id) {
			continue;
		}

		timepoint_ns pose_ts = 0;
		xrt_space_relation relation;
		device->sensor_fusion->getTrackedPose(when_ns, pose_ts, relation, blended);

		// No flags just means pass directly
		if (relation.relation_flags == 0) {
			out_relation = relation;
		} else {
			// Predict to the requested time
			m_predict_relation(&relation, time_ns_to_s(when_ns - pose_ts), &out_relation);
		}

		if (this->data_recorder) {
			this->data_recorder->recordOutputPose(DatasetOutputPose{
			    .device_id = device_id,
			    .query_ns = os_monotonic_get_ns(),
			    .when_ns = when_ns,
			    .state_ns = pose_ts,
			    .relation = out_relation,
			});
		}

		return;
	}

	throw std::runtime_error("Device missing, failed to get tracked pose.");
}

}; // namespace xrt::tracking::constellation

using namespace xrt::tracking::constellation;

void *
constellation_tracker_camera_slow_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->slow_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->slow_processing_thread)) {
		// A frame handed over while the last one was searched is already here, its signal came before the wait.
		if (!camera->slow_processing_thread_data.sample.has_value()) {
			os_thread_helper_wait_locked(&camera->slow_processing_thread);
		}

		std::optional<CameraSample> maybe_sample = camera->slow_processing_thread_data.sample;
		camera->slow_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->slow_processing_thread);

		if (auto sample = maybe_sample) {
			const timepoint_ns start_ns = os_monotonic_get_ns();
			camera->processSampleSlow(*sample);
			const timepoint_ns end_ns = os_monotonic_get_ns();

			auto &stats = camera->slow_stats;
			const double ms = time_ns_to_ms_f(end_ns - start_ns);
			stats.runs++;
			stats.total_ms += ms;
			stats.max_ms = std::max(stats.max_ms, ms);
			if (stats.window_start_ns == 0) {
				stats.window_start_ns = start_ns;
			}
			if (end_ns - stats.window_start_ns >= U_TIME_1S_IN_NS) {
				auto &data = camera->slow_processing_thread_data;
				CT_INFO(
				    camera->tracker,
				    "Camera %zu: slow search ran %u time(s) in the last second, %.1f ms on average, "
				    "%.1f at "
				    "most, %u search(es) out of time; %u search(es) with no prior left out %.1f static "
				    "spot(s) on average; %" PRIu64 " frame(s) skipped while it ran.",
				    camera->index, stats.runs, stats.total_ms / stats.runs, stats.max_ms,
				    data.cs->num_timeouts + data.cs_blind->num_timeouts, data.blind_searches,
				    data.blind_searches > 0
				        ? static_cast<double>(data.blind_spots_left_out) / data.blind_searches
				        : 0.0,
				    camera->slow_skipped.exchange(0));
				data.cs->num_timeouts = 0;
				data.cs_blind->num_timeouts = 0;
				data.blind_searches = 0;
				data.blind_spots_left_out = 0;
				stats = {};
			}
		}

		os_thread_helper_lock(&camera->slow_processing_thread);
	}
	os_thread_helper_unlock(&camera->slow_processing_thread);

	return NULL;
}

void *
constellation_tracker_camera_fast_thread(void *ptr)
{
	Camera *camera = (Camera *)ptr;

	os_thread_helper_lock(&camera->fast_processing_thread);
	while (os_thread_helper_is_running_locked(&camera->fast_processing_thread)) {
		// A frame handed over while the last one was processed is already here, its signal came before the
		// wait.
		if (!camera->fast_processing_thread_data.sample.has_value()) {
			os_thread_helper_wait_locked(&camera->fast_processing_thread);
		}

		std::optional<CameraSample> maybe_sample = camera->fast_processing_thread_data.sample;
		camera->fast_processing_thread_data.sample.reset();

		os_thread_helper_unlock(&camera->fast_processing_thread);

		if (auto sample = maybe_sample) {
			const timepoint_ns start_ns = os_monotonic_get_ns();
			const bool need_slow = camera->processSampleFast(*sample);
			const timepoint_ns end_ns = os_monotonic_get_ns();

			// A frame takes far less than a frame period, a long one means the thread waited on something.
			if (end_ns - start_ns > 15 * U_TIME_1MS_IN_NS) {
				CT_WARN(camera->tracker,
				        "Camera %zu: fast processing of the frame at %" PRIi64
				        " took %.1f ms, until %" PRIi64 ".",
				        camera->index, sample->timestamp_ns, time_ns_to_ms_f(end_ns - start_ns),
				        end_ns);
			}

			if (need_slow) {
				CT_TRACE(camera->tracker,
				         "Fast processing for camera %p failed, deferring to slow thread",
				         (void *)camera);
				camera->deferSampleToSlowThread(*sample);
			}
		}

		os_thread_helper_lock(&camera->fast_processing_thread);
	}
	os_thread_helper_unlock(&camera->fast_processing_thread);

	return NULL;
}

void
Camera::classifyStatic(CameraSample &sample)
{
	const long static_ms = debug_get_num_option_constellation_tracker_static_spot_ms();
	sample.blob_static = {};
	if (static_ms <= 0) {
		this->static_last = 0;
		return;
	}

	const timepoint_ns now_ns = sample.timestamp_ns;
	const timepoint_ns static_ns = static_cast<timepoint_ns>(static_ms) * U_TIME_1MS_IN_NS;
	uint32_t count = 0;
	for (uint32_t i = 0; i < sample.blob_count; i++) {
		const t_blob &b = sample.blobs[i];
		const bool labelled = b.matched_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID;
		auto [it, added] = this->static_spots.try_emplace(b.blob_id);
		StaticSpot &spot = it->second;

		const float dx = b.center_distorted.x - spot.x;
		const float dy = b.center_distorted.y - spot.y;
		const bool moved = dx * dx + dy * dy > CT_STATIC_SPOT_PX * CT_STATIC_SPOT_PX;
		if (added || labelled || moved || now_ns - spot.last_ns > CT_STATIC_SPOT_GAP_NS ||
		    now_ns < spot.last_ns) {
			spot.x = b.center_distorted.x;
			spot.y = b.center_distorted.y;
			spot.since_ns = now_ns;
		} else {
			// Follows a slow drift, a reflection moves as the sun does.
			spot.x += 0.05f * dx;
			spot.y += 0.05f * dy;
		}
		spot.last_ns = now_ns;

		if (!labelled && now_ns - spot.since_ns >= static_ns) {
			sample.blob_static[i] = true;
			count++;
		}
	}
	this->static_last = count;

	if (now_ns - this->static_spots_pruned_ns > U_TIME_1S_IN_NS) {
		this->static_spots_pruned_ns = now_ns;
		for (auto it = this->static_spots.begin(); it != this->static_spots.end();) {
			it = now_ns - it->second.last_ns > U_TIME_1S_IN_NS ? this->static_spots.erase(it)
			                                                   : std::next(it);
		}
	}
}

void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo)
{
	Camera *camera = Camera::Get(tbs);
	ConstellationTracker *tracker = camera->tracker;

	CT_TRACE(tracker, "Received blob observation at %" PRIi64 " with %u blobs", tbo->timestamp_ns, tbo->num_blobs);

	// Frames come in at the sensor's rate, a gap means they were late or lost before reaching the tracker.
	const timepoint_ns now_ns = os_monotonic_get_ns();
	const timepoint_ns last_ns = camera->last_push_ns.exchange(now_ns);
	if (last_ns == 0) {
		camera->first_push_ns.store(now_ns);
	}
	if (last_ns != 0 && now_ns - last_ns > 30 * U_TIME_1MS_IN_NS) {
		/*
		 * Once, at its start: a sensor set to take its exposures in step with the headset's LEDs re-times its frames,
		 * and one or two are lost (the Oculus runtime's own capture shows the same, 48 ms, at that step). Anything
		 * longer, later, or a second time is frames lost on the way.
		 */
		const timepoint_ns first_ns = camera->first_push_ns.load();
		const bool at_start = now_ns - first_ns < static_cast<timepoint_ns>(3) * U_TIME_1S_IN_NS;
		if (at_start && now_ns - last_ns <= 150 * U_TIME_1MS_IN_NS && !camera->start_gap_seen.exchange(true)) {
			CT_INFO(tracker, "Camera %zu: %.1f ms between frames as it starts taking them in step with the headset.",
			        camera->index, time_ns_to_ms_f(now_ns - last_ns));
		} else {
			CT_WARN(tracker, "Camera %zu: no frame arrived for %.1f ms, until %" PRIi64 " (frame at %" PRIi64 ").",
			        camera->index, time_ns_to_ms_f(now_ns - last_ns), now_ns, tbo->timestamp_ns);
		}
	}

	// The sensors' state for status views, about once a second (one camera's thread does it).
	int64_t published_ns = tracker->status_published_ns.load();
	if (now_ns - published_ns > U_TIME_1S_IN_NS &&
	    tracker->status_published_ns.compare_exchange_strong(published_ns, now_ns)) {
		tracker->publishStatus(now_ns);
	}

	if (tbo->num_blobs == 0) {
		CT_TRACE(tracker, "No blobs in observation, skipping processing");
		return;
	}

	if (tracker->single_threaded) {
		auto sample = CameraSample(*tbo, camera);
		camera->classifyStatic(sample);

		// If we're in single-threaded mode, just process the sample immediately on the fast thread
		if (camera->processSampleFast(sample)) {
			CT_TRACE(tracker,
			         "Fast processing for camera %p failed in single-threaded mode, doing slow processing",
			         (void *)camera);

			camera->processSampleSlow(sample);
		}
	} else {
		// Send to the fast thread
		CameraSample sample(*tbo, camera);
		camera->classifyStatic(sample);

		uint64_t dropped = 0;
		os_thread_helper_lock(&camera->fast_processing_thread);
		{
			if (camera->fast_processing_thread_data.sample.has_value()) {
				dropped = camera->dropped_fast_samples.tick(now_ns);
			}

			camera->fast_processing_thread_data.sample = std::move(sample);
			os_thread_helper_signal_locked(&camera->fast_processing_thread);
		}
		os_thread_helper_unlock(&camera->fast_processing_thread);

		if (dropped > 0) {
			CT_WARN(tracker,
			        "Camera %zu: dropped %" PRIu64
			        " sample(s) in the last second, the tracker is running slow.",
			        camera->index, dropped);
		}
	}
}

void
constellation_tracker_camera_destroy(t_blob_sink *tbs)
{
	// do nothing, the constellation tracker will clean up the blob sinks when it is destroyed.
}

void
constellation_tracker_node_break_apart(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	tracker->stopCameraThreads();
}

void
constellation_tracker_node_destroy(xrt_frame_node *node)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(node);

	u_var_remove_root(tracker);

	delete tracker;
}

int
t_constellation_tracker_create(xrt_frame_context *xfctx,
                               t_constellation_tracker_params *params,
                               t_constellation_tracker **out_tracker)
{
	try {
		ConstellationTracker *tracker = new ConstellationTracker(params);

		// Add us to the frame context
		xrt_frame_context_add(xfctx, &tracker->node);

		*out_tracker = (t_constellation_tracker *)tracker;

		tracker->setupVariableTracking();
	} catch (const std::exception &e) {
		U_LOG_E("Failed to create constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_add_device(t_constellation_tracker *raw_tracker,
                                   t_constellation_tracker_device_params *params,
                                   t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		t_constellation_device_id_t device_id = tracker->addDevice(params, device);
		*out_device_id = device_id;
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to add device to constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

int
t_constellation_tracker_remove_device(t_constellation_tracker *raw_tracker, t_constellation_device_id_t device)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->removeDevice(device);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to remove device from constellation tracker: %s", e.what());
		return -1;
	}

	return 0;
}

xrt_tracking_origin *
t_constellation_tracker_get_tracking_origin(t_constellation_tracker *raw_tracker)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	return &tracker->tracking_origin;
}

int
t_constellation_tracker_get_tracked_pose(struct t_constellation_tracker *raw_tracker,
                                         t_constellation_device_id_t device,
                                         timepoint_ns when_ns,
                                         struct xrt_space_relation *out_relation)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->getTrackedPose(device, when_ns, *out_relation);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to get the tracked pose for a device from constellation tracker: %s",
		         e.what());
		return -1;
	}

	return 0;
}

void
t_constellation_tracker_set_device_worn(struct t_constellation_tracker *raw_tracker,
                                        t_constellation_device_id_t device_id,
                                        bool worn)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);
	std::shared_lock lock(tracker->device_lock);
	for (const auto &device : tracker->devices) {
		if (device->id == device_id) {
			if (device->worn.exchange(worn) != worn) {
				CT_INFO(tracker, "Device %d %s.", device_id, worn ? "worn" : "not worn");
			}
			return;
		}
	}
}

int
t_constellation_tracker_cameras_placed(struct t_constellation_tracker *raw_tracker)
{
	return ConstellationTracker::Get(raw_tracker)->cameras_placed.load();
}

bool
t_constellation_tracker_device_tracked(struct t_constellation_tracker *raw_tracker, t_constellation_device_id_t device_id)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	std::shared_lock lock(tracker->device_lock);
	for (const auto &device : tracker->devices) {
		if (device->id == device_id) {
			return device->enabled.load();
		}
	}
	return false;
}

int
t_constellation_tracker_get_solved_pose(struct t_constellation_tracker *raw_tracker,
                                        t_constellation_device_id_t device,
                                        timepoint_ns when_ns,
                                        struct xrt_space_relation *out_relation)
{
	ConstellationTracker *tracker = ConstellationTracker::Get(raw_tracker);

	try {
		tracker->getTrackedPose(device, when_ns, *out_relation, false);
	} catch (const std::exception &e) {
		CT_ERROR(tracker, "Failed to get the solved pose for a device from constellation tracker: %s",
		         e.what());
		return -1;
	}

	return 0;
}
