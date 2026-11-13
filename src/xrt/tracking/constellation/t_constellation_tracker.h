// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Header defining the constellation tracker parameters and functions.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_tracking.h"

#include "tracking/t_constellation.h"

#include "tracking/t_tracking.h"


#ifdef __cplusplus
extern "C" {
#endif

#define XRT_CONSTELLATION_MAX_TRACKING_MOSAICS (1)

struct t_constellation_tracker;

/*!
 * @public @memberof t_constellation_tracker

 * A constellation tracker camera is a single camera that the constellation tracker will use to track devices. The
 * constellation tracker will provide the blob sink for this camera.
 */
struct t_constellation_tracker_camera
{
	//! The calibration for this camera
	struct t_camera_calibration calibration;

	//! The position of this camera, in the mosaic's tracking origin.
	struct xrt_pose pose_in_origin;
	/*
	 * Whether this camera has a concrete pose in the tracking origin, or if we don't know the current position and
	 * need to compute it.
	 */
	bool has_concrete_pose;

	/*!
	 * A stable identifier for the physical camera, such as its USB serial number, or empty if there is none.
	 * The poses a rig calibration finds for cameras with one are saved, and restored when a tracker is created
	 * with the same cameras again, unless the camera already has a concrete pose.
	 */
	char serial[32];

	/*!
	 * The blob sink for this camera, this is an out parameter filled in by the constellation tracker, and you are
	 * expected to pass this to your blobwatch implementation.
	 */
	struct t_blob_sink *blob_sink;
};

/*!
 * @public @memberof t_constellation_tracker
 *
 * A constellation tracker camera mosaic is a set of cameras that may or may not be physically attached, but
 * importantly, they all fire at the same time and are synchronized with each other.
 */
struct t_constellation_tracker_camera_mosaic
{
	/*
	 * The constellation tracking source for this mosaic, if any. This is used as the origin of any cameras in the
	 * mosaic.
	 */
	struct t_constellation_tracker_tracking_source *tracking_origin;

	//! The cameras in this mosaic, with their blob sinks filled in by the constellation tracker.
	struct t_constellation_tracker_camera cameras[XRT_TRACKING_MAX_CAMS];
	//! The number of cameras in this mosaic.
	size_t num_cameras;
};

/*!
 * @public @memberof t_constellation_tracker
 *
 * Parameters for adding a device to the constellation tracker.
 */
struct t_constellation_tracker_device_params
{
	//! The constellation pattern for this device.
	struct t_constellation_tracker_led_model led_model;

	/*
	 * An optional tracking source to give the constellation tracker extra information to better throw out bad
	 * guesses when finding the device.
	 */
	struct t_constellation_tracker_tracking_source *tracking_source;

	/*!
	 * The IMU sink for this device. This pointer is filled in by the constellation tracker. After adding the device
	 * to the constellation tracker, drivers should send IMU samples to this sink.
	 */
	struct xrt_imu_sink *imu_sink;

	/*!
	 * Optional: lit LEDs on an adjustable part of the device, such as the Rift CV1's rear headband, at their
	 * factory place (same axes as @ref led_model) and not in @ref led_model. The tracker measures where the part
	 * really sits from frames in which the rest of the device is tracked, then tracks its LEDs with the device's
	 * own, and keeps checking the place. Only the LEDs are read; led_count 0 for none.
	 */
	struct t_constellation_tracker_led_model loose_part;

	//! A stable identifier of the device, such as its serial number, to save the part's measured place under.
	char serial[32];

	/*!
	 * The device is worn on the head: unseen by every sensor, the pose applications get is held, turning about the
	 * neck, see SensorFusion::getTrackedPose. Controllers leave it false.
	 */
	bool head;
};

enum t_constellation_tracker_flags
{
	T_CONSTELLATION_TRACKER_FLAGS_NONE = 0,
	/*!
	 * Whether the constellation tracker should be deterministic.
	 * Turn this on if you require consistent results, at the cost of performance.
	 */
	T_CONSTELLATION_TRACKER_FLAGS_DETERMINISTIC = 1 << 0,
};

/*!
 * @public @memberof t_constellation_tracker
 */
struct t_constellation_tracker_params
{
	//! Flags for the constellation tracker, see t_constellation_tracker_flags
	enum t_constellation_tracker_flags flags;
	struct t_constellation_tracker_camera_mosaic mosaics[XRT_CONSTELLATION_MAX_TRACKING_MOSAICS];
	size_t num_mosaics;
};

/*!
 * @public @memberof t_constellation_tracker
 */
int
t_constellation_tracker_create(struct xrt_frame_context *xfctx,
                               struct t_constellation_tracker_params *params,
                               struct t_constellation_tracker **out_tracker);

int
t_constellation_tracker_add_device(struct t_constellation_tracker *tracker,
                                   struct t_constellation_tracker_device_params *params,
                                   struct t_constellation_tracker_device *device,
                                   t_constellation_device_id_t *out_device_id);

int
t_constellation_tracker_remove_device(struct t_constellation_tracker *tracker, t_constellation_device_id_t device);

struct xrt_tracking_origin *
t_constellation_tracker_get_tracking_origin(struct t_constellation_tracker *tracker);

int
t_constellation_tracker_get_tracked_pose(struct t_constellation_tracker *tracker,
                                         t_constellation_device_id_t device,
                                         timepoint_ns when_ns,
                                         struct xrt_space_relation *out_relation);

/*!
 * As @ref t_constellation_tracker_get_tracked_pose, without the blending in of the latest solve's correction: the pose
 * to search for the device's LEDs with, where the tracked pose is the one to show.
 */
int
t_constellation_tracker_get_solved_pose(struct t_constellation_tracker *tracker,
                                        t_constellation_device_id_t device,
                                        timepoint_ns when_ns,
                                        struct xrt_space_relation *out_relation);

/*!
 * Whether the tracker looks for the device with its cameras now: not while the room is measured (a camera not placed
 * yet, or all of them measured again), when only the headset is tracked. Its last place is then no place to show it.
 */
bool
t_constellation_tracker_device_tracked(struct t_constellation_tracker *tracker, t_constellation_device_id_t device);

/*!
 * How many cameras are placed and in use; 0 while the room is measured (the first start, or every sensor measured
 * again), when no camera gives a pose in the room and a device's orientation is its gyro's alone; -1 not known yet.
 */
int
t_constellation_tracker_cameras_placed(struct t_constellation_tracker *tracker);

/*!
 * Whether the device is worn, from a headset's wear sensor: its adjustable part (the rear headband) is only measured
 * and checked while it is, since a headband sits elsewhere off the head. Unknown counts as worn.
 */
void
t_constellation_tracker_set_device_worn(struct t_constellation_tracker *tracker,
                                        t_constellation_device_id_t device,
                                        bool worn);

/*!
 * Whether a controller's solves steadily disagree with its IMU on which way is down (RiftLinux): only counted, nothing
 * in tracking reads it. See t_constellation_tracker_imu_advice.
 */
struct t_constellation_imu_advice
{
	//! A calibration would help: it disagrees in enough orientations, seen so by every sensor, the headset not.
	bool advised;
	//! Orientations it steadily disagrees in.
	uint32_t steady_orientations;
	//! Sensors that saw enough of it at rest, all disagreeing with its IMU.
	uint32_t sensors;
	//! Readings counted, and those off by more than the limit, from those sensors.
	uint32_t samples;
	uint32_t off;
};

//! How many LEDs the sensors matched for a device just now, 0 when it has not been seen lately: for guiding a user.
uint32_t
t_constellation_tracker_imu_calibration_leds(struct t_constellation_tracker *tracker,
                                             t_constellation_device_id_t device);

/*!
 * Reads the counts behind a calibration's advice: resting, from many LEDs, a few a second at most.
 *
 * @param device    The controller.
 * @param reference The headset: a sensor that also sees it off from the IMU's down is misplaced, the controller is fine.
 *
 * @return 0, or -1 for no such devices.
 */
int
t_constellation_tracker_imu_advice(struct t_constellation_tracker *tracker,
                                   t_constellation_device_id_t device,
                                   t_constellation_device_id_t reference,
                                   struct t_constellation_imu_advice *out_advice);

//! The counts behind t_constellation_tracker_imu_advice, in the log at info level.
void
t_constellation_tracker_imu_advice_log(struct t_constellation_tracker *tracker, t_constellation_device_id_t device);

/*!
 * A controller's IMU alignment learned in the background (RiftLinux): how its IMU sits against its LEDs and its
 * accelerometer's bias, used for the gravity check once it is clearly a fault (CONSTELLATION_TRACKER_IMU_ALIGN).
 *
 * @param out_rot     The turn taking the IMU's readings into the device's frame.
 * @param out_bias    The accelerometer's bias, m/s2, in the IMU's frame (taken off before the turn).
 * @param out_applied Whether it is in use.
 *
 * @return 0, or -1 for no such device.
 */
int
t_constellation_tracker_get_imu_align(struct t_constellation_tracker *tracker,
                                      t_constellation_device_id_t device,
                                      struct xrt_quat *out_rot,
                                      struct xrt_vec3 *out_bias,
                                      bool *out_applied);

/*!
 * An alignment learned in an earlier session, in use from now on until the solves say otherwise (then dropped), as the
 * Oculus runtime starts from what it kept and checks it.
 *
 * @return 0, or -1 for no such device.
 */
int
t_constellation_tracker_set_imu_align(struct t_constellation_tracker *tracker,
                                      t_constellation_device_id_t device,
                                      const struct xrt_quat *rot,
                                      const struct xrt_vec3 *bias);

/*!
 * A controller's IMU seat calibration (RiftLinux), see t_constellation_tracker_calibrate_imu.
 */
enum t_constellation_imu_calibration_state
{
	T_CONSTELLATION_IMU_CALIBRATION_IDLE = 0,
	//! Collecting: the device is to be turned slowly through many orientations in view of the sensors.
	T_CONSTELLATION_IMU_CALIBRATION_COLLECTING = 1,
	//! Done: @ref t_constellation_imu_calibration_status::correction is the result, for the driver to keep.
	T_CONSTELLATION_IMU_CALIBRATION_DONE = 2,
	//! Failed (not enough orientations in time, or the solves disagreed among themselves): nothing to keep.
	T_CONSTELLATION_IMU_CALIBRATION_FAILED = 3,
};

//! Why a calibration ended without a result.
enum t_constellation_imu_calibration_reason
{
	T_CONSTELLATION_IMU_CALIBRATION_REASON_NONE = 0,
	T_CONSTELLATION_IMU_CALIBRATION_REASON_CANCELLED = 1,
	//! Nothing new measured for a long time.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_NO_PROGRESS = 2,
	//! The solves' down directions do not agree among themselves.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_SOLVES_DISAGREE = 3,
	//! The correction came out too large to be how the IMU sits.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_TOO_LARGE = 4,
	//! The accelerometer's bias came out too large to trust.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_BIAS_TOO_LARGE = 5,
	/*!
	 * While it goes on (not an end): the pose just held with the trigger was turned away, as not the one shown (6: the
	 * opposite of it, near enough) or as nearly one already done (7). Cleared when the trigger is let go; the same pose is asked for again.
	 */
	T_CONSTELLATION_IMU_CALIBRATION_REASON_STEP_NOT_AS_SHOWN = 6,
	T_CONSTELLATION_IMU_CALIBRATION_REASON_STEP_DONE_BEFORE = 7,
	//! While it goes on: the trigger was pressed to go on from "come closer" before the sensors saw it well enough.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_NOT_CLOSE_ENOUGH = 8,
	//! The poses held, several times over, gave readings too unsettled for a result.
	T_CONSTELLATION_IMU_CALIBRATION_REASON_UNSETTLED = 9,
};

/*!
 * The orientations a calibration asks for. Each is the direction that is up, in the device's frame in OpenXR
 * coordinates: 0 +X, 1 -X, 2 +Y, 3 -Y, 4 +Z, 5 -Z, then the diagonals, with k = 0.577: 6 (k,k,k), 7 (k,k,-k), 8 (k,-k,k),
 * 9 (k,-k,-k), 10 (-k,k,k), 11 (-k,k,-k), 12 (-k,-k,k), 13 (-k,-k,-k).
 */
#define T_CONSTELLATION_IMU_ORIENTATIONS 14
/*!
 * The guided steps: first "come closer" (next_orientation -2, progress how well the sensors see it, the trigger goes
 * on once they see it well enough), then orientations 5, 4, 0, 1 in this order, each taken with the trigger.
 */

struct t_constellation_imu_calibration_status
{
	enum t_constellation_imu_calibration_state state;
	//! The round (1 on), each a fit of what the previous ones left.
	int round;
	/*!
	 * The pose asked for now (@ref next_orientation), held with the trigger: how much of what it needs it has, 0-100;
	 * 100 with no @ref reason when it is done, when it was turned away with one (6, 7).
	 */
	int progress;
	/*!
	 * The rotation the device's IMU samples are to be turned by (OpenXR coordinates in the device's frame), found so
	 * far: relative to the samples as they came when the calibration started, the rounds composed.
	 */
	struct xrt_quat correction;
	float correction_deg;
	/*!
	 * The accelerometer's bias (OpenXR coordinates in the device's frame, m/s²), found so far, on the same samples:
	 * an accelerometer sample becomes correction * (sample - accel_bias), a gyro sample correction * sample.
	 */
	struct xrt_vec3 accel_bias;
	//! How far the solves' down directions are from the corrected IMU's after the last fit, 90th percentile, degrees.
	float residual_p90_deg;
	//! Counts up whenever @ref correction changes, so that the driver applies each once.
	uint32_t generation;
	//! Why it ended, when it ended without a result.
	enum t_constellation_imu_calibration_reason reason;
	//! Over all rounds, 0-100; unlike @ref progress it does not start over.
	int overall_progress;
	//! The poses asked for: 1 when done this round, the one asked for now by its progress, the rest 0.
	float orientation_fill[T_CONSTELLATION_IMU_ORIENTATIONS];
	//! The orientation it rests in now, -1 when it does not rest.
	int orientation_now;
	//! The orientation to ask for next, -1 when it needs none.
	int next_orientation;
	//! Whether the device rests (with the trigger held: still enough for a hand).
	bool resting;
	//! Whether the trigger is held (t_constellation_tracker_imu_calibration_hold).
	bool held;
};

/*!
 * Starts (@p start) or cancels the IMU calibration of @p device: its accelerometer's bias, from the resting samples
 * having one length in every orientation (no sensor needed), and where its IMU sits against its LEDs, from the down
 * direction its solves show against its IMU's while it rests, over many orientations. While collecting,
 * the device's solves are checked against its IMU's down with a wide tolerance. The driver turns the device's IMU
 * samples by each round's correction as it comes (t_constellation_tracker_imu_calibration), and keeps the final one.
 */
int
t_constellation_tracker_calibrate_imu(struct t_constellation_tracker *tracker,
                                      t_constellation_device_id_t device,
                                      bool start);

/*!
 * The person holds the pose shown (@p held, the trigger): while held, the device's readings count towards that pose,
 * still enough for a hand; once it has enough it is done or turned away (the status says), and letting go goes on to
 * the next pose or asks for the same again.
 */
void
t_constellation_tracker_imu_calibration_hold(struct t_constellation_tracker *tracker,
                                             t_constellation_device_id_t device,
                                             bool held);

//! The state of @p device's IMU seat calibration; < 0 for no such device.
int
t_constellation_tracker_imu_calibration(struct t_constellation_tracker *tracker,
                                        t_constellation_device_id_t device,
                                        struct t_constellation_imu_calibration_status *out_status);

#ifdef __cplusplus
}
#endif
