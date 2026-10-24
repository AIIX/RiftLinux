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

#ifdef __cplusplus
}
#endif
