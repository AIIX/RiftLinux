// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Whether the tracker's room is level: the headset's gravity against the down its sensors' placements give.
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <cstddef>
#include <cstdint>

namespace xrt::tracking::constellation {

//! A window's outcome: each sensor's tilt, the IMU's seat in the headset, and how well they were told apart.
struct LevelResult
{
	static constexpr size_t kMaxCameras = 8;

	//! The rotation that levels the room (apply it to every sensor's place): the sensors' common tilt, undone.
	//! OpenCV world axes, radians.
	xrt_vec3 level_rotvec{0, 0, 0};
	//! The common tilt, and how far the farthest sensor's own tilt is from it, degrees.
	float tilt_deg{0};
	float spread_deg{0};
	//! The IMU's turn in the headset (its down against the LEDs'), OpenCV device axes.
	float mount_deg{0};
	//! How far the headset turned about the vertical within the window, and the fit's leftover, degrees.
	float yaw_spread_deg{0};
	float residual_deg{0};
	size_t solves{0};

	//! The sensors with enough solves in the window: their key (as given to LevelWatch::add), solves, tilt.
	size_t cameras{0};
	uint32_t camera_key[kMaxCameras]{};
	size_t camera_solves[kMaxCameras]{};
	float camera_tilt_deg[kMaxCameras]{};
};

/*!
 * Collects good solves of a device with an IMU: its orientation in the room (through the sensor's placement) and its
 * IMU's down in its own frame. Down from the IMU should be the room's down; when it is not, either the sensor's place
 * is tilted (the same in the world for every solve through that sensor, as when sensors were tilted while the driver
 * was off) or the IMU sits turned in the device (the same in the device for every solve). Turning the head tells the
 * two apart: a least squares fit of the IMU's seat and of each sensor's tilt, all small rotations, over a window of
 * solves. Sensors tilted alike tilt the room: levelled by turning them all back. A sensor tilted on its own is a moved
 * sensor (the moved-sensor watch's case), shown by a spread between the sensors' tilts.
 *
 * The Oculus runtime levels the same way (its log's "DoGravityAlignment": the tracked objects' gravity rotates all
 * sensors together, pitch and roll only).
 */
class LevelWatch
{
public:
	//! Solves per window.
	static constexpr size_t kWindow = 300;
	//! A sensor takes part in a window's result with at least this many of its solves.
	static constexpr size_t kMinCameraSolves = 60;

	/*!
	 * @param camera_key       which sensor the solve came through (any number, the same for the same sensor)
	 * @param Rcv_world_device the device's orientation in the room, OpenCV axes
	 * @param imu_down_cv      the IMU's down in the device's frame, OpenCV axes, unit length
	 * @return true when a window completed, its result in @p out
	 */
	bool
	add(uint32_t camera_key, const xrt_quat &Rcv_world_device, const xrt_vec3 &imu_down_cv, LevelResult &out);

	void
	reset();

private:
	static constexpr size_t kMaxCameras = LevelResult::kMaxCameras;
	static constexpr size_t kParams = 3 + 3 * kMaxCameras;

	double ata[kParams][kParams]{};
	double atb[kParams]{};
	size_t count{0};
	// The sensors seen in this window, by slot.
	uint32_t keys[kMaxCameras]{};
	size_t key_solves[kMaxCameras]{};
	size_t key_count{0};
	// Which way the device faced, by 10 degree bins about the vertical.
	bool yaw_bins[36]{};
	// For the leftover: the solves of the window, kept small (one in four).
	static constexpr size_t kKept = kWindow / 4 + 1;
	xrt_quat kept_r[kKept]{};
	xrt_vec3 kept_d[kKept]{};
	uint8_t kept_slot[kKept]{};
	size_t kept{0};
};

} // namespace xrt::tracking::constellation
