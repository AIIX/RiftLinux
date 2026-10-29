// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Short notices for the user about what the tracking is doing (a sensor moved and is being placed again),
 *         passed on by whatever shows things to the user: the SteamVR driver hands them to SteamVR.
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//! What a notice is about, so that each kind can be switched off on its own.
enum u_notice_kind
{
	U_NOTICE_SENSORS = 0,
	U_NOTICE_CALIBRATION = 1,
	U_NOTICE_TRACKING = 2,
	//! The headset's picture (RiftLinux: a setting of the system that spoils it, e.g. NVIDIA and adaptive sync).
	U_NOTICE_DISPLAY = 3,
	//! The system around the driver (RiftLinux: real-time priority refused, RealtimeKit missing).
	U_NOTICE_SYSTEM = 4,
};

/*!
 * A notice: @p key names it, for example "sensor-moved", and @p number is what it is about, a sensor's number counted
 * from 1, 0 for none. The receiver words it.
 */
typedef void (*u_notice_sink_func_t)(void *ptr, enum u_notice_kind kind, const char *key, int number);

//! Sets who receives notices, NULL for nobody (the default: notices are dropped).
void
u_notice_set_sink(u_notice_sink_func_t func, void *ptr);

//! Posts a notice to the sink, if there is one. Safe from any thread; the sink is called on it.
void
u_notice_post(enum u_notice_kind kind, const char *key, int number);

/*!
 * The sensors' state for status views (RiftLinux: the Rift tab, the driver's socket), kept current by the tracker
 * about once a second.
 */
struct u_notice_sensors
{
	//! Sensors the tracker has.
	int found;
	//! Of them, sending frames now.
	int streaming;
	//! Of them, placed in the room and in use for tracking.
	int placed;
	//! A sensor is being placed (again) or the room measured.
	bool measuring;
};

//! Sets the sensors' state. Safe from any thread.
void
u_notice_set_sensors(const struct u_notice_sensors *sensors);

//! The sensors' state; false until a tracker set it.
bool
u_notice_get_sensors(struct u_notice_sensors *out_sensors);

/*!
 * Measures every sensor again from the user's movement, whatever was placed or saved before, as on a first start
 * (RiftLinux: the Rift tab's button, riftlinux-setup recalibrate). The tracker answers; true when measuring starts.
 */
typedef bool (*u_notice_recalibrate_func_t)(void *ptr);

//! Sets who answers u_notice_request_recalibration (the tracker), NULL for nobody. Waits for a call in progress.
void
u_notice_set_recalibrator(u_notice_recalibrate_func_t func, void *ptr);

//! Asks for every sensor to be measured again: false when nothing tracks, or a solve is running. Any thread.
bool
u_notice_request_recalibration(void);

//! Cameras a recalibration result can describe.
#define U_NOTICE_MAX_CAMERAS 8

/*!
 * What a recalibration changed, for whoever keeps the room and a floor measured from the tracking space (its origin is
 * a camera). Kept: the cameras that kept their place put the new measurement back where the room was, so the room and
 * the floor stay as they were. Else the tracking space moved by what nobody can tell: the room is placed on the user
 * again, and per camera its height after minus before, metres along gravity (the space is levelled), carries the
 * floor over: a camera that moved up or down moves the floor by as much in the new space, one that did not keeps it.
 */
struct u_notice_recalibration
{
	//! Counts up with every recalibration finished, so a reader sees a new one.
	uint32_t sequence;
	int count;
	//! Placed before and after, so its change is known.
	bool known[U_NOTICE_MAX_CAMERAS];
	float height_change_m[U_NOTICE_MAX_CAMERAS];
	//! The room is where it was (rl_room.hpp), by this many cameras that kept their place.
	bool kept;
	int anchors;
};

//! Set by the tracker when a recalibration is solved, before it posts "calibration-done". Any thread.
void
u_notice_set_recalibration(const struct u_notice_recalibration *result);

//! The last recalibration's result; false if there was none.
bool
u_notice_get_recalibration(struct u_notice_recalibration *out_result);


/*!
 * The headset's display dark at once, whatever the wear sensor says (RiftLinux: the process ends without the driver's
 * own shutdown, SteamVR's having stuck: no frozen picture left lit on the panel). Any thread; the headset's driver
 * acts on it at its next report.
 */
void
u_notice_request_display_off(void);

//! Whether u_notice_request_display_off was called.
bool
u_notice_display_off_requested(void);

/*!
 * The headset's picture black while it is not worn (RiftLinux, headset.not_worn "black"): its driver asks, the compositor
 * driving the headset's display shows pure black (no layers, no lens or panel processing) and says so. The display stays
 * switched on: switched off, the CV1 leaves HDMI, and the desktop rebuilds its monitors at every unplug and plug (GNOME
 * 48 on NVIDIA kept about 210 MiB of GPU memory each time). Any thread.
 */
void
u_notice_request_display_black(bool black);

//! Whether the headset's driver wants the picture black (u_notice_request_display_black).
bool
u_notice_display_black_requested(void);

//! The compositor: it shows the headset black now, or the picture again.
void
u_notice_set_display_black(bool black);

//! Whether a compositor shows the headset black.
bool
u_notice_display_black(void);

#ifdef __cplusplus
}
#endif
