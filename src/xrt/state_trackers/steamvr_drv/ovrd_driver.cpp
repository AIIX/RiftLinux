// Copyright 2020,2024 Collabora, Ltd.
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Main driver code for @ref st_ovrd.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Christoph Haag <christoph.haag@collabora.com>
 * @author Daniel Willmott <web@dan-w.com>
 * @author Moshi Turner <moshiturner@protonmail.com>
 * @author Korcan Hussein <korcan.hussein@collabora.com>
 * @ingroup st_ovrd
 */

#include <ctime>
#include <cstring>
#include <cassert>
#include <array>
#include <thread>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <string>
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <vector>
#include <map>

#include "math/m_api.h"
#include "ovrd_log.hpp"
#include "openvr_driver.h"

extern "C" {
#include "ovrd_interface.h"

#include <math.h>

#include <math/m_space.h>
#include "os/os_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_tracking.h"
#include "util/u_builder_helpers.h"

#include "xrt/xrt_space.h"
#include "xrt/xrt_system.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_instance.h"

#include "b_ovrd_generated_bindings.h"
}
#include "math/m_vec3.h"
#include "xrt/xrt_config_os.h"

#include <chrono>

#ifdef XRT_OS_LINUX
#include <dlfcn.h>
#include <mutex>
#include <optional>
#include <climits>
#include <cstdlib>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include "util/u_notice.h"
#ifdef XRT_OS_LINUX
#include "util/u_linux.h"
#endif
#include "util/u_json.h"
#include "xrt/xrt_compositor.h"
#include "xrt/xrt_session.h"
#include "ovrd_direct_mode.hpp"
#include "ovrd_socket.hpp"
#include "riftlinux/rl_floor.hpp"
#include "riftlinux/rl_room.hpp"
#include "main/comp_main_interface.h"
#include "render/render_output_settings.h"
extern char **environ;
#if __has_include(<pulse/pulseaudio.h>)
#include <pulse/pulseaudio.h>
#define OVRD_HAVE_PULSE_HEADERS
#include "riftlinux/rl_audio_switch.hpp"
#endif
#endif

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#endif

//! When set, all controllers pretend to be Index controllers. Provides best
//! compatibility with legacy games due to steamvr's legacy binding for Index
//! controllers, but input mapping may be incomplete or not ideal.
DEBUG_GET_ONCE_BOOL_OPTION(emulate_index_controller, "STEAMVR_EMULATE_INDEX_CONTROLLER", false)

DEBUG_GET_ONCE_NUM_OPTION(scale_percentage, "XRT_COMPOSITOR_SCALE_PERCENTAGE", 140)

#define MODELNUM_LEN (XRT_DEVICE_NAME_LEN + 9) // "[Monado] "

#define OPENVR_BONE_COUNT 31

// Debug define(s), always off.
#undef DUMP_POSE
#undef DUMP_POSE_CONTROLLERS


/*
 * Controller
 */

struct MonadoInputComponent
{
	bool has_component;
	bool x;
	bool y;
};

struct SteamVRDriverControl
{
	const char *steamvr_control_path;
	vr::VRInputComponentHandle_t control_handle;
};

struct SteamVRDriverControlInput : SteamVRDriverControl
{
	enum xrt_input_type monado_input_type;
	enum xrt_input_name monado_input_name;

	struct MonadoInputComponent component;
};

struct SteamVRDriverControlOutput : SteamVRDriverControl
{
	enum xrt_output_type monado_output_type;
	enum xrt_output_name monado_output_name;
};

static void
copy_vec3(struct xrt_vec3 *from, double *to)
{
	to[0] = from->x;
	to[1] = from->y;
	to[2] = from->z;
}

static void
copy_quat(struct xrt_quat *from, vr::HmdQuaternion_t *to)
{
	to->x = from->x;
	to->y = from->y;
	to->z = from->z;
	to->w = from->w;
}

/*!
 * Where SteamVR's world sits in the driver's space: the tracking space's origin is a sensor metres away, which put
 * the user outside SteamVR Home's room until they pressed buttons. It is set once, from the HMD's first tracked pose,
 * so the user starts at the origin facing -Z, at the height the tracking gives; Room Setup works on top of it.
 */
//! Before the headset's first tracked position (the first sensor calibration): its eye height in SteamVR's room, m.
static constexpr float kStandingEyeHeightM = 1.6f;

static struct
{
	std::atomic<bool> set{false};
	/*!
	 * Placed again on the user once the head has settled (rl::room::Settle): after measuring the sensors again
	 * moved the room with no sensor left in place to tell where it went. Placed at once, the head's pose was still
	 * the one from before the new measurement and left the user off SteamVR's room, turned.
	 */
	std::atomic<bool> recentre{false};
	//! Guards the two below once they can change (a recentre), read by every pose.
	std::mutex lock;
	vr::HmdQuaternion_t rotation;
	double translation[3];
	/*!
	 * The floor, metres up from the driver's space: Monado's tracking has no floor (its origin is a sensor), and
	 * Room Setup, which would measure it, is not run. Set in the Rift tab ("Set floor": a controller on the floor),
	 * plus the user's own raise or lower (the tab's slider; seated games: from the recentred spot). Saved in
	 * settings.json {"tracking": {"floor_m", "height_cm"}}.
	 */
	std::atomic<float> floor_m{0.0f};
	std::atomic<float> height_m{0.0f};
	//! "Set floor" asked: the lower controller lies on the floor.
	std::atomic<bool> floor_request{false};
	//! A floor was set once (or carried over since): the head's heights then teach the standing eye height.
	std::atomic<bool> floor_known{false};
} g_world_from_driver;

/*!
 * Measuring the sensors again (the Rift tab's button, the driver's socket), so nothing has to be set again: the
 * tracker puts the new measurement back where the room was by the sensors that kept their place (rl_room.hpp), and
 * the room and the floor stay. When no sensor can tell, the world is placed on the user anew and the floor carried
 * over (rl_floor.hpp), from the sensors' height changes and the user's standing eye height, learned here while the
 * floor is known (settings.json {"tracking": {"eye_height_m"}}).
 */
static struct
{
	//! The Rift tab asked. RunFrame asks the tracker: never under the notices' lock, as the tracker posts notices
	//! under its own locks.
	std::atomic<bool> asked{false};
	//! The tracker is measuring on our request: its "calibration-done" carries the floor over.
	std::atomic<bool> measuring{false};
	//! The last measuring result handled (u_notice_recalibration::sequence): ours, or a first measure from nothing.
	uint32_t seen_sequence = 0;
	std::mutex lock;
	rl::floor::Keeper keeper;
	//! A floor decided, to be saved (RunFrame).
	bool floor_to_save = false;
} g_recal;

static double
floor_now_s()
{
	return (double)os_monotonic_get_ns() / 1e9;
}

//! Socket asked (riftlinux-setup, test tools): SteamVR quits the way its own Exit does, from RunFrame.
static std::atomic<bool> g_quit_requested{false};

//! Asks the tracker to measure every sensor again; true when it starts (logged either way).
static bool
request_recalibration(const char *from)
{
	const bool started = u_notice_request_recalibration();
	if (started) {
		g_recal.measuring.store(true);
	}
	ovrd_log("Sensors: measuring every sensor again, asked from %s: %s\n", from,
	         started ? "started" : "refused (no tracker, or the sensors are being solved right now)");
	return started;
}

//! The head, from the pose thread: its heights teach the eye height, or after a recalibration decide the floor.
static void
floor_watch_head(struct xrt_device *xdev, const struct xrt_space_relation *head);

//! SteamVR's dashboard is open (riftlinux-notices says so): the Oculus button then closes it.
static std::atomic<bool> g_dashboard_visible{false};
//! Until when the left Touch's menu button reads pressed although it is not: SteamVR's own way to close its
//! dashboard, which an application cannot.
static std::atomic<int64_t> g_dashboard_toggle_until_ns{0};

#ifdef XRT_OS_LINUX
/*!
 * Driver direct mode (ovrd_direct_mode.hpp): SteamVR's images shown by Monado's compositor. Null when off, which is
 * the default: SteamVR's compositor owns the display.
 */
static OvrdDirectMode *g_direct_mode = nullptr;
#endif

//! The headset reported as worn whatever its sensor says (direct mode capture runs, unattended).
static std::atomic<bool> g_report_worn{false};

/*!
 * The Rift tab's picture row: Auto-HDR on or off and its strength, applied at once to Monado's
 * compositor, which runs in this process in direct mode. Strength scales contrast and highlights, colour the colour
 * (vibrance): 50 % is the settings' strengths (or the defaults), 100 % twice those, 0 % none.
 */
static struct
{
	std::mutex lock;
	bool direct = false;
	bool on = false;
	//! Contrast and highlights, %.
	int strength = 50;
	//! Colour (vibrance), %.
	int colour = 50;
	//! Motion smoothing, switched live from the tab.
	bool smoothing = false;
	//! HDR Bright: HDR content placed brighter on the panel.
	bool bright = false;
	struct render_output_settings base = {};
} g_picture;

//! Whether Monado's compositor shows HDR content now (1) or not (0): the Rift tab says so at HDR Bright.
static std::atomic<int> g_hdr_content{0};

//! The Rift tab's status lines as last sent ("cmd-status ...", "cmd-display ..."), under g_notices' lock; sent again
//! to a helper just started.
static std::string g_tab_status_line;
static std::string g_tab_display_line;

//! The Rift tab's display card: direct mode and the render scale as saved (SteamVR's next start) and as
//! running now.
static struct
{
	std::atomic<bool> direct_saved{true};
	std::atomic<bool> direct_running{false};
	std::atomic<int> scale_saved{100};
	std::atomic<int> scale_running{100};
	//! The Performance card's switch (settings.json {"tab": {"performance"}}): the helper reads frame timings.
	std::atomic<bool> perf{false};
} g_display;

//! Monado's picture settings from @ref g_picture; others (dither, black level) kept as they are.
static void
picture_apply_locked()
{
	struct render_output_settings s;
	render_output_settings_get(&s);
	const float k = std::clamp(g_picture.strength, 0, 100) / 50.0f;
	const float c = std::clamp(g_picture.colour, 0, 100) / 50.0f;
	s.auto_hdr = g_picture.on;
	s.auto_hdr_highlights = g_picture.base.auto_hdr_highlights * k;
	s.auto_hdr_shadows = g_picture.base.auto_hdr_shadows * k;
	s.auto_hdr_colour = g_picture.base.auto_hdr_colour * c;
	s.motion_smoothing = g_picture.smoothing;
	s.hdr_bright = g_picture.bright;
	render_output_settings_set(&s);
}

static void
notices_on_put_on();

//! Whether @p xdev reports being worn, true for one that cannot tell.
static bool
head_worn(struct xrt_device *xdev)
{
	if (!xdev->supported.presence || xrt_device_update_inputs(xdev) != XRT_SUCCESS) {
		return true;
	}
	for (uint32_t i = 0; i < xdev->input_count; i++) {
		if (xdev->inputs[i].name == XRT_INPUT_GENERIC_HEAD_DETECT) {
			return xdev->inputs[i].value.boolean;
		}
	}
	return true;
}

static void
floor_watch_head(struct xrt_device *xdev, const struct xrt_space_relation *head)
{
	const bool tracked = (head->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0 &&
	                     (head->relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;
	if (!tracked) {
		return;
	}
	// Ten heights a second are plenty; the pose thread (only it calls this) runs at a kilohertz.
	static int64_t last_ns = 0;
	const int64_t now_ns = os_monotonic_get_ns();
	if (now_ns - last_ns < 100 * U_TIME_1MS_IN_NS) {
		return;
	}
	last_ns = now_ns;
	// The device's inputs are read without the lock held.
	if (g_recal.measuring.load() || !head_worn(xdev)) {
		return;
	}
	const std::optional<float> floor =
	    g_world_from_driver.floor_known.load() ? std::optional<float>(g_world_from_driver.floor_m.load()) : std::nullopt;
	std::lock_guard<std::mutex> lock(g_recal.lock);
	g_recal.keeper.head(head->pose.position.y, (double)now_ns / 1e9, floor);
}

/*!
 * Measured again ("calibration-done" with a new result: on our request, or a start with no sensor placed): the world
 * placed on the user anew, and the floor carried over at once when the sensors agree on it, else from the head's
 * heights over the next seconds (RunFrame). From the tracker's thread, under the notices' lock: nothing here takes
 * the tracker's.
 */
static void
recalibration_done()
{
	struct u_notice_recalibration change = {};
	(void)u_notice_get_recalibration(&change);
	if (change.kept) {
		ovrd_log("Sensors measured again: the room is where it was (%d sensors kept their place), floor %.3f m kept\n",
		         change.anchors, g_world_from_driver.floor_m.load());
		return;
	}
	g_world_from_driver.recentre.store(true);
	ovrd_log("Sensors measured again: no sensor kept its place, the world is placed on you once your head settles\n");
	if (!g_world_from_driver.floor_known.load()) {
		ovrd_log("Sensors measured again; no floor was set, nothing to carry over\n");
		return;
	}
	std::lock_guard<std::mutex> lock(g_recal.lock);
	const float before = g_world_from_driver.floor_m.load();
	const std::optional<rl::floor::Result> r = g_recal.keeper.done(before, change, floor_now_s());
	if (r) {
		g_world_from_driver.floor_m.store(r->floor_m);
		g_recal.floor_to_save = true;
		ovrd_log("Sensors measured again: floor %.3f m (was %.3f), %s\n", r->floor_m, before, r->how.c_str());
		return;
	}
	ovrd_log("Sensors measured again: a sensor moved up or down; the floor follows from your head's height in %.0f s\n",
	         rl::floor::Keeper::kWaitS);
}

/*!
 * Places the world once the headset is worn and its heading has held within 8 degrees for a second, the way the
 * Monado service recenters on a put-on: placed at the first pose, it faced wherever the headset pointed in a hand.
 * Placed again, once the head has settled in the new measurement, when measuring the sensors again moved the room.
 */
/*!
 * How long the headset has to be worn before it may be placed on: a headset taken as worn while it moves (its wear
 * sensor missing the face) was placed while still being put on, from a moment it was held still in the hands, and the
 * world faced 60 degrees off.
 */
static constexpr int64_t kPlaceWornForNs = static_cast<int64_t>(3) * U_TIME_1S_IN_NS;

static void
set_world_from_driver_once(struct xrt_device *xdev, const struct xrt_space_relation *head)
{
	static int64_t settle_since_ns = 0;
	static float settle_yaw = 0.0f;
	static int64_t last_check_ns = 0;
	static int64_t worn_since_ns = 0;
	// Only this (the HMD's pose) thread.
	static rl::room::Settle resettle;

	const bool tracked = (head->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0 &&
	                     (head->relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;
	const int64_t now_ns = os_monotonic_get_ns();
	if (g_world_from_driver.recentre.exchange(false)) {
		resettle.arm((double)now_ns / 1e9);
	}
	const bool placed = g_world_from_driver.set.load(std::memory_order_acquire);
	if ((placed && !resettle.armed()) || !tracked) {
		resettle.lost();
		return;
	}

	// The pose thread runs at a kilohertz, 20 checks a second are plenty.
	if (now_ns - last_check_ns < 50 * U_TIME_1MS_IN_NS) {
		return;
	}
	last_check_ns = now_ns;

	const struct xrt_vec3 minus_z = {0.0f, 0.0f, -1.0f};
	struct xrt_vec3 forward;
	math_quat_rotate_vec3(&head->pose.orientation, &minus_z, &forward);
	const float yaw = atan2f(-forward.x, -forward.z);

	const bool recentre = placed;
	if (recentre) {
		if (!head_worn(xdev)) {
			resettle.lost();
			return;
		}
		if (!resettle.head((double)now_ns / 1e9, yaw, head->pose.position)) {
			return;
		}
	}

	float d = (yaw - settle_yaw) * 180.0f / (float)M_PI;
	d = d > 180.0f ? d - 360.0f : (d < -180.0f ? d + 360.0f : d);
	if (!recentre) {
		const bool worn = head_worn(xdev);
		if (!worn) {
			worn_since_ns = 0;
		} else if (worn_since_ns == 0) {
			worn_since_ns = now_ns;
		}
		if (!worn || now_ns - worn_since_ns < kPlaceWornForNs || settle_since_ns == 0 || fabsf(d) > 8.0f ||
		    fabsf(forward.y) > 0.6f) {
			settle_since_ns = now_ns;
			settle_yaw = yaw;
			return;
		}
		if (now_ns - settle_since_ns < U_TIME_1S_IN_NS) {
			return;
		}
	}

	// Turn the head's heading to -Z, then move its spot on the floor plane to the origin.
	struct xrt_quat q;
	const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
	math_quat_from_angle_vector(-yaw, &up, &q);
	const struct xrt_vec3 flat = {head->pose.position.x, 0.0f, head->pose.position.z};
	struct xrt_vec3 moved;
	math_quat_rotate_vec3(&q, &flat, &moved);

	{
		std::lock_guard<std::mutex> lock(g_world_from_driver.lock);
		g_world_from_driver.rotation = {q.w, q.x, q.y, q.z};
		g_world_from_driver.translation[0] = -moved.x;
		g_world_from_driver.translation[1] = 0.0;
		g_world_from_driver.translation[2] = -moved.z;
	}
	g_world_from_driver.set.store(true, std::memory_order_release);

	ovrd_log("World %s on the user: heading %.1f deg, at %.2f %.2f m\n", recentre ? "recentred" : "placed",
	         yaw * 180.0f / (float)M_PI, head->pose.position.x, head->pose.position.z);
}

static void
apply_world_from_driver(vr::DriverPose_t *pose)
{
	if (!g_world_from_driver.set.load(std::memory_order_acquire)) {
		return;
	}
	std::lock_guard<std::mutex> lock(g_world_from_driver.lock);
	pose->qWorldFromDriverRotation = g_world_from_driver.rotation;
	for (int i = 0; i < 3; i++) {
		pose->vecWorldFromDriverTranslation[i] = g_world_from_driver.translation[i];
	}
	pose->vecWorldFromDriverTranslation[1] += g_world_from_driver.floor_m.load() + g_world_from_driver.height_m.load();
}

//! A pose in SteamVR's world back in the driver's space: the inverse of the world placement.
static void
driver_from_world(struct xrt_pose *pose)
{
	if (!g_world_from_driver.set.load(std::memory_order_acquire)) {
		return;
	}
	struct xrt_pose world_from_driver = XRT_POSE_IDENTITY;
	{
		std::lock_guard<std::mutex> lock(g_world_from_driver.lock);
		const vr::HmdQuaternion_t &q = g_world_from_driver.rotation;
		world_from_driver.orientation = {(float)q.x, (float)q.y, (float)q.z, (float)q.w};
		// As apply_world_from_driver: the floor and the user's height are part of the placement.
		world_from_driver.position = {(float)g_world_from_driver.translation[0],
		                              (float)g_world_from_driver.translation[1] +
		                                  g_world_from_driver.floor_m.load() + g_world_from_driver.height_m.load(),
		                              (float)g_world_from_driver.translation[2]};
	}
	struct xrt_pose driver_from_world_pose;
	math_pose_invert(&world_from_driver, &driver_from_world_pose);
	struct xrt_pose out;
	math_pose_transform(&driver_from_world_pose, pose, &out);
	*pose = out;
}

DEBUG_GET_ONCE_OPTION(ovrd_angular_velocity, "OVRD_ANGULAR_VELOCITY", "local")

static void
apply_pose(struct xrt_space_relation *rel, vr::DriverPose_t *m_pose)
{
	if ((rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0) {
		copy_quat(&rel->pose.orientation, &m_pose->qRotation);
	} else {
		m_pose->result = vr::TrackingResult_Running_OutOfRange;
		m_pose->poseIsValid = false;
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
		copy_vec3(&rel->pose.position, m_pose->vecPosition);
	} else {
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0) {
		// linear velocity in world space
		copy_vec3(&rel->linear_velocity, m_pose->vecVelocity);
	}

	if ((rel->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
		/*
		 * Monado reports angular velocity in world space. Which frame SteamVR predicts with is not documented:
		 * OVRD_ANGULAR_VELOCITY=local (default, as before), world, or none. With local the Rift CV1's view
		 * skewed while the head turned in SteamVR Home, where Monado's own compositor showed nothing of it.
		 */
		const char *frame = debug_get_option_ovrd_angular_velocity();
		if (strcmp(frame, "world") == 0) {
			copy_vec3(&rel->angular_velocity, m_pose->vecAngularVelocity);
		} else if (strcmp(frame, "none") == 0) {
			m_pose->vecAngularVelocity[0] = m_pose->vecAngularVelocity[1] = m_pose->vecAngularVelocity[2] =
			    0.0;
		} else {
			struct xrt_quat orientation_inv;
			math_quat_invert(&rel->pose.orientation, &orientation_inv);

			struct xrt_vec3 vel;
			math_quat_rotate_derivative(&orientation_inv, &rel->angular_velocity, &vel);

			copy_vec3(&vel, m_pose->vecAngularVelocity);
		}
	}
}

#define OPENVR_BONE_COUNT 31

// these are currently only used for the root and wrist transforms, but are kept here as they are useful for debugging.
vr::VRBoneTransform_t rightOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}}, // Root
    {{0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, 0.920279f, -0.379296f}},
    //
    {{0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.567418f, -0.464112f, 0.623374f, -0.272106f}}, // Thumb
    {{-0.040406f, -0.000000f, 0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{-0.032517f, -0.000000f, -0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{-0.030464f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{-0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.421979f, -0.644251f, 0.422133f, 0.478202f}}, // Index
    {{-0.074204f, 0.005002f, -0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{-0.043930f, 0.000000f, 0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{-0.028695f, -0.000000f, -0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{-0.022821f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{-0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.541276f, -0.546723f, 0.460749f, 0.442520f}}, // Middle
    {{-0.070953f, -0.000779f, -0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{-0.043108f, -0.000000f, -0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{-0.033266f, -0.000000f, -0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{-0.025892f, 0.000000f, -0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},
    //
    {{-0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.550143f, -0.516692f, 0.429888f, 0.495548f}}, // Ring
    {{-0.065876f, -0.001786f, -0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{-0.040697f, -0.000000f, -0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{-0.028747f, 0.000000f, 0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{-0.022430f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.523940f, -0.526918f, 0.326740f, 0.584025f}}, // Pinky
    {{-0.062878f, -0.002844f, -0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{-0.030220f, -0.000000f, -0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{-0.018187f, -0.000000f, -0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{-0.018018f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, -0.594267f, -0.249441f}}, // Aux
    {{0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, 0.663809f, 0.293734f}},
    {{0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, 0.659285f, 0.265683f}},
    {{0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, 0.634757f, 0.143936f}},
    {{0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, 0.639342f, 0.126678f}},
};

vr::VRBoneTransform_t leftOpenPose[OPENVR_BONE_COUNT] = {
    {{0.000000f, 0.000000f, 0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},   // Root
                                                                                                      //
    {{-0.034038f, 0.036503f, 0.164722f, 1.000000f}, {-0.055147f, -0.078608f, -0.920279f, 0.379296f}}, // Thumb
    {{-0.012083f, 0.028070f, 0.025050f, 1.000000f}, {0.464112f, 0.567418f, 0.272106f, 0.623374f}},
    {{0.040406f, 0.000000f, -0.000000f, 1.000000f}, {0.994838f, 0.082939f, 0.019454f, 0.055130f}},
    {{0.032517f, 0.000000f, 0.000000f, 1.000000f}, {0.974793f, -0.003213f, 0.021867f, -0.222015f}},
    {{0.030464f, -0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, -0.000000f, 0.000000f}},
    //
    {{0.000632f, 0.026866f, 0.015002f, 1.000000f}, {0.644251f, 0.421979f, -0.478202f, 0.422133f}}, // Index
    {{0.074204f, -0.005002f, 0.000234f, 1.000000f}, {0.995332f, 0.007007f, -0.039124f, 0.087949f}},
    {{0.043930f, -0.000000f, -0.000000f, 1.000000f}, {0.997891f, 0.045808f, 0.002142f, -0.045943f}},
    {{0.028695f, 0.000000f, 0.000000f, 1.000000f}, {0.999649f, 0.001850f, -0.022782f, -0.013409f}},
    {{0.022821f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, -0.000000f, 0.000000f, -0.000000f}},
    //
    {{0.002177f, 0.007120f, 0.016319f, 1.000000f}, {0.546723f, 0.541276f, -0.442520f, 0.460749f}}, // Middle
    {{0.070953f, 0.000779f, 0.000997f, 1.000000f}, {0.980294f, -0.167261f, -0.078959f, 0.069368f}},
    {{0.043108f, 0.000000f, 0.000000f, 1.000000f}, {0.997947f, 0.018493f, 0.013192f, 0.059886f}},
    {{0.033266f, 0.000000f, 0.000000f, 1.000000f}, {0.997394f, -0.003328f, -0.028225f, -0.066315f}},
    {{0.025892f, -0.000000f, 0.000000f, 1.000000f}, {0.999195f, -0.000000f, 0.000000f, 0.040126f}},

    {{0.000513f, -0.006545f, 0.016348f, 1.000000f}, {0.516692f, 0.550143f, -0.495548f, 0.429888f}}, // Ring
    {{0.065876f, 0.001786f, 0.000693f, 1.000000f}, {0.990420f, -0.058696f, -0.101820f, 0.072495f}},
    {{0.040697f, 0.000000f, 0.000000f, 1.000000f}, {0.999545f, -0.002240f, 0.000004f, 0.030081f}},
    {{0.028747f, -0.000000f, -0.000000f, 1.000000f}, {0.999102f, -0.000721f, -0.012693f, 0.040420f}},
    {{0.022430f, -0.000000f, 0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.002478f, -0.018981f, 0.015214f, 1.000000f}, {0.526918f, 0.523940f, -0.584025f, 0.326740f}}, // Pinky
    {{0.062878f, 0.002844f, 0.000332f, 1.000000f}, {0.986609f, -0.059615f, -0.135163f, 0.069132f}},
    {{0.030220f, 0.000000f, 0.000000f, 1.000000f}, {0.994317f, 0.001896f, -0.000132f, 0.106446f}},
    {{0.018187f, 0.000000f, 0.000000f, 1.000000f}, {0.995931f, -0.002010f, -0.052079f, -0.073526f}},
    {{0.018018f, 0.000000f, -0.000000f, 1.000000f}, {1.000000f, 0.000000f, 0.000000f, 0.000000f}},

    {{-0.006059f, 0.056285f, 0.060064f, 1.000000f}, {0.737238f, 0.202745f, 0.594267f, 0.249441f}}, // Aux
    {{-0.040416f, -0.043018f, 0.019345f, 1.000000f}, {-0.290331f, 0.623527f, -0.663809f, -0.293734f}},
    {{-0.039354f, -0.075674f, 0.047048f, 1.000000f}, {-0.187047f, 0.678062f, -0.659285f, -0.265683f}},
    {{-0.038340f, -0.090987f, 0.082579f, 1.000000f}, {-0.183037f, 0.736793f, -0.634757f, -0.143936f}},
    {{-0.031806f, -0.087214f, 0.121015f, 1.000000f}, {-0.003659f, 0.758407f, -0.639342f, -0.126678f}},
};

template <class T, class U>
void
convert_quaternion(const T &p_quatA, U &p_quatB)
{
	p_quatB.x = p_quatA.x;
	p_quatB.y = p_quatA.y;
	p_quatB.z = p_quatA.z;
	p_quatB.w = p_quatA.w;
}

xrt_quat
apply_bone_hand_transform(xrt_quat p_rot, xrt_hand hand)
{
	std::swap(p_rot.x, p_rot.z);
	p_rot.z *= -1.f;
	if (hand == XRT_HAND_RIGHT)
		return p_rot;

	p_rot.x *= -1.f;
	p_rot.y *= -1.f;
	return p_rot;
}

void
metacarpal_joints_to_bone_transform(struct xrt_hand_joint_set *hand_joint_set,
                                    vr::VRBoneTransform_t *out_bone_transforms,
                                    xrt_hand hand)
{
	struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

	// Apply orientations for four-finger metacarpals.
	for (int joint :
	     {XRT_HAND_JOINT_THUMB_METACARPAL, XRT_HAND_JOINT_INDEX_METACARPAL, XRT_HAND_JOINT_MIDDLE_METACARPAL,
	      XRT_HAND_JOINT_RING_METACARPAL, XRT_HAND_JOINT_LITTLE_METACARPAL}) {
		struct xrt_hand_joint_value *current_joint = &joint_values[joint];
		struct xrt_hand_joint_value *parent_joint = &joint_values[XRT_HAND_JOINT_WRIST];

		xrt_quat diff_openxr;
		// These should do the exact same things.
		xrt_quat parent_inv;
		math_quat_invert(&parent_joint->relation.pose.orientation, &parent_inv);
		math_quat_rotate(&parent_inv, &current_joint->relation.pose.orientation, &diff_openxr);
		xrt_quat diff_openvr = apply_bone_hand_transform(diff_openxr, hand);


		/**
		 * * if you try applying the metacarpal transforms without the magic quaternion, everything from the
		 * metacarpals onwards is rotated 90 degrees.
		 * In the neutral pose sample, all the metacarpals have a
		 * rotation relatively close to {w=0.5, x=0.5, y=-0.5, z=0.5} which is an Important Quaternion because
		 * it probably represents some 90 degree rotation. Maybe, and this was just a random guess, if I took
		 * the regular metacarpal orientations and rotated them by that quat, everything would work.
		 */
		xrt_quat magic_prerotate = XRT_QUAT_IDENTITY;
		magic_prerotate.w = 0.5;
		magic_prerotate.x = 0.5;
		magic_prerotate.y = -0.5;
		magic_prerotate.z = 0.5;

		if (hand == XRT_HAND_RIGHT) {
			magic_prerotate.y *= -1.f;
			magic_prerotate.x *= -1.f;
		}

		xrt_quat final_diff;
		math_quat_rotate(&magic_prerotate, &diff_openvr, &final_diff);
		convert_quaternion(final_diff, out_bone_transforms[joint].orientation);

		xrt_vec3 global_diff_from_this_to_parent =
		    m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);

		xrt_vec3 translation_wrist_rel;
		math_quat_rotate_vec3(&parent_inv, &global_diff_from_this_to_parent, &translation_wrist_rel);

		// Y = X?
		out_bone_transforms[joint].position.v[0] = translation_wrist_rel.y;
		out_bone_transforms[joint].position.v[1] = translation_wrist_rel.x;
		out_bone_transforms[joint].position.v[2] = -translation_wrist_rel.z;
		out_bone_transforms[joint].position.v[3] = 1.f;

		if (hand == XRT_HAND_RIGHT)
			out_bone_transforms[joint].position.v[1] *= -1.f;
	}
}

void
flexion_joints_to_bone_transform(struct xrt_hand_joint_set *hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 xrt_hand hand)
{
	struct xrt_hand_joint_value *joint_values = hand_joint_set->values.hand_joint_set_default;

	// Apply orientations for four-finger pxm and onward
	int parent = -1;
	for (int joint = XRT_HAND_JOINT_THUMB_METACARPAL; joint < XRT_HAND_JOINT_COUNT; joint++) {
		if (u_hand_joint_is_metacarpal((xrt_hand_joint)joint)) {
			parent = joint;
			continue;
		}
		struct xrt_hand_joint_value *current_joint = &joint_values[joint];
		struct xrt_hand_joint_value *parent_joint = &joint_values[parent];


		xrt_quat diff_openxr;
		math_quat_unrotate(&parent_joint->relation.pose.orientation, &current_joint->relation.pose.orientation,
		                   &diff_openxr);

		xrt_quat diff_openvr = apply_bone_hand_transform(diff_openxr, hand);
		convert_quaternion(diff_openvr, out_bone_transforms[joint].orientation);
		xrt_vec3 global_diff_from_this_to_parent =
		    m_vec3_sub(current_joint->relation.pose.position, parent_joint->relation.pose.position);


		float bone_length = m_vec3_len(global_diff_from_this_to_parent);
		// OpenVR left hand has +X forward. Weird, huh?
		out_bone_transforms[joint].position = {bone_length, 0, 0, 1};

		if (hand == XRT_HAND_RIGHT)
			out_bone_transforms[joint].position.v[0] *= -1.f;

		parent = joint;
	}
}

void
hand_joint_set_to_bone_transform(struct xrt_hand_joint_set hand_joint_set,
                                 vr::VRBoneTransform_t *out_bone_transforms,
                                 xrt_hand hand)
{
	// fill bone transforms with a default open pose to manipulate later
	for (int i : {XRT_HAND_JOINT_WRIST, XRT_HAND_JOINT_PALM}) {
		out_bone_transforms[i] = hand == XRT_HAND_LEFT ? leftOpenPose[i] : rightOpenPose[i];
	}

	metacarpal_joints_to_bone_transform(&hand_joint_set, out_bone_transforms, hand);
	flexion_joints_to_bone_transform(&hand_joint_set, out_bone_transforms, hand);
}

#ifdef XRT_OS_LINUX
static void
riftlinux_helper_command(const char *command);
static cJSON *
riftlinux_settings_load();
static bool
user_room_complete();
static std::string
default_room_json();
#endif

class CDeviceDriver_Monado_Controller : public vr::ITrackedDeviceServerDriver
{
public:
	CDeviceDriver_Monado_Controller(struct xrt_instance *xinst, struct xrt_device *xdev, enum xrt_hand hand)
	    : m_xdev(xdev), m_hand(hand)
	{
		ovrd_log("Creating Controller %s\n", xdev->str);

		m_handed_controller = true;

		m_emulate_index_controller = debug_get_bool_option_emulate_index_controller();

		if (m_emulate_index_controller) {
			ovrd_log("Emulating Index Controller\n");
		} else {
			ovrd_log("Using Monado Controller profile\n");
		}

		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_pose = {};

		snprintf(m_sModelNumber, MODELNUM_LEN, "[Monado] %s", xdev->str);
		strncpy(m_sSerialNumber, xdev->serial, XRT_DEVICE_NAME_LEN);
		/*
		 * SteamVR will not add a device without a serial number, and a Rift's Touch controllers only get theirs
		 * once the radio has read it, after SteamVR has asked: fall back to the device's name, which tells left
		 * from right.
		 */
		if (m_sSerialNumber[0] == '\0') {
			snprintf(m_sSerialNumber, XRT_DEVICE_NAME_LEN, "%s", xdev->str);
		}

		switch (this->m_xdev->name) {
		case XRT_DEVICE_INDEX_CONTROLLER:
			if (hand == XRT_HAND_LEFT) {
				m_render_model =
				    "{indexcontroller}valve_controller_knu_1_0_"
				    "left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model =
				    "{indexcontroller}valve_controller_knu_1_0_"
				    "right";
			}
			break;
		case XRT_DEVICE_TOUCH_CONTROLLER:
		case XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1:
			if (hand == XRT_HAND_LEFT) {
				m_render_model = "oculus_cv1_controller_left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model = "oculus_cv1_controller_right";
			}
			break;
		case XRT_DEVICE_VIVE_WAND: m_render_model = "vr_controller_vive_1_5"; break;
		case XRT_DEVICE_VIVE_TRACKER_GEN1:
		case XRT_DEVICE_VIVE_TRACKER_GEN2:
		case XRT_DEVICE_VIVE_TRACKER_GEN3:
		case XRT_DEVICE_VIVE_TRACKER_TUNDRA: m_render_model = "{htc}vr_tracker_vive_1_0"; break;
		case XRT_DEVICE_FLIPVR:
			if (hand == XRT_HAND_LEFT) {
				m_render_model = "{shiftall}flipvr_controller_vc1b_left";
			}
			if (hand == XRT_HAND_RIGHT) {
				m_render_model = "{shiftall}flipvr_controller_vc1b_right";
			}
			break;
		case XRT_DEVICE_PSMV:
		case XRT_DEVICE_HYDRA:
		case XRT_DEVICE_DAYDREAM:
		case XRT_DEVICE_GENERIC_HMD:
		default: m_render_model = "locator_one_sided"; break;
		}

		ovrd_log("Render model based on Monado: %s\n", m_render_model);

		vr::VRServerDriverHost()->TrackedDeviceAdded(GetSerialNumber().c_str(),
		                                             vr::TrackedDeviceClass_Controller, this);
	}
	virtual ~CDeviceDriver_Monado_Controller() {}

	bool
	HasInput(enum xrt_input_name name)
	{
		for (uint32_t i = 0; i < m_xdev->input_count; i++) {
			if (m_xdev->inputs[i].name == name) {
				return true;
			}
		}
		return false;
	}

	void
	AddControl(const char *steamvr_control_path,
	           enum xrt_input_name monado_input_name,
	           struct MonadoInputComponent *component)
	{
		// A profile lists both hands' controls; one the device lacks was looked up and logged every frame.
		if (!HasInput(monado_input_name)) {
			ovrd_log("Skipping %s, the device has no such input\n", steamvr_control_path);
			return;
		}

		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		SteamVRDriverControlInput in;

		in.monado_input_type = monado_input_type;
		in.steamvr_control_path = steamvr_control_path;
		in.monado_input_name = monado_input_name;
		if (component != NULL) {
			in.component = *component;
		} else {
			in.component.has_component = false;
		}

		if (monado_input_type == XRT_INPUT_TYPE_BOOLEAN) {
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, steamvr_control_path,
			                                            &in.control_handle);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE) {
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedTwoSided);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE) {
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedOneSided);

		} else if (monado_input_type == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {
			// 2D values are added as 2 1D values
			// usually those are [-1,1]
			vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, steamvr_control_path,
			                                           &in.control_handle, vr::VRScalarType_Absolute,
			                                           vr::VRScalarUnits_NormalizedTwoSided);
		}

		m_input_controls.push_back(in);
		ovrd_log("Added input %s\n", steamvr_control_path);
	}

	void
	AddOutputControl(enum xrt_output_name monado_output_name, const char *steamvr_control_path)
	{
		SteamVRDriverControlOutput out;

		/// @todo when there are multiple output types: XRT_GET_OUTPUT_TYPE(monado_output_name);
		enum xrt_output_type monado_output_type = XRT_OUTPUT_TYPE_VIBRATION;

		out.monado_output_type = monado_output_type;
		out.steamvr_control_path = steamvr_control_path;
		out.monado_output_name = monado_output_name;

		vr::VRDriverInput()->CreateHapticComponent(m_ulPropertyContainer, out.steamvr_control_path,
		                                           &out.control_handle);

		m_output_controls.push_back(out);
		ovrd_log("Added output %s\n", steamvr_control_path);
	}

	void
	AddSkeletonControl(const char *steamvr_skeleton_name,
	                   const char *steamvr_control_path,
	                   enum xrt_input_name monado_input_name,
	                   vr::EVRSkeletalTrackingLevel tracking_level = vr::VRSkeletalTracking_Full)
	{
		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		SteamVRDriverControlInput in;

		in.monado_input_type = monado_input_type;
		in.steamvr_control_path = steamvr_control_path;
		in.monado_input_name = monado_input_name;
		in.component.has_component = false;

		vr::EVRInputError err = vr::VRDriverInput()->CreateSkeletonComponent(
		    m_ulPropertyContainer, steamvr_skeleton_name, steamvr_control_path, "/pose/raw", tracking_level, NULL,
		    OPENVR_BONE_COUNT, &in.control_handle);
		if (err) {
			ovrd_log("Error adding skeletal input: %i", err);
			return;
		}


		m_skeletal_input_control = in;
		ovrd_log("Added skeleton input %s\n", steamvr_control_path);
	}

	void
	AddEmulatedIndexControls()
	{
		switch (this->m_xdev->name) {
		case XRT_DEVICE_INDEX_CONTROLLER: {
			AddControl("/input/trigger/value", XRT_INPUT_INDEX_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_INDEX_TRIGGER_CLICK, NULL);
			AddControl("/input/trigger/touch", XRT_INPUT_INDEX_TRIGGER_TOUCH, NULL);


			AddControl("/input/system/click", XRT_INPUT_INDEX_SYSTEM_CLICK, NULL);
			AddControl("/input/system/touch", XRT_INPUT_INDEX_SYSTEM_TOUCH, NULL);

			AddControl("/input/a/click", XRT_INPUT_INDEX_A_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_INDEX_A_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_INDEX_B_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_INDEX_B_TOUCH, NULL);


			AddControl("/input/grip/force", XRT_INPUT_INDEX_SQUEEZE_FORCE, NULL);

			AddControl("/input/grip/value", XRT_INPUT_INDEX_SQUEEZE_VALUE, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/thumbstick/click", XRT_INPUT_INDEX_THUMBSTICK_CLICK, NULL);

			AddControl("/input/thumbstick/touch", XRT_INPUT_INDEX_THUMBSTICK_TOUCH, NULL);

			AddControl("/input/thumbstick/x", XRT_INPUT_INDEX_THUMBSTICK, &x);

			AddControl("/input/thumbstick/y", XRT_INPUT_INDEX_THUMBSTICK, &y);


			AddControl("/input/trackpad/force", XRT_INPUT_INDEX_TRACKPAD_FORCE, NULL);

			AddControl("/input/trackpad/touch", XRT_INPUT_INDEX_TRACKPAD_TOUCH, NULL);

			AddControl("/input/trackpad/x", XRT_INPUT_INDEX_TRACKPAD, &x);

			AddControl("/input/trackpad/y", XRT_INPUT_INDEX_TRACKPAD, &y);

			if (m_xdev->supported.hand_tracking) {
				ovrd_log("Enabling skeletal input as this device supports it");

				// skeletal input compatibility with games is a bit funky with any controllers
				// other than the index controller, so only do skeletal input with index
				// emulation
				const std::string str_hand = m_hand == XRT_HAND_LEFT ? "left" : "right";


				AddSkeletonControl(("/input/skeleton/" + str_hand).c_str(),
				                   ("/skeleton/hand/" + str_hand).c_str(),
				                   XRT_INPUT_HT_UNOBSTRUCTED_RIGHT);
				RunFrame();
			} else
				ovrd_log("Not enabling skeletal input as this device does not support it");


			AddOutputControl(XRT_OUTPUT_NAME_INDEX_HAPTIC, "/output/haptic");
		}

		break;
		case XRT_DEVICE_VIVE_WAND: {
			AddControl("/input/trigger/value", XRT_INPUT_VIVE_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_VIVE_TRIGGER_CLICK, NULL);


			AddControl("/input/system/click", XRT_INPUT_VIVE_SYSTEM_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_VIVE_TRACKPAD_CLICK, NULL);

			AddControl("/input/b/click", XRT_INPUT_VIVE_MENU_CLICK, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/trackpad/touch", XRT_INPUT_VIVE_TRACKPAD_TOUCH, NULL);

			AddControl("/input/trackpad/x", XRT_INPUT_VIVE_TRACKPAD, &x);

			AddControl("/input/trackpad/y", XRT_INPUT_VIVE_TRACKPAD, &y);

			AddOutputControl(XRT_OUTPUT_NAME_VIVE_HAPTIC, "/output/haptic");
		} break;
		case XRT_DEVICE_PSMV: {

			AddControl("/input/trigger/value", XRT_INPUT_PSMV_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_PSMV_MOVE_CLICK, NULL);

			AddControl("/input/system/click", XRT_INPUT_PSMV_PS_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_PSMV_CROSS_CLICK, NULL);

			AddControl("/input/b/click", XRT_INPUT_PSMV_SQUARE_CLICK, NULL);

			AddOutputControl(XRT_OUTPUT_NAME_PSMV_RUMBLE_VIBRATION, "/output/haptic");
		} break;
		case XRT_DEVICE_FLIPVR: {

			AddControl("/input/trigger/value", XRT_INPUT_FLIPVR_TRIGGER_VALUE, NULL);

			AddControl("/input/trigger/click", XRT_INPUT_FLIPVR_TRIGGER_CLICK, NULL);
			AddControl("/input/trigger/touch", XRT_INPUT_FLIPVR_TRIGGER_TOUCH, NULL);

			AddControl("/input/system/click", XRT_INPUT_FLIPVR_SYSTEM_CLICK, NULL);

			AddControl("/input/a/click", XRT_INPUT_FLIPVR_A_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_FLIPVR_A_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_FLIPVR_B_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_FLIPVR_B_TOUCH, NULL);

			AddControl("/input/a/click", XRT_INPUT_FLIPVR_X_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_FLIPVR_X_TOUCH, NULL);

			AddControl("/input/b/click", XRT_INPUT_FLIPVR_Y_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_FLIPVR_Y_TOUCH, NULL);

			AddControl("/input/grip/value", XRT_INPUT_FLIPVR_SQUEEZE_VALUE, NULL);

			AddControl("/input/grip/click", XRT_INPUT_FLIPVR_SQUEEZE_CLICK, NULL);

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl("/input/thumbstick/click", XRT_INPUT_FLIPVR_THUMBSTICK_CLICK, NULL);

			AddControl("/input/thumbstick/touch", XRT_INPUT_FLIPVR_THUMBSTICK_TOUCH, NULL);

			AddControl("/input/thumbstick/x", XRT_INPUT_FLIPVR_THUMBSTICK, &x);

			AddControl("/input/thumbstick/y", XRT_INPUT_FLIPVR_THUMBSTICK, &y);

			AddOutputControl(XRT_OUTPUT_NAME_FLIPVR_HAPTIC, "/output/haptic");
		}

		case XRT_DEVICE_TOUCH_CONTROLLER: break;    // TODO
		case XRT_DEVICE_WMR_CONTROLLER: break;      // TODO
		case XRT_DEVICE_XBOX_CONTROLLER: break;     // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN1: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN2: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_GEN3: break;   // TODO
		case XRT_DEVICE_VIVE_TRACKER_TUNDRA: break; // TODO
		case XRT_DEVICE_REALSENSE: break;
		case XRT_DEVICE_DEPTHAI: break;

		case XRT_DEVICE_HAND_INTERACTION: break;  // there is no hardware
		case XRT_DEVICE_GO_CONTROLLER: break;     // hardware has no haptics
		case XRT_DEVICE_DAYDREAM: break;          // hardware has no haptics
		case XRT_DEVICE_HYDRA: break;             // hardware has no haptics
		case XRT_DEVICE_SIMPLE_CONTROLLER: break; // shouldn't happen
		case XRT_DEVICE_HAND_TRACKER: break;      // shouldn't happen
		case XRT_DEVICE_GENERIC_HMD:
		case XRT_DEVICE_VIVE_PRO: break; // no
		case XRT_DEVICE_EXT_HAND_INTERACTION: break;

		default: break;
		}
	}

	static struct profile_template *
	get_profile_template(enum xrt_device_name device_name)
	{
		for (int i = 0; i < OXR_BINDINGS_PROFILE_TEMPLATE_COUNT; i++) {
			if (profile_templates[i].name == device_name)
				return &profile_templates[i];
		}
		return NULL;
	}

	void
	AddMonadoInput(struct binding_template *b)
	{
		enum xrt_input_name monado_input_name = b->input;
		const char *steamvr_path = b->steamvr_path;

		enum xrt_input_type monado_input_type = XRT_GET_INPUT_TYPE(monado_input_name);

		switch (monado_input_type) {
		case XRT_INPUT_TYPE_BOOLEAN:
		case XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE:
		case XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE: AddControl(steamvr_path, monado_input_name, NULL); break;

		case XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE: {
			std::string xpath = std::string(steamvr_path) + std::string("/x");
			std::string ypath = std::string(steamvr_path) + std::string("/y");

			struct MonadoInputComponent x = {true, true, false};
			struct MonadoInputComponent y = {true, false, true};

			AddControl(xpath.c_str(), monado_input_name, &x);
			AddControl(ypath.c_str(), monado_input_name, &y);
		} break;
		case XRT_INPUT_TYPE_POSE:
			//! @todo how to handle poses?
		case XRT_INPUT_TYPE_HAND_TRACKING:
		case XRT_INPUT_TYPE_FACE_TRACKING:
		case XRT_INPUT_TYPE_BODY_TRACKING:
		case XRT_INPUT_TYPE_VEC3_MINUS_ONE_TO_ONE: break;
		}
	}

	/*!
	 * The controls of SteamVR's Oculus Touch input profile ({oculus}/input/touch_profile.json). Its system button
	 * is the left Touch's menu button, which opens the SteamVR dashboard as it does on Windows; the right one's
	 * Oculus button has no place in it.
	 */
	void
	AddOculusTouchControls()
	{
		struct MonadoInputComponent x = {true, true, false};
		struct MonadoInputComponent y = {true, false, true};

		if (m_hand == XRT_HAND_LEFT) {
			AddControl("/input/x/click", XRT_INPUT_TOUCH_X_CLICK, NULL);
			AddControl("/input/x/touch", XRT_INPUT_TOUCH_X_TOUCH, NULL);
			AddControl("/input/y/click", XRT_INPUT_TOUCH_Y_CLICK, NULL);
			AddControl("/input/y/touch", XRT_INPUT_TOUCH_Y_TOUCH, NULL);
			AddControl("/input/system/click", XRT_INPUT_TOUCH_MENU_CLICK, NULL);
		} else {
			AddControl("/input/a/click", XRT_INPUT_TOUCH_A_CLICK, NULL);
			AddControl("/input/a/touch", XRT_INPUT_TOUCH_A_TOUCH, NULL);
			AddControl("/input/b/click", XRT_INPUT_TOUCH_B_CLICK, NULL);
			AddControl("/input/b/touch", XRT_INPUT_TOUCH_B_TOUCH, NULL);
		}

		AddControl("/input/joystick/click", XRT_INPUT_TOUCH_THUMBSTICK_CLICK, NULL);
		AddControl("/input/joystick/touch", XRT_INPUT_TOUCH_THUMBSTICK_TOUCH, NULL);
		AddControl("/input/joystick/x", XRT_INPUT_TOUCH_THUMBSTICK, &x);
		AddControl("/input/joystick/y", XRT_INPUT_TOUCH_THUMBSTICK, &y);
		AddControl("/input/trigger/value", XRT_INPUT_TOUCH_TRIGGER_VALUE, NULL);
		AddControl("/input/trigger/touch", XRT_INPUT_TOUCH_TRIGGER_TOUCH, NULL);
		AddControl("/input/grip/value", XRT_INPUT_TOUCH_SQUEEZE_VALUE, NULL);
		AddControl("/input/thumbrest/touch", XRT_INPUT_TOUCH_THUMBREST_TOUCH, NULL);

		AddOutputControl(XRT_OUTPUT_NAME_TOUCH_HAPTIC, "/output/haptic");

		/*
		 * Finger poses from what the Touch senses, as the Oculus driver gives SteamVR (VRChat's hands, No Man's
		 * Sky's); "partial", the fingers being estimated, not tracked. The profile names the skeletons, SteamVR
		 * warned when they were missing.
		 */
		if (m_xdev->supported.hand_tracking) {
			const std::string side = m_hand == XRT_HAND_LEFT ? "left" : "right";
			AddSkeletonControl(("/input/skeleton/" + side).c_str(), ("/skeleton/hand/" + side).c_str(),
			                   m_hand == XRT_HAND_LEFT ? XRT_INPUT_HT_CONFORMING_LEFT
			                                           : XRT_INPUT_HT_CONFORMING_RIGHT,
			                   vr::VRSkeletalTracking_Partial);
		}
	}

	void
	AddMonadoControls()
	{
		struct profile_template *p = get_profile_template(m_xdev->name);
		if (!p) {
			ovrd_log("No profile template for %s\n", m_xdev->str);
			return;
		}

		for (size_t i = 0; i < p->binding_count; i++) {
			struct binding_template *b = &p->bindings[i];

			if (b->input != 0) {
				AddMonadoInput(b);
			}
			if (b->output != 0) {
				AddOutputControl(b->output, b->steamvr_path);
			}
		}
	}

	void
	PoseUpdateThreadFunction()
	{
		ovrd_log("Starting controller pose update thread for %s\n", m_xdev->str);

		while (m_poseUpdating) {
			//! @todo figure out the best pose update rate
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

			if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, GetPose(),
				                                                   sizeof(vr::DriverPose_t));
			}
		}

		ovrd_log("Stopping controller pose update thread for %s\n", m_xdev->str);
	}

	vr::EVRInitError
	Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		ovrd_log("Activating Controller SteamVR[%d]\n", unObjectId);

		if (!m_handed_controller) {
			//! @todo handle trackers etc
			ovrd_log("Unhandled: %s\n", m_xdev->str);
			return vr::VRInitError_Unknown;
		}

		m_unObjectId = unObjectId;

		if (this->m_xdev == NULL) {
			ovrd_log("Error: xdev NULL\n");
			return vr::VRInitError_Init_InterfaceNotFound;
		}

		// clang-format off


		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, 2);
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_Controller);
		// SteamVR shows the battery at the controller's icon, as other headsets' drivers report it (RunFrame).
		if (m_xdev->supported.battery_status) {
			vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceProvidesBatteryStatus_Bool, true);
		}

		if (m_hand == XRT_HAND_LEFT) {
			ovrd_log("Left Controller\n");
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_LeftHand);
		} else if (m_hand == XRT_HAND_RIGHT) {
			ovrd_log("Right Controller\n");
			vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_ControllerRoleHint_Int32, vr::TrackedControllerRole_RightHand);
		}
		m_pose.poseIsValid = false;
		m_pose.deviceIsConnected = true;
		m_pose.result = vr::TrackingResult_Uninitialized;
		m_pose.willDriftInYaw = !m_xdev->supported.position_tracking;

		if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1) {
			/*
			 * As SteamVR's own Oculus driver presents it, so SteamVR and its apps use their Touch bindings:
			 * with Monado's own profile there were none, no dashboard on the menu button and no haptics.
			 */
			m_input_profile = std::string("{oculus}/input/touch_profile.json");
			m_controller_type = "oculus_touch";
		} else if (m_emulate_index_controller) {
			m_input_profile = std::string("{indexcontroller}/input/index_controller_profile.json");
			m_controller_type = "knuckles";
			if (m_hand == XRT_HAND_LEFT) {
				m_render_model = "{indexcontroller}valve_controller_knu_1_0_left";
			} else if (m_hand == XRT_HAND_RIGHT) {
				m_render_model = "{indexcontroller}valve_controller_knu_1_0_right";
			}
		} else {

			struct profile_template *p = get_profile_template(m_xdev->name);

			if (p == NULL) {
				ovrd_log("Monado device has unknown profile: %d\n", m_xdev->name);
				return vr::VRInitError_Unknown;
			}

			m_input_profile = std::string("{monado}/input/") + std::string(p->steamvr_input_profile_path);
			m_controller_type = p->steamvr_controller_type;
		}

		ovrd_log("Using input profile %s\n", m_input_profile.c_str());
		ovrd_log("Using render model%s\n", m_render_model);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String, m_input_profile.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, m_render_model);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_xdev->str);

		// clang-format on

		/*
		 * Every controller a manufacturer: Unity's OpenVR plugin leaves out a controller without one, and Unity
		 * games on it (VTOL VR) had no controllers at all while the HMD was found. The Touch
		 * named as SteamVR's Oculus driver names it, which games also go by for their Touch prompts and models.
		 */
		if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1) {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Oculus");
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String,
			                                      m_hand == XRT_HAND_LEFT ? "Oculus Rift CV1 (Left Controller)"
			                                                              : "Oculus Rift CV1 (Right Controller)");
		} else {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Monado");
		}

		m_input_controls.clear();
		if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1) {
			AddOculusTouchControls();
		} else if (m_emulate_index_controller) {
			AddEmulatedIndexControls();
		} else {
			AddMonadoControls();
		}

		ovrd_log("Controller %d activated\n", m_unObjectId);

		m_poseUpdateThread = new std::thread(&CDeviceDriver_Monado_Controller::PoseUpdateThreadFunction, this);
		if (!m_poseUpdateThread) {
			ovrd_log("Unable to create pose updated thread for %s\n", m_xdev->str);
			return vr::VRInitError_Driver_Failed;
		}

		return vr::VRInitError_None;
	}

	void
	Deactivate()
	{
		ovrd_log("deactivate controller\n");
		m_poseUpdating = false;
		m_poseUpdateThread->join();
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	void
	EnterStandby()
	{
		ovrd_log("standby controller\n");
	}

	void *
	GetComponent(const char *pchComponentNameAndVersion)
	{
		// deprecated API
		// ovrd_log("get controller component %s\n",
		// pchComponentNameAndVersion);
		return NULL;
	}

	/** debug request from a client */
	void
	DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	vr::DriverPose_t
	GetPose()
	{
		// monado predicts pose "now", see xrt_device_get_tracked_pose
		m_pose.poseTimeOffset = 0;

		m_pose.poseIsValid = true;
		m_pose.result = vr::TrackingResult_Running_OK;
		m_pose.deviceIsConnected = true;

		enum xrt_input_name grip_name;

		//! @todo better method to find grip name
		if (m_xdev->name == XRT_DEVICE_VIVE_WAND) {
			grip_name = XRT_INPUT_VIVE_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_INDEX_CONTROLLER) {
			grip_name = XRT_INPUT_INDEX_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_FLIPVR) {
			grip_name = XRT_INPUT_FLIPVR_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_PSMV) {
			grip_name = XRT_INPUT_PSMV_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_DAYDREAM) {
			grip_name = XRT_INPUT_DAYDREAM_POSE;
		} else if (m_xdev->name == XRT_DEVICE_HYDRA) {
			grip_name = XRT_INPUT_HYDRA_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER ||
		           m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1) {
			// The Rift CV1's Touch is its own device name, with the Touch's inputs.
			grip_name = XRT_INPUT_TOUCH_GRIP_POSE;
		} else if (m_xdev->name == XRT_DEVICE_SIMPLE_CONTROLLER) {
			grip_name = XRT_INPUT_SIMPLE_GRIP_POSE;
		} else {
			ovrd_log("Unhandled device name %u\n", m_xdev->name);
			grip_name = XRT_INPUT_GENERIC_HEAD_POSE; // ???
		}

		timepoint_ns now_ns = os_monotonic_get_ns();

		struct xrt_space_relation rel;
		xrt_device_get_tracked_pose(m_xdev, grip_name, now_ns, &rel);

		struct xrt_pose *offset = &m_xdev->tracking_origin->initial_offset;

		struct xrt_relation_chain chain = {};
		/*
		 * SteamVR draws a CV1 Touch, and its laser, from the Oculus driver's raw pose, where the CV1 model's
		 * openxr_grip part sits 20.6 degrees up and 10 cm forward: the grip pose taken as the raw one put the
		 * laser 39 degrees low. Undone here, so SteamVR's own grip is the grip again.
		 */
		if (m_xdev->name == XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1) {
			const struct xrt_pose model_grip = {
			    .orientation = {.x = 0.1788017f, .y = 0.0f, .z = 0.0f, .w = 0.9838847f},
			    .position = {.x = m_hand == XRT_HAND_LEFT ? 0.007f : -0.007f,
			                 .y = -0.00182941f,
			                 .z = 0.1019482f},
			};
			m_relation_chain_push_inverted_pose_if_not_identity(&chain, &model_grip);
		}
		// Tracking as SteamVR gets it, logged on change (the left Touch once disappeared in Home).
		{
			const bool rot = (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT) != 0;
			const bool pos = (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
			if (rot != m_logged_rot || pos != m_logged_pos) {
				ovrd_log("Controller %s: orientation %s, position %s\n", m_hand == XRT_HAND_LEFT ? "left" : "right",
				         rot ? "tracked" : "LOST", pos ? "tracked" : "not tracked (held)");
				m_logged_rot = rot;
				m_logged_pos = pos;
			}
		}
		m_relation_chain_push_relation(&chain, &rel);
		m_relation_chain_push_pose_if_not_identity(&chain, offset);
		m_relation_chain_resolve(&chain, &rel);

		apply_pose(&rel, &m_pose);
		// Before its first tracked position (no sensor placed yet) a controller would sit on the floor at the
		// room's middle: not shown until it is tracked; after that its last position is held (m_pose keeps it).
		if ((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
			m_position_known = true;
		} else if (!m_position_known) {
			m_pose.poseIsValid = false;
			m_pose.result = vr::TrackingResult_Running_OutOfRange;
		}

#ifdef DUMP_POSE_CONTROLLERS
		ovrd_log("get controller %d pose %f %f %f %f, %f %f %f\n", m_unObjectId, m_pose.qRotation.x,
		         m_pose.qRotation.y, m_pose.qRotation.z, m_pose.qRotation.w, m_pose.vecPosition[0],
		         m_pose.vecPosition[1], m_pose.vecPosition[2]);
#endif
		vr::HmdQuaternion_t identityquat{1, 0, 0, 0};
		m_pose.qWorldFromDriverRotation = identityquat;
		m_pose.qDriverFromHeadRotation = identityquat;
		m_pose.vecDriverFromHeadTranslation[0] = 0.f;
		m_pose.vecDriverFromHeadTranslation[1] = 0.f;
		m_pose.vecDriverFromHeadTranslation[2] = 0.f;

		/*
		 * After the identity above, which overwrote the world rotation and left its translation: the
		 * controllers got half the HMD's transform and sat 0.3-1.2 m to one side of the user's hands.
		 */
		apply_world_from_driver(&m_pose);

		return m_pose;
	}

	void
	RunFrame()
	{
		m_xdev->update_inputs(m_xdev);

		// The battery, every 5 s: for SteamVR's controller icons and the Rift tab's status.
		const int64_t now_ns = os_monotonic_get_ns();
		if (m_xdev->supported.battery_status && m_unObjectId != vr::k_unTrackedDeviceIndexInvalid &&
		    now_ns - m_battery_checked_ns > 5LL * U_TIME_1S_IN_NS) {
			m_battery_checked_ns = now_ns;
			bool present = false, charging = false;
			float charge = 0.0f;
			if (xrt_device_get_battery_status(m_xdev, &present, &charging, &charge) == XRT_SUCCESS && present) {
				// The controller's own figure (its radio's ADC channel 0x21: 0-100; Oculus' runtime read the same
				// 100 from these controllers on Windows, in a capture).
				const int percent = std::clamp((int)std::lround(charge * 100.0f), 0, 100);
				if (percent != m_battery_percent.load()) {
					ovrd_log("Controller %s: battery %d %%%s\n", m_hand == XRT_HAND_LEFT ? "left" : "right", percent,
					         charging ? ", charging" : "");
					m_battery_percent.store(percent);
					vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer,
					                                     vr::Prop_DeviceBatteryPercentage_Float,
					                                     std::clamp(charge, 0.0f, 1.0f));
					vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DeviceIsCharging_Bool,
					                                    charging);
				}
			}
		}


		for (const auto &in : m_input_controls) {

			// ovrd_log("Update %d: %s\n", i,
			// m_controls[i].steamvr_control_path);

			enum xrt_input_name binding_name = in.monado_input_name;

			struct xrt_input *input = NULL;
			for (uint32_t ii = 0; ii < m_xdev->input_count; ii++) {
				if (m_xdev->inputs[ii].name == binding_name) {
					input = &m_xdev->inputs[ii];
					break;
				}
			}

			if (input == NULL) {
				ovrd_log("Input for %s not found!\n", in.steamvr_control_path);
				continue;
			}

			vr::VRInputComponentHandle_t handle = in.control_handle;

			if (in.monado_input_type == XRT_INPUT_TYPE_BOOLEAN) {
				bool state = input->value.boolean;
				if (binding_name == XRT_INPUT_TOUCH_MENU_CLICK &&
				    os_monotonic_get_ns() < g_dashboard_toggle_until_ns.load()) {
					state = true;
				}
				vr::VRDriverInput()->UpdateBooleanComponent(handle, state, 0);
				// ovrd_log("Update %s: %d\n",
				// m_controls[i].steamvr_control_path, state);
				// U_LOG_D("Update %s: %d",
				//       m_controls[i].steamvr_control_path,
				//       state);
			}
			if (in.monado_input_type == XRT_INPUT_TYPE_VEC1_MINUS_ONE_TO_ONE ||
			    in.monado_input_type == XRT_INPUT_TYPE_VEC1_ZERO_TO_ONE ||
			    in.monado_input_type == XRT_INPUT_TYPE_VEC2_MINUS_ONE_TO_ONE) {

				float value;
				if (in.component.has_component && in.component.x) {
					value = input->value.vec2.x;
				} else if (in.component.has_component && in.component.y) {
					value = input->value.vec2.y;
				} else {
					value = input->value.vec1.x;
				}

				vr::VRDriverInput()->UpdateScalarComponent(handle, value, 0);
				// ovrd_log("Update %s: %f\n",
				// m_controls[i].steamvr_control_path,
				// state->x);
				// U_LOG_D("Update %s: %f",
				//       m_controls[i].steamvr_control_path,
				//       value);
			}
		}

#ifdef XRT_OS_LINUX
		/*
		 * The right Touch's Oculus button, which SteamVR's Touch profile has no place for: as on Windows, a
		 * press opens the Rift's own panel (SteamVR's dashboard on the Rift tab: volume and mute; SteamVR on
		 * Linux has no volume control) or closes the dashboard when it is open; held for a second it recentres as
		 * SteamVR's own Recenter does (the seated origin; the helper asks SteamVR). It moved the driver's world
		 * before, which SteamVR does not know of: its dashboard and seated games' seats ended up off.
		 */
		if (m_hand == XRT_HAND_RIGHT) {
			bool down = false;
			for (uint32_t ii = 0; ii < m_xdev->input_count; ii++) {
				if (m_xdev->inputs[ii].name == XRT_INPUT_TOUCH_SYSTEM_CLICK) {
					down = m_xdev->inputs[ii].value.boolean;
					break;
				}
			}
			const int64_t now_ns = os_monotonic_get_ns();
			if (down && !m_oculusButtonDown) {
				m_oculusButtonSinceNs = now_ns;
				m_oculusButtonHeld = false;
			} else if (down && !m_oculusButtonHeld && now_ns - m_oculusButtonSinceNs >= U_TIME_1S_IN_NS) {
				m_oculusButtonHeld = true;
				riftlinux_helper_command("cmd-recentre");
			} else if (!down && m_oculusButtonDown && !m_oculusButtonHeld) {
				if (g_dashboard_visible.load()) {
					// A press and release of the left menu button, SteamVR's dashboard toggle.
					g_dashboard_toggle_until_ns.store(now_ns + 120 * U_TIME_1MS_IN_NS);
					ovrd_log("Rift panel: closing the dashboard\n");
				} else {
					riftlinux_helper_command("cmd-open-quick");
				}
			}
			m_oculusButtonDown = down;
		}
#endif

		if (m_xdev->supported.hand_tracking && m_skeletal_input_control.control_handle) {
			constexpr const std::size_t sk_comp_size = 2;

			using ht_input_name_list = std::array<const enum xrt_input_name, sk_comp_size>;
			constexpr const std::array<const ht_input_name_list, sk_comp_size> ht_input_name_map = {
			    ht_input_name_list{
			        XRT_INPUT_HT_UNOBSTRUCTED_LEFT,
			        XRT_INPUT_HT_CONFORMING_LEFT,
			    },
			    ht_input_name_list{
			        XRT_INPUT_HT_UNOBSTRUCTED_RIGHT,
			        XRT_INPUT_HT_CONFORMING_RIGHT,
			    },
			};
			using bone_transform_list = std::array<vr::VRBoneTransform_t, OPENVR_BONE_COUNT>;
			using bone_transforms_list = std::vector<bone_transform_list>;

			bone_transforms_list bone_transforms;
			bone_transforms.reserve(sk_comp_size);
			const auto &ht_input_names = ht_input_name_map[m_hand];
			const timepoint_ns now_ns = os_monotonic_get_ns();

			for (size_t i = 0; i < ht_input_names.size(); ++i) {
				int64_t out_timestamp_ns;
				struct xrt_hand_joint_set joint_set_value = {};
				if (m_xdev->get_hand_tracking(m_xdev, ht_input_names[i], now_ns, &joint_set_value,
				                              &out_timestamp_ns) != XRT_SUCCESS) {
					continue;
				}
				auto &bts = bone_transforms.emplace_back();
				hand_joint_set_to_bone_transform(joint_set_value, bts.data(), m_hand);
				// hand_joint_set_to_bone_transforms(out_joint_set_value, bone_transforms);
			}

			if (bone_transforms.empty())
				return;
			if (bone_transforms.size() < sk_comp_size) {
				bone_transforms.push_back(bone_transforms.back());
			}
			assert(bone_transforms.size() >= sk_comp_size);

			vr::EVRInputError err = vr::VRDriverInput()->UpdateSkeletonComponent(
			    m_skeletal_input_control.control_handle, vr::VRSkeletalMotionRange_WithoutController,
			    bone_transforms[0].data(), OPENVR_BONE_COUNT);
			if (err != vr::VRInputError_None) {
				ovrd_log("error updating skeleton: %i ", err);
			}

			err = vr::VRDriverInput()->UpdateSkeletonComponent(
			    m_skeletal_input_control.control_handle, vr::VRSkeletalMotionRange_WithController,
			    bone_transforms[1].data(), OPENVR_BONE_COUNT);
			if (err != vr::VRInputError_None) {
				ovrd_log("error updating skeleton: %i ", err);
			}
		}
	}


	vr::VRControllerState_t
	GetControllerState()
	{
		// deprecated API
		vr::VRControllerState_t controllerstate{};
		return controllerstate;
	}

	bool
	TriggerHapticPulse(uint32_t unAxisId, uint16_t usPulseDurationMicroseconds)
	{
		// deprecated API
		return false;
	}

	std::string
	GetSerialNumber() const
	{
		ovrd_log("get controller serial number: %s\n", m_sSerialNumber);
		return m_sSerialNumber;
	}


	struct xrt_device *m_xdev;
	vr::DriverPose_t m_pose;
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	bool m_emulate_index_controller = false;

	//! The battery as last read, %; -1 not read yet. For the Rift tab's status (SteamVR gets it as a property).
	std::atomic<int> m_battery_percent{-1};
	int64_t m_battery_checked_ns = 0;

	std::vector<struct SteamVRDriverControlInput> m_input_controls;
	struct SteamVRDriverControlInput m_skeletal_input_control;

	std::vector<struct SteamVRDriverControlOutput> m_output_controls;

private:
	char m_sSerialNumber[XRT_DEVICE_NAME_LEN];
	char m_sModelNumber[MODELNUM_LEN];

	const char *m_controller_type = NULL;

	const char *m_render_model = NULL;
	enum xrt_hand m_hand;
	//! Tracking last logged, see GetPose.
	bool m_logged_rot = true;
	bool m_logged_pos = true;
	//! The controller had a tracked position since SteamVR started (before that it is not shown, see GetPose).
	bool m_position_known = false;
	bool m_handed_controller;

	std::string m_input_profile;

	//! The right Touch's Oculus button: down, since when, held long enough to recentre.
	bool m_oculusButtonDown = false;
	int64_t m_oculusButtonSinceNs = 0;
	bool m_oculusButtonHeld = false;

	bool m_poseUpdating = true;
	std::thread *m_poseUpdateThread = NULL;
};

/*
 *
 * Device driver
 *
 */

#ifdef OVRD_HAVE_PULSE_HEADERS
/*
 * The headset's headphones and microphone as the default audio output and input while SteamVR runs, as SteamVR does
 * on Windows. On Linux its PipeWire routing kept SteamVR Home's sound on the desktop's default (an S/PDIF port nothing
 * is plugged into) although the HMD names its playback device.
 *
 * Through libpulse, loaded at run time (rl_audio_switch.hpp, shared with RiftLinux's own runtime): SteamVR started
 * from Steam runs in Valve's runtime container, which has the library and the desktop's sound server socket but no
 * pactl (the driver ran pactl before: no sound in games). PipeWire answers the same protocol. Not linked, so the driver
 * still loads where there is no libpulse.
 */
static rl::audio::Switch g_audio_switch;

static void
audio_log(const std::string &line)
{
	ovrd_log("%s\n", line.c_str());
}
#endif

class CDeviceDriver_Monado : public vr::ITrackedDeviceServerDriver, public vr::IVRDisplayComponent
{
public:
	CDeviceDriver_Monado(struct xrt_instance *xinst, struct xrt_device *xdev) : m_xdev(xdev)
	{
		//! @todo latency
		m_flSecondsFromVsyncToPhotons = 0.011f;

		float ns = (float)m_xdev->hmd->screens->nominal_frame_interval_ns;
		m_flDisplayFrequency = 1.f / ns * 1000.f * 1000.f * 1000.f;
		ovrd_log("display frequency from device: %f\n", m_flDisplayFrequency);

		// steamvr can really misbehave when freq is inf or so
		if (m_flDisplayFrequency < 0 || m_flDisplayFrequency > 1000) {
			ovrd_log("Setting display frequency to 60 Hz!\n");
			m_flDisplayFrequency = 60.f;
		}

		//! @todo get ipd user setting from monado session
		float ipd_meters = 0.063f;
		struct xrt_vec3 ipd_vec = {ipd_meters, 0, 0};

		timepoint_ns now_ns = os_monotonic_get_ns();

		//! @todo more than 2 views
		struct xrt_space_relation head_relation;
		xrt_device_get_view_poses(xdev, &ipd_vec, now_ns, XRT_VIEW_TYPE_STEREO, 2, &head_relation, m_fovs,
		                          m_view_pose);

		//! @todo more versatile IPD calculation
		float actual_ipd = -m_view_pose[0].position.x + m_view_pose[1].position.x;

		m_flIPD = actual_ipd;

		ovrd_log("Seconds from Vsync to Photons: %f\n", m_flSecondsFromVsyncToPhotons);
		ovrd_log("Display Frequency: %f\n", m_flDisplayFrequency);
		ovrd_log("IPD: %f\n", m_flIPD);
	};
	// clang-format off
	virtual ~CDeviceDriver_Monado() {};

	// ITrackedDeviceServerDriver
	virtual vr::EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);
	virtual void Deactivate();
	virtual void EnterStandby();
	virtual void *GetComponent(const char *pchComponentNameAndVersion);
	virtual void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize);
	virtual vr::DriverPose_t GetPose();

	// IVRDisplayComponent
	virtual void GetWindowBounds(int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight);
	virtual bool IsDisplayOnDesktop();
	virtual bool IsDisplayRealDisplay();
	virtual void GetRecommendedRenderTargetSize(uint32_t *pnWidth, uint32_t *pnHeight);
	virtual void GetEyeOutputViewport(vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight);
	virtual void GetProjectionRaw(vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom);
	virtual vr::DistortionCoordinates_t ComputeDistortion(vr::EVREye eEye, float fU, float fV);
	virtual bool ComputeInverseDistortion(vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV);

	// clang-format on

private:
	struct xrt_device *m_xdev = NULL;

	// clang-format off

	vr::TrackedDeviceIndex_t m_trackedDeviceIndex = 0;
	vr::PropertyContainerHandle_t m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

	float m_flSecondsFromVsyncToPhotons = -1;
	float m_flDisplayFrequency = -1;
	float m_flIPD = -1;

	struct xrt_fov m_fovs[2];
	struct xrt_pose m_view_pose[2];

	bool m_poseUpdating = true;
	//! The headset's last tracked position, held while it has none (see GetPose).
	struct xrt_vec3 m_last_position = {0.0f, 0.0f, 0.0f};
	bool m_position_known = false;
	bool m_logged_standing = false;
	std::thread *m_poseUpdateThread = NULL;
	virtual void PoseUpdateThreadFunction();

	//! Makes the headset's headphones and microphone the default output and input, see g_audio_switch.
	std::thread *m_audioThread = NULL;

	//! The headset's wear sensor as SteamVR's /proximity, see Activate.
	vr::VRInputComponentHandle_t m_proximity = vr::k_ulInvalidInputComponentHandle;
	struct xrt_input *m_headDetect = NULL;

	// clang-format on
};

static void
create_translation_rotation_matrix(struct xrt_pose *pose, struct vr::HmdMatrix34_t *res)
{
	struct xrt_vec3 t = pose->position;
	struct xrt_quat r = pose->orientation;
	res->m[0][0] = (1.0f - 2.0f * (r.y * r.y + r.z * r.z));
	res->m[1][0] = (r.x * r.y + r.z * r.w) * 2.0f;
	res->m[2][0] = (r.x * r.z - r.y * r.w) * 2.0f;
	res->m[0][1] = (r.x * r.y - r.z * r.w) * 2.0f;
	res->m[1][1] = (1.0f - 2.0f * (r.x * r.x + r.z * r.z));
	res->m[2][1] = (r.y * r.z + r.x * r.w) * 2.0f;
	res->m[0][2] = (r.x * r.z + r.y * r.w) * 2.0f;
	res->m[1][2] = (r.y * r.z - r.x * r.w) * 2.0f;
	res->m[2][2] = (1.0f - 2.0f * (r.x * r.x + r.y * r.y));
	res->m[0][3] = t.x;
	res->m[1][3] = t.y;
	res->m[2][3] = t.z;
}

void
CDeviceDriver_Monado::PoseUpdateThreadFunction()
{
	ovrd_log("Starting HMD pose update thread\n");

	int ticks = 0;
	while (m_poseUpdating) {
		//! @todo figure out the best pose update rate
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_trackedDeviceIndex, GetPose(),
		                                                   sizeof(vr::DriverPose_t));

		if (m_headDetect != NULL && ++ticks % 50 == 0) {
			xrt_device_update_inputs(m_xdev);
			static bool was_worn = false;
			if (m_headDetect->value.boolean && !was_worn) {
				notices_on_put_on();
			}
			was_worn = m_headDetect->value.boolean;
			// Capture runs (headset on a desk): reported worn, so SteamVR stays out of standby and shows the
			// application.
			vr::VRDriverInput()->UpdateBooleanComponent(m_proximity,
			                                            m_headDetect->value.boolean || g_report_worn.load(), 0);
		}
	}
	ovrd_log("Stopping HMD pose update thread\n");
}

vr::EVRInitError
CDeviceDriver_Monado::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
	ovrd_log("Activate tracked device %u: %s\n", unObjectId, m_xdev->str);

	m_trackedDeviceIndex = unObjectId;

	// clang-format off

	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
	//! @todo: proper serial and model number

	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, vr::Prop_DeviceClass_Int32, vr::TrackedDeviceClass_HMD);
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_TrackingSystemName_String, "monado");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "Monado");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, m_xdev->str);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, m_flIPD);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.f);
	vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayFrequency_Float, m_flDisplayFrequency);
	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons);

	// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
	vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, vr::Prop_CurrentUniverseId_Uint64, 2);

	// clang-format on

#ifdef XRT_OS_LINUX
	/*
	 * No Room Setup needed: without a room of the user's own, the driver gives SteamVR one, as SteamVR's Oculus
	 * driver gives it the Guardian on Windows. Its walls are not real ones, so they are not shown.
	 */
	if (!user_room_complete()) {
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_DriverProvidedChaperoneJson_String,
		                                      default_room_json().c_str());
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_DriverProvidedChaperoneVisibility_Bool,
		                                    false);
		ovrd_log("Room: none set up; the driver gives SteamVR a standing room (2 x 2 m, walls hidden)\n");
	}
#endif

#ifdef XRT_OS_LINUX
	/*
	 * SteamVR's compositor has a floor for the darkest output it sends the display (Prop_DisplayGCBlackClamp_Float),
	 * which this driver never set. In the headset the CV1's blacks looked grey and levels 1-4 of 255 merged with
	 * black (test-dark panel). {"display": {"black_clamp": 0.0}} in settings.json sets it, to compare;
	 * without, SteamVR keeps its own.
	 */
	{
		vr::ETrackedPropertyError perr = vr::TrackedProp_Success;
		const float before =
		    vr::VRProperties()->GetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayGCBlackClamp_Float, &perr);
		ovrd_log("Display black clamp: SteamVR's is %f (property error %d)\n", before, static_cast<int>(perr));
		cJSON *settings = riftlinux_settings_load();
		const cJSON *clamp = cJSON_GetObjectItemCaseSensitive(
		    cJSON_GetObjectItemCaseSensitive(settings, "display"), "black_clamp");
		if (cJSON_IsNumber(clamp)) {
			vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_DisplayGCBlackClamp_Float,
			                                     static_cast<float>(clamp->valuedouble));
			ovrd_log("Display black clamp: set to %f from settings.json\n", clamp->valuedouble);
		}
		cJSON_Delete(settings);
	}
#endif
	/*
	 * The Rift CV1's headphones are a USB audio device named after the headset's serial; SteamVR switches the
	 * playback to it while it runs. Without, nothing was heard in SteamVR Home.
	 */
	if (strncmp(m_xdev->serial, "WMHD", 4) == 0) {
		char sink[256];
		// The audio device's USB serial is the HMD's first 13 characters (WMHD316G100C9 of WMHD316G100C9Z, 13
		// characters).
		snprintf(sink, sizeof(sink), "alsa_output.usb-Oculus_Rift_Audio_%.13s-00.analog-stereo",
		         m_xdev->serial);
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
		                                      vr::Prop_Audio_DefaultPlaybackDeviceId_String, sink);
		ovrd_log("Audio playback device: %s\n", sink);

		/*
		 * SteamVR asks the headset whether it is worn (/user/head/proximity) and found no such input on it; the
		 * Oculus driver's Rift profile has one, fed here from the CV1's wear sensor.
		 */
		for (uint32_t i = 0; i < m_xdev->input_count; i++) {
			if (m_xdev->inputs[i].name == XRT_INPUT_GENERIC_HEAD_DETECT) {
				m_headDetect = &m_xdev->inputs[i];
			}
		}
		if (m_headDetect != NULL) {
			vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_InputProfilePath_String,
			                                      "{oculus}/input/rift_profile.json");
			vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/proximity", &m_proximity);
			/*
			 * And that the headset has one: without it SteamVR judged presence by movement alone, and after its
			 * standby (or at start) the picture stayed black until the head moved.
			 */
			vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_ContainsProximitySensor_Bool,
			                                    true);
		}
		/*
		 * Direct mode: SteamVR's per-application throttling and prediction settings apply, as for the
		 * headsets whose drivers reproject themselves; motion smoothing's half-rate lock goes the same way.
		 */
		if (g_direct_mode != nullptr) {
			vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_Hmd_SupportsAppThrottling_Bool,
			                                    true);
			// SteamVR paces to the driver's vsync events (Monado's display) rather than its own vsync model.
			vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer,
			                                    vr::Prop_DriverDirectModeSendsVsyncEvents_Bool, true);
			g_direct_mode->start_vsync_events();
		}
#ifdef OVRD_HAVE_PULSE_HEADERS
		/*
		 * SteamVR knew the headphones only: the microphone, on the same USB audio device, is the headset's recording
		 * device too, named once the sound server has it.
		 */
		m_audioThread = new std::thread(
		    [this, playback = std::string(sink), mic = rl::audio::rift_source_prefix(m_xdev->serial)] {
			    g_audio_switch.to_headset(playback, mic, audio_log);
			    const std::string found = g_audio_switch.found_source();
			    if (!found.empty()) {
				    vr::VRProperties()->SetStringProperty(m_ulPropertyContainer,
				                                          vr::Prop_Audio_DefaultRecordingDeviceId_String,
				                                          found.c_str());
				    ovrd_log("Audio recording device: %s\n", found.c_str());
			    }
		    });
#endif
	}
	// clang-format off

	// avoid "not fullscreen" warnings from vrmonitor
	//vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);

	// clang-format on


	//! @todo update when ipd changes
	vr::HmdMatrix34_t left;
	create_translation_rotation_matrix(&m_view_pose[0], &left);
	vr::HmdMatrix34_t right;
	create_translation_rotation_matrix(&m_view_pose[1], &right);

	vr::VRServerDriverHost()->SetDisplayEyeToHead(m_trackedDeviceIndex, left, right);


	m_poseUpdateThread = new std::thread(&CDeviceDriver_Monado::PoseUpdateThreadFunction, this);
	if (!m_poseUpdateThread) {
		ovrd_log("Unable to create pose updated thread for %s\n", m_xdev->str);
		return vr::VRInitError_Driver_Failed;
	}

	return vr::VRInitError_None;
}

void
CDeviceDriver_Monado::Deactivate()
{
	m_poseUpdating = false;
	m_poseUpdateThread->join();
#ifdef OVRD_HAVE_PULSE_HEADERS
	if (m_audioThread != NULL) {
		m_audioThread->join();
		delete m_audioThread;
		m_audioThread = NULL;
	}
	g_audio_switch.restore(audio_log);
#endif
	ovrd_log("Deactivate\n");
}

void
CDeviceDriver_Monado::EnterStandby()
{
	ovrd_log("Enter Standby\n");
}

void *
CDeviceDriver_Monado::GetComponent(const char *pchComponentNameAndVersion)
{
	// clang-format off
	if (strcmp(pchComponentNameAndVersion, vr::IVRDisplayComponent_Version) == 0) {
		return (vr::IVRDisplayComponent *)this;
	}
	// clang-format on
#ifdef XRT_OS_LINUX
	if (g_direct_mode != nullptr && strcmp(pchComponentNameAndVersion, vr::IVRDriverDirectModeComponent_Version) == 0) {
		return (vr::IVRDriverDirectModeComponent *)g_direct_mode;
	}
#endif

	return NULL;
}

void
CDeviceDriver_Monado::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
	//! @todo
}

static constexpr inline vr::HmdQuaternion_t
HmdQuaternion_Init(double w, double x, double y, double z)
{
	return vr::HmdQuaternion_t{
	    .w = w,
	    .x = x,
	    .y = y,
	    .z = z,
	};
}

vr::DriverPose_t
CDeviceDriver_Monado::GetPose()
{

	timepoint_ns now_ns = os_monotonic_get_ns();
	struct xrt_space_relation rel;
	xrt_device_get_tracked_pose(m_xdev, XRT_INPUT_GENERIC_HEAD_POSE, now_ns, &rel);

	struct xrt_pose *offset = &m_xdev->tracking_origin->initial_offset;

	struct xrt_relation_chain chain = {};
	m_relation_chain_push_relation(&chain, &rel);
	m_relation_chain_push_pose_if_not_identity(&chain, offset);
	m_relation_chain_resolve(&chain, &rel);

	vr::DriverPose_t t = {
	    // monado predicts pose "now", see xrt_device_get_tracked_pose
	    .poseTimeOffset = 0,
	    .qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0),
	    .vecWorldFromDriverTranslation = {0, 0, 0},

	    .qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0),
	    .vecDriverFromHeadTranslation = {0, 0, 0},

	    .vecPosition = {0, 0, 0},
	    .vecVelocity = {0, 0, 0},
	    .vecAcceleration = {0, 0, 0},

	    .qRotation = HmdQuaternion_Init(1, 0, 0, 0),

	    .vecAngularVelocity = {0, 0, 0},
	    .vecAngularAcceleration = {0, 0, 0},

	    .result = vr::TrackingResult_Running_OK,
	    .poseIsValid = (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0,
	    .willDriftInYaw = !m_xdev->supported.position_tracking,
	    //! @todo: Monado head model?
	    .shouldApplyHeadModel = !m_xdev->supported.position_tracking,
	    .deviceIsConnected = true,
	};
	set_world_from_driver_once(m_xdev, &rel);
	floor_watch_head(m_xdev, &rel);
	apply_pose(&rel, &t);
	/*
	 * A frame without a tracked position: apply_pose leaves it at zero, which SteamVR takes as the head on the floor
	 * (the first start on a fresh install: "half under the floor" for the 36 s of the first sensor
	 * calibration, when no sensor is placed yet). The last tracked position is held, not moving; before there is one,
	 * the head stands at an eye height in SteamVR's room (the world is not placed yet), turning only.
	 */
	/*
	 * Unseen by every sensor the tracker holds the headset, turning about the neck: valid but not tracked, with the
	 * velocity of that turn (a pose of the IMU's orientation alone never has one, nor a position of its own). Shown as
	 * handed out, as the Oculus runtime shows a headset it cannot see; it jumped when the copy below was shown.
	 */
	const bool held = (rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) == 0 &&
	                  (rel.relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) != 0 &&
	                  (rel.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0;
	if ((rel.relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0) {
		m_last_position = rel.pose.position;
		m_position_known = true;
	} else if (held && m_position_known) {
		m_last_position = rel.pose.position;
		copy_vec3(&rel.pose.position, t.vecPosition);
	} else {
		struct xrt_vec3 standing = {0.0f, kStandingEyeHeightM + g_world_from_driver.height_m.load(), 0.0f};
		copy_vec3(m_position_known ? &m_last_position : &standing, t.vecPosition);
		t.vecVelocity[0] = t.vecVelocity[1] = t.vecVelocity[2] = 0.0;
		if (!m_position_known && !m_logged_standing) {
			m_logged_standing = true;
			ovrd_log("Headset: no position yet (sensors not placed), held at a standing eye height of %.2f m\n",
			         standing.y);
		}
	}
	apply_world_from_driver(&t);

#ifdef DUMP_POSE
	ovrd_log("get hmd pose %f %f %f %f, %f %f %f\n", t.qRotation.x, t.qRotation.y, t.qRotation.z, t.qRotation.w,
	         t.vecPosition[0], t.vecPosition[1], t.vecPosition[2]);
#endif

	//! @todo
	// copy_vec3(&rel.angular_velocity, t.vecAngularVelocity);
	// copy_vec3(&rel.angular_acceleration, t.vecAngularAcceleration);

	// ovrd_log("Vel: %f %f %f\n", t.vecAngularVelocity[0],
	// t.vecAngularVelocity[1], t.vecAngularVelocity[2]); ovrd_log("Accel:
	// %f %f %f\n", t.vecAngularAcceleration[0],
	// t.vecAngularAcceleration[1], t.vecAngularAcceleration[2]);

	return t;
}

void
CDeviceDriver_Monado::GetWindowBounds(int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight)
{
	// offset in extended mode, e.g. to the right of a 1920x1080 monitor
	*pnX = 1920;
	*pnY = 0;

	*pnWidth = m_xdev->hmd->screens[0].w_pixels;
	*pnHeight = m_xdev->hmd->screens[0].h_pixels;
	;

	ovrd_log("Window Bounds: %dx%d\n", *pnWidth, *pnHeight);
}

bool
CDeviceDriver_Monado::IsDisplayOnDesktop()
{
	return false;
}

bool
CDeviceDriver_Monado::IsDisplayRealDisplay()
{
#ifdef XRT_OS_LINUX
	// In direct mode Monado's compositor drives the display, not SteamVR's.
	if (g_direct_mode != nullptr) {
		return false;
	}
#endif
	return true;
}

void
CDeviceDriver_Monado::GetRecommendedRenderTargetSize(uint32_t *pnWidth, uint32_t *pnHeight)
{
	/*
	 * The CV1's own 1:1 size (Rift driver, from its lens: one rendered pixel per panel pixel at the lens centre, as
	 * Oculus' pixel density 1), times display.render_scale in settings.json (percent of pixels, as SteamVR's
	 * resolution setting; default 100). SteamVR's own resolution setting applies on top. It was the panel times
	 * 140 % per side (Monado's default, from the Vive's lenses), and with SteamVR's automatic 150 % games drew 1.8
	 * times the pixels Oculus asks for (No Man's Sky 1852x2056 a view instead of about 1332x1586).
	 * XRT_COMPOSITOR_SCALE_PERCENTAGE set by hand keeps the old way (per side, of the panel).
	 */
	const struct xrt_view &v = m_xdev->hmd->views[0];
	if (v.native.w_pixels > 0 && getenv("XRT_COMPOSITOR_SCALE_PERCENTAGE") == nullptr) {
		const char *rs = getenv("RIFTLINUX_RENDER_SCALE");
		const float percent = rs != nullptr ? std::clamp((float)atof(rs), 25.0f, 400.0f) : 100.0f;
		const float side = sqrtf(percent / 100.0f);
		*pnWidth = (uint32_t)lroundf((float)v.native.w_pixels * side);
		*pnHeight = (uint32_t)lroundf((float)v.native.h_pixels * side);
		ovrd_log("Render Target Size: %ux%u a view (%.0f %% of the 1:1 size %ux%u)\n", *pnWidth, *pnHeight, percent,
		         v.native.w_pixels, v.native.h_pixels);
		return;
	}

	int scale = debug_get_num_option_scale_percentage();
	float fscale = (float)scale / 100.f;

	/*
	 * Per eye, as SteamVR takes it: the whole screen's width asked a Rift CV1 for 3024x1680 per eye, twice the
	 * pixels, and SteamVR ran at about 20 frames a second.
	 */
	*pnWidth = m_xdev->hmd->views[0].display.w_pixels * fscale;
	*pnHeight = m_xdev->hmd->views[0].display.h_pixels * fscale;

	ovrd_log("Render Target Size: %dx%d (%fx)\n", *pnWidth, *pnHeight, fscale);
}

void
CDeviceDriver_Monado::GetEyeOutputViewport(
    vr::EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight)
{
	*pnWidth = m_xdev->hmd->views[eEye].viewport.w_pixels;
	*pnHeight = m_xdev->hmd->views[eEye].viewport.h_pixels;

	*pnX = m_xdev->hmd->views[eEye].viewport.x_pixels;
	*pnY = m_xdev->hmd->views[eEye].viewport.y_pixels;

	ovrd_log("Output Viewport for eye %d: %dx%d offset %dx%d\n", eEye, *pnWidth, *pnHeight, *pnX, *pnY);
}

void
CDeviceDriver_Monado::GetProjectionRaw(vr::EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom)
{
	*pfLeft = tanf(m_xdev->hmd->distortion.fov[eEye].angle_left);
	*pfRight = tanf(m_xdev->hmd->distortion.fov[eEye].angle_right);
	/*
	 * SteamVR's top is the downward extent and its bottom the upward one, as drv_steamvr_lh reads Valve's own
	 * drivers (angle_up = atan(bottom)). They were the other way around: invisible on a headset whose vertical
	 * field of view is near symmetric, on a Rift CV1 (41.6 degrees up, 48 down) SteamVR rendered 48 up and 41.6
	 * down, and through the lenses the view squeezed looking up and stretched looking down.
	 */
	*pfTop = tanf(m_xdev->hmd->distortion.fov[eEye].angle_down);
	*pfBottom = tanf(m_xdev->hmd->distortion.fov[eEye].angle_up);
	ovrd_log("Projection Raw: L%f R%f T%f B%f\n", *pfLeft, *pfRight, *pfTop, *pfBottom);
}

vr::DistortionCoordinates_t
CDeviceDriver_Monado::ComputeDistortion(vr::EVREye eEye, float fU, float fV)
{
	/** Used to return the post-distortion UVs for each color channel.
	 * UVs range from 0 to 1 with 0,0 in the upper left corner of the
	 * source render target. The 0,0 to 1,1 range covers a single eye. */

	struct xrt_vec2 *rot = m_xdev->hmd->views[eEye].rot.vecs;

	// multiply 2x2 rotation matrix with fU, fV scaled to [-1, 1]
	float U = rot[0].x * (fU * 2 - 1) + rot[0].y * (fV * 2 - 1);
	float V = rot[1].x * (fU * 2 - 1) + rot[1].y * (fV * 2 - 1);

	// scale U, V back to [0, 1]
	U = (U + 1) / 2;
	V = (V + 1) / 2;

	struct xrt_uv_triplet d;

	xrt_result_t xret = m_xdev->compute_distortion(m_xdev, eEye, U, V, &d);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Failed to compute distortion for view %d at %f,%f! Got error %d\n", eEye, U, V, xret);

		vr::DistortionCoordinates_t coordinates;
		coordinates.rfBlue[0] = U;
		coordinates.rfBlue[1] = V;
		coordinates.rfGreen[0] = U;
		coordinates.rfGreen[1] = V;
		coordinates.rfRed[0] = U;
		coordinates.rfRed[1] = V;
		return coordinates;
	}

	vr::DistortionCoordinates_t coordinates;
	coordinates.rfRed[0] = d.r.x;
	coordinates.rfRed[1] = d.r.y;
	coordinates.rfGreen[0] = d.g.x;
	coordinates.rfGreen[1] = d.g.y;
	coordinates.rfBlue[0] = d.b.x;
	coordinates.rfBlue[1] = d.b.y;

	// ovrd_log("Computed distortion for view %d at %f,%f -> %f,%f | %f,%f |
	// %f,%f!\n", eEye, U, V, d.r.x, d.r.y, d.g.x, d.g.y, d.b.x, d.b.y);

	return coordinates;
}

bool
CDeviceDriver_Monado::ComputeInverseDistortion(
    vr::HmdVector2_t *pResult, vr::EVREye eEye, uint32_t unChannel, float fU, float fV)
{
	// Unsupported, so return false
	return false;
}


/*
 *
 * Device driver server
 *
 */

#ifdef XRT_OS_LINUX
/*
 * Notices in the headset (a sensor moved and is being placed again, see u_notice.h). A driver has no way to show
 * anything in the headset, so they go to riftlinux-notices, a small SteamVR overlay app next to this driver, started
 * with SteamVR; it shows each for a few seconds in front of the view, and is the dashboard's Rift tab (volume). Each kind can be switched off in
 * ~/.config/riftlinux/settings.json, which the KDE app is to write, read again whenever it changes:
 *   {"notices": {"enabled": true, "sensors": true, "calibration": true, "tracking": true, "display": true}}
 * Missing file or key: on. "test_at_start": "<panel name>" there shows that panel once, 15 s after start.
 */
static struct
{
	std::mutex lock;
	int fd = -1;
	pid_t pid = 0;
	//! Helpers not yet reaped, each waited for by its own thread (see notices_spawn_locked).
	std::vector<pid_t> live;
	int64_t last_spawn_ns = 0;
	bool enabled = true;
	bool kinds[5] = {true, true, true, true, true};
	//! The welcome panel was shown once (notices.welcome_shown; false shows it again at the next put-on).
	bool welcome_shown = false;
	//! When the headset was first put on since SteamVR started (0: not yet), and whether the controller hint ran.
	int64_t first_put_on_ns = 0;
	bool controller_hint_done = false;
	//! When the first measuring of the sensors was last said (0: not since it began), see notices_first_calibration.
	int64_t first_calibration_shown_ns = 0;
	//! The headset worn, as last seen once a second (RunFrame).
	bool worn = false;
	//! Test: as if the headset was put on this many seconds after the driver started (notices.test_put_on_s, desk).
	int test_put_on_s = 0;
	int64_t settings_mtime_ns = -1;
	//! SteamVR is shutting the driver down: nothing more is started.
	bool stopped = false;
	//! Seconds after SteamVR's start before the helper is (notices.start_delay_s, a test key: 0 meets a compositor
	//! not up yet).
	int start_delay_s = 5;
	//! A helper that ended by failing is started again from SteamVR's frame (riftlinux_helper_poll) at this time.
	int64_t restart_due_ns = 0;
	int restarts = 0;
	//! Notices posted before the helper runs (the compositor's, at start), shown once it does.
	std::vector<std::string> pending;
	//! Notices about the picture, posted at start while the headset may still lie on the desk: shown again the first
	//! time it is put on.
	std::vector<std::string> on_put_on;
} g_notices;

static std::string
riftlinux_settings_path()
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	const std::string dir = xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "") + "/.config";
	return dir + "/riftlinux/settings.json";
}

//! The settings file parsed, or NULL (none, unreadable); free with cJSON_Delete.
static cJSON *
riftlinux_settings_load()
{
	FILE *file = fopen(riftlinux_settings_path().c_str(), "r");
	if (file == nullptr) {
		return nullptr;
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0 && text.size() < 65536) {
		text.append(buf, n);
	}
	fclose(file);
	return cJSON_Parse(text.c_str());
}

/*
 * Out of the box (install the package once and the CV1 works, regardless of room setup):
 * what SteamVR or OpenXR would otherwise need the user to set up, the driver provides itself, never replacing
 * what the user chose.
 */

//! A small file's text, or empty.
static std::string
read_small_file(const std::string &path)
{
	std::string text;
	FILE *file = fopen(path.c_str(), "r");
	if (file == nullptr) {
		return text;
	}
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0 && text.size() < (1u << 20)) {
		text.append(buf, n);
	}
	fclose(file);
	return text;
}

//! SteamVR's install directory, from this process (vrserver is SteamVR/bin/linux64/vrserver), or empty.
static std::string
steamvr_install_dir()
{
	char buf[4096];
	ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
	if (n <= 0) {
		return {};
	}
	std::string p(buf, (size_t)n);
	for (int i = 0; i < 3; i++) {
		size_t slash = p.find_last_of('/');
		if (slash == std::string::npos || slash == 0) {
			return {};
		}
		p.resize(slash);
	}
	return p;
}

//! SteamVR's config directories: openvrpaths.vrpath's "config" entries, else Steam's usual one.
static std::vector<std::string>
steamvr_config_dirs()
{
	std::vector<std::string> dirs;
	const char *home = getenv("HOME");
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const std::string cfg = xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "") + "/.config";
	cJSON *vp = cJSON_Parse(read_small_file(cfg + "/openvr/openvrpaths.vrpath").c_str());
	const cJSON *c = cJSON_GetObjectItemCaseSensitive(vp, "config");
	const cJSON *e = nullptr;
	cJSON_ArrayForEach(e, c)
	{
		if (cJSON_IsString(e)) {
			dirs.push_back(e->valuestring);
		}
	}
	cJSON_Delete(vp);
	if (home != nullptr) {
		dirs.push_back(std::string(home) + "/.steam/steam/config");
	}
	return dirs;
}

/*!
 * The driver's own room (default_room_json) as SteamVR saves it into the user's file: SteamVR's recenter writes the
 * room it was given back with the seated spot. Taken for a room of the user's own, its walls were shown, 1 m around
 * the spot the world was placed on, in blue at arm's length.
 */
static bool
is_driver_room(const cJSON *universe)
{
	const cJSON *area = cJSON_GetObjectItemCaseSensitive(universe, "play_area");
	const cJSON *walls = cJSON_GetObjectItemCaseSensitive(universe, "collision_bounds");
	if (cJSON_GetArraySize(area) != 2 || cJSON_GetArraySize(walls) != 4) {
		return false;
	}
	const cJSON *v = nullptr;
	cJSON_ArrayForEach(v, area)
	{
		if (!cJSON_IsNumber(v) || fabs(v->valuedouble - 2.0) > 1e-3) {
			return false;
		}
	}
	const cJSON *wall = nullptr;
	cJSON_ArrayForEach(wall, walls)
	{
		if (cJSON_GetArraySize(wall) != 4) {
			return false;
		}
		const cJSON *corner = nullptr;
		cJSON_ArrayForEach(corner, wall)
		{
			const cJSON *x = cJSON_GetArrayItem(corner, 0);
			const cJSON *y = cJSON_GetArrayItem(corner, 1);
			const cJSON *z = cJSON_GetArrayItem(corner, 2);
			if (cJSON_GetArraySize(corner) != 3 || !cJSON_IsNumber(x) || !cJSON_IsNumber(y) || !cJSON_IsNumber(z) ||
			    fabs(fabs(x->valuedouble) - 1.0) > 1e-3 || fabs(fabs(z->valuedouble) - 1.0) > 1e-3 ||
			    (fabs(y->valuedouble) > 1e-3 && fabs(y->valuedouble - 2.43) > 1e-3)) {
				return false;
			}
		}
	}
	return true;
}

//! Whether the user has a complete room for this driver's universe (SteamVR's Room Setup, or riftlinux-setup's).
static bool
user_room_complete()
{
	for (const std::string &d : steamvr_config_dirs()) {
		cJSON *c = cJSON_Parse(read_small_file(d + "/chaperone_info.vrchap").c_str());
		bool complete = false;
		const cJSON *u = nullptr;
		cJSON_ArrayForEach(u, cJSON_GetObjectItemCaseSensitive(c, "universes"))
		{
			const cJSON *id = cJSON_GetObjectItemCaseSensitive(u, "universeID");
			if (!cJSON_IsString(id) || strcmp(id->valuestring, "2") != 0) {
				continue;
			}
			complete = cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(u, "play_area")) == 2 &&
			           cJSON_GetArraySize(cJSON_GetObjectItemCaseSensitive(u, "collision_bounds")) > 0 &&
			           cJSON_GetObjectItemCaseSensitive(u, "standing") != nullptr &&
			           cJSON_GetObjectItemCaseSensitive(u, "seated") != nullptr && !is_driver_room(u);
		}
		cJSON_Delete(c);
		if (complete) {
			return true;
		}
	}
	return false;
}

//! A standing room of 2 x 2 m on the floor where the driver places it, seated at the origin (SteamVR's format).
static std::string
default_room_json()
{
	const char *walls = "[[[-1,0,-1],[-1,2.43,-1],[1,2.43,-1],[1,0,-1]],[[1,0,-1],[1,2.43,-1],[1,2.43,1],[1,0,1]],"
	                    "[[1,0,1],[1,2.43,1],[-1,2.43,1],[-1,0,1]],[[-1,0,1],[-1,2.43,1],[-1,2.43,-1],[-1,0,-1]]]";
	return std::string("{\"jsonid\":\"chaperone_info\",\"version\":5,\"universes\":[{\"collision_bounds\":") + walls +
	       ",\"play_area\":[2,2],\"seated\":{\"translation\":[0,0,0],\"yaw\":0},"
	       "\"standing\":{\"translation\":[0,0,0],\"yaw\":0},\"universeID\":\"2\"}]}";
}

//! OpenXR games find SteamVR (else they open on the monitor): set it as the active runtime when none is set.
static void
openxr_runtime_default()
{
	const char *home = getenv("HOME");
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const std::string cfg = xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "") + "/.config";
	const std::string user = cfg + "/openxr/1/active_runtime.json";
	struct stat st;
	if (lstat(user.c_str(), &st) == 0) {
		return; // The user's choice (or a dangling one: also theirs).
	}
	const char *dirs = getenv("XDG_CONFIG_DIRS");
	std::string list = dirs != nullptr && dirs[0] != '\0' ? dirs : "/etc/xdg";
	size_t start = 0;
	while (start <= list.size()) {
		size_t end = list.find(':', start);
		std::string d = list.substr(start, end == std::string::npos ? std::string::npos : end - start);
		if (!d.empty() && stat((d + "/openxr/1/active_runtime.json").c_str(), &st) == 0) {
			return; // A system-wide runtime.
		}
		if (end == std::string::npos) {
			break;
		}
		start = end + 1;
	}
	const std::string steamxr = steamvr_install_dir() + "/steamxr_linux64.json";
	if (stat(steamxr.c_str(), &st) != 0) {
		return;
	}
	const std::string dir = cfg + "/openxr/1";
	mkdir((cfg + "/openxr").c_str(), 0755);
	mkdir(dir.c_str(), 0755);
	if (symlink(steamxr.c_str(), user.c_str()) == 0) {
		ovrd_log("OpenXR: no runtime was set; SteamVR set (%s)\n", steamxr.c_str());
	}
}

//! Sets @p section.@p key to @p item (taken) in the settings file, keeping everything else in it.
static void
riftlinux_settings_set_item(const char *section, const char *key, cJSON *item)
{
	// Read, changed, written whole: one writer at a time, or a change made at the same moment is lost.
	static std::mutex write_lock;
	std::lock_guard<std::mutex> lock(write_lock);
	cJSON *root = riftlinux_settings_load();
	if (root == nullptr) {
		root = cJSON_CreateObject();
	}
	cJSON *sec = cJSON_GetObjectItemCaseSensitive(root, section);
	if (!cJSON_IsObject(sec)) {
		cJSON_DeleteItemFromObjectCaseSensitive(root, section);
		sec = cJSON_AddObjectToObject(root, section);
	}
	cJSON_DeleteItemFromObjectCaseSensitive(sec, key);
	cJSON_AddItemToObject(sec, key, item);
	char *text = cJSON_Print(root);
	cJSON_Delete(root);
	if (text == nullptr) {
		return;
	}
	const std::string path = riftlinux_settings_path();
	const std::string dir = path.substr(0, path.rfind('/'));
	mkdir(dir.c_str(), 0755);
	const std::string tmp = path + ".tmp";
	FILE *file = fopen(tmp.c_str(), "w");
	if (file != nullptr) {
		fputs(text, file);
		fputc('\n', file);
		fclose(file);
		rename(tmp.c_str(), path.c_str());
	}
	cJSON_free(text);
}

//! Sets @p section.@p key to the number @p value in the settings file.
static void
riftlinux_settings_set_number(const char *section, const char *key, double value)
{
	riftlinux_settings_set_item(section, key, cJSON_CreateNumber(value));
}

//! Sets @p section.@p key to true or false in the settings file (keys read as such, e.g. display.direct_mode).
static void
riftlinux_settings_set_bool(const char *section, const char *key, bool value)
{
	riftlinux_settings_set_item(section, key, cJSON_CreateBool(value));
}

//! The floor and the user's height from the settings file.
static void
riftlinux_load_floor()
{
	cJSON *settings = riftlinux_settings_load();
	const cJSON *tracking = cJSON_GetObjectItemCaseSensitive(settings, "tracking");
	const cJSON *floor = cJSON_GetObjectItemCaseSensitive(tracking, "floor_m");
	const cJSON *height = cJSON_GetObjectItemCaseSensitive(tracking, "height_cm");
	const cJSON *eye = cJSON_GetObjectItemCaseSensitive(tracking, "eye_height_m");
	g_world_from_driver.floor_m.store(cJSON_IsNumber(floor) ? (float)floor->valuedouble : 0.0f);
	g_world_from_driver.floor_known.store(cJSON_IsNumber(floor));
	g_world_from_driver.height_m.store(cJSON_IsNumber(height) ? (float)height->valuedouble / 100.0f : 0.0f);
	if (cJSON_IsNumber(eye)) {
		std::lock_guard<std::mutex> lock(g_recal.lock);
		g_recal.keeper.set_saved_eye((float)eye->valuedouble);
	}
	cJSON_Delete(settings);
	ovrd_log("Floor %.3f m, height %+.0f cm (settings.json)\n", g_world_from_driver.floor_m.load(),
	         g_world_from_driver.height_m.load() * 100.0f);
}

/*!
 * From RunFrame: the floor after a recalibration once the head's heights are in (see recalibration_done), and what
 * is to be saved: the floor, and the learned standing eye height (after 30 more seconds of it, when it changed).
 */
static void
floor_run_frame()
{
	std::optional<float> save_floor;
	std::optional<float> save_eye;
	{
		std::lock_guard<std::mutex> lock(g_recal.lock);
		const float before = g_world_from_driver.floor_m.load();
		const std::optional<rl::floor::Result> r = g_recal.keeper.tick(floor_now_s());
		if (r) {
			g_world_from_driver.floor_m.store(r->floor_m);
			g_recal.floor_to_save = true;
			ovrd_log("Sensors measured again: floor %.3f m (was %.3f), %s\n", r->floor_m, before, r->how.c_str());
		}
		if (g_recal.floor_to_save) {
			g_recal.floor_to_save = false;
			save_floor = g_world_from_driver.floor_m.load();
		}
		save_eye = g_recal.keeper.eye_to_save();
	}
	if (save_floor) {
		riftlinux_settings_set_number("tracking", "floor_m", *save_floor);
	}
	if (save_eye) {
		riftlinux_settings_set_number("tracking", "eye_height_m", *save_eye);
		ovrd_log("Standing eye height learned: %.2f m\n", *save_eye);
	}
}

//! Reads the switches if the file changed since; also the test panel, when @p out_test is given. Lock held.
static void
notices_read_settings_locked(std::string *out_test)
{
	const std::string path = riftlinux_settings_path();
	struct stat st;
	const int64_t mtime = stat(path.c_str(), &st) == 0
	                          ? static_cast<int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec
	                          : -2;
	if (mtime == g_notices.settings_mtime_ns && out_test == nullptr) {
		return;
	}
	g_notices.settings_mtime_ns = mtime;
	g_notices.enabled = true;
	for (bool &kind : g_notices.kinds) {
		kind = true;
	}
	g_notices.welcome_shown = false;
	if (mtime == -2) {
		return;
	}

	cJSON *root = riftlinux_settings_load();
	const cJSON *notices = cJSON_GetObjectItemCaseSensitive(root, "notices");
	auto flag = [&](const char *name, bool &out) {
		const cJSON *item = cJSON_GetObjectItemCaseSensitive(notices, name);
		if (cJSON_IsBool(item)) {
			out = cJSON_IsTrue(item);
		}
	};
	flag("enabled", g_notices.enabled);
	flag("sensors", g_notices.kinds[U_NOTICE_SENSORS]);
	flag("calibration", g_notices.kinds[U_NOTICE_CALIBRATION]);
	flag("tracking", g_notices.kinds[U_NOTICE_TRACKING]);
	flag("display", g_notices.kinds[U_NOTICE_DISPLAY]);
	flag("system", g_notices.kinds[U_NOTICE_SYSTEM]);
	flag("welcome_shown", g_notices.welcome_shown);
	const cJSON *test_put_on = cJSON_GetObjectItemCaseSensitive(notices, "test_put_on_s");
	if (cJSON_IsNumber(test_put_on)) {
		g_notices.test_put_on_s = std::clamp(test_put_on->valueint, 0, 600);
	}
	const cJSON *delay = cJSON_GetObjectItemCaseSensitive(notices, "start_delay_s");
	if (cJSON_IsNumber(delay)) {
		g_notices.start_delay_s = std::clamp(delay->valueint, 0, 60);
	}
	const cJSON *test = cJSON_GetObjectItemCaseSensitive(notices, "test_at_start");
	if (out_test != nullptr && cJSON_IsString(test)) {
		*out_test = test->valuestring;
	}
	cJSON_Delete(root);
	ovrd_log("Notices: %s (sensors %d, calibration %d, tracking %d, display %d, system %d; welcome %s), from %s\n",
	         g_notices.enabled ? "on" : "off", g_notices.kinds[0], g_notices.kinds[1], g_notices.kinds[2],
	         g_notices.kinds[3], g_notices.kinds[4], g_notices.welcome_shown ? "shown" : "to show", path.c_str());
}

//! Starts riftlinux-notices, at most every 30 s. Lock held.
static bool
notices_spawn_locked()
{
	const int64_t now_ns = os_monotonic_get_ns();
	if (g_notices.stopped) {
		return false;
	}
	if (g_notices.last_spawn_ns != 0 && now_ns - g_notices.last_spawn_ns < 30LL * U_TIME_1S_IN_NS) {
		return false;
	}
	g_notices.last_spawn_ns = now_ns;

	// Next to this driver; the panels in the package's resources; SteamVR's vrclient.so next to vrserver.
	Dl_info info;
	char exe[PATH_MAX];
	const ssize_t exe_len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
	if (dladdr(reinterpret_cast<void *>(&notices_spawn_locked), &info) == 0 || info.dli_fname == nullptr ||
	    exe_len <= 0) {
		ovrd_log("Notices: can't find riftlinux-notices or SteamVR's vrclient.so\n");
		return false;
	}
	exe[exe_len] = '\0';
	const std::string so = info.dli_fname;
	const std::string dir = so.substr(0, so.rfind('/'));
	std::string helper = dir + "/riftlinux-notices";
	std::string panels = dir + "/../../resources/notices";
	const std::string exe_path = exe;
	std::string vrclient = exe_path.substr(0, exe_path.rfind('/')) + "/vrclient.so";

	int sv[2];
	if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sv) != 0) {
		return false;
	}
	posix_spawn_file_actions_t actions;
	posix_spawn_file_actions_init(&actions);
	posix_spawn_file_actions_adddup2(&actions, sv[1], STDIN_FILENO);
	char *argv[] = {helper.data(), vrclient.data(), panels.data(), nullptr};
	pid_t pid = 0;
	const int rc = posix_spawn(&pid, helper.c_str(), &actions, nullptr, argv, environ);
	posix_spawn_file_actions_destroy(&actions);
	close(sv[1]);
	if (rc != 0) {
		close(sv[0]);
		ovrd_log("Notices: could not start %s (%s)\n", helper.c_str(), strerror(rc));
		return false;
	}
	g_notices.fd = sv[0];
	g_notices.pid = pid;
	g_notices.live.push_back(pid);
	/*
	 * Reaped the moment it ends. It is vrserver's child; at shutdown vrserver waits for overlay applications to
	 * quit, and a helper that had quit but was left a zombie (reaped only from RunFrame, which no longer runs then)
	 * kept vrserver waiting for good, and Steam counted SteamVR as running.
	 */
	std::thread([pid]() {
		int status = 0;
		while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
		}
		std::lock_guard<std::mutex> lock(g_notices.lock);
		g_notices.live.erase(std::remove(g_notices.live.begin(), g_notices.live.end(), pid), g_notices.live.end());
		if (g_notices.pid == pid) {
			g_notices.pid = 0;
		}
		/*
		 * Ended by failing while SteamVR runs (it found SteamVR's compositor not up in time: a slow SteamVR start,
		 * or it crashed): the Rift tab, the notices and the frame lock went with it. Started again
		 * from SteamVR's frame, a few seconds on, at most ten times. Not one that quit or was stopped (SteamVR
		 * closing down asks it to quit, then kills it), else it came back while SteamVR shuts down.
		 */
		const bool stopped_from_outside =
		    WIFSIGNALED(status) && (WTERMSIG(status) == SIGKILL || WTERMSIG(status) == SIGTERM ||
		                            WTERMSIG(status) == SIGINT || WTERMSIG(status) == SIGHUP);
		const bool failed = WIFEXITED(status) ? WEXITSTATUS(status) != 0 : !stopped_from_outside;
		if (failed && !g_notices.stopped && g_notices.restarts < 10) {
			g_notices.restart_due_ns = os_monotonic_get_ns() + 5LL * U_TIME_1S_IN_NS;
			ovrd_log("Notices: riftlinux-notices (pid %d) ended (%s %d); starting it again\n", static_cast<int>(pid),
			         WIFEXITED(status) ? "exit" : "signal", WIFEXITED(status) ? WEXITSTATUS(status) : WTERMSIG(status));
		}
	}).detach();
	ovrd_log("Notices: started riftlinux-notices (pid %d)\n", static_cast<int>(pid));
	return true;
}

/*!
 * Sends one line (a panel name or a command) to the helper; dropped while there is none. A helper that failed is
 * started again from SteamVR's frame (riftlinux_helper_poll); sending no longer starts one: at SteamVR's shutdown a
 * message to the gone helper started a new one that SteamVR then had to refuse and kill. Lock held.
 */
static void
notices_send_locked(const std::string &name)
{
	if (g_notices.fd < 0) {
		return;
	}
	const std::string line = name + "\n";
	// Never blocks a tracker thread, never raises SIGPIPE in vrserver; a full socket drops the line.
	const ssize_t n = send(g_notices.fd, line.data(), line.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
	if (n == static_cast<ssize_t>(line.size()) || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
		return;
	}
	// Gone; its thread reaps it.
	close(g_notices.fd);
	g_notices.fd = -1;
	g_notices.pid = 0;
}

//! A panel shown now, or once the helper runs (notices before it are kept, up to 8). Lock held.
static void
notices_show_locked(const std::string &name)
{
	if (g_notices.fd < 0 && !g_notices.stopped) {
		if (g_notices.pending.size() < 8) {
			g_notices.pending.push_back(name);
		}
		return;
	}
	notices_send_locked(name);
}

static void
notices_sink(void *ptr, enum u_notice_kind kind, const char *key, int number)
{
	(void)ptr;
	std::lock_guard<std::mutex> lock(g_notices.lock);
	// A measuring result (ours, or a first measure with no sensor placed; whatever the notice switches say): world and
	// floor follow.
	if (key != nullptr && strcmp(key, "calibration-done") == 0) {
		g_recal.measuring.store(false);
		struct u_notice_recalibration change = {};
		if (u_notice_get_recalibration(&change) && change.sequence != g_recal.seen_sequence) {
			g_recal.seen_sequence = change.sequence;
			recalibration_done();
		}
	} else if (key != nullptr && strcmp(key, "calibration-failed") == 0 && g_recal.measuring.load()) {
		ovrd_log("Sensors: a solve of the measuring failed; the tracker keeps measuring\n");
	}
	notices_read_settings_locked(nullptr);
	if (!g_notices.enabled || static_cast<int>(kind) < 0 || static_cast<int>(kind) > U_NOTICE_SYSTEM || !g_notices.kinds[kind]) {
		return;
	}
	ovrd_log("Notice: %s %d\n", key, number);
	const std::string name = number > 0 ? std::string(key) + "-" + std::to_string(number) : std::string(key);
	// The tracker's own post counts as said: its panel is up for 10 s, and is seen if the headset goes on meanwhile.
	if (name == "calibration-start") {
		g_notices.first_calibration_shown_ns = os_monotonic_get_ns();
	}
	// Posted at start, while the headset may still lie on the desk: shown again the first time it is put on.
	if (kind == U_NOTICE_DISPLAY || kind == U_NOTICE_SYSTEM) {
		g_notices.on_put_on.push_back(name);
	}
	notices_show_locked(name);
}

/*
 * The headset put on (nothing to set up by hand, so the headset says how it works): the first time
 * ever, the welcome panel (the Oculus button, recentring, the floor, waking the controllers); every time, the notices
 * posted while it lay unworn. The first put-on since SteamVR started also starts the controller hint's 30 s.
 */
static void
notices_on_put_on()
{
	std::lock_guard<std::mutex> lock(g_notices.lock);
	notices_read_settings_locked(nullptr);
	if (g_notices.first_put_on_ns == 0) {
		g_notices.first_put_on_ns = os_monotonic_get_ns();
	}
	if (g_notices.enabled && !g_notices.welcome_shown) {
		g_notices.welcome_shown = true;
		riftlinux_settings_set_bool("notices", "welcome_shown", true);
		ovrd_log("Notices: welcome shown (once; notices.welcome_shown false shows it again)\n");
		notices_show_locked("welcome");
	}
	for (const std::string &name : g_notices.on_put_on) {
		notices_show_locked(name);
	}
	g_notices.on_put_on.clear();
}

/*
 * The first measuring of the sensors (a fresh install, or the calibration removed): no sensor is placed, so the
 * headset only turns and the controllers are hidden (CDeviceDriver_Monado::GetPose), which looks broken unless said.
 * The measuring panel ("... The controllers appear when it is done.") is shown while the headset
 * is worn, again every 20 s, until a sensor is placed; then the tracker's "Sensors measured" follows. The tracker
 * posts the same panel when measuring starts, often before the headset is on. From RunFrame, once a second.
 */
static void
notices_first_calibration(bool unplaced, bool worn)
{
	std::lock_guard<std::mutex> lock(g_notices.lock);
	g_notices.worn = worn;
	if (!unplaced) {
		g_notices.first_calibration_shown_ns = 0;
		return;
	}
	if (!worn || !g_notices.enabled || !g_notices.kinds[U_NOTICE_CALIBRATION]) {
		return;
	}
	const int64_t now_ns = os_monotonic_get_ns();
	if (g_notices.first_calibration_shown_ns != 0 && now_ns - g_notices.first_calibration_shown_ns < 20LL * U_TIME_1S_IN_NS) {
		return;
	}
	g_notices.first_calibration_shown_ns = now_ns;
	ovrd_log("Notice: calibration-start (shown while the sensors are first measured, the controllers hidden)\n");
	notices_show_locked("calibration-start");
}

/*
 * 30 s after the headset was first put on, a controller that never reported (asleep, flat battery, not paired to this
 * headset) is named in the headset, once each SteamVR start. "Reported" is having sent its battery level, which a
 * controller does within a second of waking. From RunFrame. @p left, @p right: whether each has.
 */
static void
notices_controller_hint(bool left, bool right, bool sensors_placed)
{
	std::lock_guard<std::mutex> lock(g_notices.lock);
	// Not while no sensor is placed: the controllers are hidden then on purpose, and the measuring panel says so.
	if (g_notices.controller_hint_done || g_notices.first_put_on_ns == 0 || !sensors_placed ||
	    os_monotonic_get_ns() - g_notices.first_put_on_ns < 30LL * U_TIME_1S_IN_NS) {
		return;
	}
	g_notices.controller_hint_done = true;
	if (left && right) {
		return;
	}
	const char *name = !left && !right ? "controllers-none" : !left ? "controller-left-none" : "controller-right-none";
	ovrd_log("Notice: %s (30 s after the headset was put on)\n", name);
	if (g_notices.enabled && g_notices.kinds[U_NOTICE_TRACKING]) {
		notices_show_locked(name);
	}
}

//! A command for riftlinux-notices (the Oculus button: "cmd-open-quick"), whatever the switches.
static void
riftlinux_helper_command(const char *command)
{
	std::lock_guard<std::mutex> lock(g_notices.lock);
	if (g_notices.stopped) {
		return;
	}
	ovrd_log("Rift panel: %s\n", command);
	notices_send_locked(command);
}

//! The Rift tab's picture card as the driver has it ("cmd-picture direct on strength colour smoothing bright").
static std::string
picture_line()
{
	std::lock_guard<std::mutex> plock(g_picture.lock);
	return "cmd-picture " + std::to_string(g_picture.direct ? 1 : 0) + " " + std::to_string(g_picture.on ? 1 : 0) +
	       " " + std::to_string(g_picture.strength) + " " + std::to_string(g_picture.colour) + " " +
	       std::to_string(g_picture.smoothing ? 1 : 0) + " " + std::to_string(g_picture.bright ? 1 : 0);
}

/*
 * settings.json's display section read again when the file changes: each HDR part's switch and the
 * picture settings the Rift tab also has, live, so the KDE app or an edit by hand changes them without restarting
 * SteamVR. A key is applied when its value in the file changed since the last read (SteamVR's start applied the file
 * as it was; a value the environment set stays until the file changes it). Direct mode only: the rest is Monado's
 * compositor's. From RunFrame, once a second.
 */
static void
display_read_settings_live()
{
	static int64_t mtime_seen = -1;
	static std::map<std::string, double> file_values;
	const std::string path = riftlinux_settings_path();
	struct stat st;
	const int64_t mtime = stat(path.c_str(), &st) == 0
	                          ? static_cast<int64_t>(st.st_mtim.tv_sec) * 1000000000LL + st.st_mtim.tv_nsec
	                          : -2;
	if (mtime == mtime_seen) {
		return;
	}
	// A file being written by an editor may not parse yet: read again next time.
	cJSON *root = riftlinux_settings_load();
	if (root == nullptr && mtime != -2) {
		return;
	}
	const bool first = mtime_seen == -1;
	mtime_seen = mtime;

	static const char *const kKeys[] = {
	    "auto_hdr",         "auto_hdr_strength", "auto_hdr_colour_strength", "hdr_bright",
	    "hdr_white",        "hdr_white_bright",  "hdr_float_layers",         "hdr_metadata",
	    "hdr_measured_peak", "hdr_players",      "motion_smoothing",         "direct_mode_fast_path",
	};
	std::vector<std::pair<std::string, double>> changed;
	const cJSON *display = cJSON_GetObjectItemCaseSensitive(root, "display");
	for (const char *key : kKeys) {
		const cJSON *v = cJSON_GetObjectItemCaseSensitive(display, key);
		if (!cJSON_IsBool(v) && !cJSON_IsNumber(v)) {
			file_values.erase(key);
			continue;
		}
		const double d = cJSON_IsBool(v) ? (cJSON_IsTrue(v) ? 1.0 : 0.0) : v->valuedouble;
		auto it = file_values.find(key);
		if (!first && (it == file_values.end() || it->second != d)) {
			changed.emplace_back(key, d);
		}
		file_values[key] = d;
	}
	cJSON_Delete(root);
	if (changed.empty() || g_direct_mode == nullptr) {
		return;
	}

	bool tab = false;
	{
		std::lock_guard<std::mutex> plock(g_picture.lock);
		struct render_output_settings rs;
		render_output_settings_get(&rs);
		bool compositor = false;
		for (const auto &[key, d] : changed) {
			const bool on = d != 0.0;
			// A value that differs from the running one: taken, logged; the tab's own are sent to it.
			auto set = [&](auto &field, auto value, bool &flag) {
				if (field != value) {
					field = value;
					flag = true;
					ovrd_log("Settings: display.%s = %g (settings.json, live)\n", key.c_str(), d);
				}
			};
			if (key == "auto_hdr") {
				set(g_picture.on, on, tab);
			} else if (key == "auto_hdr_strength") {
				set(g_picture.strength, std::clamp((int)std::lround(d), 0, 100), tab);
			} else if (key == "auto_hdr_colour_strength") {
				set(g_picture.colour, std::clamp((int)std::lround(d), 0, 100), tab);
			} else if (key == "hdr_bright") {
				set(g_picture.bright, on, tab);
			} else if (key == "motion_smoothing") {
				set(g_picture.smoothing, on, tab);
			} else if (key == "hdr_white") {
				set(rs.hdr_white, std::clamp((float)d, 0.5f, 0.98f), compositor);
			} else if (key == "hdr_white_bright") {
				set(rs.hdr_white_bright, std::clamp((float)d, 0.5f, 0.98f), compositor);
			} else if (key == "hdr_float_layers") {
				set(rs.hdr_float_layers, on, compositor);
			} else if (key == "hdr_metadata") {
				set(rs.hdr_metadata, on, compositor);
			} else if (key == "hdr_measured_peak") {
				set(rs.hdr_measured_peak, on, compositor);
			} else if (key == "direct_mode_fast_path") {
				set(rs.fast_path, on, compositor);
			} else if (key == "hdr_players") {
				if (g_direct_mode->m_players_hdr.exchange(on) != on) {
					ovrd_log("Settings: display.hdr_players = %d (settings.json, live): players' HDR descriptions %s\n",
					         on ? 1 : 0, on ? "used" : "not used");
				}
			}
		}
		if (compositor) {
			render_output_settings_set(&rs);
		}
		// The tab's switches on top (Auto-HDR's strengths, smoothing, HDR Bright).
		picture_apply_locked();
	}
	if (tab) {
		std::lock_guard<std::mutex> lock(g_notices.lock);
		notices_send_locked(picture_line());
	}
}

//! The Rift tab's settings, for a helper just started: the height, and the picture's (direct mode only). Lock held.
static void
notices_send_state_locked()
{
	const long mm = std::lround(g_world_from_driver.height_m.load() * 1000.0f);
	notices_send_locked("cmd-height " + std::to_string(mm));
	notices_send_locked(picture_line());
	notices_send_locked("cmd-hdr-content " + std::to_string(g_hdr_content.load()));
	if (!g_tab_status_line.empty()) {
		notices_send_locked(g_tab_status_line);
	}
	if (!g_tab_display_line.empty()) {
		notices_send_locked(g_tab_display_line);
	}
	// Notices that came before the helper ran.
	for (const std::string &name : g_notices.pending) {
		notices_send_locked(name);
	}
	g_notices.pending.clear();
}

//! What riftlinux-notices says back: "dashboard 1" / "dashboard 0". Never waits; on SteamVR's frame thread.
static void
riftlinux_helper_poll()
{
	static std::string partial;
	std::lock_guard<std::mutex> lock(g_notices.lock);
	if (g_notices.fd < 0) {
		// A helper that failed, started again (see notices_spawn_locked); retried while it can't be started yet.
		if (g_notices.restart_due_ns != 0 && !g_notices.stopped && os_monotonic_get_ns() >= g_notices.restart_due_ns) {
			g_notices.restart_due_ns = 0;
			if (notices_spawn_locked()) {
				g_notices.restarts++;
				notices_send_state_locked();
			} else {
				g_notices.restart_due_ns = os_monotonic_get_ns() + 5LL * U_TIME_1S_IN_NS;
			}
		}
		return;
	}
	char buf[256];
	const ssize_t n = recv(g_notices.fd, buf, sizeof(buf), MSG_DONTWAIT);
	if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
		// The helper is gone (its thread reaps it and, if it failed, has it started again). A crashed one leaves a
		// reset connection, not an end of file.
		close(g_notices.fd);
		g_notices.fd = -1;
		g_notices.pid = 0;
		g_dashboard_visible.store(false);
		return;
	}
	if (n < 0) {
		return;
	}
	partial.append(buf, static_cast<size_t>(n));
	size_t eol;
	while ((eol = partial.find('\n')) != std::string::npos) {
		const std::string line = partial.substr(0, eol);
		partial.erase(0, eol + 1);
		if (line == "dashboard 1" || line == "dashboard 0") {
			g_dashboard_visible.store(line == "dashboard 1");
		} else if (line.rfind("height ", 0) == 0) {
			const float cm = (float)std::lround(atof(line.c_str() + 7) / 10.0);
			g_world_from_driver.height_m.store(cm / 100.0f);
			riftlinux_settings_set_number("tracking", "height_cm", cm);
			ovrd_log("Rift panel: height %+.0f cm\n", cm);
		} else if (line == "floor") {
			g_world_from_driver.floor_request.store(true);
		} else if (line == "recalibrate") {
			g_recal.asked.store(true);
		} else if (line == "autohdr 1" || line == "autohdr 0") {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				g_picture.on = line == "autohdr 1";
				picture_apply_locked();
				riftlinux_settings_set_number("display", "auto_hdr", g_picture.on ? 1 : 0);
				ovrd_log("Rift panel: Auto-HDR %s\n", g_picture.on ? "on" : "off");
			}
		} else if (line == "direct-mode 1" || line == "direct-mode 0") {
			// For SteamVR's next start (the manifest always says the driver drives the display; off by settings).
			g_display.direct_saved.store(line == "direct-mode 1");
			riftlinux_settings_set_bool("display", "direct_mode", g_display.direct_saved.load());
			ovrd_log("Rift panel: direct mode %s from SteamVR's next start\n",
			         g_display.direct_saved.load() ? "on" : "off");
		} else if (line.rfind("render-scale ", 0) == 0) {
			const int percent = std::clamp(atoi(line.c_str() + 13), 50, 200);
			g_display.scale_saved.store(percent);
			riftlinux_settings_set_number("display", "render_scale", percent);
			ovrd_log("Rift panel: render scale %d %% from SteamVR's next start\n", percent);
		} else if (line == "perf 1" || line == "perf 0") {
			g_display.perf.store(line == "perf 1");
			riftlinux_settings_set_bool("tab", "performance", line == "perf 1");
			ovrd_log("Rift panel: performance numbers %s\n", line == "perf 1" ? "on" : "off");
		} else if (line == "fast-path 1" || line == "fast-path 0") {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				struct render_output_settings rs;
				render_output_settings_get(&rs);
				rs.fast_path = line == "fast-path 1";
				render_output_settings_set(&rs);
				riftlinux_settings_set_bool("display", "direct_mode_fast_path", rs.fast_path);
				ovrd_log("Rift panel: fast path %s\n", rs.fast_path ? "on" : "off");
			}
		} else if (line == "hdr-bright 1" || line == "hdr-bright 0") {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				g_picture.bright = line == "hdr-bright 1";
				picture_apply_locked();
				riftlinux_settings_set_number("display", "hdr_bright", g_picture.bright ? 1 : 0);
				ovrd_log("Rift panel: HDR Bright %s\n", g_picture.bright ? "on" : "off");
			}
		} else if (line == "smoothing 1" || line == "smoothing 0") {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				g_picture.smoothing = line == "smoothing 1";
				picture_apply_locked();
				riftlinux_settings_set_number("display", "motion_smoothing", g_picture.smoothing ? 1 : 0);
				ovrd_log("Rift panel: motion smoothing %s\n", g_picture.smoothing ? "on" : "off");
			}
		} else if (line.rfind("autohdr-colour ", 0) == 0) {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				g_picture.colour = std::clamp(atoi(line.c_str() + 15), 0, 100);
				picture_apply_locked();
				riftlinux_settings_set_number("display", "auto_hdr_colour_strength", g_picture.colour);
				ovrd_log("Rift panel: Auto-HDR colour %d %%\n", g_picture.colour);
			}
		} else if (line.rfind("autohdr-strength ", 0) == 0) {
			std::lock_guard<std::mutex> plock(g_picture.lock);
			if (g_picture.direct) {
				g_picture.strength = std::clamp(atoi(line.c_str() + 17), 0, 100);
				picture_apply_locked();
				riftlinux_settings_set_number("display", "auto_hdr_strength", g_picture.strength);
				ovrd_log("Rift panel: Auto-HDR strength %d %%\n", g_picture.strength);
			}
		}
	}
	if (partial.size() > 4096) {
		partial.clear();
	}
}

static void
notices_start()
{
	std::string test;
	int delay_s = 5;
	{
		std::lock_guard<std::mutex> lock(g_notices.lock);
		g_notices.stopped = false;
		notices_read_settings_locked(&test);
		delay_s = g_notices.start_delay_s;
	}
	u_notice_set_sink(notices_sink, nullptr);
	/*
	 * Started with SteamVR rather than at the first notice: it also gives the dashboard its Rift tab (volume).
	 * SteamVR takes overlay apps only once it is up; the helper keeps trying for a while.
	 */
	std::thread([test, delay_s]() {
		std::this_thread::sleep_for(std::chrono::seconds(delay_s));
		{
			std::lock_guard<std::mutex> lock(g_notices.lock);
			if (g_notices.stopped) {
				return;
			}
			if (g_notices.fd < 0) {
				notices_spawn_locked();
			}
			notices_send_state_locked();
		}
		if (test.empty()) {
			return;
		}
		std::this_thread::sleep_for(std::chrono::seconds(10));
		std::lock_guard<std::mutex> lock(g_notices.lock);
		if (g_notices.enabled && !g_notices.stopped) {
			ovrd_log("Notice: test %s\n", test.c_str());
			notices_send_locked(test);
		}
	}).detach();
}

static void
notices_stop()
{
	u_notice_set_sink(nullptr, nullptr);
	{
		std::lock_guard<std::mutex> lock(g_notices.lock);
		g_notices.stopped = true;
		if (g_notices.fd >= 0) {
			// It ends when its input closes.
			close(g_notices.fd);
			g_notices.fd = -1;
		}
		g_notices.pid = 0;
	}
	// Every helper ended and reaped before the driver goes (their threads live in it): 2 s, then killed.
	for (int i = 0; i < 125; i++) {
		{
			std::lock_guard<std::mutex> lock(g_notices.lock);
			if (g_notices.live.empty()) {
				return;
			}
			if (i == 100) {
				for (pid_t p : g_notices.live) {
					ovrd_log("Notices: riftlinux-notices (pid %d) did not end, killed\n", static_cast<int>(p));
					kill(p, SIGKILL);
				}
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}
#endif

class CServerDriver_Monado : public vr::IServerTrackedDeviceProvider
{
public:
	CServerDriver_Monado() : m_MonadoDeviceDriver(NULL) {}

	// clang-format off
	virtual ~CServerDriver_Monado() {};
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
	virtual void Cleanup();
	virtual const char *const * GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}
	virtual void HandleHapticEvent(vr::VREvent_t *event);
	// clang-format on

private:
	struct xrt_instance *m_xinst = NULL;
	struct xrt_system *m_xsys = NULL;
	struct xrt_system_devices *m_xsysd = NULL;
	struct xrt_space_overseer *m_xso = NULL;
	//! Direct mode only: Monado's compositor, and this driver as its one client.
	struct xrt_system_compositor *m_xsysc = NULL;
	struct xrt_compositor_native *m_xcn = NULL;
	struct xrt_device *m_xhmd = NULL;

	CDeviceDriver_Monado *m_MonadoDeviceDriver = NULL;
	CDeviceDriver_Monado_Controller *m_left = NULL;
	CDeviceDriver_Monado_Controller *m_right = NULL;
};

CServerDriver_Monado g_serverDriverMonado;

#define NUM_XDEVS 16

/*!
 * Monado's compositor for direct mode, on a thread of its own, waited for until @p deadline_s after the driver's start
 * (on a locked KDE session the start hung after the headset's lease, before Vulkan answered, until
 * SteamVR's watchdog ended SteamVR at ~22 s and blocked the driver). Not up by then: null, SteamVR's own compositor
 * drives the headset this time, SteamVR keeps running; a compositor that comes up later is let go at once.
 */
static struct xrt_system_compositor *
direct_mode_compositor_start(struct xrt_device *head, int64_t init_start_ns, float deadline_s)
{
	struct Start
	{
		std::mutex lock;
		std::condition_variable done_cv;
		bool done = false;
		bool abandoned = false;
		xrt_result_t xret = XRT_SUCCESS;
		struct xrt_system_compositor *xsysc = nullptr;
	};
	auto st = std::make_shared<Start>();
	const int64_t begin_ns = os_monotonic_get_ns();
	std::thread([st, head]() {
		struct xrt_system_compositor *xsysc = nullptr;
		const xrt_result_t xret = comp_main_create_system_compositor(head, NULL, NULL, &xsysc);
		std::lock_guard<std::mutex> lock(st->lock);
		if (st->abandoned) {
			ovrd_log("Direct mode: Monado's compositor came up after its deadline; let go\n");
			xrt_syscomp_destroy(&xsysc);
			return;
		}
		st->xret = xret;
		st->xsysc = xsysc;
		st->done = true;
		st->done_cv.notify_all();
	}).detach();

	const int64_t deadline_ns = init_start_ns + (int64_t)(deadline_s * 1e9);
	std::unique_lock<std::mutex> lock(st->lock);
	const int64_t wait_ns = std::max<int64_t>(deadline_ns - os_monotonic_get_ns(), 0);
	st->done_cv.wait_for(lock, std::chrono::nanoseconds(wait_ns), [&] { return st->done; });
	if (!st->done) {
		st->abandoned = true;
		ovrd_log("Direct mode: Monado's compositor not up %.1f s after the driver's start (SteamVR ends a driver's "
		         "start at ~22 s); SteamVR's compositor this time\n",
		         (double)(os_monotonic_get_ns() - init_start_ns) / 1e9);
		return nullptr;
	}
	if (st->xret < 0 || st->xsysc == nullptr) {
		ovrd_log("Direct mode: no Monado compositor (%d), SteamVR's compositor instead\n", (int)st->xret);
		return nullptr;
	}
	ovrd_log("Direct mode: Monado's compositor up in %.1f s\n", (double)(os_monotonic_get_ns() - begin_ns) / 1e9);
	return st->xsysc;
}

vr::EVRInitError
CServerDriver_Monado::Init(vr::IVRDriverContext *pDriverContext)
{

	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	ovrd_log_init(vr::VRDriverLog());

	ovrd_log("Initializing Monado driver\n");
	const int64_t init_start_ns = os_monotonic_get_ns();

#ifdef XRT_OS_LINUX
	// Before the driver's threads, which inherit it: SteamVR started from a background session runs in its class.
	if (!u_linux_leave_background_scheduling(U_LOGGING_INFO, "SteamVR driver")) {
		ovrd_log("SteamVR runs at the lowest CPU priority (started from a background session): start Steam from the "
		         "desktop\n");
	}
#endif

#ifdef XRT_OS_LINUX
	riftlinux_load_floor();
#endif

#ifdef XRT_OS_LINUX
	// Before the devices: the tracker may have something to say right away (placing the sensors).
	notices_start();
#endif

	//! @todo instance initialization is difficult to replicate

	xrt_result_t xret;
	xret = xrt_instance_create(NULL, &m_xinst);
	if (xret != XRT_SUCCESS) {
		ovrd_log("Failed to create instance\n");
		return vr::VRInitError_Init_HmdNotFound;
	}

	bool direct_mode = false;
	float direct_mode_guard_ms = 0.0f;
	bool direct_mode_compositor_unorm = false;
	bool direct_mode_app_unorm = false;
	bool direct_mode_fast_path = true;
	int direct_mode_max_layers = 16;
	// Seconds after the driver's start by which Monado's compositor must be up (display.direct_mode_start_deadline_s,
	// a test key: a short one shows the fallback).
	float direct_mode_deadline_s = 18.0f;
	int render_scale = 100;
	int direct_mode_capture = 0;
	float direct_mode_capture_delay_s = 20.0f;
	int capture_output = 0;
	int hdr_test = 0;
	bool hdr_players = true;
#ifdef XRT_OS_LINUX
	{
		cJSON *settings = riftlinux_settings_load();
		const cJSON *display = cJSON_GetObjectItemCaseSensitive(settings, "display");
		// On unless switched off: the package's manifest always says the driver drives the display; with direct
		// mode off SteamVR's compositor takes the headset itself (tested), so no manifest is edited.
		const cJSON *dm = cJSON_GetObjectItemCaseSensitive(display, "direct_mode");
		direct_mode = cJSON_IsBool(dm) ? cJSON_IsTrue(dm) : true;
		g_display.perf.store(
		    cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(cJSON_GetObjectItemCaseSensitive(settings, "tab"), "performance")));
		const cJSON *guard = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_guard_ms");
		if (cJSON_IsNumber(guard)) {
			direct_mode_guard_ms = (float)guard->valuedouble;
		}
		const cJSON *fc = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_format_compositor");
		const cJSON *fa = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_format_app");
		if (cJSON_IsString(fc)) {
			direct_mode_compositor_unorm = strcmp(fc->valuestring, "unorm") == 0;
		}
		if (cJSON_IsString(fa)) {
			direct_mode_app_unorm = strcmp(fa->valuestring, "unorm") == 0;
		}
		const cJSON *fp = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_fast_path");
		if (cJSON_IsBool(fp)) {
			direct_mode_fast_path = cJSON_IsTrue(fp);
		}
		const cJSON *cap = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_capture");
		if (cJSON_IsNumber(cap)) {
			direct_mode_capture = cap->valueint;
		}
		const cJSON *capd = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_capture_delay_s");
		if (cJSON_IsNumber(capd)) {
			direct_mode_capture_delay_s = (float)capd->valuedouble;
		}
		const cJSON *dl = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_start_deadline_s");
		if (cJSON_IsNumber(dl)) {
			direct_mode_deadline_s = std::clamp((float)dl->valuedouble, 0.0f, 20.0f);
		}
		const cJSON *ml = cJSON_GetObjectItemCaseSensitive(display, "direct_mode_max_layers");
		if (cJSON_IsNumber(ml)) {
			direct_mode_max_layers = ml->valueint;
		}
		const cJSON *rsc = cJSON_GetObjectItemCaseSensitive(display, "render_scale");
		if (cJSON_IsNumber(rsc)) {
			render_scale = std::clamp(rsc->valueint, 25, 400);
		}

		// display.hdr_test: true or "static" (MaxCLL 4000), "none" (measured), "scene" (scene peak 1000 nits).
		const cJSON *ht = cJSON_GetObjectItemCaseSensitive(display, "hdr_test");
		if (cJSON_IsTrue(ht)) {
			hdr_test = 1;
		} else if (cJSON_IsString(ht)) {
			hdr_test = strcmp(ht->valuestring, "none") == 0 ? 2 : strcmp(ht->valuestring, "scene") == 0 ? 3 : 1;
		}
		// display.hdr_players: video players' HDR descriptions over the socket used, on by default.
		const cJSON *hp = cJSON_GetObjectItemCaseSensitive(display, "hdr_players");
		if (cJSON_IsBool(hp) || cJSON_IsNumber(hp)) {
			hdr_players = cJSON_IsBool(hp) ? cJSON_IsTrue(hp) : hp->valuedouble != 0.0;
		}
		const cJSON *co = cJSON_GetObjectItemCaseSensitive(display, "capture_output");
		if (cJSON_IsNumber(co)) {
			capture_output = co->valueint;
		}
		// Motion smoothing's capture is unattended too.
		const cJSON *mc = cJSON_GetObjectItemCaseSensitive(display, "motion_capture");
		if (cJSON_IsNumber(mc) && mc->valueint > 0) {
			capture_output = std::max(capture_output, 1);
		}
		cJSON_Delete(settings);
	}
#endif
	/*
	 * Direct mode: single-layer frames take Monado's fast path (the lens pass reads the game's image itself). Every
	 * frame went through the layer renderer from the first direct mode spike on, against dark frames with bright
	 * flashes, whose cause was then found elsewhere (SteamVR's and Monado's image descriptions differed) and fixed,
	 * but the detour stayed: the game's image was drawn into an image of 1512x1680 and that sampled again by the
	 * lens pass, a softer picture in every game (it looked blurry). Pace test, panel output at the centre: 37 %
	 * sharper on the fast path, the same colours (1-2 % in linear light). In the headset it first made the
	 * edges of the view blink: it read past the area the game drew, and applied Auto-HDR after taking red,
	 * green and blue from three places. Both fixed in distortion.comp (black outside the drawn area and Auto-HDR per
	 * sample, as the layer renderer); Home and No Man's Sky then fine. On by default; display.direct_mode_fast_path
	 * false goes back to the layer renderer.
	 */
#ifdef XRT_OS_LINUX
	openxr_runtime_default();
#endif
	// Live (the Rift tab's display card, render_output_settings), not XRT_COMPOSITOR_DISABLE_FAST_PATH's for good.
	setenv("XRT_COMPOSITOR_FAST_PATH", direct_mode_fast_path ? "1" : "0", 0);
	g_display.direct_saved.store(direct_mode);
	// The render scale (display.render_scale) also without direct mode: the size SteamVR's games render at either way
	// (GetRecommendedRenderTargetSize); with direct mode the option list below sets it too.
	if (!direct_mode) {
		setenv("RIFTLINUX_RENDER_SCALE", std::to_string(render_scale).c_str(), 0);
	}
	g_display.scale_saved.store(render_scale);
	{
		const char *rs = getenv("RIFTLINUX_RENDER_SCALE");
		g_display.scale_running.store(rs != nullptr ? std::clamp(atoi(rs), 25, 400) : render_scale);
	}
	// Direct mode: only targets that take the headset's display (Wayland lease, X11/XWayland RandR lease, NVIDIA,
	// VK_KHR_display), never a desktop window; none working -> SteamVR's compositor (comp_compositor.c).
	if (direct_mode) {
		setenv("XRT_COMPOSITOR_DIRECT_ONLY", "1", 0);
	}
	// SteamVR's direct mode has its own frame lock (riftlinux-notices, ovrd_direct_mode.cpp) built on Monado's
	// application pacing as it is; the native one (u_pacing_app.c) stays off here.
	setenv("U_PACING_APP_FRAME_LOCK", "0", 1);
	/*
	 * Monado's output processing (distortion.comp) from settings.json display.*: dither_bits (0 off,
	 * default 8), auto_hdr (and _highlights, _shadows, _colour, _knee), vibrance (0 off), black_level (0 off),
	 * tonemap_shoulder (0 off), float_scratch (default on), motion_smoothing (default off; motion_capture,
	 * motion_capture_start, motion_test_half, motion_test_yaw_mdeg, motion_test_shadow for tests), capture_output
	 * (frames of the panel's output to ~/.cache/riftlinux/output-capture, for tests); environment variables already
	 * set win.
	 */
	if (direct_mode) {
		cJSON *settings = riftlinux_settings_load();
		const cJSON *display = cJSON_GetObjectItemCaseSensitive(settings, "display");
		const struct
		{
			const char *key;
			const char *env;
		} options[] = {
		    {"dither_bits", "XRT_COMPOSITOR_DITHER_BITS"},
		    {"vibrance", "XRT_COMPOSITOR_VIBRANCE"},
		    {"black_level", "XRT_COMPOSITOR_BLACK_LEVEL"},
		    {"tonemap_shoulder", "XRT_COMPOSITOR_TONEMAP_SHOULDER"},
		    {"auto_hdr", "XRT_COMPOSITOR_AUTO_HDR"},
		    {"auto_hdr_highlights", "XRT_COMPOSITOR_AUTO_HDR_HIGHLIGHTS"},
		    {"auto_hdr_shadows", "XRT_COMPOSITOR_AUTO_HDR_SHADOWS"},
		    {"auto_hdr_colour", "XRT_COMPOSITOR_AUTO_HDR_COLOUR"},
		    {"auto_hdr_knee", "XRT_COMPOSITOR_AUTO_HDR_KNEE"},
		    {"capture_output", "XRT_COMPOSITOR_CAPTURE_OUTPUT"},
		    {"capture_output_start", "XRT_COMPOSITOR_CAPTURE_OUTPUT_START"},
		    {"capture_output_ab", "XRT_COMPOSITOR_CAPTURE_OUTPUT_AB"},
		    {"float_scratch", "XRT_COMPOSITOR_FLOAT_SCRATCH"},
		    {"hdr_white", "XRT_COMPOSITOR_HDR_WHITE"},
		    {"hdr_float_layers", "XRT_COMPOSITOR_HDR_FLOAT_LAYERS"},
		    {"hdr_metadata", "XRT_COMPOSITOR_HDR_METADATA"},
		    {"hdr_measured_peak", "XRT_COMPOSITOR_HDR_MEASURED_PEAK"},
		    {"hdr_bright", "XRT_COMPOSITOR_HDR_BRIGHT"},
		    {"hdr_white_bright", "XRT_COMPOSITOR_HDR_WHITE_BRIGHT"},
		    // Presenting to the headset (tests on Wayland leases, NVIDIA bug 5372097): "fifo", "mailbox", "immediate".
		    {"present_mode", "XRT_COMPOSITOR_PRESENT_MODE"},
		    {"present_wait", "XRT_COMPOSITOR_USE_PRESENT_WAIT"},
		    {"comp_time_fraction", "U_PACING_COMP_TIME_FRACTION_PERCENT"},
		    {"present_to_display_offset_ms", "U_PACING_COMP_PRESENT_TO_DISPLAY_OFFSET_MS"},
		    {"motion_smoothing", "XRT_COMPOSITOR_MOTION_SMOOTHING"},
		    {"render_scale", "RIFTLINUX_RENDER_SCALE"},
		    {"render_scale", "XRT_COMPOSITOR_RENDER_SCALE"},
		    {"motion_pick", "XRT_COMPOSITOR_MOTION_PICK"},
		    {"motion_flow", "XRT_COMPOSITOR_MOTION_FLOW"},
		    {"motion_subpixel", "XRT_COMPOSITOR_MOTION_SUBPIXEL"},
		    {"scratch_headroom", "XRT_COMPOSITOR_SCRATCH_HEADROOM"},
		    {"motion_capture", "XRT_COMPOSITOR_MOTION_CAPTURE"},
		    {"motion_capture_start", "XRT_COMPOSITOR_MOTION_CAPTURE_START"},
		    {"motion_test_half", "XRT_COMPOSITOR_MOTION_TEST_HALF"},
		    {"motion_test_yaw_mdeg", "XRT_COMPOSITOR_MOTION_TEST_YAW_MDEG"},
		    {"motion_test_shadow", "XRT_COMPOSITOR_MOTION_TEST_SHADOW"},
		    {"motion_record", "XRT_COMPOSITOR_MOTION_RECORD"},
		    {"motion_lock_test", "RIFTLINUX_FRAME_LOCK_TEST"},
		    {"motion_test_present_delay_ms", "RIFTLINUX_TEST_PRESENT_DELAY_MS"},
		    {"motion_pacing_trace", "RIFTLINUX_PACING_TRACE"},
		    {"motion_pacing_trace_after_s", "RIFTLINUX_PACING_TRACE_AFTER_S"},
		    {"motion_record_frames", "XRT_COMPOSITOR_MOTION_RECORD_FRAMES"},
		    {"motion_record_every_s", "XRT_COMPOSITOR_MOTION_RECORD_EVERY_S"},
		};
		for (const auto &o : options) {
			const cJSON *v = cJSON_GetObjectItemCaseSensitive(display, o.key);
			char text[32];
			if (cJSON_IsNumber(v)) {
				snprintf(text, sizeof(text), "%g", v->valuedouble);
			} else if (cJSON_IsString(v)) {
				snprintf(text, sizeof(text), "%s", v->valuestring);
			} else if (cJSON_IsBool(v)) {
				snprintf(text, sizeof(text), "%d", cJSON_IsTrue(v) ? 1 : 0);
			} else {
				continue;
			}
			setenv(o.env, text, 0);
			ovrd_log("Display: %s = %s\n", o.key, text);
		}
		/*
		 * The compositor's scratch images follow the CV1's 1:1 size with 25 % headroom a side for SteamVR's
		 * supersampling (comp_compositor.c); a larger display.render_scale gets as much more headroom.
		 */
		const char *rs = getenv("RIFTLINUX_RENDER_SCALE");
		if (rs != nullptr && getenv("XRT_COMPOSITOR_SCRATCH_HEADROOM") == nullptr && atof(rs) > 0.0) {
			char text[32];
			snprintf(text, sizeof(text), "%.0f", 125.0 * sqrt(atof(rs) / 100.0));
			setenv("XRT_COMPOSITOR_SCRATCH_HEADROOM", text, 0);
			ovrd_log("Display: scratch headroom %s %% from render_scale %s\n", text, rs);
		}
		/*
		 * The CV1's panel shows sRGB levels 1-3 as black: lifted by default so dark detail stays (test
		 * pattern, by eye: 3 made level 1 too bright, 2.3 "much better"); display.black_level overrides, 0 off.
		 */
		setenv("XRT_COMPOSITOR_BLACK_LEVEL", "2.3", 0);
		// The Rift tab's picture row: the strengths above are 50 %; display.auto_hdr_strength, 0-100.
		std::lock_guard<std::mutex> plock(g_picture.lock);
		render_output_settings_get(&g_picture.base);
		g_picture.direct = true;
		g_picture.on = g_picture.base.auto_hdr;
		g_picture.smoothing = g_picture.base.motion_smoothing;
		g_picture.bright = g_picture.base.hdr_bright;
		const cJSON *strength = cJSON_GetObjectItemCaseSensitive(display, "auto_hdr_strength");
		if (cJSON_IsNumber(strength)) {
			g_picture.strength = std::clamp(strength->valueint, 0, 100);
		}
		const cJSON *colour = cJSON_GetObjectItemCaseSensitive(display, "auto_hdr_colour_strength");
		if (cJSON_IsNumber(colour)) {
			g_picture.colour = std::clamp(colour->valueint, 0, 100);
		}
		picture_apply_locked();
		ovrd_log("Display: Auto-HDR %s, strength %d %%, colour %d %%, motion smoothing %s, HDR Bright %s\n",
		         g_picture.on ? "on" : "off", g_picture.strength, g_picture.colour, g_picture.smoothing ? "on" : "off",
		         g_picture.bright ? "on" : "off");
		cJSON_Delete(settings);
	}
	// Capture runs are unattended (the headset on a desk): the panel stays on a minute unworn, not 5 s.
	if (direct_mode && (direct_mode_capture > 0 || capture_output > 0)) {
		setenv("RIFT_DISPLAY_OFF_UNWORN_MS", "90000", 1);
		g_report_worn.store(true);
	}
	/*
	 * diagnostics.record (settings.json): the tracker's and the headset's info lines into SteamVR's log and a
	 * recording of the session (the cameras' blobs, solves, IMU, output poses) into diagnostics.dir (default the
	 * cache folder's riftlinux, which SteamVR's container shares), so a session that goes wrong can be replayed and
	 * understood. Off by default (about 3 MB a second).
	 */
	{
		cJSON *settings = riftlinux_settings_load();
		const cJSON *diag = cJSON_GetObjectItemCaseSensitive(settings, "diagnostics");
		const cJSON *record = cJSON_GetObjectItemCaseSensitive(diag, "record");
		if (cJSON_IsTrue(record)) {
			const cJSON *dir = cJSON_GetObjectItemCaseSensitive(diag, "dir");
			std::string where;
			if (cJSON_IsString(dir)) {
				where = dir->valuestring;
			} else {
				const char *cache = getenv("XDG_CACHE_HOME");
				const char *home = getenv("HOME");
				where = std::string(cache != nullptr && cache[0] != '\0' ? cache
				                    : home != nullptr                     ? std::string(home) + "/.cache"
				                                                          : std::string("/tmp")) +
				        "/riftlinux";
				// Its parent exists; the folder itself may not yet (the driver's socket makes it later).
				mkdir(where.c_str(), 0700);
			}
			char stamp[32];
			const time_t now = time(nullptr);
			struct tm tm_now;
			localtime_r(&now, &tm_now);
			strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_now);
			const std::string file = where + "/riftlinux-" + stamp + ".ctdata";
			setenv("CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT", file.c_str(), 0);
			setenv("CONSTELLATION_TRACKER_LOG", "info", 0);
			setenv("RIFT_LOG", "info", 0);
			ovrd_log("Diagnostics: the tracker's and the headset's info log, the session recorded to %s\n",
			         file.c_str());
		}
		cJSON_Delete(settings);
	}
	// The devices; with direct mode Monado's compositor follows, which takes the headset's display.
	xret = xrt_instance_create_system(m_xinst, &m_xsys, &m_xsysd, &m_xso, NULL);
	ovrd_log("Devices up in %.1f s\n", (double)(os_monotonic_get_ns() - init_start_ns) / 1e9);
	if (xret >= 0 && direct_mode && m_xsysd->static_roles.head != NULL) {
		m_xsysc = direct_mode_compositor_start(m_xsysd->static_roles.head, init_start_ns, direct_mode_deadline_s);
		if (m_xsysc == NULL) {
			direct_mode = false;
		}
	}
	if (xret < 0) {
		ovrd_log("Failed to create system devices\n");
		xrt_instance_destroy(&m_xinst);
		return vr::VRInitError_Init_HmdNotFound;
	}
	if (m_xsysd->static_roles.head == NULL) {
		ovrd_log("Didn't get a HMD device!\n");
		xrt_space_overseer_destroy(&m_xso);
		xrt_system_devices_destroy(&m_xsysd);
		xrt_system_destroy(&m_xsys);
		xrt_instance_destroy(&m_xinst);
		return vr::VRInitError_Init_HmdNotFound;
	}

	m_xhmd = m_xsysd->static_roles.head;

#ifdef XRT_OS_LINUX
	if (direct_mode && m_xsysc != NULL) {
		static struct xrt_session_event_sink events = {
		    [](struct xrt_session_event_sink *, const union xrt_session_event *) { return XRT_SUCCESS; }};
		struct xrt_session_info xsi = {};
		struct xrt_begin_session_info bsi = {};
		bsi.view_type = XRT_VIEW_TYPE_STEREO;
		if (xrt_syscomp_create_native_compositor(m_xsysc, &xsi, &events, &m_xcn) == XRT_SUCCESS &&
		    xrt_comp_begin_session(&m_xcn->base, &bsi) == XRT_SUCCESS) {
			xrt_syscomp_set_state(m_xsysc, &m_xcn->base, true, true, os_monotonic_get_ns());
			g_direct_mode = new OvrdDirectMode(m_xcn, m_xhmd, driver_from_world, direct_mode_guard_ms,
			                                   direct_mode_compositor_unorm, direct_mode_app_unorm);
			g_direct_mode->m_max_layers = direct_mode_max_layers;
			g_direct_mode->m_hdr_test = hdr_test;
			g_direct_mode->m_players_hdr.store(hdr_players);
			g_direct_mode->m_capture_count = g_direct_mode->m_capture_left = direct_mode_capture;
			g_direct_mode->m_capture_delay_s = direct_mode_capture_delay_s;
		} else {
			ovrd_log("Direct mode: Monado's compositor took no client, SteamVR's compositor instead\n");
			xrt_comp_native_destroy(&m_xcn);
			xrt_syscomp_destroy(&m_xsysc);
		}
	}

	// The driver's socket (ovrd_socket.hpp): video players' HDR descriptions, status for tools.
	{
		OvrdSocketHandlers handlers;
		handlers.hdr = [](uint32_t pid, const struct xrt_layer_hdr_data *hdr) {
			// Only Monado's compositor (direct mode) shows them; SteamVR's has no use for them.
			if (g_direct_mode != nullptr) {
				g_direct_mode->set_hdr(pid, hdr);
			}
		};
		const std::string hmd = m_xhmd->str;
		handlers.status = [hmd](cJSON *status) {
			cJSON_AddStringToObject(status, "hmd", hmd.c_str());
			cJSON_AddBoolToObject(status, "direct_mode", g_direct_mode != nullptr);
			cJSON_AddStringToObject(status, "compositor", g_direct_mode != nullptr ? "monado" : "steamvr");
			// Which HDR parts are on; they act in direct mode only.
			struct render_output_settings rs;
			render_output_settings_get(&rs);
			cJSON *hdr = cJSON_AddObjectToObject(status, "hdr");
			cJSON_AddBoolToObject(hdr, "auto_hdr", rs.auto_hdr);
			cJSON_AddBoolToObject(hdr, "float_layers", rs.hdr_float_layers);
			cJSON_AddBoolToObject(hdr, "players", g_direct_mode != nullptr && g_direct_mode->m_players_hdr.load());
			cJSON_AddBoolToObject(hdr, "metadata", rs.hdr_metadata);
			cJSON_AddBoolToObject(hdr, "measured_peak", rs.hdr_measured_peak);
			cJSON_AddBoolToObject(hdr, "bright", rs.hdr_bright);
		};
		handlers.recalibrate = []() { return request_recalibration("the driver's socket"); };
		handlers.quit = []() { g_quit_requested.store(true); };
		ovrd_socket_start(handlers);
	}
#endif

	g_display.direct_running.store(g_direct_mode != nullptr);
	ovrd_log("Selected HMD %s\n", m_xhmd->str);
	m_MonadoDeviceDriver = new CDeviceDriver_Monado(m_xinst, m_xhmd);
	//! @todo provide a serial number
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_xhmd->str, vr::TrackedDeviceClass_HMD, m_MonadoDeviceDriver);

	xrt_system_roles system_roles = XRT_SYSTEM_ROLES_INIT;
	xrt_system_devices_get_roles(m_xsysd, &system_roles);

	struct xrt_device *left_xdev = nullptr;
	if (system_roles.left >= 0 && system_roles.left < XRT_SYSTEM_MAX_DEVICES) {
		left_xdev = m_xsysd->static_xdevs[system_roles.left];
	}
	struct xrt_device *right_xdev = nullptr;
	if (system_roles.right >= 0 && system_roles.right < XRT_SYSTEM_MAX_DEVICES) {
		right_xdev = m_xsysd->static_xdevs[system_roles.right];
	}

	// use steamvr room setup instead
	struct xrt_vec3 offset = {0, 0, 0};
	u_builder_setup_tracking_origins(m_xhmd, nullptr, left_xdev, right_xdev, nullptr, &offset);

	if (left_xdev) {
		m_left = new CDeviceDriver_Monado_Controller(m_xinst, left_xdev, XRT_HAND_LEFT);
		ovrd_log("Added left Controller: %s\n", left_xdev->str);
	}
	if (right_xdev) {
		m_right = new CDeviceDriver_Monado_Controller(m_xinst, right_xdev, XRT_HAND_RIGHT);
		ovrd_log("Added right Controller: %s\n", right_xdev->str);
	}

	return vr::VRInitError_None;
}

void
CServerDriver_Monado::Cleanup()
{
#ifdef XRT_OS_LINUX
	// Before direct mode goes: the socket's descriptions are handed to it.
	ovrd_socket_stop();
	notices_stop();
#endif

	if (m_MonadoDeviceDriver != NULL) {
		delete m_MonadoDeviceDriver;
		m_MonadoDeviceDriver = NULL;
	}

#ifdef XRT_OS_LINUX
	// SteamVR's images and Monado's compositor go before the devices it shows.
	if (g_direct_mode != nullptr) {
		delete g_direct_mode;
		g_direct_mode = nullptr;
	}
	if (m_xcn != NULL) {
		xrt_comp_end_session(&m_xcn->base);
		xrt_comp_native_destroy(&m_xcn);
	}
	xrt_syscomp_destroy(&m_xsysc);
#endif

	xrt_space_overseer_destroy(&m_xso);
	xrt_system_devices_destroy(&m_xsysd);
	xrt_system_destroy(&m_xsys);
	m_xhmd = NULL;
	m_left->m_xdev = NULL;
	m_right->m_xdev = NULL;

	if (m_xinst) {
		xrt_instance_destroy(&m_xinst);
	}
}

void
CServerDriver_Monado::HandleHapticEvent(vr::VREvent_t *event)
{
	float freq = event->data.hapticVibration.fFrequency;
	float amp = event->data.hapticVibration.fAmplitude;
	float duration = event->data.hapticVibration.fDurationSeconds;

	ovrd_log("Haptic vibration %fs %fHz %famp\n", duration, freq, amp);

	CDeviceDriver_Monado_Controller *controller = NULL;

	if (m_left && m_left->m_ulPropertyContainer == event->data.hapticVibration.containerHandle) {
		controller = m_left;
		ovrd_log("Haptic vibration left\n");
	} else if (m_right && m_right->m_ulPropertyContainer == event->data.hapticVibration.containerHandle) {
		controller = m_right;
		ovrd_log("Haptic vibration right\n");
	} else {
		ovrd_log("Haptic vibration ignored\n");
		return;
	}

	// The type was left unset: the Touch driver turned down every SteamVR vibration as not one (no buzz).
	struct xrt_output_value out = {};
	out.type = XRT_OUTPUT_VALUE_TYPE_VIBRATION;
	out.vibration.amplitude = amp;
	if (duration > 0.00001) {
		out.vibration.duration_ns = (time_duration_ns)(duration * 1000.f * 1000.f * 1000.f);
	} else {
		out.vibration.duration_ns = XRT_MIN_HAPTIC_DURATION;
	}
	out.vibration.frequency = freq;

	if (controller->m_output_controls.empty()) {
		ovrd_log("Controller %s has no outputs\n", controller->m_xdev->str);
		return;
	}

	// TODO: controllers with more than 1 haptic motors
	SteamVRDriverControlOutput *control = &controller->m_output_controls.at(0);

	enum xrt_output_name name = control->monado_output_name;
	xrt_result_t xret = controller->m_xdev->set_output(controller->m_xdev, name, &out);
	ovrd_log("Haptic vibration %s, %d: %d\n", control->steamvr_control_path, name, (int)xret);
}

void
CServerDriver_Monado::RunFrame()
{
#ifdef XRT_OS_LINUX
	riftlinux_helper_poll();
	/*
	 * The Rift tab's second page: the headset's state and the display settings, once a second, sent when
	 * they change.
	 */
	{
		static int64_t status_at_ns = 0;
		const int64_t now_ns = os_monotonic_get_ns();
		if (now_ns - status_at_ns > U_TIME_1S_IN_NS) {
			status_at_ns = now_ns;
			display_read_settings_live();
			u_notice_sensors placed_now = {};
			const bool placed_known = u_notice_get_sensors(&placed_now);
			const bool worn_now = m_xhmd != nullptr && head_worn(m_xhmd);

			notices_first_calibration(placed_known && placed_now.found > 0 && placed_now.placed == 0, worn_now);
			notices_controller_hint(m_left != nullptr && m_left->m_battery_percent.load() >= 0,
			                        m_right != nullptr && m_right->m_battery_percent.load() >= 0,
			                        !placed_known || placed_now.placed > 0);
			// Desk test (notices.test_put_on_s): as if the headset was put on, once.
			static const int64_t started_ns = now_ns;
			static bool test_put_on_done = false;
			int test_put_on_s = 0;
			{
				std::lock_guard<std::mutex> lock(g_notices.lock);
				test_put_on_s = g_notices.test_put_on_s;
			}
			if (test_put_on_s > 0 && !test_put_on_done && now_ns - started_ns > (int64_t)test_put_on_s * U_TIME_1S_IN_NS) {
				test_put_on_done = true;
				ovrd_log("Notices: test: as if the headset was put on\n");
				notices_on_put_on();
			}
			u_notice_sensors sensors = {};
			const bool sensors_known = u_notice_get_sensors(&sensors);
			const char *wayland = getenv("WAYLAND_DISPLAY");
			const int display = g_direct_mode == nullptr ? 0 : (wayland != nullptr && wayland[0] != '\0') ? 2 : 1;
			const std::string status =
			    "cmd-status worn=" + std::to_string(m_xhmd != nullptr && head_worn(m_xhmd) ? 1 : 0) +
			    " left=" + std::to_string(m_left != nullptr ? m_left->m_battery_percent.load() : -2) +
			    " right=" + std::to_string(m_right != nullptr ? m_right->m_battery_percent.load() : -2) +
			    " sensors-known=" + std::to_string(sensors_known ? 1 : 0) +
			    " sensors-found=" + std::to_string(sensors.found) +
			    " sensors-placed=" + std::to_string(sensors.placed) +
			    " sensors-measuring=" + std::to_string(sensors.measuring || sensors.placed < sensors.found ? 1 : 0) +
			    " display=" + std::to_string(display);
			struct render_output_settings rs;
			render_output_settings_get(&rs);
			const std::string display_line =
			    "cmd-display direct-saved=" + std::to_string(g_display.direct_saved.load() ? 1 : 0) +
			    " direct-running=" + std::to_string(g_display.direct_running.load() ? 1 : 0) +
			    " scale-saved=" + std::to_string(g_display.scale_saved.load()) +
			    " scale-running=" + std::to_string(g_display.scale_running.load()) +
			    " fast-path=" + std::to_string(rs.fast_path ? 1 : 0) +
			    " perf=" + std::to_string(g_display.perf.load() ? 1 : 0);
			std::lock_guard<std::mutex> lock(g_notices.lock);
			if (status != g_tab_status_line) {
				g_tab_status_line = status;
				notices_send_locked(status);
				ovrd_log("Rift panel: %s\n", status.c_str() + 4);
			}
			if (display_line != g_tab_display_line) {
				g_tab_display_line = display_line;
				notices_send_locked(display_line);
				ovrd_log("Rift panel: %s\n", display_line.c_str() + 4);
			}
		}
	}
	// Measuring the sensors again, asked from the Rift tab (asked here, not under the notices' lock: see g_recal);
	// quitting, asked through the driver's socket; the floor kept through a recalibration.
	if (g_recal.asked.exchange(false)) {
		(void)request_recalibration("the Rift tab");
	}
	if (g_quit_requested.exchange(false)) {
		ovrd_log("Asked to quit (driver socket): SteamVR quits the way its Exit does\n");
		vr::VRServerDriverHost()->VendorSpecificEvent(vr::k_unTrackedDeviceIndex_Hmd, vr::VREvent_DriverRequestedQuit,
		                                              {}, 0);
	}
	floor_run_frame();
	/*
	 * HDR Bright acts on HDR content only (float images with highlights, a described video, the test chart), not on
	 * SteamVR Home or 8-bit games: the Rift tab says whether there is any, so the switch is not taken for broken.
	 * On after the compositor showed an HDR layer, off 1.5 s after the last one.
	 */
	if (g_direct_mode != nullptr) {
		const int64_t seen = render_output_last_hdr_layer_ns();
		const int now = seen != 0 && os_monotonic_get_ns() - seen < 1500LL * U_TIME_1MS_IN_NS ? 1 : 0;
		if (now != g_hdr_content.load()) {
			g_hdr_content.store(now);
			std::lock_guard<std::mutex> lock(g_notices.lock);
			notices_send_locked("cmd-hdr-content " + std::to_string(now));
			ovrd_log("Display: HDR content %s\n", now ? "on screen" : "gone");
		}
	}
	/*
	 * "Set floor": the lower of the two controllers lies on the floor; its origin is about 4 cm up when it lies
	 * flat. Every device follows at once.
	 */
	if (g_world_from_driver.floor_request.exchange(false)) {
		float lowest = 1e9f;
		for (CDeviceDriver_Monado_Controller *c : {m_left, m_right}) {
			if (c != nullptr && c->m_pose.poseIsValid) {
				lowest = std::min(lowest, (float)c->m_pose.vecPosition[1]);
			}
		}
		if (lowest < 1e8f) {
			const float floor = 0.04f - lowest;
			g_world_from_driver.floor_m.store(floor);
			g_world_from_driver.floor_known.store(true);
			riftlinux_settings_set_number("tracking", "floor_m", floor);
			ovrd_log("Rift panel: floor set, the lower controller at %.3f m, floor %.3f m\n", lowest, floor);
		} else {
			ovrd_log("Rift panel: no tracked controller to set the floor from\n");
		}
	}
#endif
	if (m_left) {
		m_left->RunFrame();
	}
	if (m_right) {
		m_right->RunFrame();
	}

	// https://github.com/ValveSoftware/openvr/issues/719#issuecomment-358038640
	struct vr::VREvent_t event;
	while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(struct vr::VREvent_t))) {
		switch (event.eventType) {
		case vr::VREvent_Input_HapticVibration: HandleHapticEvent(&event); break;
		case vr::VREvent_PropertyChanged:
			// ovrd_log("Property changed\n");
			break;
		case vr::VREvent_TrackedDeviceActivated:
			ovrd_log("Device activated %d\n", event.trackedDeviceIndex);
			break;
		case vr::VREvent_TrackedDeviceUserInteractionStarted:
			ovrd_log("Device interaction started %d\n", event.trackedDeviceIndex);
			break;
		case vr::VREvent_IpdChanged: ovrd_log("ipd changed to %fm\n", event.data.ipd.ipdMeters); break;
		// This event currently spams the console, so is currently commented out. see
		// https://github.com/ValveSoftware/SteamVR-for-Linux/issues/307
		// case vr::VREvent_ActionBindingReloaded: ovrd_log("action binding reloaded\n"); break;
		case vr::VREvent_StatusUpdate: ovrd_log("EVRState: %d\n", event.data.status.statusState); break;

		case vr::VREvent_TrackedDeviceRoleChanged:
			// device roles are for legacy input
		case vr::VREvent_ChaperoneUniverseHasChanged: break;
		// Which process connects and quits when: SteamVR's shutdown order (its compositor's exit crash, README).
		case vr::VREvent_Quit:
		case vr::VREvent_ProcessQuit:
		case vr::VREvent_QuitAcknowledged:
		case vr::VREvent_ProcessDisconnected:
		case vr::VREvent_ProcessConnected: {
			char comm[32] = "?";
			char path[64];
			snprintf(path, sizeof(path), "/proc/%u/comm", event.data.process.pid);
			FILE *f = fopen(path, "r");
			if (f != nullptr) {
				if (fgets(comm, sizeof(comm), f) == nullptr) {
					snprintf(comm, sizeof(comm), "?");
				}
				comm[strcspn(comm, "\n")] = '\0';
				fclose(f);
			}
			const char *what = event.eventType == vr::VREvent_Quit               ? "quit asked of"
			                   : event.eventType == vr::VREvent_ProcessQuit      ? "process quit:"
			                   : event.eventType == vr::VREvent_QuitAcknowledged ? "quit acknowledged by"
			                   : event.eventType == vr::VREvent_ProcessConnected ? "process connected:"
			                                                                      : "process disconnected:";
			ovrd_log("SteamVR event: %s pid %u (%s)%s\n", what, event.data.process.pid, comm,
			         vr::VRServerDriverHost()->IsExiting() ? ", SteamVR exiting" : "");
			break;
		}
		case vr::VREvent_DashboardActivated:
		case vr::VREvent_DashboardDeactivated:
		case vr::VREvent_Compositor_ChaperoneBoundsShown:
		case vr::VREvent_Compositor_ChaperoneBoundsHidden: break;

		default: ovrd_log("Unhandled Event: %d\n", event.eventType);
		}
	}
}


/*
 *
 * Watchdog code
 *
 */

class CWatchdogDriver_Monado : public vr::IVRWatchdogProvider
{
public:
	CWatchdogDriver_Monado()
	{
		ovrd_log("Created watchdog object\n");
		m_pWatchdogThread = nullptr;
	}

	// clang-format off
	virtual ~CWatchdogDriver_Monado() {};
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext);
	virtual void Cleanup();
	// clang-format on

private:
	std::thread *m_pWatchdogThread;
};

CWatchdogDriver_Monado g_watchdogDriverMonado;
bool g_bExiting = false;

void
WatchdogThreadFunction()
{
	while (!g_bExiting) {
#if defined(_WINDOWS)
		// on windows send the event when the Y key is pressed.
		if ((0x01 & GetAsyncKeyState('Y')) != 0) {
			// Y key was pressed.
			vr::VRWatchdogHost()->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
		}
		std::this_thread::sleep_for(std::chrono::microseconds(500));
#else
		ovrd_log("Watchdog wakeup\n");
		// for the other platforms, just send one every five seconds
		std::this_thread::sleep_for(std::chrono::seconds(1));
		vr::VRWatchdogHost()->WatchdogWakeUp(vr::ETrackedDeviceClass::TrackedDeviceClass_HMD);
#endif
	}

	ovrd_log("Watchdog exit\n");
}

vr::EVRInitError
CWatchdogDriver_Monado::Init(vr::IVRDriverContext *pDriverContext)
{
	VR_INIT_WATCHDOG_DRIVER_CONTEXT(pDriverContext);
	ovrd_log_init(vr::VRDriverLog());

	// Watchdog mode on Windows starts a thread that listens for the 'Y' key
	// on the keyboard to be pressed. A real driver should wait for a system
	// button event or something else from the the hardware that signals
	// that the VR system should start up.
	g_bExiting = false;

	ovrd_log("starting watchdog thread\n");

	m_pWatchdogThread = new std::thread(WatchdogThreadFunction);
	if (!m_pWatchdogThread) {
		ovrd_log("Unable to create watchdog thread\n");
		return vr::VRInitError_Driver_Failed;
	}

	return vr::VRInitError_None;
}

void
CWatchdogDriver_Monado::Cleanup()
{
	g_bExiting = true;
	if (m_pWatchdogThread) {
		m_pWatchdogThread->join();
		delete m_pWatchdogThread;
		m_pWatchdogThread = nullptr;
	}

	ovrd_log_cleanup();
}


/*
 *
 * 'Exported' functions.
 *
 */

void *
ovrd_hmd_driver_impl(const char *pInterfaceName, int *pReturnCode)
{
	// clang-format off
	if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) {
		return &g_serverDriverMonado;
	}
	if (0 == strcmp(vr::IVRWatchdogProvider_Version, pInterfaceName)) {
		return &g_watchdogDriverMonado;
	}
	// clang-format on

	ovrd_log("Unimplemented interface: %s\n", pInterfaceName);

	if (pReturnCode) {
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
	}

	return NULL;
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
