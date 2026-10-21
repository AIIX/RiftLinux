// Copyright 2025, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Radio state machine functions for the Oculus Rift.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "xrt/xrt_byte_order.h"
#include "math/m_relation_history.h"
#include "math/m_space.h"
#include "math/m_vec3.h"

#include "util/u_device.h"
#include "util/u_var.h"
#include "util/u_file.h"
#include "util/u_hand_simulation.h"
#include "util/u_hand_tracking.h"

#include "rift_internal.h"
#include "rift_radio.h"
#include "rift_bindings.h"
#include "rift_usb.h"

#include <errno.h>


static void
rift_update_input(struct xrt_device *device, size_t index, union xrt_input_value value, int64_t now)
{
	device->inputs[index].value = value;
	device->inputs[index].timestamp = now;
}

/*
 * Rift Touch Controller device functions
 */

/*!
 * Without a constellation solve for this long the controller's position is held, see
 * rift_touch_controller_get_tracked_pose. The sensor fusion would otherwise integrate the accelerometer twice from the
 * last solve, and past about a second extrapolate its last velocity, so a hidden controller drifted off. Replaying a
 * worn session, a controller solved only now and then had the fusion carry it at up to 8 m/s 0.3-0.5 s after its last
 * solve; with 150 ms its largest step between two frames went from 1.1 m to 0.13 m.
 */
#define RIFT_TOUCH_POSITION_HOLD_NS (150 * U_TIME_1MS_IN_NS)

/*!
 * What the app gets carried on by the sensor fusion for this long, the prior only for RIFT_TOUCH_POSITION_HOLD_NS: past
 * 150 ms, while the fusion's speed and distance from the last solve stay plausible (RIFT_TOUCH_DEAD_RECKON_MAX_*).
 * Against what Oculus tracked, 150-250 ms after the last solve the fusion was half as far off as the held position
 * (left Touch 15 / 25 mm against 31 / 59 mm, day / night), and the switch to holding jumped the controller back to
 * where it was solved: the maintainer saw it stop in space and catch up.
 */
#define RIFT_TOUCH_APP_DEAD_RECKON_NS (300 * U_TIME_1MS_IN_NS)
#define RIFT_TOUCH_DEAD_RECKON_MAX_SPEED_M_S 3.0f
#define RIFT_TOUCH_DEAD_RECKON_MAX_DISTANCE_M 0.3f

/*!
 * A hold starts where the app last saw the controller rather than at its last solve, so it stops without jumping back,
 * unless the two are further apart than this.
 */
#define RIFT_TOUCH_APP_HOLD_MAX_OFFSET_M 0.15f

/*!
 * For this long after its last solve, a lost controller's held pose is the constellation tracker's prior: where it was
 * last seen, turned as its IMU has turned it since, see
 * touch_controller_constellation_tracking_source_get_tracked_pose. As long as the tracker binds a held prior
 * (CONSTELLATION_TRACKER_HELD_PRIOR_MS).
 */
#define RIFT_TOUCH_HELD_PRIOR_NS (5LL * U_TIME_1S_IN_NS)

//! A controller found again slides from where it was held to where it is over this long, rather than jumping.
#define RIFT_TOUCH_REACQUIRE_BLEND_NS (100 * U_TIME_1MS_IN_NS)

//! How far from a requested time an IMU orientation from the history may be and still stand for it.
#define RIFT_TOUCH_IMU_HISTORY_MAX_GAP_NS (50 * U_TIME_1MS_IN_NS)

/*!
 * The controller's pose from the constellation tracker's sensor fusion, as the HMD's, while it solved the controller
 * within RIFT_TOUCH_POSITION_HOLD_NS of @p at_ns.
 *
 * Takes no driver lock: the tracker calls this for the prior while holding its device lock.
 *
 * @return false when there is no such pose.
 */
static bool
touch_get_fused_relation(struct rift_touch_controller *controller,
                         int64_t at_ns,
                         bool for_prior,
                         struct xrt_space_relation *out_relation)
{
	if (xrt_atomic_s32_load(&controller->constellation_ready) == 0) {
		return false;
	}

	const int64_t last_solve_ns = xrt_atomic_s64_load(&controller->last_solve_ns);
	const int64_t age_ns = at_ns - last_solve_ns;
	if (last_solve_ns == 0 || age_ns > (for_prior ? RIFT_TOUCH_POSITION_HOLD_NS : RIFT_TOUCH_APP_DEAD_RECKON_NS)) {
		return false;
	}

	// The prior is searched with the solved pose, the app gets it with the corrections blended in.
	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	const int ret =
	    for_prior ? t_constellation_tracker_get_solved_pose(controller->constellation_tracker,
	                                                        controller->constellation_device_id, at_ns, &relation)
	              : t_constellation_tracker_get_tracked_pose(controller->constellation_tracker,
	                                                         controller->constellation_device_id, at_ns, &relation);
	if (ret < 0) {
		return false;
	}

	const enum xrt_space_relation_flags needed =
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	if ((relation.relation_flags & needed) != needed) {
		return false;
	}

	// Carried on past RIFT_TOUCH_POSITION_HOLD_NS only while it looks like a hand moving, not a fusion running off.
	if (age_ns > RIFT_TOUCH_POSITION_HOLD_NS) {
		if ((relation.relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0 &&
		    m_vec3_len(relation.linear_velocity) > RIFT_TOUCH_DEAD_RECKON_MAX_SPEED_M_S) {
			return false;
		}
		os_mutex_lock(&controller->pose_hold.mutex);
		const bool have_hold = controller->pose_hold.have;
		const struct xrt_vec3 solved = controller->pose_hold.position;
		os_mutex_unlock(&controller->pose_hold.mutex);
		if (have_hold &&
		    m_vec3_len(m_vec3_sub(relation.pose.position, solved)) > RIFT_TOUCH_DEAD_RECKON_MAX_DISTANCE_M) {
			return false;
		}
	}

	*out_relation = relation;
	return true;
}

//! The IMU fusion's orientation nearest @p at_ns from the history. Called with pose_hold.mutex held.
static bool
touch_imu_orientation_at_locked(const struct rift_touch_controller *controller, int64_t at_ns, struct xrt_quat *out)
{
	const uint32_t count = controller->pose_hold.imu_history_count;
	const uint32_t n = MIN(count, (uint32_t)RIFT_TOUCH_IMU_HISTORY);
	int64_t best_gap = INT64_MAX;
	for (uint32_t i = 0; i < n; i++) {
		const uint32_t idx = (count - 1 - i) % RIFT_TOUCH_IMU_HISTORY;
		const int64_t ts = controller->pose_hold.imu_history[idx].timestamp_ns;
		const int64_t gap = ts > at_ns ? ts - at_ns : at_ns - ts;
		if (gap < best_gap) {
			best_gap = gap;
			*out = controller->pose_hold.imu_history[idx].orientation;
		}
	}
	return best_gap <= RIFT_TOUCH_IMU_HISTORY_MAX_GAP_NS;
}

//! Holds a tracked pose, with the IMU fusion's orientation at the same time. Called with pose_hold.mutex held.
static void
touch_update_hold_locked(struct rift_touch_controller *controller,
                         const struct xrt_pose *tracked,
                         const struct xrt_quat *imu_orientation)
{
	struct xrt_quat imu_inverse;
	math_quat_invert(imu_orientation, &imu_inverse);

	controller->pose_hold.have = true;
	controller->pose_hold.position = tracked->position;
	math_quat_rotate(&tracked->orientation, &imu_inverse, &controller->pose_hold.imu_to_world);
}

/*!
 * The pose for apps. Tracked by the cameras, it is the sensor fusion's. Then lost, the position is held where it was
 * last tracked and the orientation follows the controller's own IMU fusion, turned into the tracker's world as the
 * sensor fusion last had it, so the controller keeps turning in place rather than drifting or freezing. Found again,
 * it slides back from where it was held over RIFT_TOUCH_REACQUIRE_BLEND_NS: snapped back at once, it jerked.
 * Never tracked by the cameras, it is a 3dof controller.
 */
//! Once a second per controller, which pose the app got and from where: in recordings the Touch was off to one side.
static void
touch_log_app_relation(struct rift_touch_controller *controller,
                       int64_t at_ns,
                       const char *branch,
                       const struct xrt_space_relation *rel)
{
	static int64_t last_ns[4] = {0};
	const uint32_t i = (uint32_t)controller->device_type & 3;
	if (at_ns - last_ns[i] < U_TIME_1S_IN_NS) {
		return;
	}
	last_ns[i] = at_ns;
	const int64_t last_solve_ns = xrt_atomic_s64_load(&controller->last_solve_ns);
	HMD_INFO(controller->hmd, "Touch %d app pose: %s, at %.3f %.3f %.3f, flags 0x%x, last solve %.0f ms before",
	         controller->device_type, branch, rel->pose.position.x, rel->pose.position.y, rel->pose.position.z,
	         rel->relation_flags, last_solve_ns ? (double)(at_ns - last_solve_ns) / 1e6 : -1.0);
}

static void
touch_get_app_relation(struct rift_touch_controller *controller, int64_t at_ns, struct xrt_space_relation *out_relation)
{
	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;

	/*
	 * While the room is measured (the first start, or the sensors measured again) the tracker looks only for the
	 * headset: the last held place stood frozen in the air, used with the sticks, while the notice said the
	 * controllers come back when it is done. Not shown until the cameras look for it again.
	 */
	if (controller->constellation_tracker != NULL &&
	    controller->constellation_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID &&
	    !t_constellation_tracker_device_tracked(controller->constellation_tracker,
	                                            controller->constellation_device_id)) {
		touch_log_app_relation(controller, at_ns, "not shown while the room is measured", out_relation);
		return;
	}

	struct xrt_quat imu_orientation = XRT_QUAT_IDENTITY;
	struct xrt_vec3 imu_angular_velocity = XRT_VEC3_ZERO;
	os_mutex_lock(&controller->input.mutex);
	const bool have_imu = imu_fusion_get_prediction(controller->input.imu_fusion, (uint64_t)at_ns, &imu_orientation,
	                                                &imu_angular_velocity) == 0;
	os_mutex_unlock(&controller->input.mutex);

	struct xrt_space_relation fused;
	if (touch_get_fused_relation(controller, at_ns, false, &fused)) {
		os_mutex_lock(&controller->pose_hold.mutex);
		if (controller->pose_hold.app_was_held) {
			controller->pose_hold.app_was_held = false;
			controller->pose_hold.blend_start_ns = at_ns;
			controller->pose_hold.blend_from = controller->pose_hold.app_held_pose;
		}

		const int64_t blend_ns = at_ns - controller->pose_hold.blend_start_ns;
		if (controller->pose_hold.blend_start_ns != 0 && blend_ns >= 0 &&
		    blend_ns < RIFT_TOUCH_REACQUIRE_BLEND_NS) {
			const float t = (float)blend_ns / (float)RIFT_TOUCH_REACQUIRE_BLEND_NS;
			const struct xrt_pose from = controller->pose_hold.blend_from;
			fused.pose.position = m_vec3_lerp(from.position, fused.pose.position, t);
			math_quat_slerp(&from.orientation, &fused.pose.orientation, t, &fused.pose.orientation);
		}
		controller->pose_hold.have_app_tracked = true;
		controller->pose_hold.app_tracked_position = fused.pose.position;
		os_mutex_unlock(&controller->pose_hold.mutex);

		*out_relation = fused;
		touch_log_app_relation(controller, at_ns, "tracked", out_relation);
		return;
	}

	if (!have_imu) {
		return;
	}

	os_mutex_lock(&controller->pose_hold.mutex);
	const bool have_hold = controller->pose_hold.have;
	const struct xrt_quat imu_to_world = controller->pose_hold.imu_to_world;
	if (have_hold && !controller->pose_hold.app_was_held) {
		// The hold starts: where the app last saw it, unless that is too far from the last solve.
		const struct xrt_vec3 solved = controller->pose_hold.position;
		const struct xrt_vec3 shown = controller->pose_hold.app_tracked_position;
		controller->pose_hold.app_hold_position =
		    controller->pose_hold.have_app_tracked &&
		            m_vec3_len(m_vec3_sub(shown, solved)) <= RIFT_TOUCH_APP_HOLD_MAX_OFFSET_M
		        ? shown
		        : solved;
	}
	const struct xrt_vec3 hold_position = controller->pose_hold.app_hold_position;
	os_mutex_unlock(&controller->pose_hold.mutex);

	out_relation->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                               XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;

	if (have_hold) {
		math_quat_rotate(&imu_to_world, &imu_orientation, &out_relation->pose.orientation);
		out_relation->pose.position = hold_position;
		out_relation->relation_flags |= XRT_SPACE_RELATION_POSITION_VALID_BIT;

		os_mutex_lock(&controller->pose_hold.mutex);
		controller->pose_hold.app_was_held = true;
		controller->pose_hold.app_held_pose = out_relation->pose;
		os_mutex_unlock(&controller->pose_hold.mutex);
	} else {
		out_relation->pose.orientation = imu_orientation;
	}

	// The IMU fusion's angular velocity is in the controller's frame, apps get it in the world's.
	math_quat_rotate_vec3(&out_relation->pose.orientation, &imu_angular_velocity, &out_relation->angular_velocity);
	touch_log_app_relation(controller, at_ns, have_hold ? "held" : "orientation only", out_relation);
}

/*
 * An app's grip and aim poses on a CV1 Touch, in the frame its LEDs and IMU are tracked in. Grip: fitted against the
 * Oculus runtime's OpenXR grip poses over the day and night captures, both hands alike within 2 mm and
 * 1 degree: turned 60 degrees about the controller's x axis, 32 mm below and 40 mm behind. Aim: from the grip as
 * SteamVR's CV1 model has it (its openxr_grip and openxr_aim parts), which leaves the tracked frame 55 mm forward.
 * Both were the tracked frame itself before: SteamVR's laser, drawn from the model, pointed 39 degrees low.
 */
static const struct xrt_pose touch_grip_in_tracked = {
    .orientation = {.x = 0.5f, .y = 0.0f, .z = 0.0f, .w = 0.8660254f},
    .position = {.x = 0.0f, .y = -0.03207f, .z = 0.04045f},
};
static const struct xrt_pose touch_aim_in_tracked = {
    .orientation = {.x = 0.0f, .y = 0.0f, .z = 0.0f, .w = 1.0f},
    .position = {.x = 0.0f, .y = -0.00207f, .z = -0.05455f},
};

static xrt_result_t
rift_touch_controller_get_tracked_pose(struct xrt_device *xdev,
                                       const enum xrt_input_name name,
                                       int64_t at_timestamp_ns,
                                       struct xrt_space_relation *out_relation)
{
	struct rift_touch_controller *controller = (struct rift_touch_controller *)xdev;

	switch (name) {
	case XRT_INPUT_TOUCH_GRIP_POSE:
	case XRT_INPUT_TOUCH_AIM_POSE: {
		struct xrt_space_relation tracked;
		touch_get_app_relation(controller, at_timestamp_ns, &tracked);

		struct xrt_relation_chain xrc = {0};
		m_relation_chain_push_pose(&xrc, name == XRT_INPUT_TOUCH_GRIP_POSE ? &touch_grip_in_tracked
		                                                                   : &touch_aim_in_tracked);
		m_relation_chain_push_relation(&xrc, &tracked);
		m_relation_chain_resolve(&xrc, out_relation);
		break;
	}
	default: return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	return XRT_SUCCESS;
}

/*!
 * The prior the constellation tracker checks a new camera sample against: the sensor fusion's pose while it is fresh.
 * Its own raw solves were the prior before, and a wrong one, turned 130-180 degrees, was then confirmed frame after
 * frame for up to 47 s.
 *
 * Lost for longer, it is the held pose, position valid but not tracked, for RIFT_TOUCH_HELD_PRIOR_NS: with a prior
 * the tracker searches near it from 4 LEDs, without one from scratch and needs 7. With none, a controller lost in a
 * fast wave or behind the back stayed lost for 2-4 s in a recorded session.
 *
 * Takes only pose_hold.mutex, which is a leaf: the tracker calls this holding its own locks.
 */
static void
touch_controller_constellation_tracking_source_get_tracked_pose(
    struct t_constellation_tracker_tracking_source *tracking_source,
    timepoint_ns when_ns,
    struct xrt_space_relation *out_relation)
{
	struct rift_touch_controller *controller =
	    container_of(tracking_source, struct rift_touch_controller, constellation_tracking_source);

	struct xrt_quat imu_orientation;
	if (touch_get_fused_relation(controller, when_ns, true, out_relation)) {
		return;
	}

	*out_relation = (struct xrt_space_relation)XRT_SPACE_RELATION_ZERO;

	const int64_t last_solve_ns = xrt_atomic_s64_load(&controller->last_solve_ns);
	if (last_solve_ns == 0 || when_ns - last_solve_ns > RIFT_TOUCH_HELD_PRIOR_NS) {
		return;
	}

	os_mutex_lock(&controller->pose_hold.mutex);
	if (controller->pose_hold.have && touch_imu_orientation_at_locked(controller, when_ns, &imu_orientation)) {
		math_quat_rotate(&controller->pose_hold.imu_to_world, &imu_orientation,
		                 &out_relation->pose.orientation);
		out_relation->pose.position = controller->pose_hold.position;
		out_relation->relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		                               XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		                               XRT_SPACE_RELATION_POSITION_VALID_BIT;
	}
	os_mutex_unlock(&controller->pose_hold.mutex);
}

static void
rift_touch_controller_destroy(struct xrt_device *xdev)
{
	struct rift_touch_controller *controller = rift_touch_controller(xdev);

	controller->node.break_apart(&controller->node);
}

static xrt_result_t
rift_touch_controller_update_inputs(struct xrt_device *xdev)
{
	struct rift_touch_controller *controller = rift_touch_controller(xdev);

	os_mutex_lock(&controller->input.mutex);
	struct rift_touch_controller_input_state input_state = controller->input.state;
	os_mutex_unlock(&controller->input.mutex);

	uint64_t update_ns = controller->input.device_local_ns;

	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_A_CLICK,
	                  (union xrt_input_value){.boolean = input_state.buttons & RIFT_TOUCH_CONTROLLER_BUTTON_A},
	                  update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_B_CLICK,
	                  (union xrt_input_value){.boolean = input_state.buttons & RIFT_TOUCH_CONTROLLER_BUTTON_B},
	                  update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_SYSTEM_CLICK,
	                  (union xrt_input_value){.boolean = input_state.buttons & RIFT_TOUCH_CONTROLLER_BUTTON_MENU},
	                  update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK_CLICK,
	                  (union xrt_input_value){.boolean = input_state.buttons & RIFT_TOUCH_CONTROLLER_BUTTON_STICK},
	                  update_ns);

	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK,
	                  (union xrt_input_value){.vec2 = {CLAMP(input_state.stick.x, -1.0f, 1.0f),
	                                                   CLAMP(input_state.stick.y, -1.0f, 1.0f)}},
	                  update_ns);

	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_TRIGGER_VALUE,
	                  (union xrt_input_value){.vec1 = {CLAMP(input_state.trigger, 0.0f, 1.0f)}}, update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_SQUEEZE_VALUE,
	                  (union xrt_input_value){.vec1 = {CLAMP(input_state.grip, 0.0f, 1.0f)}}, update_ns);

	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_A_TOUCH,
	                  (union xrt_input_value){.boolean = input_state.cap_a_x >= 1}, update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_B_TOUCH,
	                  (union xrt_input_value){.boolean = input_state.cap_b_y >= 1}, update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_TRIGGER_TOUCH,
	                  (union xrt_input_value){.boolean = input_state.cap_trigger >= 1}, update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK_TOUCH,
	                  (union xrt_input_value){.boolean = input_state.cap_stick >= 1}, update_ns);
	rift_update_input(&controller->base, RIFT_TOUCH_CONTROLLER_INPUT_THUMBREST_TOUCH,
	                  (union xrt_input_value){.boolean = input_state.cap_thumbrest >= 1}, update_ns);

	return XRT_SUCCESS;
}

/*!
 * How far the index finger is curled when it only rests on the trigger, of the curl a fully pulled trigger gives.
 * Lifted off the trigger it points.
 */
#define RIFT_TOUCH_INDEX_RESTING_CURL 0.4f

/*!
 * Finger poses from what the Touch senses, as the Oculus driver gives them to SteamVR's skeletal input on Windows
 * (VRChat's hands, No Man's Sky's): the index finger straight unless it rests on the trigger, then curled as far as the
 * trigger is pulled; the other three as far as the grip is squeezed; the thumb down on a button, the stick or the thumb
 * rest, up otherwise. The Index controller's hand model takes the curls, see u_hand_simulation.h.
 */
static xrt_result_t
rift_touch_controller_get_hand_tracking(struct xrt_device *xdev,
                                        enum xrt_input_name name,
                                        int64_t requested_timestamp_ns,
                                        struct xrt_hand_joint_set *out_value,
                                        int64_t *out_timestamp_ns)
{
	struct rift_touch_controller *controller = rift_touch_controller(xdev);
	const bool left = controller->device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH;
	if (name != (left ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT)) {
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	os_mutex_lock(&controller->input.mutex);
	const struct rift_touch_controller_input_state state = controller->input.state;
	os_mutex_unlock(&controller->input.mutex);

	const float trigger = CLAMP(state.trigger, 0.0f, 1.0f);
	const float grip = CLAMP(state.grip, 0.0f, 1.0f);
	const bool thumb_down =
	    state.cap_a_x >= 1 || state.cap_b_y >= 1 || state.cap_stick >= 1 || state.cap_thumbrest >= 1;
	const struct u_hand_tracking_curl_values values = {
	    .little = grip,
	    .ring = grip,
	    .middle = grip,
	    .index = state.cap_trigger >= 1
	                 ? RIFT_TOUCH_INDEX_RESTING_CURL + (1.0f - RIFT_TOUCH_INDEX_RESTING_CURL) * trigger
	                 : 0.0f,
	    .thumb = thumb_down ? 1.0f : 0.0f,
	};

	struct xrt_space_relation hand_relation;
	xrt_result_t xret =
	    rift_touch_controller_get_tracked_pose(xdev, XRT_INPUT_TOUCH_GRIP_POSE, requested_timestamp_ns, &hand_relation);
	if (xret != XRT_SUCCESS) {
		return xret;
	}

	u_hand_sim_simulate_for_valve_index_knuckles(&values, left ? XRT_HAND_LEFT : XRT_HAND_RIGHT, &hand_relation,
	                                             out_value);
	*out_timestamp_ns = requested_timestamp_ns;
	out_value->is_active = true;
	return XRT_SUCCESS;
}

//! The shortest and the weakest vibration a Touch plays, see rift_touch_controller_set_output.
#define RIFT_TOUCH_HAPTIC_MIN_NS (40 * U_TIME_1MS_IN_NS)
#define RIFT_TOUCH_HAPTIC_MIN_AMPLITUDE 0.25f

static xrt_result_t
rift_touch_controller_set_output(struct xrt_device *xdev,
                                 enum xrt_output_name name,
                                 const struct xrt_output_value *value)
{
	struct rift_touch_controller *controller = rift_touch_controller(xdev);

	if (name != XRT_OUTPUT_NAME_TOUCH_HAPTIC) {
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}

	if (value->type != XRT_OUTPUT_VALUE_TYPE_VIBRATION) {
		return XRT_ERROR_OUTPUT_UNSUPPORTED;
	}

	timepoint_ns now = os_monotonic_get_ns();
	/*
	 * SteamVR's dashboard ticks are 11 ms long at 5-40 % strength: a Touch's motor does not get going on that, and
	 * nothing could be felt. Requests are stretched to what the motor can play, a full one stays as it is.
	 */
	timepoint_ns end_time = now + MAX(value->vibration.duration_ns, RIFT_TOUCH_HAPTIC_MIN_NS);
	const float requested = CLAMP(value->vibration.amplitude, 0.0f, 1.0f);
	const float amplitude =
	    requested > 0.0f ? RIFT_TOUCH_HAPTIC_MIN_AMPLITUDE + (1.0f - RIFT_TOUCH_HAPTIC_MIN_AMPLITUDE) * requested
	                     : 0.0f;

	// if it's higher than the middle of the two possible frequencies, pick the high frequency rumble
	bool high_freq = value->vibration.frequency > ((320.0 + 160.0) / 2.0);

	os_mutex_lock(&controller->input.mutex);
	controller->input.haptic.end_time_ns = end_time;
	controller->input.haptic.high_freq = high_freq;
	controller->input.haptic.amplitude = amplitude;
	os_mutex_unlock(&controller->input.mutex);

	return XRT_SUCCESS;
}

static xrt_result_t
rift_touch_controller_get_battery_status(struct xrt_device *xdev,
                                         bool *out_present,
                                         bool *out_charging,
                                         float *out_charge)
{
	struct rift_touch_controller *controller = rift_touch_controller(xdev);

	int32_t battery_status = xrt_atomic_s32_load(&controller->input.battery_status);

	*out_charging = false;

	if (battery_status == -1) {
		*out_present = false;
		return XRT_SUCCESS;
	}

	*out_present = true;
	*out_charge = (float)battery_status / 100.0f;

	return XRT_SUCCESS;
}

void
rift_touch_controller_node_break_apart(struct xrt_frame_node *node)
{
	struct rift_touch_controller *controller = rift_touch_controller_from_node(node);

	os_mutex_lock(&controller->constellation_mutex);

	if (controller->constellation_tracker &&
	    controller->constellation_device_id != XRT_CONSTELLATION_INVALID_DEVICE_ID) {
		t_constellation_tracker_remove_device(controller->constellation_tracker,
		                                      controller->constellation_device_id);

		controller->constellation_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
		controller->use_constellation = false;
	}

	os_mutex_unlock(&controller->constellation_mutex);
}

/*
 * The gyro bias of a controller, kept from one session to the next by serial, as the Oculus runtime keeps it in its
 * Ekf_<serial> files: a controller that is picked up without having lain still first starts right too.
 */
#define RIFT_TOUCH_GYRO_BIAS_DIR "rift"
#define RIFT_TOUCH_GYRO_BIAS_SAVE_NS (60LL * U_TIME_1S_IN_NS)
#define RIFT_TOUCH_GYRO_BIAS_SAVE_CHANGE 0.0005f

static void
rift_touch_gyro_bias_filename(struct rift_touch_controller *controller, char *out, size_t size)
{
	snprintf(out, size, "touch_%s_gyro_bias.txt", controller->base.serial);
}

static void
rift_touch_gyro_bias_load(struct rift_hmd *hmd, struct rift_touch_controller *controller)
{
	char filename[XRT_DEVICE_NAME_LEN + 32];
	rift_touch_gyro_bias_filename(controller, filename, sizeof(filename));
	FILE *f = u_file_open_file_in_config_dir_subpath(RIFT_TOUCH_GYRO_BIAS_DIR, filename, "r");
	if (f == NULL) {
		return;
	}
	struct xrt_vec3 bias;
	const int n = fscanf(f, "%f %f %f", &bias.x, &bias.y, &bias.z);
	fclose(f);
	if (n != 3 || fabsf(bias.x) > 0.1f || fabsf(bias.y) > 0.1f || fabsf(bias.z) > 0.1f) {
		HMD_WARN(hmd, "Ignoring the gyro bias saved for Touch controller %s, it does not read right.",
		         controller->base.serial);
		return;
	}
	m_gyro_rest_bias_set(&controller->input.gyro_rest_bias, &bias);
	controller->input.gyro_bias_saved = bias;
	HMD_INFO(hmd, "Touch controller %s: gyro bias %.4f %.4f %.4f rad/s from last session.", controller->base.serial,
	         bias.x, bias.y, bias.z);
}

static void
rift_touch_gyro_bias_save(struct rift_hmd *hmd, struct rift_touch_controller *controller, timepoint_ns now_ns)
{
	const struct xrt_vec3 bias = controller->input.gyro_rest_bias.bias;
	controller->input.gyro_bias_saved_ns = now_ns;
	controller->input.gyro_bias_saved = bias;

	char filename[XRT_DEVICE_NAME_LEN + 32];
	rift_touch_gyro_bias_filename(controller, filename, sizeof(filename));
	FILE *f = u_file_open_file_in_config_dir_subpath(RIFT_TOUCH_GYRO_BIAS_DIR, filename, "w");
	if (f == NULL) {
		HMD_WARN(hmd, "Could not save the gyro bias of Touch controller %s.", controller->base.serial);
		return;
	}
	fprintf(f, "%.6f %.6f %.6f\n", bias.x, bias.y, bias.z);
	fclose(f);
}

static bool
rift_touch_gyro_bias_changed(const struct rift_touch_controller *controller)
{
	const struct xrt_vec3 d = m_vec3_sub(controller->input.gyro_rest_bias.bias, controller->input.gyro_bias_saved);
	return controller->input.gyro_rest_bias.have_bias && m_vec3_len(d) > RIFT_TOUCH_GYRO_BIAS_SAVE_CHANGE;
}

void
rift_touch_controller_node_destroy(struct xrt_frame_node *node)
{
	struct rift_touch_controller *controller = rift_touch_controller_from_node(node);

	u_var_remove_root(controller);

	if (controller->input.gyro_bias_loaded && rift_touch_gyro_bias_changed(controller)) {
		rift_touch_gyro_bias_save(controller->hmd, controller, os_monotonic_get_ns());
	}

	if (controller->input.mutex_created) {
		os_mutex_destroy(&controller->input.mutex);
	}

	if (controller->constellation_mutex_created) {
		os_mutex_destroy(&controller->constellation_mutex);
	}

	if (controller->radio_data.calibration_body_json) {
		free(controller->radio_data.calibration_body_json);
	}

	if (controller->input.calibration.led_model.leds) {
		free(controller->input.calibration.led_model.leds);
	}

	if (controller->input.clock_tracker) {
		m_clock_windowed_skew_tracker_destroy(controller->input.clock_tracker);
	}

	if (controller->input.imu_fusion) {
		imu_fusion_destroy(controller->input.imu_fusion);
	}

	if (controller->pose_hold.mutex_created) {
		os_mutex_destroy(&controller->pose_hold.mutex);
	}

	u_device_free(&controller->base);
}

/*
 *
 * Rift Remote device functions
 *
 */

static xrt_result_t
rift_remote_get_tracked_pose(struct xrt_device *xdev,
                             const enum xrt_input_name name,
                             int64_t at_timestamp_ns,
                             struct xrt_space_relation *out_relation)
{
	// The remote has no tracked poses
	return XRT_ERROR_INPUT_UNSUPPORTED;
}

static void
rift_remote_destroy(struct xrt_device *xdev)
{
	// stub
}

static xrt_result_t
rift_remote_update_inputs(struct xrt_device *xdev)
{
	struct rift_remote *remote = rift_remote(xdev);

	uint64_t now = os_monotonic_get_ns();

	uint16_t buttons = xrt_atomic_s32_load(&remote->buttons);

	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_DPAD_UP,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_DPAD_UP}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_DPAD_DOWN,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_DPAD_DOWN}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_DPAD_LEFT,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_DPAD_LEFT}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_DPAD_RIGHT,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_DPAD_RIGHT}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_SELECT,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_SELECT}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_VOLUME_UP,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_VOLUME_UP}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_VOLUME_DOWN,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_VOLUME_DOWN}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_BACK,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_BACK}, now);
	rift_update_input(&remote->base, RIFT_REMOTE_INPUT_OCULUS,
	                  (union xrt_input_value){.boolean = buttons & RIFT_REMOTE_BUTTON_MASK_OCULUS}, now);

	return XRT_SUCCESS;
}

static void
rift_remote_node_break_apart(struct xrt_frame_node *node)
{}

static void
rift_remote_node_destroy(struct xrt_frame_node *node)
{
	struct rift_remote *remote = rift_remote_from_node(node);

	u_var_remove_root(remote);

	u_device_free(&remote->base);
}

/*
 * Implementation functions
 */

static struct rift_remote *
rift_remote_create(struct rift_hmd *hmd, struct xrt_frame_context *xfctx)
{
	struct rift_remote *remote =
	    U_DEVICE_ALLOCATE(struct rift_remote, U_DEVICE_ALLOC_TRACKING_NONE, RIFT_REMOTE_INPUT_COUNT, 1);
	if (remote == NULL) {
		HMD_ERROR(hmd, "Failed to allocate remote");
		return NULL;
	}

	remote->node.break_apart = rift_remote_node_break_apart;
	remote->node.destroy = rift_remote_node_destroy;

	snprintf(remote->base.str, XRT_DEVICE_NAME_LEN, "Oculus Rift Remote");
	remote->base.device_type = XRT_DEVICE_TYPE_GAMEPAD;
	remote->base.name = XRT_DEVICE_RIFT_REMOTE;

#define SET_INPUT(NAME)                                                                                                \
	do {                                                                                                           \
		remote->base.inputs[RIFT_REMOTE_INPUT_##NAME].name = XRT_INPUT_RIFT_REMOTE_##NAME##_CLICK;             \
	} while (0)
	SET_INPUT(DPAD_UP);
	SET_INPUT(DPAD_DOWN);
	SET_INPUT(DPAD_LEFT);
	SET_INPUT(DPAD_RIGHT);
	SET_INPUT(SELECT);
	SET_INPUT(VOLUME_UP);
	SET_INPUT(VOLUME_DOWN);
	SET_INPUT(BACK);
	SET_INPUT(OCULUS);
#undef SET_INPUT

	u_device_populate_function_pointers(&remote->base, rift_remote_get_tracked_pose, rift_remote_destroy);
	remote->base.update_inputs = rift_remote_update_inputs;

	remote->base.binding_profiles = remote_profile_bindings;
	remote->base.binding_profile_count = remote_profile_bindings_count;

	u_var_add_root(remote, "Rift Remote", true);
	u_var_add_ro_u32(remote, (uint32_t *)&remote->buttons, "buttons");

	xrt_frame_context_add(xfctx, &remote->node);

	return remote;
}

void
touch_controller_constellation_device_push_constellation_tracker_sample(
    struct t_constellation_tracker_device *connection, struct t_constellation_tracker_sample *sample)
{
	struct rift_touch_controller *controller =
	    container_of(connection, struct rift_touch_controller, constellation_device);

	// Cameras that have not been placed by rig calibration yet produce samples without a world pose.
	if (!sample->has_world_pose) {
		return;
	}

	// The pose itself goes to the tracker's sensor fusion, apps get it from there.
	if (sample->timestamp_ns > xrt_atomic_s64_load(&controller->last_solve_ns)) {
		xrt_atomic_s64_store(&controller->last_solve_ns, sample->timestamp_ns);
	}

	/*
	 * The hold is anchored on the solve itself, with the IMU's orientation at the same moment: not on the sensor
	 * fusion's output, which can be off by tens of degrees for seconds, and a held prior that far
	 * off binds the tracker's search away from the controller for its whole 5 s. In a recording the right
	 * controller so held was 30 degrees off and not found again. The replays hold the same way and come back in 0.2
	 * s.
	 */
	os_mutex_lock(&controller->pose_hold.mutex);
	struct xrt_quat imu_orientation;
	if (sample->timestamp_ns >= controller->pose_hold.solve_ns &&
	    touch_imu_orientation_at_locked(controller, sample->timestamp_ns, &imu_orientation)) {
		touch_update_hold_locked(controller, &sample->world_pose, &imu_orientation);
		controller->pose_hold.solve_ns = sample->timestamp_ns;
	}
	os_mutex_unlock(&controller->pose_hold.mutex);
}

static struct rift_touch_controller *
rift_touch_controller_create(struct rift_hmd *hmd,
                             enum rift_radio_device_type device_type,
                             struct xrt_frame_context *xfctx);

int
rift_radio_create_touch_controllers(struct rift_hmd *hmd)
{
	const enum rift_radio_device_type types[] = {RIFT_RADIO_DEVICE_LEFT_TOUCH, RIFT_RADIO_DEVICE_RIGHT_TOUCH};

	for (size_t i = 0; i < ARRAY_SIZE(types); i++) {
		const size_t touch_index = rift_radio_device_type_to_touch_index(types[i]);

		struct rift_touch_controller *controller = rift_touch_controller_create(hmd, types[i], hmd->xfctx);
		if (controller == NULL) {
			HMD_ERROR(hmd, "Failed to create touch controller for device type %d", types[i]);
			return -1;
		}

		os_mutex_lock(&hmd->device_mutex);
		hmd->radio_state.touch_controllers[touch_index] = controller;
		hmd->devices[hmd->device_count++] = &controller->base;
		os_mutex_unlock(&hmd->device_mutex);
	}

	return 0;
}

void
rift_touch_controller_add_to_constellation(struct rift_hmd *hmd, struct rift_touch_controller *controller)
{
	struct t_constellation_tracker *tracker = hmd->constellation_tracker;
	if (tracker == NULL || controller->device_type == RIFT_RADIO_DEVICE_TRACKED_OBJECT ||
	    !controller->input.calibration_read) {
		return;
	}

	/*
	 * Held across adding, which waits for the tracker's device lock. Safe: the tracker's threads never take this
	 * mutex while holding that lock, pushing samples to the controller doesn't lock it.
	 */
	os_mutex_lock(&controller->constellation_mutex);

	if (controller->use_constellation) {
		os_mutex_unlock(&controller->constellation_mutex);
		return;
	}

	struct t_constellation_tracker_device_params params = {
	    .led_model = controller->input.calibration.led_model,
	    .tracking_source = &controller->constellation_tracking_source,
	};

	int ret = t_constellation_tracker_add_device(tracker, &params, &controller->constellation_device,
	                                             &controller->constellation_device_id);
	if (ret < 0) {
		os_mutex_unlock(&controller->constellation_mutex);
		HMD_ERROR(hmd, "Failed to add Touch controller %d to the constellation tracker, reason %d",
		          controller->device_type, ret);
		return;
	}

	controller->constellation_tracker = tracker;
	controller->base.tracking_origin = t_constellation_tracker_get_tracking_origin(tracker);
	controller->constellation_imu_sink = params.imu_sink;
	controller->use_constellation = true;
	xrt_atomic_s32_store(&controller->constellation_ready, 1);

	os_mutex_unlock(&controller->constellation_mutex);

	HMD_INFO(hmd, "Added Touch controller %d to the constellation tracker.", controller->device_type);
}

static struct rift_touch_controller *
rift_touch_controller_create(struct rift_hmd *hmd,
                             enum rift_radio_device_type device_type,
                             struct xrt_frame_context *xfctx)
{
	int result;

	struct rift_touch_controller *controller = U_DEVICE_ALLOCATE(
	    struct rift_touch_controller, U_DEVICE_ALLOC_NO_FLAGS, RIFT_TOUCH_CONTROLLER_INPUT_COUNT, 1);
	if (controller == NULL) {
		HMD_ERROR(hmd, "Failed to allocate touch controller");
		return NULL;
	}

	controller->node.break_apart = rift_touch_controller_node_break_apart;
	controller->node.destroy = rift_touch_controller_node_destroy;

	controller->device_type = device_type;
	controller->hmd = hmd;

	controller->base.tracking_origin = hmd->base.tracking_origin;

	snprintf(controller->base.str, XRT_DEVICE_NAME_LEN, "Oculus Touch (Unknown)");
	switch (device_type) {
	case RIFT_RADIO_DEVICE_LEFT_TOUCH:
		snprintf(controller->base.str, XRT_DEVICE_NAME_LEN, "Oculus Touch (Left)");
		controller->base.device_type = XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER;
		break;
	case RIFT_RADIO_DEVICE_RIGHT_TOUCH:
		snprintf(controller->base.str, XRT_DEVICE_NAME_LEN, "Oculus Touch (Right)");
		controller->base.device_type = XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
		break;
	case RIFT_RADIO_DEVICE_TRACKED_OBJECT:
		snprintf(controller->base.str, XRT_DEVICE_NAME_LEN, "Oculus Touch (Tracked Object)");
		controller->base.device_type = XRT_DEVICE_TYPE_ANY_HAND_CONTROLLER;
		break;
	default: break; return controller;
	}
	controller->base.name = XRT_DEVICE_TOUCH_CONTROLLER_RIFT_CV1;

	controller->use_constellation = false;
	controller->constellation_tracking_source.get_tracked_pose =
	    touch_controller_constellation_tracking_source_get_tracked_pose;
	controller->constellation_device.push_constellation_tracker_sample =
	    touch_controller_constellation_device_push_constellation_tracker_sample;

#define SET_INPUT(NAME, ACTIVE)                                                                                        \
	do {                                                                                                           \
		if (ACTIVE) {                                                                                          \
			controller->base.inputs[RIFT_TOUCH_CONTROLLER_INPUT_##NAME].name = XRT_INPUT_TOUCH_##NAME;     \
			controller->base.inputs[RIFT_TOUCH_CONTROLLER_INPUT_##NAME].active = ACTIVE;                   \
		}                                                                                                      \
	} while (0)
	SET_INPUT(X_CLICK, device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH);
	SET_INPUT(X_TOUCH, device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH);
	SET_INPUT(Y_CLICK, device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH);
	SET_INPUT(Y_TOUCH, device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH);
	SET_INPUT(MENU_CLICK, device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH);

	SET_INPUT(A_CLICK, device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH);
	SET_INPUT(A_TOUCH, device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH);
	SET_INPUT(B_CLICK, device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH);
	SET_INPUT(B_TOUCH, device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH);
	SET_INPUT(SYSTEM_CLICK, device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH);

	SET_INPUT(SQUEEZE_VALUE, true);
	SET_INPUT(TRIGGER_TOUCH, true);
	SET_INPUT(TRIGGER_VALUE, true);
	SET_INPUT(THUMBSTICK_CLICK, true);
	SET_INPUT(THUMBSTICK_TOUCH, true);
	SET_INPUT(THUMBSTICK, true);
	SET_INPUT(THUMBREST_TOUCH, true);
	SET_INPUT(GRIP_POSE, true);
	SET_INPUT(AIM_POSE, true);
	SET_INPUT(TRIGGER_PROXIMITY, true);
	SET_INPUT(THUMB_PROXIMITY, true);
#undef SET_INPUT

	// Finger poses for the left and the right hand's controller, see rift_touch_controller_get_hand_tracking.
	const bool hand_controller =
	    device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH || device_type == RIFT_RADIO_DEVICE_RIGHT_TOUCH;
	if (hand_controller) {
		controller->base.inputs[RIFT_TOUCH_CONTROLLER_INPUT_HAND_TRACKING].name =
		    device_type == RIFT_RADIO_DEVICE_LEFT_TOUCH ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT;
		controller->base.inputs[RIFT_TOUCH_CONTROLLER_INPUT_HAND_TRACKING].active = true;
	}

	controller->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;

	u_device_populate_function_pointers(&controller->base, rift_touch_controller_get_tracked_pose,
	                                    rift_touch_controller_destroy);
	controller->base.update_inputs = rift_touch_controller_update_inputs;
	controller->base.set_output = rift_touch_controller_set_output;
	controller->base.get_battery_status = rift_touch_controller_get_battery_status;

	controller->base.supported.battery_status = true;
	controller->base.supported.orientation_tracking = true;
	if (hand_controller) {
		controller->base.get_hand_tracking = rift_touch_controller_get_hand_tracking;
		controller->base.supported.hand_tracking = true;
	}

	controller->base.binding_profile_count = touch_profile_bindings_count;
	controller->base.binding_profiles = touch_profile_bindings;

	result = os_mutex_init(&controller->input.mutex);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to init touch controller input mutex");
		rift_touch_controller_node_destroy(&controller->node);
		return NULL;
	}
	controller->input.mutex_created = true;

	result = os_mutex_init(&controller->constellation_mutex);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to init touch controller constellation mutex");
		rift_touch_controller_node_destroy(&controller->node);
		return NULL;
	}
	controller->constellation_mutex_created = true;

	result = os_mutex_init(&controller->pose_hold.mutex);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to init touch controller pose hold mutex");
		rift_touch_controller_node_destroy(&controller->node);
		return NULL;
	}
	controller->pose_hold.mutex_created = true;

	controller->input.clock_tracker = m_clock_windowed_skew_tracker_alloc(64);
	if (controller->input.clock_tracker == NULL) {
		HMD_ERROR(hmd, "Failed to allocate touch controller clock tracker");
		rift_touch_controller_node_destroy(&controller->node);
		return NULL;
	}

	m_gyro_rest_bias_init(&controller->input.gyro_rest_bias);

	controller->input.imu_fusion = imu_fusion_create();
	if (controller->input.imu_fusion == NULL) {
		HMD_ERROR(hmd, "Failed to create touch controller IMU fusion");
		rift_touch_controller_node_destroy(&controller->node);
		return NULL;
	}

	// -1 means unknown battery level/no data
	xrt_atomic_s32_store(&controller->input.battery_status, -1);

	u_var_add_root(controller, "Rift Touch Controller", true);
	u_var_add_ro_i64_ns(controller, &controller->input.device_remote_ns, "Device Remote Timestamp");
	u_var_add_ro_i64_ns(controller, &controller->input.device_local_ns, "Device Local Timestamp");
	u_var_add_bool(controller, &controller->input.calibration_read, "Read Calibration");

	{
		u_var_add_gui_header(controller, NULL, "Input State");
		u_var_add_ro_vec3_f64(controller, &controller->input.last_imu_sample.accel_m_s2, "Last Accel (m/s²)");
		u_var_add_ro_vec3_f64(controller, &controller->input.last_imu_sample.gyro_rad_secs,
		                      "Last Gyro (rad/s)");
		u_var_add_ro_vec3_f32(controller, &controller->input.gyro_rest_bias.bias, "Gyro bias at rest (rad/s)");
		u_var_add_ro_i32(controller, (int32_t *)&controller->input.battery_status, "battery_status");
		u_var_add_u8(controller, &controller->input.state.buttons, "buttons");
		u_var_add_f32(controller, &controller->input.state.trigger, "trigger");
		u_var_add_f32(controller, &controller->input.state.grip, "grip");
		u_var_add_f32(controller, &controller->input.state.stick.x, "stick.x");
		u_var_add_f32(controller, &controller->input.state.stick.y, "stick.y");
		u_var_add_u8(controller, &controller->input.state.haptic_counter, "haptic_counter");
		u_var_add_f32(controller, &controller->input.state.cap_stick, "cap_stick");
		u_var_add_f32(controller, &controller->input.state.cap_b_y, "cap_b_y");
		u_var_add_f32(controller, &controller->input.state.cap_a_x, "cap_a_x");
		u_var_add_f32(controller, &controller->input.state.cap_trigger, "cap_trigger");
		u_var_add_f32(controller, &controller->input.state.cap_thumbrest, "cap_thumbrest");
	}

	xrt_frame_context_add(xfctx, &controller->node);

	return controller;
}

static int
rift_radio_read_device_serial_async_locked(struct rift_hmd *hmd,
                                           enum rift_radio_device_type device_type,
                                           char serial[SERIAL_NUMBER_LENGTH + 1],
                                           bool *serial_valid)
{
	// Radio is already busy
	if (hmd->radio_state.current_command != RIFT_RADIO_COMMAND_NONE) {
		return -EBUSY;
	}

	HMD_DEBUG(hmd, "Reading serial for device type %d", device_type);

	int result = rift_send_radio_cmd(
	    hmd, true,
	    &(struct rift_radio_cmd_report){.a = 0x03, .b = RIFT_RADIO_READ_CMD_SERIAL, .c = (uint8_t)device_type});
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to send read serial command");
		return result;
	}

	hmd->radio_state.current_command = RIFT_RADIO_COMMAND_READ_SERIAL;
	hmd->radio_state.command_data.read_serial = (struct rift_radio_command_data_read_serial){
	    .serial = serial,
	    .serial_valid = serial_valid,
	};

	return 0;
}

static int
rift_radio_read_device_flash_async_locked(struct rift_hmd *hmd,
                                          void *user_data,
                                          enum rift_radio_device_type device_type,
                                          uint16_t offset,
                                          uint16_t length,
                                          uint8_t *buffer,
                                          flash_read_callback_t callback)
{
	// Radio is already busy
	if (hmd->radio_state.current_command != RIFT_RADIO_COMMAND_NONE) {
		return -EBUSY;
	}

	HMD_DEBUG(hmd, "Queueing flash read at %d of length %d for device %d", offset, length, device_type);

	int result =
	    rift_radio_send_data_read_cmd(hmd, &(struct rift_radio_data_read_cmd){.offset = offset, .length = length});
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to send data read command to radio, reason %d", result);
		return result;
	}

	result = rift_send_radio_cmd(hmd, true,
	                             &(struct rift_radio_cmd_report){
	                                 .a = 0x03, .b = RIFT_RADIO_READ_CMD_FLASH_CONTROL, .c = (uint8_t)device_type});
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to send radio command report to HMD, reason %d", result);
		return result;
	}

	hmd->radio_state.current_command = RIFT_RADIO_COMMAND_READ_FLASH;
	hmd->radio_state.command_data.read_flash = (struct rift_radio_command_data_read_flash){
	    .user_data = user_data,
	    .address = offset,
	    .length = length,
	    .buffer = buffer,
	    .read_callback = callback,
	};

	return 0;
}

static int
rift_touch_controller_calibration_body_read_callback(void *user_data, uint16_t address, uint16_t length);

static int
rift_touch_controller_calibration_read_body_async_locked(struct rift_touch_controller *controller, uint16_t address)
{
	return rift_radio_read_device_flash_async_locked(
	    controller->hmd, controller, controller->device_type, address + CALIBRATION_BODY_BYTE_OFFSET,
	    MIN(controller->radio_data.calibration_body_json_length - address, CALIBRATION_BODY_BYTE_CHUNK_LENGTH),
	    controller->radio_data.calibration_data_buffer, rift_touch_controller_calibration_body_read_callback);
}

// strlen("touchXX_.cal") + NULL == 13
#define RIFT_TOUCH_CONTROLLER_CALIBRATION_FILENAME_MAX_LEN (XRT_DEVICE_NAME_LEN + 13)

static void
rift_touch_controller_get_calibration_filename(struct rift_touch_controller *controller, char *out_filename)
{
	snprintf(out_filename, RIFT_TOUCH_CONTROLLER_CALIBRATION_FILENAME_MAX_LEN, "touch%d_%s.cal",
	         controller->device_type, controller->base.serial);
}

static int
rift_touch_controller_calibration_body_read_callback(void *user_data, uint16_t address, uint16_t length)
{
	int result;
	struct rift_touch_controller *controller = user_data;

	const uint8_t *chunk_buffer = controller->radio_data.calibration_data_buffer;

	if (address == CALIBRATION_HEADER_BYTE_OFFSET) {
		assert(length == CALIBRATION_HEADER_BYTE_LENGTH);

		if (chunk_buffer[0] != 1 || chunk_buffer[1] != 0) {
			HMD_ERROR(controller->hmd, "Got bad flash header");
			return -1;
		}

		__le16 length_raw;
		memcpy(&length_raw, chunk_buffer + 2, sizeof(length_raw));
		HMD_DEBUG(controller->hmd, "Calibration body length: %d", length_raw);

		uint16_t length = __le16_to_cpu(length_raw);

		controller->radio_data.calibration_body_json = U_TYPED_ARRAY_CALLOC(uint8_t, length + 1);
		if (controller->radio_data.calibration_body_json == NULL) {
			HMD_ERROR(controller->hmd, "Failed to allocate calibration body JSON");
			return -1;
		}
		// null terminate
		controller->radio_data.calibration_body_json[length] = '\0';
		controller->radio_data.calibration_body_json_length = length;

		result = rift_touch_controller_calibration_read_body_async_locked(controller, 0);
		assert(result != -EBUSY); // radio cannot be busy at this point
		if (result < 0) {
			return result;
		}

		return 0;
	}

	uint16_t body_offset = address - CALIBRATION_BODY_BYTE_OFFSET;

	assert(body_offset + length <= controller->radio_data.calibration_body_json_length);
	assert(length <= CALIBRATION_BODY_BYTE_CHUNK_LENGTH);

	memcpy(controller->radio_data.calibration_body_json + body_offset, chunk_buffer, length);

	if (body_offset + length < controller->radio_data.calibration_body_json_length) {
		result = rift_touch_controller_calibration_read_body_async_locked(controller, body_offset + length);
		assert(result != -EBUSY); // radio cannot be busy at this point
		if (result < 0) {
			return result;
		}

		return 0;
	}

	// Finished reading calibration body
	HMD_INFO(controller->hmd, "Finished reading touch controller calibration JSON body for controller %d",
	         controller->device_type);
#if 0
	printf("%s\n", controller->radio_data.calibration_body_json);
#endif

	os_mutex_lock(&controller->input.mutex);
	if (!rift_touch_calibration_parse((const char *)controller->radio_data.calibration_body_json,
	                                  controller->radio_data.calibration_body_json_length,
	                                  &controller->input.calibration)) {
		HMD_ERROR(controller->hmd, "Failed to parse touch controller calibration JSON");
		os_mutex_unlock(&controller->input.mutex);
		return -1;
	}

	controller->input.calibration_read = true;
	os_mutex_unlock(&controller->input.mutex);

	rift_touch_controller_add_to_constellation(controller->hmd, controller);

	char calibration_filename[RIFT_TOUCH_CONTROLLER_CALIBRATION_FILENAME_MAX_LEN];
	rift_touch_controller_get_calibration_filename(controller, calibration_filename);

	FILE *calibration_file = u_file_open_file_in_config_dir_subpath(RIFT_CONFIG_SUBDIR, calibration_filename, "wb");
	if (calibration_file == NULL) {
		HMD_ERROR(controller->hmd, "Failed to open calibration file for writing");
		return 0;
	}

	if (fwrite(controller->radio_data.calibration_hash, 1, CALIBRATION_HASH_BYTE_LENGTH, calibration_file) !=
	    CALIBRATION_HASH_BYTE_LENGTH) {
		HMD_ERROR(controller->hmd, "Failed to write calibration hash to file");
		fclose(calibration_file);
		return 0;
	}

	size_t to_write = controller->radio_data.calibration_body_json_length;
	while (to_write > 0) {
		size_t written = fwrite(controller->radio_data.calibration_body_json +
		                            controller->radio_data.calibration_body_json_length - to_write,
		                        1, to_write, calibration_file);
		if (written == 0) {
			HMD_ERROR(controller->hmd, "Failed to write calibration body to file");
			fclose(calibration_file);
			return 0;
		}
		to_write -= written;
	}
	fclose(calibration_file);

	HMD_DEBUG(controller->hmd, "Wrote calibration data to %s", calibration_filename);

	return 0;
}

static int
rift_touch_controller_calibration_hash_read_callback(void *user_data, uint16_t address, uint16_t length)
{
	assert(address == CALIBRATION_HASH_BYTE_OFFSET);
	if (length != CALIBRATION_HASH_BYTE_LENGTH) {
		return -EINVAL;
	}

	struct rift_touch_controller *controller = user_data;

	assert(controller->hmd->radio_state.current_command == RIFT_RADIO_COMMAND_NONE);

	char calibration_filename[RIFT_TOUCH_CONTROLLER_CALIBRATION_FILENAME_MAX_LEN];
	rift_touch_controller_get_calibration_filename(controller, calibration_filename);

	FILE *calibration_file = u_file_open_file_in_config_dir_subpath(RIFT_CONFIG_SUBDIR, calibration_filename, "r");
	if (calibration_file != NULL) {
		size_t file_size;
		char *calibration_data = u_file_read_content(calibration_file, &file_size);
		fclose(calibration_file);

		if (memcmp(calibration_data, controller->radio_data.calibration_hash, CALIBRATION_HASH_BYTE_LENGTH) ==
		    0) {
			HMD_INFO(controller->hmd,
			         "Calibration hash matches cached file for controller %d, loading from disk",
			         controller->device_type);
			if (rift_touch_calibration_parse(calibration_data + CALIBRATION_HASH_BYTE_LENGTH,
			                                 file_size - CALIBRATION_HASH_BYTE_LENGTH,
			                                 &controller->input.calibration)) {
				controller->input.calibration_read = true;
				free(calibration_data);
				rift_touch_controller_add_to_constellation(controller->hmd, controller);
				return 0;
			} else {
				HMD_ERROR(controller->hmd,
				          "Failed to parse touch controller calibration JSON from disk, reading from "
				          "device flash");
				free(calibration_data);
				// fall through to reading from device flash
			}
		} else {
			HMD_INFO(
			    controller->hmd,
			    "Calibration hash does not match cached file for controller %d, reading from device flash",
			    controller->device_type);
			free(calibration_data);
		}
	}

	int result = rift_radio_read_device_flash_async_locked(
	    controller->hmd, controller, controller->device_type, CALIBRATION_HEADER_BYTE_OFFSET,
	    CALIBRATION_HEADER_BYTE_LENGTH, controller->radio_data.calibration_data_buffer,
	    rift_touch_controller_calibration_body_read_callback);
	assert(result != -EBUSY); // radio cannot be busy at this point
	if (result < 0) {
		return result;
	}
	return 0;
}

static int
rift_radio_read_touch_calibration_hash_async_locked(struct rift_hmd *hmd, struct rift_touch_controller *controller)
{
	int result;

	result = rift_radio_read_device_flash_async_locked(
	    hmd, controller, controller->device_type, CALIBRATION_HASH_BYTE_OFFSET, CALIBRATION_HASH_BYTE_LENGTH,
	    controller->radio_data.calibration_hash, rift_touch_controller_calibration_hash_read_callback);
	if (result < 0) {
		return result;
	}

	return 0;
}

bool
rift_radio_read_device_serial(struct rift_hmd *hmd,
                              enum rift_radio_device_type device_type,
                              char *serial,
                              bool *serial_valid)
{
	// Early out if read.
	if (*serial_valid) {
		return true;
	}

	int result = rift_radio_read_device_serial_async_locked(hmd, device_type, serial, serial_valid);

	if (result == -EBUSY) {
		// try again later
		return false;
	}

	if (result < 0) {
		HMD_ERROR(hmd, "Failed to start reading serial for device type %d, reason %d", device_type, result);
		return false;
	}

	return true;
}

static bool
rift_radio_read_controller_calibration(struct rift_hmd *hmd, struct rift_touch_controller *controller)
{
	// Early out if read.
	if (controller->input.calibration_read) {
		return true;
	}

	int result = rift_radio_read_touch_calibration_hash_async_locked(hmd, controller);

	if (result == -EBUSY) {
		// try again later
		return false;
	}

	if (result < 0) {
		HMD_ERROR(hmd, "Failed to start reading calibration hash for device type %d, reason %d",
		          controller->device_type, result);
		return false;
	}

	return true;
}

static void
rift_touch_controller_handle_radio_input_report(struct rift_hmd *hmd,
                                                struct rift_touch_controller *controller,
                                                struct rift_radio_report_message message,
                                                int64_t receive_ns)
{
	struct rift_touch_controller_calibration *c = &controller->input.calibration;

	uint8_t tgs[5];
	memcpy(tgs, message.touch.touch_grip_stick_state, sizeof tgs);
	uint16_t raw_trigger = tgs[0] | ((tgs[1] & 0x03) << 8);
	uint16_t raw_grip = ((tgs[1] & 0xfc) >> 2) | ((tgs[2] & 0xf) << 6);
	uint16_t raw_stick_x = ((tgs[2] & 0xf0) >> 4) | ((tgs[3] & 0x3f) << 4);
	uint16_t raw_stick_y = ((tgs[3] & 0xc0) >> 6) | ((tgs[4] & 0xff) << 2);

	float trigger = rift_min_mid_max_range_to_float(c->trigger_range, (float)raw_trigger);
	float grip = rift_min_mid_max_range_to_float(c->middle_range, (float)raw_grip);

	struct xrt_vec2 joy;
	if (raw_stick_x >= c->joy_x_dead[0] && raw_stick_x <= c->joy_x_dead[1] && raw_stick_y >= c->joy_y_dead[0] &&
	    raw_stick_y <= c->joy_y_dead[1]) {
		joy.x = 0.0f;
		joy.y = 0.0f;
	} else {
		joy.x =
		    ((float)raw_stick_x - c->joy_x_range[0]) / (c->joy_x_range[1] - c->joy_x_range[0]) * 2.0f - 1.0f;
		joy.y =
		    ((float)raw_stick_y - c->joy_y_range[0]) / (c->joy_y_range[1] - c->joy_y_range[0]) * 2.0f - 1.0f;
	}

	struct xrt_vec3 raw_accel = {
	    MATH_GRAVITY_M_S2 / 2048.0 * (float)message.touch.accel[0],
	    MATH_GRAVITY_M_S2 / 2048.0 * (float)message.touch.accel[1],
	    MATH_GRAVITY_M_S2 / 2048.0 * (float)message.touch.accel[2],
	};

	// Gyro is MPU 6500, configured for 2000°/s.
	// The datasheet has 16.4 LSB/°/s, but I'm using
	// 32768 / 2000 = 16.384 here because that actually
	// yields a 2000°/s full range, then converting to
	// radians for the fusion.
	struct xrt_vec3 raw_gyro = {
	    message.touch.gyro[0] / (16.384 * 180.0) * M_PI,
	    message.touch.gyro[1] / (16.384 * 180.0) * M_PI,
	    message.touch.gyro[2] / (16.384 * 180.0) * M_PI,
	};
	struct xrt_vec3 gyro;
	struct xrt_vec3 accel;

	/*
	 * The factory offsets are the sensor's biases on its own axes: subtract them, then the matrix turns the axes
	 * into the model's and corrects their cross-coupling. Adding the offsets after the matrix, as before, left a
	 * CV1 Touch resting with 0.04-0.06 rad/s of gyro and an accelerometer reading 9.6-10.9 m/s² depending on how it
	 * lay (0.15-0.38 spread over 12 orientations); this way 0.01-0.02 rad/s and 0.06-0.16.
	 */
	struct xrt_vec3 unbiased_gyro = m_vec3_sub(raw_gyro, c->gyro_offset);
	math_matrix_3x3_transform_vec3((struct xrt_matrix_3x3 *)&c->gyro_calibration, &unbiased_gyro, &gyro);

	struct xrt_vec3 unbiased_accel = m_vec3_sub(raw_accel, c->accel_offset);
	math_matrix_3x3_transform_vec3((struct xrt_matrix_3x3 *)&c->accel_calibration, &unbiased_accel, &accel);

	/*
	 * A CV1 Touch left at rest goes idle: it keeps sending its buttons and sticks, about every 0.75 s, with the
	 * timestamp and the IMU all zero. The zeroes are no measurement. Taken as one they were free fall with the
	 * factory offsets added, a timestamp 71.6 minutes away (the 32 bit counter wrapping back to 0), and the clock
	 * tracking then put seconds of samples at the wrong time. Only the inputs are used from them.
	 */
	const bool imu_idle = message.touch.timestamp == 0 &&                               //
	                      message.touch.accel[0] == 0 && message.touch.accel[1] == 0 && //
	                      message.touch.accel[2] == 0 && message.touch.gyro[0] == 0 &&  //
	                      message.touch.gyro[1] == 0 && message.touch.gyro[2] == 0;     //

	controller->radio_data.last_report_ns = receive_ns;

	if (imu_idle != controller->input.imu_idle) {
		controller->input.imu_idle = imu_idle;
		HMD_INFO(hmd, "Touch controller %d %s.", controller->device_type,
		         imu_idle ? "is at rest, its IMU went idle" : "moved, its IMU is back");
	}

	os_mutex_lock(&controller->input.mutex);

	if (!imu_idle) {
		// Unsigned, so this is right across the 32 bit counter wrapping every 71.6 minutes.
		uint32_t delta_us = message.touch.timestamp - controller->input.last_device_remote_us;
		controller->input.last_device_remote_us = message.touch.timestamp;

		/*
		 * Reports come every couple of milliseconds. A jump of more than a second means the controller was idle
		 * or its clock restarted, which it does when it wakes up or reconnects. Start the clock tracking over.
		 */
		const bool clock_restarted = controller->input.device_remote_ns != 0 && delta_us > 1000 * 1000;
		if (clock_restarted) {
			delta_us = 2000;
			m_clock_windowed_skew_tracker_reset(controller->input.clock_tracker);
		}
		controller->input.device_remote_ns += (timepoint_ns)delta_us * U_TIME_1US_IN_NS;

		m_clock_windowed_skew_tracker_push(       //
		    controller->input.clock_tracker,      //
		    receive_ns - RIFT_RADIO_LATENCY_BIAS, //
		    controller->input.device_remote_ns);  //

		if (!m_clock_windowed_skew_tracker_to_local(controller->input.clock_tracker,
		                                            controller->input.device_remote_ns,
		                                            &controller->input.device_local_ns)) {
			HMD_WARN(hmd, "Failed to convert device remote time to local time");
			os_mutex_unlock(&controller->input.mutex);
			return;
		}
	}

	controller->input.state.buttons = message.touch.buttons & 0x0F;
	controller->input.state.trigger = trigger;
	controller->input.state.grip = grip;
	controller->input.state.stick = joy;

	switch (message.touch.adc_channel) {
	case RIFT_TOUCH_CONTROLLER_ADC_STICK:
		controller->input.state.cap_stick = rift_min_mid_max_cap(c, 0, (float)message.touch.adc_value);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_A_X:
		controller->input.state.cap_a_x = rift_min_mid_max_cap(c, 3, (float)message.touch.adc_value);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_B_Y:
		controller->input.state.cap_b_y = rift_min_mid_max_cap(c, 1, (float)message.touch.adc_value);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_TRIGGER:
		controller->input.state.cap_trigger = rift_min_mid_max_cap(c, 2, (float)message.touch.adc_value);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_THUMBREST:
		controller->input.state.cap_thumbrest = rift_min_mid_max_cap(c, 7, (float)message.touch.adc_value);
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_HAPTIC_COUNTER:
		controller->input.state.haptic_counter = (uint8_t)message.touch.adc_value;
		break;
	case RIFT_TOUCH_CONTROLLER_ADC_BATTERY:
		xrt_atomic_s32_store(&controller->input.battery_status, message.touch.adc_value);
		break;
	}

	if (imu_idle) {
		os_mutex_unlock(&controller->input.mutex);
		return;
	}

	/*
	 * The factory offsets leave a bias that turns the controller by up to a degree or more a second, the left of
	 * the tested pair 0.021 rad/s against Oculus' own estimate of 0.022. The fusion learns that
	 * only over many seconds, and the held orientation between sightings not at all. Measured whenever the
	 * controller lies still and taken out here, for both.
	 */
	if (!controller->input.gyro_bias_loaded) {
		controller->input.gyro_bias_loaded = true;
		rift_touch_gyro_bias_load(hmd, controller);
	}
	if (m_gyro_rest_bias_push(&controller->input.gyro_rest_bias, controller->input.device_local_ns, &gyro,
	                          &accel)) {
		const struct xrt_vec3 b = controller->input.gyro_rest_bias.bias;
		if (controller->input.gyro_rest_bias.still_windows == 1) {
			HMD_INFO(hmd, "Touch controller %s lies still, its gyro's bias is %.4f %.4f %.4f rad/s.",
			         controller->base.serial, b.x, b.y, b.z);
		}
		if (rift_touch_gyro_bias_changed(controller) &&
		    (controller->input.gyro_bias_saved_ns == 0 ||
		     receive_ns - controller->input.gyro_bias_saved_ns > RIFT_TOUCH_GYRO_BIAS_SAVE_NS)) {
			rift_touch_gyro_bias_save(hmd, controller, receive_ns);
		}
	}
	m_gyro_rest_bias_apply(&controller->input.gyro_rest_bias, &gyro, &gyro);

	struct xrt_imu_sample imu_sample = {
	    .timestamp_ns = controller->input.device_local_ns,
	    .gyro_rad_secs = {gyro.x, gyro.y, gyro.z},
	    .accel_m_s2 = {accel.x, accel.y, accel.z},
	};
	controller->input.last_imu_sample = imu_sample;

	os_mutex_lock(&controller->constellation_mutex);
	if (controller->constellation_imu_sink) {
		xrt_sink_push_imu(controller->constellation_imu_sink, &imu_sample);
	}
	os_mutex_unlock(&controller->constellation_mutex);

	struct xrt_vec3 accel_variance = {0.01, 0.01, 0.01};
	struct xrt_vec3 gyro_variance = {0.01, 0.01, 0.01};
	imu_fusion_incorporate_gyros_and_accelerometer(controller->input.imu_fusion, imu_sample.timestamp_ns, &gyro,
	                                               &gyro_variance, &accel, &accel_variance, NULL);

	// For the tracker's held prior, which can't take input.mutex, see touch_imu_orientation_at_locked.
	struct xrt_quat imu_orientation;
	struct xrt_vec3 imu_angular_velocity;
	if (imu_fusion_get_prediction(controller->input.imu_fusion, (uint64_t)imu_sample.timestamp_ns, &imu_orientation,
	                              &imu_angular_velocity) == 0) {
		os_mutex_lock(&controller->pose_hold.mutex);
		const uint32_t idx = controller->pose_hold.imu_history_count % RIFT_TOUCH_IMU_HISTORY;
		controller->pose_hold.imu_history[idx].timestamp_ns = imu_sample.timestamp_ns;
		controller->pose_hold.imu_history[idx].orientation = imu_orientation;
		controller->pose_hold.imu_history_count++;
		os_mutex_unlock(&controller->pose_hold.mutex);
	}

	os_mutex_unlock(&controller->input.mutex);
}

int
rift_radio_handle_read(struct rift_hmd *hmd)
{
	// int result;
	uint8_t buf[REPORT_MAX_SIZE];
	int length;

	// 1ms so the radio thread is ticking at 1khz for haptics.
	length = os_hid_read(hmd->radio_dev, buf, sizeof(buf), 1);

	timepoint_ns receive_ns = os_monotonic_get_ns() - RIFT_USB_LATENCY_BIAS;

	if (length < 0) {
		HMD_ERROR(hmd, "Got error reading from radio device, assuming fatal, reason %d", length);
		return length;
	}

	// non fatal, but nothing to do
	if (length == 0) {
		// HMD_TRACE(hmd, "Timed out waiting for packet from radio, packets should come in at 500hz");
		return 0;
	}

	// do nothing with the keepalive
	if (buf[0] == IN_REPORT_CV1_RADIO_KEEPALIVE) {
		HMD_TRACE(hmd, "Got radio keepalive(?)");
		return 0;
	}

	if (buf[0] != IN_REPORT_RADIO_DATA) {
		HMD_WARN(hmd, "Got radio IN report with bad ID (got %d, expected %d, size %d), ignoring...", buf[0],
		         IN_REPORT_RADIO_DATA, length);
		return 0;
	}

	if (length != IN_REPORT_RADIO_DATA_SIZE) {
		HMD_WARN(hmd, "Got radio IN report with bad size (got %d, expected %d), ignoring...", length,
		         IN_REPORT_RADIO_DATA_SIZE);
		return 0;
	}

	struct rift_radio_report radio_report;
	memcpy(&radio_report, buf + 1, sizeof(radio_report));

	for (uint32_t i = 0; i < ARRAY_SIZE(radio_report.messages); i++) {
		const struct rift_radio_report_message message = radio_report.messages[i];

		if (message.flags != 0x1c && message.flags != 0x05) {
			// HMD_TRACE(hmd, "Got message with unknown radio flags %04x. skipping", message.flags);

			continue;
		}

		switch (message.device_type) {
		case RIFT_RADIO_DEVICE_LEFT_TOUCH:
		case RIFT_RADIO_DEVICE_RIGHT_TOUCH:
		case RIFT_RADIO_DEVICE_TRACKED_OBJECT: {
			size_t touch_index = rift_radio_device_type_to_touch_index(message.device_type);

			struct rift_touch_controller *controller = hmd->radio_state.touch_controllers[touch_index];

			if (controller == NULL) {
				controller = hmd->radio_state.touch_controllers[touch_index] =
				    rift_touch_controller_create(hmd, message.device_type, hmd->xfctx);

				if (controller == NULL) {
					HMD_ERROR(hmd, "Failed to create touch controller for device type %d",
					          message.device_type);
					continue;
				}

				HMD_INFO(hmd, "Created touch controller for device type %d", message.device_type);

				os_mutex_lock(&hmd->device_mutex);
				hmd->devices[hmd->device_count++] = &controller->base;
				os_mutex_unlock(&hmd->device_mutex);
			}

			if (!rift_radio_read_device_serial(hmd, message.device_type, controller->base.serial,
			                                   &controller->radio_data.serial_valid)) {
				// still waiting for serial
				break;
			}

			if (!rift_radio_read_controller_calibration(hmd, controller)) {
				// still waiting for calibration
				break;
			}

			rift_touch_controller_handle_radio_input_report(hmd, controller, message, receive_ns);

			break;
		}
		case RIFT_RADIO_DEVICE_REMOTE: {
			struct rift_remote *remote = hmd->radio_state.remote;

			if (remote == NULL) {
				remote = hmd->radio_state.remote = rift_remote_create(hmd, hmd->xfctx);

				if (remote == NULL) {
					HMD_ERROR(hmd, "Failed to create remote");
					continue;
				}

				HMD_INFO(hmd, "Created remote");

				os_mutex_lock(&hmd->device_mutex);
				hmd->devices[hmd->device_count++] = &remote->base;
				os_mutex_unlock(&hmd->device_mutex);
			}

			if (!rift_radio_read_device_serial(hmd, message.device_type, remote->base.serial,
			                                   &remote->serial_valid)) {
				// still waiting for serial
				break;
			}

			xrt_atomic_s32_store(&remote->buttons, message.remote.buttons);

			break;
		}
		}
	}

	return 0;
}

static int
rift_radio_set_touch_controller_haptics_sync(struct rift_hmd *hmd,
                                             enum rift_radio_device_type device_type,
                                             bool high_freq,
                                             uint8_t amplitude)
{
	int result;
	unsigned char buf[30] = {0};

	// radio is already busy!
	if (hmd->radio_state.current_command != RIFT_RADIO_COMMAND_NONE) {
		return -EBUSY;
	}

	if (amplitude != 0) {
		buf[high_freq ? 3 /* 320hz */ : 2 /* 160hz */] = 0xa0;

		buf[6] = amplitude;
	}

	result = rift_send_radio_data(
	    hmd, &(struct rift_radio_cmd_report){.a = 0x02, .b = 0x03, .c = (uint8_t)device_type}, buf, sizeof(buf));

	if (result < 0) {
		return result;
	}

	result = rift_get_radio_cmd_response(hmd, true, true);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to send haptics command to device %d, reason %d", device_type, result);
		return result;
	}

	return 0;
}

//! A controller silent this long is set up again when it turns up, it went to sleep or lost the radio.
#define RIFT_TOUCH_GONE_NS (5LL * U_TIME_1S_IN_NS)

static int
rift_radio_update_touch_haptics(struct rift_hmd *hmd, enum rift_radio_device_type device_type, timepoint_ns now)
{
	int result = 0;
	size_t touch_index = rift_radio_device_type_to_touch_index(device_type);

	struct rift_touch_controller *controller = hmd->radio_state.touch_controllers[touch_index];

	if (controller == NULL) {
		return 0;
	}

	/*
	 * Only to a controller that is talking to the radio: one that is asleep or out of range does not answer, and
	 * each command waits for the radio's timeout (in a recording the right controller never woke after a power
	 * cut, and the recenter buzz to it stopped the radio thread).
	 */
	const timepoint_ns last_ns = controller->radio_data.last_report_ns;

	/*
	 * Logged as reports stop and start again (the left Touch once disappeared in SteamVR Home "as if a
	 * button were held"). An idle controller still reports about every 0.75 s.
	 */
	const bool silent = last_ns != 0 && now - last_ns > 1500 * U_TIME_1MS_IN_NS;
	if (silent != controller->radio_data.silent) {
		controller->radio_data.silent = silent;
		HMD_WARN(hmd, "Touch controller %d %s.", controller->device_type,
		         silent ? "stopped reporting over the radio" : "reports over the radio again");
	}

	if (last_ns == 0 || now - last_ns > RIFT_TOUCH_GONE_NS) {
		controller->input.haptic.set_enabled = false;
		controller->input.haptic.set_amplitude = -1.0f;
		return 0;
	}

	/*
	 * Copy the request under the lock set_output writes it under, then send without holding it: applications call
	 * set_output from their frame loop, and sending waits on the radio. Only this thread touches the set_ fields.
	 */
	os_mutex_lock(&controller->input.mutex);
	const timepoint_ns end_time_ns = controller->input.haptic.end_time_ns;
	const float amplitude = controller->input.haptic.amplitude;
	const bool high_freq = controller->input.haptic.high_freq;
	os_mutex_unlock(&controller->input.mutex);

	// Haptic duration has elapsed, turn it off
	if (now > end_time_ns) {
		if (controller->input.haptic.set_enabled) {
			result = rift_radio_set_touch_controller_haptics_sync(hmd, device_type, false, 0);
			// Dropped when the controller does not answer, as it turns its motor off by itself.
			if (result != -EBUSY) {
				controller->input.haptic.set_enabled = false;
				controller->input.haptic.set_amplitude = -1.0f;
			}

			return result;
		}

		// no haptic needed
		return 0;
	}

	if (amplitude != controller->input.haptic.set_amplitude ||
	    high_freq != controller->input.haptic.set_high_freq) {
		result = rift_radio_set_touch_controller_haptics_sync(hmd, device_type, high_freq,
		                                                      CLAMP(amplitude * 255.0f, 0, 255));

		// Not sent again when the controller does not answer: each try waits for the radio's timeout.
		if (result != -EBUSY) {
			controller->input.haptic.set_enabled = true;
			controller->input.haptic.set_amplitude = amplitude;
			controller->input.haptic.set_high_freq = high_freq;
		}

		return result;
	}

	return 0;
}

int
rift_radio_handle_haptics(struct rift_hmd *hmd)
{
	int result;

	timepoint_ns now = os_monotonic_get_ns();

	static const enum rift_radio_device_type to_check[] = {
	    RIFT_RADIO_DEVICE_LEFT_TOUCH,
	    RIFT_RADIO_DEVICE_RIGHT_TOUCH,
	    RIFT_RADIO_DEVICE_TRACKED_OBJECT,
	};
	for (size_t i = 0; i < ARRAY_SIZE(to_check); i++) {
		result = rift_radio_update_touch_haptics(hmd, to_check[i], now);

		// radio busy, try again later
		if (result == -EBUSY) {
			return 0;
		}

		// A controller that does not answer loses its haptics, not the radio: the radio thread stops on errors.
		if (result < 0) {
			HMD_WARN(hmd, "Failed to update haptics for device type %d, reason %d, dropped", to_check[i],
			         result);
		}
	}

	return 0;
}

/*
 *
 * Radio and Touch setup
 *
 */

//! The Touch controllers' sync period, the sensors' frame period: 19200 us.
#define RIFT_RADIO_SYNC_PERIOD_US 19200
//! How often the Oculus runtime re-sends a Touch controller's active timeout.
#define RIFT_TOUCH_KEEPALIVE_NS (30LL * U_TIME_1S_IN_NS)

static void
rift_radio_queue_config(struct rift_hmd *hmd, uint8_t a, uint8_t b, uint8_t c, const uint8_t *data, uint8_t length)
{
	if (hmd->radio_state.config_queue_count >= RIFT_RADIO_CONFIG_QUEUE_LEN) {
		HMD_WARN(hmd, "Radio setup queue full, dropping %02x %02x %02x", a, b, c);
		return;
	}

	size_t idx =
	    (hmd->radio_state.config_queue_head + hmd->radio_state.config_queue_count) % RIFT_RADIO_CONFIG_QUEUE_LEN;
	struct rift_radio_config_write *w = &hmd->radio_state.config_queue[idx];
	*w = (struct rift_radio_config_write){.a = a, .b = b, .c = c, .length = MIN(length, sizeof(w->data))};
	memcpy(w->data, data, w->length);
	hmd->radio_state.config_queue_count++;
}

/*!
 * The Oculus runtime (1.208.0, seen in a USB capture) sets up the radio and each Touch controller as it turns up,
 * which Monado did not:
 *  - radio: 04 12 05 = 08 (meaning unknown), and 04 02 05 = 19200, the sync period in microseconds, the same as the
 *    sensors' frame period and the HMD's tracking report;
 *  - each controller: 02 17 = 02 (into active mode, "FRSetPowerState" in its log), 02 18 = 60 (active timeout, s),
 *    02 19 = 10080 (standby timeout, min), 02 04 = 0x23 and 02 13 = 00 (meaning unknown); it then re-sends the active
 *    timeout every 30 s.
 * Without them a controller at rest dropped into the idle reports (IMU all zero), and its LEDs may run on another
 * period than the sensors' exposures.
 */
int
rift_radio_handle_config(struct rift_hmd *hmd)
{
	const timepoint_ns now = os_monotonic_get_ns();

	if (!hmd->radio_state.radio_configured) {
		const uint8_t unknown_12[] = {0x08};
		const uint8_t period[] = {RIFT_RADIO_SYNC_PERIOD_US & 0xff, RIFT_RADIO_SYNC_PERIOD_US >> 8};
		rift_radio_queue_config(hmd, 0x04, 0x12, 0x05, unknown_12, sizeof(unknown_12));
		rift_radio_queue_config(hmd, 0x04, 0x02, 0x05, period, sizeof(period));
		hmd->radio_state.radio_configured = true;
		HMD_INFO(hmd, "Setting up the radio: sync period %d us.", RIFT_RADIO_SYNC_PERIOD_US);
	}

	for (size_t i = 0; i < RADIO_STATE_TOUCH_CONTROLLERS_LEN; i++) {
		struct rift_touch_controller *controller = hmd->radio_state.touch_controllers[i];
		if (controller == NULL || controller->device_type == RIFT_RADIO_DEVICE_TRACKED_OBJECT) {
			continue;
		}

		const timepoint_ns last_ns = controller->radio_data.last_report_ns;
		if (last_ns == 0 || now - last_ns > RIFT_TOUCH_GONE_NS) {
			controller->radio_data.configured = false;
			continue;
		}

		// Only once its serial is known, as the Oculus runtime does after detecting it.
		if (!controller->radio_data.serial_valid) {
			continue;
		}

		const uint8_t type = (uint8_t)controller->device_type;
		const uint8_t active_timeout_s[] = {60};
		if (!controller->radio_data.configured) {
			const uint8_t power_active[] = {0x02};
			const uint8_t standby_timeout_min[] = {10080 & 0xff, 10080 >> 8};
			const uint8_t unknown_04[] = {0x23};
			const uint8_t unknown_13[] = {0x00};
			rift_radio_queue_config(hmd, 0x02, 0x17, type, power_active, sizeof(power_active));
			rift_radio_queue_config(hmd, 0x02, 0x18, type, active_timeout_s, sizeof(active_timeout_s));
			rift_radio_queue_config(hmd, 0x02, 0x19, type, standby_timeout_min,
			                        sizeof(standby_timeout_min));
			rift_radio_queue_config(hmd, 0x02, 0x04, type, unknown_04, sizeof(unknown_04));
			rift_radio_queue_config(hmd, 0x02, 0x13, type, unknown_13, sizeof(unknown_13));
			controller->radio_data.configured = true;
			controller->radio_data.keepalive_ns = now;
			HMD_INFO(hmd, "Setting up Touch controller %d: active mode, timeouts.", type);
		} else if (now - controller->radio_data.keepalive_ns > RIFT_TOUCH_KEEPALIVE_NS) {
			rift_radio_queue_config(hmd, 0x02, 0x18, type, active_timeout_s, sizeof(active_timeout_s));
			controller->radio_data.keepalive_ns = now;
		}
	}

	if (hmd->radio_state.current_command != RIFT_RADIO_COMMAND_NONE || hmd->radio_state.config_queue_count == 0) {
		return 0;
	}

	struct rift_radio_config_write w = hmd->radio_state.config_queue[hmd->radio_state.config_queue_head];
	hmd->radio_state.config_queue_head = (hmd->radio_state.config_queue_head + 1) % RIFT_RADIO_CONFIG_QUEUE_LEN;
	hmd->radio_state.config_queue_count--;

	// Two bytes of command ID, then the setting, as in the data report the haptics are sent with.
	uint8_t buf[30] = {0};
	memcpy(buf + 2, w.data, w.length);
	int result =
	    rift_send_radio_data(hmd, &(struct rift_radio_cmd_report){.a = w.a, .b = w.b, .c = w.c}, buf, sizeof(buf));
	if (result < 0) {
		HMD_WARN(hmd, "Failed to send radio setup %02x %02x %02x, reason %d", w.a, w.b, w.c, result);
		return 0;
	}

	HMD_DEBUG(hmd, "Sent radio setup %02x %02x %02x (%d bytes)", w.a, w.b, w.c, w.length);
	hmd->radio_state.current_command = RIFT_RADIO_COMMAND_CONFIG;
	return 0;
}

int
rift_radio_handle_command(struct rift_hmd *hmd)
{
	// No active command
	if (hmd->radio_state.current_command == RIFT_RADIO_COMMAND_NONE) {
		return 0;
	}

	int result = rift_get_radio_cmd_response(hmd, false, true);

	if (result == -EINPROGRESS) {
		// still waiting for response
		return 0;
	}

	if (result == -ETIMEDOUT) {
		switch (hmd->radio_state.current_command) {

		case RIFT_RADIO_COMMAND_READ_SERIAL:
			break; // @note this can be erroneous since the headset likes to send remote packets even when
			       //       there's no remote sometimes
		default:
			HMD_WARN(hmd, "Timed out waiting for radio command response %d, cancelling request",
			         hmd->radio_state.current_command);
			break;
		}

		hmd->radio_state.current_command = RIFT_RADIO_COMMAND_NONE;

		return 0;
	}

	// Unexpected error
	if (result < 0) {
		HMD_ERROR(hmd, "Unexpected error getting radio command response, reason %d", result);
		return result;
	}

	enum rift_radio_command command = hmd->radio_state.current_command;
	hmd->radio_state.current_command = RIFT_RADIO_COMMAND_NONE;

	HMD_TRACE(hmd, "Successfully received response for radio command %d", command);

	switch (command) {
	case RIFT_RADIO_COMMAND_CONFIG: break;
	case RIFT_RADIO_COMMAND_READ_SERIAL: {
		struct rift_radio_command_data_read_serial *data = &hmd->radio_state.command_data.read_serial;

		uint8_t buf[24]; // size of data response
		result = rift_radio_read_data(hmd, (uint8_t *)&buf, sizeof(buf), false);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to read serial data from radio, reason %d", result);
			break;
		}

		const char *serial_ptr = (const char *)(buf + 5); // first 5 bytes are unknown

		(*data->serial_valid) = true;

		HMD_INFO(hmd, "Read radio serial: %s", serial_ptr);
		memcpy(data->serial, (char *)serial_ptr, SERIAL_NUMBER_LENGTH);
		data->serial[SERIAL_NUMBER_LENGTH] = 0;

		break;
	}
	case RIFT_RADIO_COMMAND_READ_FLASH: {
		struct rift_radio_command_data_read_flash *data = &hmd->radio_state.command_data.read_flash;

		result = rift_radio_read_data(hmd, data->buffer, data->length, true);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to read flash data from radio, reason %d", result);
			break;
		}

		// call user callback
		result = data->read_callback(data->user_data, data->address, data->length);
		if (result < 0) {
			HMD_ERROR(hmd, "Flash read callback returned error %d", result);
			break;
		}

		break;
	}
	default: break;
	}

	return 0;
}
