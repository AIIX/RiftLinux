// Copyright 2026, Aditya Mehra
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  What an application is shown of a Touch controller around the sensors losing it: carried on by the sensor
 *         fusion past its last solve while that stays plausible, then held, then slid back in. The driver's, and the
 *         replay tool's, so that what is scored against Oculus is what ships.
 * @ingroup aux_tracking
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "util/u_time.h"

#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

//! See t_touch_app_pose_params::snap_found_m.
#define CT_TOUCH_SNAP_SETTLED_M 0.04f

struct t_touch_app_pose_params
{
	//! How long past its last solve an app's pose may be carried on by the fusion.
	int64_t carry_ns;
	//! Past this, carrying on is also checked against speed and distance.
	int64_t checked_after_ns;
	float max_speed_m_s;
	float max_distance_m;
	//! Past this, only while still this close to the last solve: a long carrying on is for a hand that is near.
	int64_t near_after_ns;
	float near_distance_m;
	/*!
	 * How far from its last solve the app may see a controller at any time: as far as a hand gets, this plus this speed
	 * times the time since the solve. Right after a solve the fusion carried controllers 0.3-1.5 m off within 150 ms
	 * (velocities of up to 10 m/s from poor solves).
	 */
	float reach_m;
	float reach_speed_m_s;
	//! Found again after a hold, the fusion's pose is taken once it is this close to the solve that found it.
	float found_again_m;
	//! A hold starts where the app last saw the controller, unless that is further than this from the last solve.
	float hold_from_shown_m;
	//! A controller found again slides from where it was held to where it is over this long, or longer at this speed.
	int64_t blend_ns;
	float blend_speed_m_s;
	/*!
	 * Found again this far (m) from where it was held or carried to, its slide is not shown as tracked: as the Oculus
	 * runtime, which reports a controller back after a gap orientation-only until its filter is seeded again, then shows
	 * it where it is. 0: every slide shown tracked.
	 */
	float return_untracked_m;
	/*!
	 * Found again this far (m) or more from where it was held or carried to, it is shown where it is at once, without
	 * the slide: as the Oculus runtime, which starts its filter over at the camera's pose after a large update. A slide
	 * of up to 38 cm from where a lost controller was held was what a user saw as it gliding back and forth. 0: slides
	 * (controllers.tracking "0.1.4").
	 */
	float snap_found_m;
};

//! What applications have been shown.
struct t_touch_app_pose
{
	bool was_held;
	struct xrt_pose held_pose;
	int64_t blend_start_ns;
	struct xrt_pose blend_from;
	//! How long this slide takes, and whether it is shown untracked (t_touch_app_pose_params::return_untracked_m).
	int64_t slide_ns;
	bool slide_untracked;
	bool have_tracked;
	struct xrt_vec3 tracked_position;
	struct xrt_vec3 hold_position;
	//! The solve that carrying on stopped from: it does not start again before the next one.
	int64_t carry_over_solve_ns;
	//! Whether the last pose shown was carried on past t_touch_app_pose_params::checked_after_ns, and that pose.
	bool shown_carried;
	bool carrying;
	struct xrt_pose shown_pose;
};

static inline void
t_touch_app_pose_params_default(struct t_touch_app_pose_params *p)
{
	/*
	 * As Oculus: with the LEDs out of the frames, its runtime carried on 860-1030 ms. Against the truth (the sensors made
	 * blind in real play, the same play with sight), in the last 400 ms of that second carried on is 6-42 mm off at the
	 * median and 24-113 mm at the 90th percentile, held still 6-180 mm and 80-672 mm. But a second carried on as far as the
	 * hand might go put a controller 30 cm off in some live-like replays: past 300 ms, as before, only within 15 cm of
	 * where it was last seen. Against what Oculus tracked, day and night, that kept every number at least as it was.
	 */
	p->carry_ns = 1000 * U_TIME_1MS_IN_NS;
	p->near_after_ns = 300 * U_TIME_1MS_IN_NS;
	p->near_distance_m = 0.15f;
	p->checked_after_ns = 150 * U_TIME_1MS_IN_NS;
	p->max_speed_m_s = 3.0f;
	p->max_distance_m = 0.3f;
	p->reach_m = 0.10f;
	p->reach_speed_m_s = 3.0f;
	p->found_again_m = 0.15f;
	p->hold_from_shown_m = 0.55f;
	p->blend_ns = 100 * U_TIME_1MS_IN_NS;
	p->blend_speed_m_s = 4.0f;
	/*
	 * 0.10, the Oculus runtime's own line for snapping to the camera: against what it tracked in the recorded day and
	 * night sessions, the controllers' jumps at night went from 2 and 3 to none (as Oculus), time more than 10 cm off
	 * from 1.44 and 0.10 % to 1.01 and 0.00 at night, 0.26 to 0.16 % for the right by day. Overridden by
	 * RIFT_TOUCH_RETURN_UNTRACKED_M (controllers.tracking "0.1.4" sets 0).
	 */
	const char *untracked = getenv("RIFT_TOUCH_RETURN_UNTRACKED_M");
	p->return_untracked_m = untracked != NULL ? (float)atof(untracked) : 0.10f;
	const char *snap = getenv("RIFT_TOUCH_SNAP_FOUND_M");
	p->snap_found_m = snap != NULL ? (float)atof(snap) : 0.10f;
}

//! Whether @p fused, the fusion's pose at @p at_ns, may be shown as far as time, speed and distance go.
static inline bool
t_touch_app_pose_carried(struct t_touch_app_pose *s,
                         const struct t_touch_app_pose_params *p,
                         int64_t last_solve_ns,
                         int64_t at_ns,
                         const struct xrt_space_relation *fused,
                         bool have_solved,
                         const struct xrt_vec3 *solved)
{
	const int64_t age_ns = at_ns - last_solve_ns;
	if (last_solve_ns == 0 || age_ns > p->carry_ns || s->carry_over_solve_ns == last_solve_ns) {
		return false;
	}
	s->carrying = age_ns > p->checked_after_ns;
	const float reach_m = p->reach_m + p->reach_speed_m_s * (float)age_ns / (float)U_TIME_1S_IN_NS;
	if (have_solved && m_vec3_len(m_vec3_sub(fused->pose.position, *solved)) > reach_m) {
		if (age_ns > p->checked_after_ns) {
			s->carry_over_solve_ns = last_solve_ns;
		}
		return false;
	}
	if (age_ns > p->checked_after_ns) {
		const bool too_fast = (fused->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) != 0 &&
		                      m_vec3_len(fused->linear_velocity) > p->max_speed_m_s;
		const float limit_m =
		    age_ns > p->near_after_ns && p->near_distance_m > 0.0f ? p->near_distance_m : p->max_distance_m;
		const bool too_far = have_solved && m_vec3_len(m_vec3_sub(fused->pose.position, *solved)) > limit_m;
		if (too_fast || too_far) {
			s->carry_over_solve_ns = last_solve_ns;
			return false;
		}
	}
	return true;
}

/*!
 * Takes @p fused as what the app sees, unless it was held and the fusion is still far from the solve that found it: seen
 * again after a hold, the fusion had not taken in the new solve yet, still carried on from the last one, 1.2 and 1.8 m from
 * where the sensors saw the controller. Slides in from where it was held, or from where it was carried to, which a second
 * of the IMU alone leaves some centimetres off.
 *
 * @return false to go on holding.
 */
static inline bool
t_touch_app_pose_show(struct t_touch_app_pose *s,
                      const struct t_touch_app_pose_params *p,
                      int64_t at_ns,
                      bool have_solved,
                      const struct xrt_vec3 *solved,
                      struct xrt_space_relation *fused)
{
	if (s->was_held && have_solved && m_vec3_len(m_vec3_sub(fused->pose.position, *solved)) > p->found_again_m) {
		return false;
	}
	// Shown at once (snap_found_m) only where the fusion has settled on the solve: it moved 11 cm a frame after.
	if (s->was_held && have_solved && p->snap_found_m > 0.0f &&
	    m_vec3_len(m_vec3_sub(fused->pose.position, s->held_pose.position)) >= p->snap_found_m &&
	    m_vec3_len(m_vec3_sub(fused->pose.position, *solved)) > CT_TOUCH_SNAP_SETTLED_M) {
		return false;
	}
	const bool from_held = s->was_held;
	if (from_held || (s->shown_carried && !s->carrying)) {
		s->was_held = false;
		s->blend_start_ns = at_ns;
		s->blend_from = from_held ? s->held_pose : s->shown_pose;
		// A long way back takes longer: half a metre in 100 ms was 5 cm a frame.
		const float far_m = m_vec3_len(m_vec3_sub(fused->pose.position, s->blend_from.position));
		const int64_t at_speed_ns = (int64_t)(far_m / p->blend_speed_m_s * (float)U_TIME_1S_IN_NS);
		s->slide_ns = at_speed_ns > p->blend_ns ? at_speed_ns : p->blend_ns;
		s->slide_untracked = p->return_untracked_m > 0.0f && far_m > p->return_untracked_m;
		// Only after a hold, shown untracked meanwhile, as Oculus: a carried pose (shown tracked) still slides.
		if (p->snap_found_m > 0.0f && from_held && far_m >= p->snap_found_m) {
			s->slide_ns = 0;
		}
	}
	s->shown_carried = s->carrying;
	const int64_t blend_ns = at_ns - s->blend_start_ns;
	if (s->blend_start_ns != 0 && blend_ns >= 0 && blend_ns < s->slide_ns) {
		const float t = (float)blend_ns / (float)s->slide_ns;
		const struct xrt_pose from = s->blend_from;
		fused->pose.position = m_vec3_lerp(from.position, fused->pose.position, t);
		math_quat_slerp(&from.orientation, &fused->pose.orientation, t, &fused->pose.orientation);
		if (s->slide_untracked) {
			fused->relation_flags =
			    (enum xrt_space_relation_flags)(fused->relation_flags & ~XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
		}
	}
	s->have_tracked = true;
	s->tracked_position = fused->pose.position;
	s->shown_pose = fused->pose;
	return true;
}

//! Where a held controller stands: chosen as the hold starts, where the app last saw it if that is near the last solve.
static inline struct xrt_vec3
t_touch_app_pose_hold_position(struct t_touch_app_pose *s,
                               const struct t_touch_app_pose_params *p,
                               const struct xrt_vec3 *solved)
{
	if (!s->was_held) {
		s->hold_position = s->have_tracked && m_vec3_len(m_vec3_sub(s->tracked_position, *solved)) <= p->hold_from_shown_m
		                       ? s->tracked_position
		                       : *solved;
	}
	return s->hold_position;
}

//! The pose the app saw held.
static inline void
t_touch_app_pose_held(struct t_touch_app_pose *s, const struct xrt_pose *pose)
{
	s->was_held = true;
	s->held_pose = *pose;
	s->shown_carried = false;
}

#ifdef __cplusplus
}
#endif
