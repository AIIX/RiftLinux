// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The room kept where it was through a recalibration, and the head settling before the world is placed on the
 *         user again: shared by the tracker, the SteamVR driver and RiftLinux's own runtime.
 *
 * Measuring the sensors again solves their places anew, in a tracking space whose origin is whichever sensor the solve
 * picks: one measurement turned the whole space 21.5 degrees and moved it 1.07 m, with no sensor moved,
 * and the room the user stood in went with it. Each sensor measured both times tells how the new space sits in the
 * old one; the sensors that did not move all tell the same, a moved one something else. The new measurement is turned
 * and moved back by what the sensors that kept their place agree on: the room, the floor and SteamVR's walls stay
 * where they were. A sensor tilted by more than a solve's own spread has moved for sure; when no sensor is left, or
 * those left disagree (two sensors, one slid along the floor), nothing can tell where the room went.
 */

#pragma once

#include "xrt/xrt_defines.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace rl::room {

//! Two sensors' views of the new space this close agree: neither moved. Measured: 5 mm, 0.09 degrees.
constexpr float kAgreeM = 0.03f;
constexpr float kAgreeDeg = 1.0f;
//! Tilted by more than this, a sensor moved: a fresh solve's levelling varies by about 0.8 degrees.
constexpr float kTiltMovedDeg = 2.0f;

//! A sensor measured before and after, in the old and the new tracking space.
struct Sensor
{
	struct xrt_pose before;
	struct xrt_pose after;
};

//! The new space in the old one: a turn about up, then a shift (old = turn(new) + shift).
struct Turn
{
	float yaw_rad = 0.0f;
	struct xrt_vec3 shift = {0.0f, 0.0f, 0.0f};
};

struct Result
{
	//! The room is where it was once the new placements are turned by @ref turn.
	bool kept = false;
	struct Turn turn;
	//! Per sensor: it kept its place (one of those the room is kept by).
	std::vector<bool> anchor;
	int anchors = 0;
	//! For the log.
	std::string how;
};

namespace detail {

inline struct xrt_quat
mul(const struct xrt_quat &a, const struct xrt_quat &b)
{
	return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
	        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

inline struct xrt_quat
conj(const struct xrt_quat &q)
{
	return {-q.x, -q.y, -q.z, q.w};
}

inline struct xrt_vec3
rotate(const struct xrt_quat &q, const struct xrt_vec3 &v)
{
	const struct xrt_quat p = mul(mul(q, {v.x, v.y, v.z, 0.0f}), conj(q));
	return {p.x, p.y, p.z};
}

inline struct xrt_quat
about_up(float yaw_rad)
{
	return {0.0f, std::sin(yaw_rad / 2.0f), 0.0f, std::cos(yaw_rad / 2.0f)};
}

//! The heading of a rotation: where it turns -Z (forward), about +Y.
inline float
yaw_of(const struct xrt_quat &q)
{
	const struct xrt_vec3 f = rotate(q, {0.0f, 0.0f, -1.0f});
	return std::atan2(-f.x, -f.z);
}

inline float
wrap(float rad)
{
	while (rad > (float)M_PI) {
		rad -= 2.0f * (float)M_PI;
	}
	while (rad < -(float)M_PI) {
		rad += 2.0f * (float)M_PI;
	}
	return rad;
}

//! Where the new space's origin sits in the old space for a sensor, given the turn.
inline struct xrt_vec3
shift_for(const Sensor &s, float yaw_rad)
{
	const struct xrt_vec3 r = rotate(about_up(yaw_rad), s.after.position);
	return {s.before.position.x - r.x, s.before.position.y - r.y, s.before.position.z - r.z};
}

inline float
dist(const struct xrt_vec3 &a, const struct xrt_vec3 &b)
{
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

} // namespace detail

//! A placement of the new measurement put into the old space.
inline struct xrt_pose
apply(const Turn &t, const struct xrt_pose &p)
{
	const struct xrt_quat q = detail::about_up(t.yaw_rad);
	const struct xrt_vec3 r = detail::rotate(q, p.position);
	struct xrt_pose out;
	out.orientation = detail::mul(q, p.orientation);
	out.position = {r.x + t.shift.x, r.y + t.shift.y, r.z + t.shift.z};
	return out;
}

//! How the new measurement goes back to where the room was, if the sensors that kept their place can tell.
inline Result
keep(const std::vector<Sensor> &sensors)
{
	Result r;
	r.anchor.assign(sensors.size(), false);
	const size_t n = sensors.size();
	std::vector<float> yaw(n, 0.0f);
	std::vector<bool> level(n, false);
	size_t candidates = 0;
	for (size_t i = 0; i < n; i++) {
		// The new space in the old as this sensor sees it: before * inverse(after).
		const struct xrt_quat c = detail::mul(sensors[i].before.orientation, detail::conj(sensors[i].after.orientation));
		const struct xrt_vec3 up = detail::rotate(c, {0.0f, 1.0f, 0.0f});
		const float tilt_deg = std::acos(std::fmax(-1.0f, std::fmin(1.0f, up.y))) * 180.0f / (float)M_PI;
		yaw[i] = detail::yaw_of(c);
		level[i] = tilt_deg <= kTiltMovedDeg;
		candidates += level[i] ? 1 : 0;
	}

	// The largest group of level sensors that agree on the new space.
	std::vector<size_t> best;
	for (size_t s = 0; s < n; s++) {
		if (!level[s]) {
			continue;
		}
		const struct xrt_vec3 seed = detail::shift_for(sensors[s], yaw[s]);
		std::vector<size_t> group;
		for (size_t j = 0; j < n; j++) {
			if (level[j] && std::fabs(detail::wrap(yaw[j] - yaw[s])) * 180.0f / (float)M_PI <= kAgreeDeg &&
			    detail::dist(detail::shift_for(sensors[j], yaw[s]), seed) <= kAgreeM) {
				group.push_back(j);
			}
		}
		if (group.size() > best.size()) {
			best = group;
		}
	}

	char buf[192];
	if (n == 0) {
		r.how = "no sensor was measured both times: nothing tells where the room was";
		return r;
	}
	if (candidates == 0) {
		r.how = "every sensor was tilted (moved): nothing tells where the room was";
		return r;
	}
	// More than half of the level sensors: two that disagree can't tell which one moved.
	if (best.size() * 2 <= candidates) {
		snprintf(buf, sizeof(buf), "the sensors disagree on where the room was (%zu of %zu agree): one moved",
		         best.size(), candidates);
		r.how = buf;
		return r;
	}

	// Their turn averaged on the circle, then the shift under it.
	float s = 0.0f;
	float c = 0.0f;
	for (size_t j : best) {
		s += std::sin(yaw[j]);
		c += std::cos(yaw[j]);
	}
	r.turn.yaw_rad = std::atan2(s, c);
	struct xrt_vec3 sum = {0.0f, 0.0f, 0.0f};
	for (size_t j : best) {
		const struct xrt_vec3 v = detail::shift_for(sensors[j], r.turn.yaw_rad);
		sum = {sum.x + v.x, sum.y + v.y, sum.z + v.z};
	}
	const float k = 1.0f / (float)best.size();
	r.turn.shift = {sum.x * k, sum.y * k, sum.z * k};
	for (size_t j : best) {
		r.anchor[j] = true;
	}
	r.anchors = (int)best.size();
	r.kept = true;
	snprintf(buf, sizeof(buf), "%d of %zu sensors kept their place: the room is where it was (the new measurement "
	         "was turned %.1f deg and moved %.2f m)", r.anchors, n, r.turn.yaw_rad * 180.0f / (float)M_PI,
	         std::sqrt(r.turn.shift.x * r.turn.shift.x + r.turn.shift.z * r.turn.shift.z));
	r.how = buf;
	return r;
}

/*!
 * The head settled for placing the world on the user again: armed when a measurement moved the room, then worn and
 * tracked with its heading within 8 degrees and its place within 10 cm for a second, from kAfterS on (the sensor
 * fusion moves over to the new measurement in the first moments). Not thread-safe: one caller.
 */
class Settle
{
public:
	static constexpr double kAfterS = 1.0;
	static constexpr double kHoldS = 1.0;

	void
	arm(double t)
	{
		armed_ = true;
		not_before_ = t + kAfterS;
		since_ = -1.0;
	}

	bool
	armed() const
	{
		return armed_;
	}

	//! A worn, tracked head at @p t: true once, when it has settled (and disarms).
	bool
	head(double t, float yaw_rad, const struct xrt_vec3 &position)
	{
		if (!armed_ || t < not_before_) {
			return false;
		}
		if (since_ < 0.0 || std::fabs(detail::wrap(yaw_rad - yaw_)) * 180.0f / (float)M_PI > 8.0f ||
		    detail::dist(position, position_) > 0.10f) {
			since_ = t;
			yaw_ = yaw_rad;
			position_ = position;
			return false;
		}
		if (t - since_ < kHoldS) {
			return false;
		}
		armed_ = false;
		return true;
	}

	//! The head was not worn or tracked: it starts settling again.
	void
	lost()
	{
		since_ = -1.0;
	}

private:
	bool armed_ = false;
	double not_before_ = 0.0;
	double since_ = -1.0;
	float yaw_ = 0.0f;
	struct xrt_vec3 position_ = {0.0f, 0.0f, 0.0f};
};

} // namespace rl::room
