// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The floor kept through a recalibration, without asking the user to set it again: shared by the SteamVR
 *         driver and RiftLinux's own runtime.
 *
 * The floor is set once ("Set floor") as a height in the tracking space, whose origin is a sensor. Measuring the
 * sensors again moves that origin when a sensor moved up or down. Each sensor's height change between the two
 * measurements (u_notice_recalibration) gives a candidate for the new floor: right for every sensor that did not move
 * up or down. The origin's own sensor always reads "unchanged" (it is the origin both times), so when every sensor
 * moved, by as much or not, no candidate is right and none can tell. The user's standing eye height (learned while
 * they play with the floor known) checks the candidates against the head's heights after the measuring: it picks the
 * right one when they disagree, overrules them when they all miss it, and gives the floor itself.
 */

#pragma once

#include "util/u_notice.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace rl::floor {

//! Candidates within this of each other agree: no sensor moved up or down (or all by as much).
constexpr float kAgreeM = 0.02f;
//! A candidate fits the eye height when the head is then within this of it (both are 90th percentiles of the
//! same standing, so posture mostly cancels).
constexpr float kEyeFitM = 0.04f;

/*!
 * The user's standing eye height above the floor, learned from the head's heights while the floor is known and the
 * headset worn: the 90th percentile over at least kMinSeconds, in 1 cm steps (standing is the most common high).
 */
class EyeHeight
{
public:
	static constexpr float kMinM = 0.80f;
	static constexpr float kMaxM = 2.20f;
	static constexpr double kMinSeconds = 20.0;

	//! A head height above the floor, metres, @p dt seconds after the previous one.
	void
	add(float eye_m, double dt)
	{
		if (!(eye_m >= kMinM && eye_m < kMaxM) || !(dt > 0.0 && dt < 0.5)) {
			return;
		}
		bins[static_cast<size_t>((eye_m - kMinM) * 100.0f)] += dt;
		seconds += dt;
	}

	//! The estimate, once there is enough.
	std::optional<float>
	estimate() const
	{
		if (seconds < kMinSeconds) {
			return std::nullopt;
		}
		double below = 0.0;
		for (size_t i = 0; i < bins.size(); i++) {
			below += bins[i];
			if (below >= 0.9 * seconds) {
				return kMinM + (static_cast<float>(i) + 0.5f) / 100.0f;
			}
		}
		return std::nullopt;
	}

	double seconds = 0.0;

private:
	std::array<double, 140> bins{};
};

//! The floor after a recalibration and what decided it.
struct Result
{
	//! The same kind of number as before (metres added to the tracking space's heights).
	float floor_m;
	//! The sensors' candidates agreed: decided without the head.
	bool sensors_agree;
	//! For the log.
	std::string how;
};

/*!
 * The floor after a recalibration. @p head_m: the head's height in the tracking space now (standing, after the
 * measuring), if known.
 */
inline Result
after_recalibration(float floor_before,
                    const struct u_notice_recalibration &change,
                    std::optional<float> eye_height_m,
                    std::optional<float> head_m)
{
	std::array<float, U_NOTICE_MAX_CAMERAS> candidates{};
	int n = 0;
	for (int i = 0; i < change.count && i < U_NOTICE_MAX_CAMERAS; i++) {
		if (change.known[i]) {
			candidates[n++] = floor_before - change.height_change_m[i];
		}
	}
	const bool head_known = eye_height_m.has_value() && head_m.has_value();
	if (n == 0) {
		if (head_known) {
			return {*eye_height_m - *head_m, false, "no sensor measured both times: from your standing eye height"};
		}
		return {floor_before, false, "no sensor measured both times, eye height not yet learned: kept"};
	}
	const auto [lo, hi] = std::minmax_element(candidates.begin(), candidates.begin() + n);
	if (*hi - *lo <= kAgreeM) {
		float sum = 0.0f;
		for (int i = 0; i < n; i++) {
			sum += candidates[i];
		}
		const float agreed = sum / static_cast<float>(n);
		if (head_known && std::fabs(*head_m + agreed - *eye_height_m) > kEyeFitM) {
			return {*eye_height_m - *head_m, false,
			        "the sensors agree but miss your standing eye height (every sensor moved alike): from it"};
		}
		return {agreed, true, head_known ? "the sensors agree, your standing eye height too" : "the sensors agree"};
	}
	if (!head_known) {
		// Undecided without the eye height: the floor that moves least, so a wrong pick is the smaller error.
		const float pick = std::fabs(*lo - floor_before) <= std::fabs(*hi - floor_before) ? *lo : *hi;
		return {pick, false, "a sensor moved up or down, eye height not yet learned: the smaller change"};
	}
	float best = candidates[0];
	for (int i = 1; i < n; i++) {
		if (std::fabs(*head_m + candidates[i] - *eye_height_m) < std::fabs(*head_m + best - *eye_height_m)) {
			best = candidates[i];
		}
	}
	if (std::fabs(*head_m + best - *eye_height_m) <= kEyeFitM) {
		return {best, false, "a sensor moved up or down: the sensor that fits your standing eye height"};
	}
	return {*eye_height_m - *head_m, false, "both sensors moved: from your standing eye height"};
}

/*!
 * The floor kept through recalibrations, the same in the SteamVR driver and RiftLinux's own runtime: it learns the
 * standing eye height from the head's heights while the floor is known, and when the sensors are measured again
 * decides the new floor, at once when the sensors agree on it, else from the head's heights over kWaitS. Not
 * thread-safe: the caller locks. Times in seconds of one clock.
 */
class Keeper
{
public:
	static constexpr double kWaitS = 8.0;

	//! The eye height saved earlier (settings.json), used until this run has learned its own.
	void
	set_saved_eye(std::optional<float> eye_m)
	{
		eye_saved = eye_m;
	}

	//! A head height in the tracking space (worn, tracked) at @p t; @p floor_m: the floor, if one is known.
	void
	head(float y, double t, std::optional<float> floor_m)
	{
		const double dt = last_t < 0.0 ? 0.0 : t - last_t;
		last_t = t;
		if (due_t >= 0.0) {
			after.push_back(y);
			return;
		}
		if (floor_m.has_value()) {
			eye.add(y + *floor_m, dt);
		}
	}

	/*!
	 * Measured again: the new floor now when the sensors agree on it, else nullopt. Either way tick checks it against
	 * the head's heights kWaitS later (every sensor moved alike looks like agreement).
	 */
	std::optional<Result>
	done(float floor_before, const struct u_notice_recalibration &change_in, double t)
	{
		change = change_in;
		before = floor_before;
		after.clear();
		due_t = t + kWaitS;
		Result r = after_recalibration(floor_before, change, std::nullopt, std::nullopt);
		applied = r.sensors_agree ? std::optional<float>(r.floor_m) : std::nullopt;
		if (r.sensors_agree) {
			return r;
		}
		return std::nullopt;
	}

	//! The floor once the head's heights are in (kWaitS after done), when it differs from the one applied; else
	//! nullopt.
	std::optional<Result>
	tick(double t)
	{
		if (due_t < 0.0 || t < due_t) {
			return std::nullopt;
		}
		due_t = -1.0;
		std::optional<float> head_m;
		if (after.size() >= 10) {
			std::vector<float> v = after;
			const size_t k = v.size() * 9 / 10;
			std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
			head_m = v[k];
		}
		std::optional<float> e = eye.estimate();
		if (!e) {
			e = eye_saved;
		}
		Result r = after_recalibration(before, change, e, head_m);
		if (applied && std::fabs(r.floor_m - *applied) < 0.01f) {
			applied.reset();
			return std::nullopt;
		}
		applied.reset();
		char extra[96];
		snprintf(extra, sizeof(extra), " (eye height %s, head %s)", e ? std::to_string(*e).substr(0, 4).c_str() : "unknown",
		         head_m ? std::to_string(*head_m).substr(0, 5).c_str() : "unknown");
		r.how += extra;
		return r;
	}

	//! True while the floor after a recalibration waits for the head's heights.
	bool
	waiting() const
	{
		return due_t >= 0.0;
	}

	//! The learned eye height when worth saving (30 s more of it, changed by 1 cm or more), else nullopt.
	std::optional<float>
	eye_to_save()
	{
		const std::optional<float> e = eye.estimate();
		if (!e || eye.seconds - saved_seconds < 30.0 || (eye_saved && std::fabs(*e - *eye_saved) < 0.01f)) {
			return std::nullopt;
		}
		eye_saved = e;
		saved_seconds = eye.seconds;
		return e;
	}

private:
	EyeHeight eye;
	std::optional<float> eye_saved;
	double saved_seconds = 0.0;
	double last_t = -1.0;
	double due_t = -1.0;
	float before = 0.0f;
	//! The floor done applied at once (the sensors agreed), checked by tick.
	std::optional<float> applied;
	struct u_notice_recalibration change = {};
	std::vector<float> after;
};

} // namespace rl::floor
