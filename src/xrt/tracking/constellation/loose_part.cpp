// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Measuring where a device's adjustable LED part sits, see loose_part.hpp.
 * @ingroup aux_tracking
 */

#include "loose_part.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace xrt::tracking::constellation {

namespace {

using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

//! Measured again every this many new frames, and checked as often once in use.
constexpr size_t kEvaluateEvery = 150;
//! The newest frames kept.
constexpr size_t kMaxFrames = 3000;
//! Frames needed to measure: both halves need a few hundred.
constexpr size_t kMinFramesToMeasure = 400;
//! Both halves' offsets must agree this well, and each fit this tightly, to be accepted.
constexpr double kHalvesAgreeM = 0.005;
constexpr double kHalvesAgreeDeg = 1.0;
/*!
 * Frames paired across two cameras carry the cameras' placement error (0.7 degrees is several pixels at 3 m), so a
 * correct fit can be this loose; that both halves agree is what tells a right offset from a wrong one.
 */
constexpr double kMaxP50Px = 1.6;
constexpr size_t kMinMatchesPerHalf = 500;
//! From a camera's own solves alone (exact frames): this many frames and matches per half suffice.
constexpr size_t kMinOwnFramesToMeasure = 200;
constexpr size_t kMinMatchesPerHalfOwn = 300;
//! On average at least this many of the part's LEDs per frame that has any: one blob a frame fits anything.
constexpr double kMinMatchesPerFrame = 2.0;
//! A camera's frame is only kept when the device moved this much since its last kept one: a device lying still gives
//! the same view hundreds of times, where a stray static blob can pass for an LED.
constexpr double kNewViewM = 0.02;
constexpr double kNewViewDeg = 3.0;
//! Once in use: checked on the newest frames; this share of the predicted LEDs must land on a blob.
constexpr size_t kCheckFrames = 300;
constexpr size_t kMinPredictedForCheck = 150;
constexpr double kMinShareInUse = 0.35;
//! A new measurement replaces the one in use only when it moved at least this much.
constexpr double kReplaceM = 0.003;
constexpr double kReplaceDeg = 0.5;
//! The grid: the part's tilt about the device's side axis, and up/down, forward/back.
constexpr double kGridTiltDeg = 40.0;
constexpr double kGridTiltStepDeg = 5.0;
constexpr double kGridShiftM = 0.06;
constexpr double kGridShiftStepM = 0.015;
constexpr size_t kGridFrames = 150;
constexpr double kGridGatePx = 6.0;
constexpr double kHuberPx = 2.0;

struct Rt
{
	Matrix3d R{Matrix3d::Identity()};
	Vector3d t{Vector3d::Zero()};
};

Rt
from_pose(const xrt_pose &p)
{
	Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
	return {q.normalized().toRotationMatrix(), Vector3d(p.position.x, p.position.y, p.position.z)};
}

xrt_pose
to_pose(const Rt &rt)
{
	Eigen::Quaterniond q(rt.R);
	q.normalize();
	xrt_pose p;
	p.orientation = {(float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()};
	p.position = {(float)rt.t.x(), (float)rt.t.y(), (float)rt.t.z()};
	return p;
}

Matrix3d
rotvec(const Vector3d &r)
{
	const double a = r.norm();
	if (a < 1e-12) {
		return Matrix3d::Identity();
	}
	return Eigen::AngleAxisd(a, r / a).toRotationMatrix();
}

Vector6d
to_params(const Rt &rt)
{
	Eigen::AngleAxisd aa(rt.R);
	Vector6d x;
	x.head<3>() = aa.axis() * aa.angle();
	x.tail<3>() = rt.t;
	return x;
}

Rt
from_params(const Vector6d &x)
{
	return {rotvec(x.head<3>()), x.tail<3>()};
}

double
angle_deg(const Matrix3d &a, const Matrix3d &b)
{
	Eigen::AngleAxisd aa(a.transpose() * b);
	return std::abs(aa.angle()) * 180.0 / M_PI;
}

struct Led
{
	Vector3d p;
	Vector3d n;
};

struct Frame
{
	Rt cd; //!< camera from device
	double fx, fy, cx, cy;
	std::vector<Vector2d> blobs;
};

struct Match
{
	size_t frame;
	size_t led;
	Vector2d blob;
	double dist;
};

std::vector<Led>
leds_of(const std::vector<t_constellation_tracker_led> &part)
{
	std::vector<Led> out;
	out.reserve(part.size());
	for (const auto &l : part) {
		out.push_back({Vector3d(l.position.x, l.position.y, l.position.z),
		               Vector3d(l.normal.x, l.normal.y, l.normal.z)});
	}
	return out;
}

std::vector<Frame>
frames_of(const std::vector<const LoosePartFrame *> &in)
{
	std::vector<Frame> out;
	out.reserve(in.size());
	for (const LoosePartFrame *f : in) {
		Frame g{from_pose(f->Tcv_cam_device), f->fx, f->fy, f->cx, f->cy, {}};
		g.blobs.reserve(f->blobs.size());
		for (const auto &b : f->blobs) {
			g.blobs.emplace_back(b.x, b.y);
		}
		out.push_back(std::move(g));
	}
	return out;
}

//! Where the part's LED lands in the frame, or false when it faces away or is behind the camera.
inline bool
project(const Frame &f, const Rt &part, const Led &led, Vector2d &out)
{
	const Vector3d P = f.cd.R * (part.R * led.p + part.t) + f.cd.t;
	if (P.z() <= 0.05) {
		return false;
	}
	const Vector3d N = f.cd.R * (part.R * led.n);
	if (N.dot(-P) / P.norm() <= 0.2) {
		return false;
	}
	out = Vector2d(f.fx * P.x() / P.z() + f.cx, f.fy * P.y() / P.z() + f.cy);
	return true;
}

//! Each predicted LED to the nearest blob not yet taken in its frame, within @p gate_px.
void
associate(const std::vector<Frame> &frames,
          const std::vector<size_t> &sel,
          const std::vector<Led> &leds,
          const Rt &part,
          double gate_px,
          std::vector<Match> &out,
          size_t *out_predicted = nullptr)
{
	out.clear();
	size_t predicted = 0;
	std::vector<char> used;
	for (size_t fi : sel) {
		const Frame &f = frames[fi];
		used.assign(f.blobs.size(), 0);
		for (size_t li = 0; li < leds.size(); li++) {
			Vector2d px;
			if (!project(f, part, leds[li], px)) {
				continue;
			}
			predicted++;
			double best = gate_px;
			size_t best_j = SIZE_MAX;
			for (size_t j = 0; j < f.blobs.size(); j++) {
				if (used[j]) {
					continue;
				}
				const double d = (f.blobs[j] - px).norm();
				if (d < best) {
					best = d;
					best_j = j;
				}
			}
			if (best_j != SIZE_MAX) {
				used[best_j] = 1;
				out.push_back({fi, li, f.blobs[best_j], best});
			}
		}
	}
	if (out_predicted != nullptr) {
		*out_predicted = predicted;
	}
}

//! The residuals of all matches at @p x (pixels, two per match); false when an LED left the view.
bool
residuals(const std::vector<Frame> &frames,
          const std::vector<Led> &leds,
          const std::vector<Match> &matches,
          const Vector6d &x,
          Eigen::VectorXd &r)
{
	const Rt part = from_params(x);
	r.resize((Eigen::Index)(2 * matches.size()));
	for (size_t i = 0; i < matches.size(); i++) {
		const Match &m = matches[i];
		const Frame &f = frames[m.frame];
		const Vector3d P = f.cd.R * (part.R * leds[m.led].p + part.t) + f.cd.t;
		if (P.z() <= 1e-3) {
			return false;
		}
		r[(Eigen::Index)(2 * i)] = f.fx * P.x() / P.z() + f.cx - m.blob.x();
		r[(Eigen::Index)(2 * i + 1)] = f.fy * P.y() / P.z() + f.cy - m.blob.y();
	}
	return true;
}

double
huber_cost(const Eigen::VectorXd &r, Eigen::VectorXd *weights)
{
	double cost = 0;
	if (weights != nullptr) {
		weights->resize(r.size());
	}
	for (Eigen::Index i = 0; i + 1 < r.size(); i += 2) {
		const double e = std::hypot(r[i], r[i + 1]);
		const double w = e <= kHuberPx ? 1.0 : kHuberPx / e;
		cost += e <= kHuberPx ? e * e : 2.0 * kHuberPx * e - kHuberPx * kHuberPx;
		if (weights != nullptr) {
			(*weights)[i] = w;
			(*weights)[i + 1] = w;
		}
	}
	return cost;
}

//! Levenberg-Marquardt on the matches, Huber-weighted, numeric derivatives.
Vector6d
refine(const std::vector<Frame> &frames, const std::vector<Led> &leds, const std::vector<Match> &matches, Vector6d x)
{
	Eigen::VectorXd r, w, r2;
	if (!residuals(frames, leds, matches, x, r)) {
		return x;
	}
	double cost = huber_cost(r, &w);
	double lambda = 1e-3;
	Eigen::MatrixXd J((Eigen::Index)r.size(), 6);
	for (int iter = 0; iter < 40; iter++) {
		for (int k = 0; k < 6; k++) {
			const double h = k < 3 ? 1e-6 : 1e-6;
			Vector6d xp = x, xm = x;
			xp[k] += h;
			xm[k] -= h;
			Eigen::VectorXd rp, rm;
			if (!residuals(frames, leds, matches, xp, rp) || !residuals(frames, leds, matches, xm, rm)) {
				return x;
			}
			J.col(k) = (rp - rm) / (2.0 * h);
		}
		const Eigen::MatrixXd WJ = w.asDiagonal() * J;
		Matrix6d H = J.transpose() * WJ;
		const Vector6d g = WJ.transpose() * r;
		bool improved = false;
		for (int tries = 0; tries < 10 && !improved; tries++) {
			Matrix6d Hd = H;
			Hd.diagonal() *= (1.0 + lambda);
			const Vector6d dx = -Hd.fullPivLu().solve(g);
			const Vector6d xn = x + dx;
			if (residuals(frames, leds, matches, xn, r2)) {
				const double cost2 = huber_cost(r2, nullptr);
				if (cost2 < cost) {
					improved = true;
					x = xn;
					r = r2;
					cost = huber_cost(r, &w);
					lambda = std::max(lambda / 3.0, 1e-9);
					if (dx.norm() < 1e-8) {
						return x;
					}
				}
			}
			if (!improved) {
				lambda *= 4.0;
			}
		}
		if (!improved) {
			break;
		}
	}
	return x;
}

std::vector<size_t>
all_of(size_t n)
{
	std::vector<size_t> v(n);
	for (size_t i = 0; i < n; i++) {
		v[i] = i;
	}
	return v;
}

bool
fit(const std::vector<Led> &leds, const std::vector<Frame> &frames, const Rt *start, LoosePartFit &out)
{
	if (frames.empty() || leds.empty()) {
		return false;
	}
	const std::vector<size_t> all = all_of(frames.size());
	std::vector<Match> matches;
	Rt x0;
	if (start != nullptr) {
		x0 = *start;
	} else {
		std::vector<size_t> sub;
		const size_t step = std::max<size_t>(1, frames.size() / kGridFrames);
		for (size_t i = 0; i < frames.size(); i += step) {
			sub.push_back(i);
		}
		size_t best = 0;
		for (double tilt = -kGridTiltDeg; tilt <= kGridTiltDeg + 1e-9; tilt += kGridTiltStepDeg) {
			for (double ty = -kGridShiftM; ty <= kGridShiftM + 1e-9; ty += kGridShiftStepM) {
				for (double tz = -kGridShiftM; tz <= kGridShiftM + 1e-9; tz += kGridShiftStepM) {
					Rt c{Eigen::AngleAxisd(tilt * M_PI / 180.0, Vector3d::UnitX()).toRotationMatrix(),
					     Vector3d(0.0, ty, tz)};
					associate(frames, sub, leds, c, kGridGatePx, matches);
					if (matches.size() > best) {
						best = matches.size();
						x0 = c;
					}
				}
			}
		}
		if (best == 0) {
			return false;
		}
	}
	Vector6d x = to_params(x0);
	for (double gate : {10.0, 6.0, 4.0}) {
		associate(frames, all, leds, from_params(x), gate, matches);
		if (matches.size() < 30) {
			return false;
		}
		x = refine(frames, leds, matches, x);
	}
	associate(frames, all, leds, from_params(x), 4.0, matches);
	if (matches.empty()) {
		return false;
	}
	std::vector<double> d;
	d.reserve(matches.size());
	for (const auto &m : matches) {
		d.push_back(m.dist);
	}
	std::sort(d.begin(), d.end());
	size_t frames_matched = 0;
	for (size_t i = 0; i < matches.size(); i++) {
		if (i == 0 || matches[i].frame != matches[i - 1].frame) {
			frames_matched++;
		}
	}
	out.offset = to_pose(from_params(x));
	out.frames = frames.size();
	out.matches = matches.size();
	out.frames_matched = frames_matched;
	out.p50_px = (float)d[d.size() / 2];
	out.p90_px = (float)d[(d.size() * 9) / 10];
	return true;
}

} // namespace

bool
loose_part_fit(const std::vector<t_constellation_tracker_led> &part_leds,
               const std::vector<const LoosePartFrame *> &frames,
               const xrt_pose *start,
               LoosePartFit &out)
{
	const Rt s = start != nullptr ? from_pose(*start) : Rt{};
	return fit(leds_of(part_leds), frames_of(frames), start != nullptr ? &s : nullptr, out);
}

void
loose_part_check(const std::vector<t_constellation_tracker_led> &part_leds,
                 const std::vector<const LoosePartFrame *> &frames,
                 const xrt_pose &offset,
                 float gate_px,
                 size_t &out_predicted,
                 size_t &out_matched)
{
	const std::vector<Frame> f = frames_of(frames);
	std::vector<Match> matches;
	associate(f, all_of(f.size()), leds_of(part_leds), from_pose(offset), gate_px, matches, &out_predicted);
	out_matched = matches.size();
}

LoosePartEstimator::LoosePartEstimator(std::vector<t_constellation_tracker_led> part_leds_cv,
                                       bool deterministic,
                                       ResultFn on_result)
    : part_leds(std::move(part_leds_cv)), deterministic(deterministic), on_result(std::move(on_result))
{
	if (!this->deterministic) {
		this->worker = std::thread([this] { this->run(); });
	}
}

LoosePartEstimator::~LoosePartEstimator()
{
	{
		std::unique_lock<std::mutex> l(this->lock);
		this->stop = true;
	}
	this->cond.notify_all();
	if (this->worker.joinable()) {
		this->worker.join();
	}
}

void
LoosePartEstimator::setInUse(const xrt_pose &offset)
{
	std::unique_lock<std::mutex> l(this->lock);
	this->in_use = offset;
}

size_t
LoosePartEstimator::frameCount()
{
	std::unique_lock<std::mutex> l(this->lock);
	return this->frames.size();
}

bool
LoosePartEstimator::inUse()
{
	std::unique_lock<std::mutex> l(this->lock);
	return this->in_use.has_value();
}

void
LoosePartEstimator::addFrame(LoosePartFrame &&frame)
{
	// Only frames in which some of the part may face the camera: a front camera sees the device's front all the
	// time, and those frames would crowd out the ones that matter. Generous, the part may sit tens of degrees off
	// its factory place.
	const Rt cd = from_pose(frame.Tcv_cam_device);
	bool facing = false;
	for (const auto &led : this->part_leds) {
		const Vector3d P = cd.R * Vector3d(led.position.x, led.position.y, led.position.z) + cd.t;
		const Vector3d N = cd.R * Vector3d(led.normal.x, led.normal.y, led.normal.z);
		if (P.z() > 0.05 && N.dot(-P) / P.norm() > -0.3) {
			facing = true;
			break;
		}
	}
	if (!facing) {
		return;
	}

	{
		std::unique_lock<std::mutex> l(this->lock);
		auto it = this->last_kept.find(frame.camera);
		if (it != this->last_kept.end()) {
			const Rt last = from_pose(it->second);
			if ((last.t - cd.t).norm() < kNewViewM && angle_deg(last.R, cd.R) < kNewViewDeg) {
				return;
			}
		}
		this->last_kept[frame.camera] = frame.Tcv_cam_device;
	}

	std::vector<LoosePartFrame> copy;
	std::optional<xrt_pose> use;
	{
		std::unique_lock<std::mutex> l(this->lock);
		this->frames.push_back(std::move(frame));
		while (this->frames.size() > kMaxFrames) {
			this->frames.pop_front();
		}
		this->frames_since_evaluation++;
		const bool due = this->frames_since_evaluation >= kEvaluateEvery &&
		                 (this->in_use.has_value() || this->frames.size() >= kMinFramesToMeasure);
		if (!due) {
			return;
		}
		this->frames_since_evaluation = 0;
		if (!this->deterministic) {
			this->evaluate_requested = true;
			l.unlock();
			this->cond.notify_one();
			return;
		}
		copy.assign(this->frames.begin(), this->frames.end());
		use = this->in_use;
	}
	this->evaluate(std::move(copy), use);
}

void
LoosePartEstimator::run()
{
	for (;;) {
		std::vector<LoosePartFrame> copy;
		std::optional<xrt_pose> use;
		{
			std::unique_lock<std::mutex> l(this->lock);
			this->cond.wait(l, [this] { return this->stop || this->evaluate_requested; });
			if (this->stop) {
				return;
			}
			this->evaluate_requested = false;
			copy.assign(this->frames.begin(), this->frames.end());
			use = this->in_use;
		}
		this->evaluate(std::move(copy), use);
	}
}

//! CONSTELLATION_TRACKER_LOOSE_PART_DEBUG=1: every measuring attempt and why it was turned down, on stderr.
static bool
debug_attempts()
{
	static const bool on = [] {
		const char *v = getenv("CONSTELLATION_TRACKER_LOOSE_PART_DEBUG");
		return v != nullptr && atoi(v) != 0;
	}();
	return on;
}

static void
debug_fit(const char *what, const LoosePartFit &f)
{
	if (!debug_attempts()) {
		return;
	}
	const Rt rt = from_pose(f.offset);
	fprintf(stderr, "loose part: %s: %zu frames, %zu matched (%zu frames with any), p50 %.2f px, p90 %.2f px, %.1f mm, %.1f deg\n",
	        what, f.frames, f.matches, f.frames_matched, (double)f.p50_px, (double)f.p90_px, rt.t.norm() * 1000.0,
	        angle_deg(Matrix3d::Identity(), rt.R));
}

bool
LoosePartEstimator::measure(const std::vector<const LoosePartFrame *> &frames, LoosePartFit &out, const char *&why)
{
	// The exact frames alone when there are enough of them: frames through two cameras carry their placement error.
	std::vector<const LoosePartFrame *> own;
	for (const LoosePartFrame *f : frames) {
		if (f->own) {
			own.push_back(f);
		}
	}
	bool ok = false;
	const char *which = "all frames";
	if (own.size() >= kMinOwnFramesToMeasure) {
		ok = this->measureOnce(own, kMinMatchesPerHalfOwn, out, why);
		which = "the frames from a camera's own solve";
		if (debug_attempts()) {
			fprintf(stderr, "loose part: attempt on %zu of %zu frames (own solve): %s\n", own.size(), frames.size(),
			        ok ? "accepted" : why);
		}
	}
	if (!ok) {
		ok = this->measureOnce(frames, kMinMatchesPerHalf, out, why);
		which = "all frames";
		if (debug_attempts()) {
			fprintf(stderr, "loose part: attempt on %zu frames: %s\n", frames.size(), ok ? "accepted" : why);
		}
	}
	(void)which;
	return ok;
}

bool
LoosePartEstimator::measureOnce(const std::vector<const LoosePartFrame *> &frames,
                                size_t min_matches_per_half,
                                LoosePartFit &out,
                                const char *&why)
{
	if (frames.size() < kMinFramesToMeasure / 2) {
		why = "not enough frames";
		return false;
	}
	// Halves by a hash of the frame's place, not every other frame: frames of different kinds come in turn.
	std::vector<const LoosePartFrame *> a, b;
	for (size_t i = 0; i < frames.size(); i++) {
		uint64_t z = (uint64_t)i + 0x9E3779B97F4A7C15ULL;
		z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
		z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
		z ^= z >> 31;
		((z & 1) == 0 ? a : b).push_back(frames[i]);
	}
	LoosePartFit fa, fb;
	if (!loose_part_fit(this->part_leds, a, nullptr, fa) || !loose_part_fit(this->part_leds, b, nullptr, fb)) {
		why = "no fit";
		return false;
	}
	debug_fit("half A", fa);
	debug_fit("half B", fb);
	if (fa.p50_px > kMaxP50Px || fb.p50_px > kMaxP50Px || fa.matches < min_matches_per_half ||
	    fb.matches < min_matches_per_half) {
		why = "the halves fit too loosely or too few LEDs matched";
		return false;
	}
	if ((double)fa.matches < kMinMatchesPerFrame * (double)fa.frames_matched ||
	    (double)fb.matches < kMinMatchesPerFrame * (double)fb.frames_matched) {
		why = "too few LEDs per frame";
		return false;
	}
	const Rt ra = from_pose(fa.offset), rb = from_pose(fb.offset);
	if ((ra.t - rb.t).norm() > kHalvesAgreeM || angle_deg(ra.R, rb.R) > kHalvesAgreeDeg) {
		why = "the halves disagree";
		return false;
	}
	if (!loose_part_fit(this->part_leds, frames, &fa.offset, out) || out.p50_px > kMaxP50Px) {
		why = "the whole fits too loosely";
		return false;
	}
	return true;
}

void
LoosePartEstimator::evaluate(std::vector<LoosePartFrame> frames, std::optional<xrt_pose> use)
{
	std::vector<const LoosePartFrame *> ptrs;
	ptrs.reserve(frames.size());
	for (const auto &f : frames) {
		ptrs.push_back(&f);
	}

	if (!use.has_value()) {
		LoosePartFit fitted;
		const char *why = nullptr;
		if (this->measure(ptrs, fitted, why)) {
			{
				std::unique_lock<std::mutex> l(this->lock);
				this->in_use = fitted.offset;
			}
			this->on_result(fitted, "measured");
		}
		return;
	}

	// In use: do its LEDs still land on blobs in the newest frames?
	const size_t n = std::min(kCheckFrames, ptrs.size());
	const std::vector<const LoosePartFrame *> newest(ptrs.end() - (std::ptrdiff_t)n, ptrs.end());
	size_t predicted = 0, matched = 0;
	loose_part_check(this->part_leds, newest, use.value(), 4.0f, predicted, matched);
	if (debug_attempts()) {
		fprintf(stderr, "loose part: check on %zu frames: %zu of %zu predicted LEDs on a blob\n", newest.size(), matched,
		        predicted);
	}
	if (predicted < kMinPredictedForCheck || (double)matched >= kMinShareInUse * (double)predicted) {
		return;
	}

	// Moved: measure again on the frames since, or stop using it.
	const size_t m = std::min<size_t>(ptrs.size(), 4 * kCheckFrames);
	const std::vector<const LoosePartFrame *> recent(ptrs.end() - (std::ptrdiff_t)m, ptrs.end());
	LoosePartFit fitted;
	const char *why = nullptr;
	if (this->measure(recent, fitted, why)) {
		const Rt old_rt = from_pose(use.value()), new_rt = from_pose(fitted.offset);
		if ((old_rt.t - new_rt.t).norm() < kReplaceM && angle_deg(old_rt.R, new_rt.R) < kReplaceDeg) {
			return;
		}
		{
			std::unique_lock<std::mutex> l(this->lock);
			this->in_use = fitted.offset;
			this->frames.clear();
			this->last_kept.clear();
		}
		this->on_result(fitted, "measured again: its LEDs no longer landed where expected");
		return;
	}
	{
		std::unique_lock<std::mutex> l(this->lock);
		this->in_use.reset();
		this->frames.clear();
		this->last_kept.clear();
	}
	this->on_result(std::nullopt, "its LEDs no longer land where expected; measuring it again");
}

} // namespace xrt::tracking::constellation
