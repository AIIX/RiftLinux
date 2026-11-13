// Copyright 2026, Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief A Touch controller's error-state Kalman filter, driven by its IMU at full rate (RiftLinux).
 *
 * The prediction the camera checks compare each solve with: carried by the gyro and the accelerometer from one camera
 * pose to the next, so a fast punch or wrist turn does not leave it behind the hand. Fifteen error states, as the
 * Oculus runtime's own Touch filter has (position, velocity, orientation, accelerometer and gyro bias; its gravity tilt
 * states are left out): every accepted solve is an update weighted by its own uncertainty, and one far off by both the
 * Mahalanobis distance and more than 10 cm starts the filter over at the solve, as there.
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <optional>

class TouchEkf
{
public:
	using Vec3 = Eigen::Vector3d;
	using Mat3 = Eigen::Matrix3d;
	using Quat = Eigen::Quaterniond;
	using Mat15 = Eigen::Matrix<double, 15, 15>;

	//! Pushes one IMU sample (device frame, OpenXR axes, the accelerometer's specific force in m/s²).
	void
	propagate(int64_t timestamp_ns, const xrt_vec3 &accel_m_s2, const xrt_vec3 &gyro_rad_s)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		Vec3 f(accel_m_s2.x, accel_m_s2.y, accel_m_s2.z);
		const Vec3 w(gyro_rad_s.x, gyro_rad_s.y, gyro_rad_s.z);
		last_gyro_ = w;
		// How hard it moved lately (for the offset's learning): the largest turn rate and the specific force's distance
		// from 1 g, decaying over about 0.3 s.
		motion_gyro_ = std::max(w.norm(), motion_gyro_ * 0.99);
		motion_accel_ = std::max(std::fabs(f.norm() - rest_accel_len_), motion_accel_ * 0.99);
		// A CV1 Touch's accelerometer reads 10.1-10.5 m/s² at rest: its scale, learned while still, taken out.
		const double f_len = f.norm();
		if (w.norm() < 0.1 && std::fabs(f_len - rest_accel_len_) < 1.5) {
			rest_accel_len_ += 0.01 * (f_len - rest_accel_len_);
		}
		if (rest_accel_len_ > 1.0) {
			f *= 9.80665 / rest_accel_len_;
		}
		if (!valid_) {
			last_imu_ns_ = timestamp_ns;
			return;
		}
		const double dt = (timestamp_ns - last_imu_ns_) / 1e9;
		if (dt <= 0.0) {
			return;
		}
		last_imu_ns_ = timestamp_ns;
		if (dt > 0.1) {
			// A gap: nothing known of what happened in it.
			P_.block<3, 3>(3, 3) += Mat3::Identity() * 1.0;
			P_.block<3, 3>(6, 6) += Mat3::Identity() * 0.01;
			push_history(timestamp_ns);
			return;
		}

		const Vec3 wc = w - bg_;
		const Vec3 fc = f - ba_;
		const Mat3 R = q_.toRotationMatrix();
		const Vec3 a_world = R * fc + kGravity;

		p_ += v_ * dt + 0.5 * a_world * dt * dt;
		v_ += a_world * dt;
		const double angle = wc.norm() * dt;
		if (angle > 1e-12) {
			q_ = (q_ * Quat(Eigen::AngleAxisd(angle, wc.normalized()))).normalized();
		}

		// Error state: [dp, dv, dtheta (local), dba, dbg].
		Mat15 F = Mat15::Identity();
		F.block<3, 3>(0, 3) = Mat3::Identity() * dt;
		F.block<3, 3>(3, 6) = -R * skew(fc) * dt;
		F.block<3, 3>(3, 9) = -R * dt;
		F.block<3, 3>(6, 6) = Mat3::Identity() - skew(wc) * dt;
		F.block<3, 3>(6, 12) = -Mat3::Identity() * dt;

		// Motion dependent noise, as the Oculus runtime's (Rift.dll, per IMU sample).
		const double va = 2.5e-5 / 3.0 * fc.squaredNorm() + accel_noise_ * accel_noise_ + 9e-6 * wc.squaredNorm();
		const double vg = gyro_noise_ * gyro_noise_ + 2.5e-5 / 3.0 * wc.squaredNorm();
		P_ = F * P_ * F.transpose();
		for (int i = 0; i < 3; i++) {
			P_(0 + i, 0 + i) += dt * dt * dt * dt / 4.0 * va;
			P_(0 + i, 3 + i) += dt * dt * dt / 2.0 * va;
			P_(3 + i, 0 + i) += dt * dt * dt / 2.0 * va;
			P_(3 + i, 3 + i) += dt * dt * va;
			P_(6 + i, 6 + i) += dt * dt * vg;
			P_(9 + i, 9 + i) = std::max(P_(9 + i, 9 + i), kAccelBiasFloor);
			P_(12 + i, 12 + i) = std::max(P_(12 + i, 12 + i), kGyroBiasFloor);
		}
		push_history(timestamp_ns);
	}

	/*!
	 * A camera pose of the device in the world at @p timestamp_ns, its position and orientation sigmas. Applied to the
	 * state the filter had at that time and carried to now (the Oculus runtime does the same, P not carried back).
	 */
	void
	update(int64_t timestamp_ns, const xrt_pose &world_pose, double sigma_pos_m, double sigma_rot_rad)
	{
		Eigen::Matrix<double, 6, 6> Rm = Eigen::Matrix<double, 6, 6>::Zero();
		Rm.block<3, 3>(0, 0) = Mat3::Identity() * sigma_pos_m * sigma_pos_m;
		Rm.block<3, 3>(3, 3) = Mat3::Identity() * sigma_rot_rad * sigma_rot_rad;
		update(timestamp_ns, world_pose, Rm);
	}

	//! As above, with the pose's full 6x6 covariance (world position, then local orientation), as the Oculus runtime's.
	void
	update(int64_t timestamp_ns, const xrt_pose &world_pose, const Eigen::Matrix<double, 6, 6> &Rm)
	{
		const double sigma_pos_m = std::sqrt(std::max(Rm.block<3, 3>(0, 0).trace() / 3.0, 1e-10));
		const double sigma_rot_rad = std::sqrt(std::max(Rm.block<3, 3>(3, 3).trace() / 3.0, 1e-12));
		std::lock_guard<std::mutex> lock(mutex_);
		const Vec3 zp(world_pose.position.x, world_pose.position.y, world_pose.position.z);
		const Quat zq(world_pose.orientation.w, world_pose.orientation.x, world_pose.orientation.y,
		              world_pose.orientation.z);
		if (!valid_ || last_imu_ns_ == 0) {
			reset_to(timestamp_ns, zp, zq, sigma_pos_m, sigma_rot_rad);
			return;
		}

		// The state at the pose's time, from the history (or now, if newer than any).
		Vec3 hp = p_;
		Quat hq = q_;
		for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
			if (it->ns <= timestamp_ns) {
				hp = it->p;
				hq = it->q;
				break;
			}
		}

		Eigen::Matrix<double, 6, 1> y;
		y.head<3>() = zp - hp;
		Quat dq = hq.conjugate() * zq;
		if (dq.w() < 0.0) {
			dq.coeffs() = -dq.coeffs();
		}
		const Eigen::AngleAxisd aa(dq);
		y.tail<3>() = aa.axis() * aa.angle();

		Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
		H.block<3, 3>(0, 0) = Mat3::Identity();
		H.block<3, 3>(3, 6) = Mat3::Identity();

		// How far the prediction was from this pose, binned (for the log).
		{
			const double pm = y.head<3>().norm();
			const double rd = y.tail<3>().norm() * 57.29578;
			stats_pos_[pm < 0.01 ? 0 : pm < 0.03 ? 1 : pm < 0.10 ? 2 : 3]++;
			stats_rot_[rd < 2.0 ? 0 : rd < 5.0 ? 1 : rd < 15.0 ? 2 : 3]++;
		}
		const Eigen::Matrix<double, 6, 6> S = H * P_ * H.transpose() + Rm;
		const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt(S);
		if (ldlt.info() != Eigen::Success) {
			reset_to(timestamp_ns, zp, zq, sigma_pos_m, sigma_rot_rad);
			return;
		}
		const double chi2 = y.dot(ldlt.solve(y));
		// As the Oculus runtime: far off by both measures, start over at the camera; otherwise always taken in.
		if (chi2 > kResetChi2 && y.head<3>().squaredNorm() > kResetPosM * kResetPosM) {
			reset_to(timestamp_ns, zp, zq, sigma_pos_m, sigma_rot_rad);
			resets_++;
			return;
		}

		const Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * ldlt.solve(Eigen::Matrix<double, 6, 6>::Identity());
		const Eigen::Matrix<double, 15, 1> dx = K * y;
		p_ += dx.segment<3>(0);
		v_ += dx.segment<3>(3);
		const Vec3 dth = dx.segment<3>(6);
		if (dth.norm() > 1e-12) {
			q_ = (q_ * Quat(Eigen::AngleAxisd(dth.norm(), dth.normalized()))).normalized();
		}
		// The offset is learned only while the controller moves calmly: in a punch the filter took the hand's own
		// acceleration for the sensor's offset and lost the controller (T_CONSTELLATION_TOUCH_EKF_BIAS_CALM_*).
		if (motion_gyro_ <= bias_calm_gyro() && motion_accel_ <= bias_calm_accel()) {
			ba_ += clamp_norm(dx.segment<3>(9), bias_step());
		}
		bg_ += clamp_norm(dx.segment<3>(12), kGyroBiasStep);
		const Mat15 IKH = Mat15::Identity() - K * H;
		P_ = IKH * P_ * IKH.transpose() + K * Rm * K.transpose();
		P_ = 0.5 * (P_ + P_.transpose());
		last_update_ns_ = std::max(last_update_ns_, timestamp_ns);
		// The history from here on is what the filter now holds, moved by the same correction.
		for (auto &h : history_) {
			if (h.ns >= timestamp_ns) {
				h.p += dx.segment<3>(0);
				if (dth.norm() > 1e-12) {
					h.q = (h.q * Quat(Eigen::AngleAxisd(dth.norm(), dth.normalized()))).normalized();
				}
			}
		}
	}

	/*!
	 * The device's relation in the world at @p when_ns: from the history while it reaches that far back, carried on
	 * from now by its velocities otherwise. False until a camera pose started the filter or past @p fresh_ns since the
	 * last one.
	 */
	bool
	relationAt(int64_t when_ns, int64_t fresh_ns, xrt_space_relation &out) const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		if (!valid_ || last_update_ns_ == 0 || when_ns - last_update_ns_ > fresh_ns) {
			return false;
		}
		Vec3 p = p_;
		Quat q = q_;
		Vec3 v = v_;
		if (when_ns <= last_imu_ns_) {
			for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
				if (it->ns <= when_ns) {
					p = it->p;
					q = it->q;
					v = it->v;
					break;
				}
			}
		} else {
			const double dt = std::min((when_ns - last_imu_ns_) / 1e9, 0.05);
			p += v * dt;
			const Vec3 wc = last_gyro_ - bg_;
			if (wc.norm() * dt > 1e-12) {
				q = (q * Quat(Eigen::AngleAxisd(wc.norm() * dt, wc.normalized()))).normalized();
			}
		}
		const Vec3 w_world = q.toRotationMatrix() * (last_gyro_ - bg_);
		out = XRT_SPACE_RELATION_ZERO;
		out.pose.position = {(float)p.x(), (float)p.y(), (float)p.z()};
		out.pose.orientation = {(float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()};
		out.linear_velocity = {(float)v.x(), (float)v.y(), (float)v.z()};
		out.angular_velocity = {(float)w_world.x(), (float)w_world.y(), (float)w_world.z()};
		out.relation_flags = (enum xrt_space_relation_flags)(
		    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
		    XRT_SPACE_RELATION_POSITION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
		    XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT | XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
		return true;
	}

	//! The prediction's distance from the poses since the last call: position <1, <3, <10, >=10 cm; rotation <2, <5, <15,
	//! >=15 degrees. Cleared.
	void
	takeStats(uint32_t out_pos[4], uint32_t out_rot[4])
	{
		std::lock_guard<std::mutex> lock(mutex_);
		for (int i = 0; i < 4; i++) {
			out_pos[i] = stats_pos_[i];
			out_rot[i] = stats_rot_[i];
			stats_pos_[i] = 0;
			stats_rot_[i] = 0;
		}
	}

	//! The IMU's noise floors per sample: accelerometer m/s², gyro rad/s (the Oculus runtime's: 0.05 and 0.002).
	void
	setNoise(double accel_noise, double gyro_noise)
	{
		std::lock_guard<std::mutex> lock(mutex_);
		accel_noise_ = accel_noise;
		gyro_noise_ = gyro_noise;
	}

	uint64_t
	resets() const
	{
		std::lock_guard<std::mutex> lock(mutex_);
		return resets_;
	}

private:
	struct HistoryEntry
	{
		int64_t ns;
		Vec3 p;
		Vec3 v;
		Quat q;
	};

	static Mat3
	skew(const Vec3 &v)
	{
		Mat3 m;
		m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
		return m;
	}

	static Vec3
	clamp_norm(const Vec3 &v, double max_norm)
	{
		const double n = v.norm();
		return n > max_norm ? Vec3(v * (max_norm / n)) : v;
	}

	void
	push_history(int64_t ns)
	{
		history_.push_back({ns, p_, v_, q_});
		while (!history_.empty() && ns - history_.front().ns > kHistoryNs) {
			history_.pop_front();
		}
	}

	void
	reset_to(int64_t ns, const Vec3 &p, const Quat &q, double sigma_pos_m, double sigma_rot_rad)
	{
		p_ = p;
		q_ = q.normalized();
		v_ = Vec3::Zero();
		const bool had = valid_;
		const Mat3 keep_ba = P_.block<3, 3>(9, 9);
		const Mat3 keep_bg = P_.block<3, 3>(12, 12);
		P_ = Mat15::Zero();
		P_.block<3, 3>(0, 0) = Mat3::Identity() * sigma_pos_m * sigma_pos_m;
		P_.block<3, 3>(3, 3) = Mat3::Identity() * 0.0625;
		P_.block<3, 3>(6, 6) = Mat3::Identity() * sigma_rot_rad * sigma_rot_rad;
		P_.block<3, 3>(9, 9) = had ? keep_ba : Mat3::Identity() * start_bias_var();
		P_.block<3, 3>(12, 12) = had ? keep_bg : Mat3::Identity() * 5.0 * kGyroBiasFloor;
		valid_ = true;
		last_update_ns_ = std::max(last_update_ns_, ns);
		if (last_imu_ns_ == 0) {
			last_imu_ns_ = ns;
		}
		history_.clear();
		push_history(std::max(ns, last_imu_ns_));
	}

	// World gravity, OpenXR axes (up is +Y): the accelerometer reads the opposite at rest.
	inline static const Vec3 kGravity{0.0, -9.80665, 0.0};
	static constexpr double kResetChi2 = 750.0;
	static constexpr double kResetPosM = 0.10;
	static constexpr double kAccelBiasFloor = 2.638367e-7;
	static constexpr double kGyroBiasFloor = 7.615435e-9;
	static constexpr double kAccelBiasStep = 0.004;

	/*!
	 * How unsure the accelerometer's offset is when the filter starts (variance, (m/s²)²). The Oculus runtime starts from
	 * the offset it saved (Ekf_ file) and its floor; with none saved, the floor kept the offset at 0 while a CV1 Touch
	 * reads 0.4-1.1 m/s² off (Oculus logs, our IMU alignment): the prediction ran 12 cm off in 0.5 s, 50 cm in 1 s.
	 * T_CONSTELLATION_TOUCH_EKF_BIAS_SIGMA (m/s²) sets it; T_CONSTELLATION_TOUCH_EKF_BIAS_STEP the most one pose moves it.
	 */
	static double
	start_bias_var()
	{
		static const double v = [] {
			const char *e = getenv("T_CONSTELLATION_TOUCH_EKF_BIAS_SIGMA");
			const double sigma = e != nullptr ? atof(e) : 0.0;
			return sigma > 0.0 ? sigma * sigma : 5.0 * kAccelBiasFloor;
		}();
		return v;
	}
	static double
	bias_calm_gyro()
	{
		static const double v = [] {
			const char *e = getenv("T_CONSTELLATION_TOUCH_EKF_BIAS_CALM_GYRO");
			return e != nullptr && atof(e) > 0.0 ? atof(e) : 1e9;
		}();
		return v;
	}
	static double
	bias_calm_accel()
	{
		static const double v = [] {
			const char *e = getenv("T_CONSTELLATION_TOUCH_EKF_BIAS_CALM_ACCEL");
			return e != nullptr && atof(e) > 0.0 ? atof(e) : 1e9;
		}();
		return v;
	}
	static double
	bias_step()
	{
		static const double v = [] {
			const char *e = getenv("T_CONSTELLATION_TOUCH_EKF_BIAS_STEP");
			return e != nullptr && atof(e) > 0.0 ? atof(e) : kAccelBiasStep;
		}();
		return v;
	}
	static constexpr double kGyroBiasStep = 8.7266e-5;
	static constexpr int64_t kHistoryNs = 300 * U_TIME_1MS_IN_NS;

	mutable std::mutex mutex_;
	bool valid_{false};
	int64_t last_imu_ns_{0};
	int64_t last_update_ns_{0};
	uint64_t resets_{0};
	Vec3 p_{Vec3::Zero()};
	Vec3 v_{Vec3::Zero()};
	Quat q_{Quat::Identity()};
	Vec3 ba_{Vec3::Zero()};
	Vec3 bg_{Vec3::Zero()};
	Vec3 last_gyro_{Vec3::Zero()};
	double motion_gyro_{0.0};
	double motion_accel_{0.0};
	double rest_accel_len_{9.80665};
	double accel_noise_{0.05};
	double gyro_noise_{0.002};
	Mat15 P_{Mat15::Identity() * 1e-2};
	std::deque<HistoryEntry> history_;
	uint32_t stats_pos_[4] = {0, 0, 0, 0};
	uint32_t stats_rot_[4] = {0, 0, 0, 0};
};
