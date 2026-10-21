// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Whether the tracker's room is level, see level_watch.hpp.
 * @ingroup aux_tracking
 */

#include "level_watch.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <algorithm>
#include <cmath>

namespace xrt::tracking::constellation {

namespace {

Eigen::Matrix3d
skew(const Eigen::Vector3d &v)
{
	Eigen::Matrix3d m;
	m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
	return m;
}

Eigen::Matrix3d
rot(const xrt_quat &q)
{
	return Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
}

double
angle_between(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
	return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0));
}

} // namespace

void
LevelWatch::reset()
{
	*this = LevelWatch{};
}

bool
LevelWatch::add(uint32_t camera_key, const xrt_quat &Rcv_world_device, const xrt_vec3 &imu_down_cv, LevelResult &out)
{
	// The sensor's slot in this window; sensors beyond the slots are left out.
	size_t slot = this->key_count;
	for (size_t i = 0; i < this->key_count; i++) {
		if (this->keys[i] == camera_key) {
			slot = i;
			break;
		}
	}
	if (slot == this->key_count) {
		if (this->key_count >= kMaxCameras) {
			return false;
		}
		this->keys[slot] = camera_key;
		this->key_solves[slot] = 0;
		this->key_count++;
	}

	const Eigen::Matrix3d R = rot(Rcv_world_device);
	const Eigen::Vector3d d = Eigen::Vector3d(imu_down_cv.x, imu_down_cv.y, imu_down_cv.z).normalized();
	const Eigen::Vector3d e(0.0, 1.0, 0.0); // down in the room, OpenCV

	/*
	 * R (d + m x d) = e + t x e, all rotations small: m the IMU's turn in the device, t this sensor's tilt (where the
	 * IMU's down is in the room as this sensor's place gives it). Linear: R d - e = R [d]x m - [e]x t.
	 */
	const size_t tcol = 3 + 3 * slot;
	Eigen::Matrix<double, 3, kParams> A = Eigen::Matrix<double, 3, kParams>::Zero();
	A.block<3, 3>(0, 0) = R * skew(d);
	A.block<3, 3>(0, tcol) = -skew(e);
	const Eigen::Vector3d b = R * d - e;
	// Only the IMU's columns and this sensor's are non-zero.
	const size_t cols[6] = {0, 1, 2, tcol, tcol + 1, tcol + 2};
	for (size_t i : cols) {
		for (size_t j : cols) {
			this->ata[i][j] += A.col(i).dot(A.col(j));
		}
		this->atb[i] += A.col(i).dot(b);
	}
	this->key_solves[slot]++;

	// Which way it faces about the vertical: its forward (OpenCV +z) on the floor.
	const Eigen::Vector3d fwd = R * Eigen::Vector3d(0, 0, 1);
	if (std::hypot(fwd.x(), fwd.z()) > 0.3) {
		const double yaw = std::atan2(fwd.x(), fwd.z());
		int bin = (int)std::floor((yaw + M_PI) / (2.0 * M_PI) * 36.0);
		this->yaw_bins[(bin % 36 + 36) % 36] = true;
	}
	if (this->count % 4 == 0 && this->kept < kKept) {
		this->kept_r[this->kept] = Rcv_world_device;
		this->kept_d[this->kept] = imu_down_cv;
		this->kept_slot[this->kept] = (uint8_t)slot;
		this->kept++;
	}
	if (++this->count < kWindow) {
		return false;
	}

	Eigen::Matrix<double, kParams, kParams> N;
	Eigen::Matrix<double, kParams, 1> v;
	for (size_t i = 0; i < kParams; i++) {
		for (size_t j = 0; j < kParams; j++) {
			N(i, j) = this->ata[i][j];
		}
		v(i) = this->atb[i];
	}
	// The turns about each down are not seen (they change no down), nor are unseen sensors': kept at zero.
	N += Eigen::Matrix<double, kParams, kParams>::Identity() * (1e-3 * (double)this->count);
	const Eigen::Matrix<double, kParams, 1> x = N.fullPivLu().solve(v);
	const Eigen::Vector3d m = x.head<3>();
	auto tilt_of = [&](size_t s) -> Eigen::Vector3d { return x.segment<3>(3 + 3 * s); };

	// The sensors that take part, and their common tilt (weighted by their solves).
	Eigen::Vector3d common = Eigen::Vector3d::Zero();
	size_t weight = 0;
	out = LevelResult{};
	for (size_t s = 0; s < this->key_count; s++) {
		if (this->key_solves[s] < kMinCameraSolves) {
			continue;
		}
		common += tilt_of(s) * (double)this->key_solves[s];
		weight += this->key_solves[s];
		const size_t c = out.cameras++;
		out.camera_key[c] = this->keys[s];
		out.camera_solves[c] = this->key_solves[s];
		out.camera_tilt_deg[c] = (float)(tilt_of(s).norm() * 180.0 / M_PI);
	}
	if (weight > 0) {
		common /= (double)weight;
	}
	double spread = 0.0;
	for (size_t s = 0; s < this->key_count; s++) {
		if (this->key_solves[s] >= kMinCameraSolves) {
			spread = std::max(spread, (tilt_of(s) - common).norm());
		}
	}

	// The fit's leftover over the kept solves.
	auto residual_of = [&](const Eigen::Matrix<double, kParams, 1> &sol) -> double {
		const Eigen::Vector3d ms = sol.head<3>();
		double sum = 0.0;
		size_t used = 0;
		for (size_t i = 0; i < this->kept; i++) {
			const size_t s = this->kept_slot[i];
			if (this->key_solves[s] < kMinCameraSolves) {
				continue;
			}
			const Eigen::Matrix3d Ri = rot(this->kept_r[i]);
			const Eigen::Vector3d di =
			    Eigen::Vector3d(this->kept_d[i].x, this->kept_d[i].y, this->kept_d[i].z).normalized();
			const Eigen::Vector3d predicted = Ri * (di + ms.cross(di));
			const Eigen::Vector3d room = e + Eigen::Vector3d(sol.segment<3>(3 + 3 * s)).cross(e);
			sum += std::pow(angle_between(predicted, room), 2);
			used++;
		}
		return used > 0 ? std::sqrt(sum / (double)used) * 180.0 / M_PI : 0.0;
	};
	size_t bins = 0;
	for (bool bin : this->yaw_bins) {
		bins += bin ? 1 : 0;
	}

	// The room is level once turned back by the common tilt: apply -t to every sensor's place.
	out.level_rotvec = {(float)-common.x(), (float)-common.y(), (float)-common.z()};
	out.tilt_deg = (float)(common.norm() * 180.0 / M_PI);
	out.spread_deg = (float)(spread * 180.0 / M_PI);
	out.mount_deg = (float)(m.norm() * 180.0 / M_PI);
	out.yaw_spread_deg = (float)bins * 10.0f;
	out.residual_deg = (float)residual_of(x);
	out.solves = this->count;
	this->reset();
	return true;
}

} // namespace xrt::tracking::constellation
