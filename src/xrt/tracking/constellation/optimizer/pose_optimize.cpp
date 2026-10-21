// Copyright 2015, Philipp Zabel
// Copyright 2020-2023, Jan Schmidt
// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PnP pose optimization using ceres.
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jan Schmidt <jan@centricular.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "util/u_debug.h"
#include "util/u_logging.h"

#include "tracking/t_camera_models.hpp"
#include "tracking/t_constellation.h"

#include "math/m_api.h"
#include "math/m_eigen_interop.hpp"
#include "math/m_quatexpmap.hpp"
#include "math/m_quatexpmap_bigceres.hpp"

#include "constellation/camera_model.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "ceres/autodiff_cost_function.h"
#include "ceres/problem.h"
#include "ceres/solver.h"

#include "pose_optimize.hpp"
#include "ransac.hpp"
#include "internal_math.hpp"

#include <iostream>
#include <stdio.h>


namespace {

//! Run a RANSAC inlier optimization before the final optimization.
constexpr bool kRunRansac = true;

/*
 * A Touch's ring seen from a few metres is close to a flat target, and the LEDs of a flat target fit two poses: the
 * right one and its mirror, tilted 25-45 degrees about the ring's own axis, both to a tenth of a pixel. RANSAC builds
 * its pose from random LED triples and lands on either, and the refinement then stays where it started. The tested
 * left controller, held still at arm's length, came out mirrored on about every other frame from its better camera,
 * and the IMU's gravity check then threw those frames away. The pose handed in is a prior or a
 * search candidate that passed the gravity check, so it is refined too, and its result kept when it fits about as
 * well as RANSAC's.
 */
DEBUG_GET_ONCE_BOOL_OPTION(constellation_tracker_keep_given_branch, "CONSTELLATION_TRACKER_KEEP_GIVEN_BRANCH", true)
//! RANSAC's pose has to turn at least this far from the given one before the given one is refined as well.
constexpr float kGivenBranchMinAngleRad = 3.0f * static_cast<float>(M_PI) / 180.0f;
//! How much worse the given pose's fit may be, as RMS pixels per LED, and still be kept.
constexpr double kGivenBranchSlackPx = 0.25;

/*
 * A Touch seen nearly edge-on shows 4-5 LEDs in a short row: they pin its position, but its tilt only weakly, and the
 * refinement tilted a left controller 10-20 degrees off on the camera at that side (in a recording, 1200 of its 1800
 * rejected solves). Its IMU knows which way is down to a couple of degrees. So, as a filter fusing the
 * IMU would, the refinement weighs the IMU's down too, with this standard deviation: the LEDs then set the position
 * and the heading, the IMU the tilt, and the mirror pose above loses on its 25-45 degrees of tilt. 0 turns it off.
 * Only callers with a prior pass the IMU's down: the slow search relies on the gravity check to turn wrong LED sets
 * away, which a refinement held upright would get past.
 */
DEBUG_GET_ONCE_FLOAT_OPTION(constellation_tracker_gravity_prior_deg, "CONSTELLATION_TRACKER_GRAVITY_PRIOR_DEG", 3.0f)
//! Only for poses from this many LEDs or fewer: from more the LEDs hold the tilt better than the IMU's estimate.
DEBUG_GET_ONCE_NUM_OPTION(constellation_tracker_gravity_prior_max_leds,
                          "CONSTELLATION_TRACKER_GRAVITY_PRIOR_MAX_LEDS",
                          8)

using namespace xrt::auxiliary::math;
using namespace xrt::auxiliary::tracking::camera_models;
using namespace xrt::tracking::constellation;
using namespace xrt::tracking::constellation::optimizer;

// @todo tune this number
#define BAD_COVARIANCE_MATRIX (PoseStateCovarianceMatrix::Identity() * 1e6)

struct PnPOptimizeCostFunctor
{
private: // Fields
	uint32_t num_leds;
	const std::vector<Eigen::Vector2f> &blob_positions;
	const std::vector<Eigen::Vector3f> &T_model_leds;
	const camera_model &params;

public: // Methods
	PnPOptimizeCostFunctor(uint32_t num_leds,
	                       const std::vector<Eigen::Vector2f> &blob_positions,
	                       const std::vector<Eigen::Vector3f> &T_model_leds,
	                       const camera_model &params)
	    : num_leds(num_leds), blob_positions(blob_positions), T_model_leds(T_model_leds), params(params)
	{}

	int
	numResiduals() const
	{
		// X and Y for each LED. Ceres likes it's ints.
		return static_cast<int>(this->num_leds * kNumLedResiduals);
	}

	template <typename T>
	bool
	operator()(const T *const parameters, T *residuals) const
	{
		const auto pose = Pose<T>(Eigen::Map<const Eigen::Vector<T, kPoseStateSize>>(parameters));

		for (uint32_t i = 0; i < this->num_leds; i++) {
			const Eigen::Vector2<T> blob_position_2d = this->blob_positions[i].cast<T>();
			const Eigen::Vector3<T> T_model_led = this->T_model_leds[i].cast<T>();

			Eigen::Map<Eigen::Vector2<T>> residual(&residuals[i * kNumLedResiduals]);

			computeLedResidual(this->params,     //
			                   pose.translation, //
			                   pose.rotation,    //
			                   blob_position_2d, //
			                   T_model_led,      //
			                   residual);        //
		}

		return true;
	}
};

typedef ceres::AutoDiffCostFunction<PnPOptimizeCostFunctor, ceres::DYNAMIC, kPoseStateSize> CostFunction;

//! The device's down, turned into the camera by the pose, against the camera's down; see the gravity prior above.
struct GravityPriorCostFunctor
{
	Eigen::Vector3d camera_gravity;
	Eigen::Vector3d device_gravity;
	//! Pixels per radian: an LED's blob sigma over the IMU's tilt sigma, so both weigh in as the same unit.
	double weight;

	template <typename T>
	bool
	operator()(const T *const parameters, T *residuals) const
	{
		const auto pose = Pose<T>(Eigen::Map<const Eigen::Vector<T, kPoseStateSize>>(parameters));
		const Eigen::Vector3<T> posed = pose.rotation * this->device_gravity.cast<T>();
		Eigen::Map<Eigen::Vector3<T>> residual(residuals);
		residual = (posed - this->camera_gravity.cast<T>()) * T(this->weight);
		return true;
	}
};

typedef ceres::AutoDiffCostFunction<GravityPriorCostFunctor, 3, kPoseStateSize> GravityCostFunction;

uint32_t
pickLabelledBlobs(t_blob *blobs,
                  uint32_t num_blobs,
                  t_constellation_tracker_led_model *leds_model,
                  t_constellation_device_id_t device_id,
                  std::vector<Eigen::Vector2f> &points2d,
                  std::vector<Eigen::Vector3f> &points3d,
                  std::vector<t_constellation_led_id_it> &led_ids,
                  std::vector<Eigen::Vector3f> &normals3d,
                  std::vector<t_blob *> &inlier_blobs)
{
	uint64_t taken = 0;
	uint32_t num_leds = 0;
	for (uint32_t i = 0; i < num_blobs; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// Invalid or LED id for another object
		if (blob_device_id != device_id) {
			continue;
		}

		// Labelled with an id this model doesn't have
		int32_t led_index = findLedIndexById(leds_model, blob_led_id);
		if (led_index < 0) {
			continue;
		}

		// If this LED is already taken by another blob, skip it. Keyed on the model index rather than the id,
		// since the mask only has room for MAX_OBJECT_LEDS entries and ids need not fit in that range.
		if (taken & (1ULL << led_index)) {
			continue;
		}

		// Mark this LED as taken by this blob.
		taken |= (1ULL << led_index);
		num_leds++;
	}

	points2d.reserve(num_leds);
	points3d.reserve(num_leds);
	led_ids.reserve(num_leds);
	normals3d.reserve(num_leds);
	inlier_blobs.reserve(num_leds);

	taken = 0;
	num_leds = 0;
	for (uint32_t i = 0; i < num_blobs; i++) {
		t_constellation_device_id_t blob_device_id = blobs[i].matched_device_id;
		t_constellation_led_id_it blob_led_id = blobs[i].matched_device_led_id;

		// Invalid or LED id for another object
		if (blob_device_id != device_id) {
			continue;
		}

		// Labelled with an id this model doesn't have
		int32_t led_index = findLedIndexById(leds_model, blob_led_id);
		if (led_index < 0) {
			continue;
		}

		// If this LED is already taken by another blob, skip it.
		if (taken & (1ULL << led_index)) {
			continue;
		}

		t_constellation_tracker_led &led = leds_model->leds[led_index];

		// Get the center we're using
		xrt_vec2 center;
		if constexpr (kOptimizeUndistortedPoints) {
			center = blobs[i].center_undistorted;
		} else {
			center = blobs[i].center_distorted;
		}

		// We have a valid LED, add it to the optimization.
		points2d.push_back({center.x, center.y});
		points3d.push_back({led.position.x, led.position.y, led.position.z});
		led_ids.push_back(led.id);
		normals3d.push_back({led.normal.x, led.normal.y, led.normal.z});
		inlier_blobs.push_back(&blobs[i]);

		// Mark this LED as taken by this blob.
		taken |= (1ULL << led_index);
		num_leds++;
	}

	return num_leds;
}

void
setupProblem(Eigen::Vector<double, kPoseStateSize> &solver_params,
             PoseManifold &pose_manifold,
             CostFunction &cost_function,
             ceres::Solver::Options &out_options,
             ceres::Problem &out_problem)
{
	ceres::Problem::Options problem_options{};
	problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
	problem_options.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

	ceres::Problem problem(problem_options);
	problem.AddParameterBlock(solver_params.data(), kPoseStateSize, &pose_manifold);
	problem.AddResidualBlock(&cost_function, nullptr, solver_params.data());

	ceres::Solver::Options options{};
	options.max_num_iterations = 100;
	options.linear_solver_type = ceres::DENSE_QR;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	options.num_threads = 1;
	options.logging_type = ceres::SILENT;
	options.minimizer_progress_to_stdout = false;

	out_options = std::move(options);
	out_problem = std::move(problem);
}

bool
computeCovariance(uint32_t num_residuals,
                  double rss,
                  const PoseStateCovarianceMatrix &H,
                  Eigen::Ref<PoseStateCovarianceMatrix> out_covariance)
{
	assert(num_residuals > kPoseCovarianceSize);

	const double sigma2 = rss / static_cast<double>(num_residuals - kPoseCovarianceSize);
	const double sigma = std::max(std::sqrt(sigma2), kBlobPositionSigmaPixels);

	Eigen::LLT<PoseStateCovarianceMatrix> llt(H);
	if (llt.info() != Eigen::Success) {
		return false;
	}

	out_covariance = (sigma * sigma) * llt.solve(PoseStateCovarianceMatrix::Identity());

	if (!out_covariance.allFinite()) {
		return false;
	}

	return true;
}

bool
computeProblemHessian(ceres::Problem &problem,
                      uint32_t &out_num_residuals,
                      double &out_rss,
                      PoseStateCovarianceMatrix &out_H)
{
	ceres::CRSMatrix jacobian_crs;
	double cost = 0.0;

	if (!problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, nullptr, nullptr, &jacobian_crs)) {
		return false;
	}

	assert(jacobian_crs.num_cols == kPoseCovarianceSize);

	Eigen::MatrixXd J = Eigen::MatrixXd::Zero(jacobian_crs.num_rows, jacobian_crs.num_cols);
	for (int row = 0; row < jacobian_crs.num_rows; row++) {
		for (int i = jacobian_crs.rows[row]; i < jacobian_crs.rows[row + 1]; i++) {
			J(row, jacobian_crs.cols[i]) = jacobian_crs.values[i];
		}
	}

	out_num_residuals = static_cast<uint32_t>(jacobian_crs.num_rows);
	// Ceres' cost is 0.5 * ||r||^2, so twice it is the residual sum of squares computeCovariance wants.
	out_rss = cost * 2.0;
	out_H = J.transpose() * J;

	return true;
}

}; // namespace

namespace xrt::tracking::constellation::optimizer {

bool
optimizePose(u_logging_level log_level,
             bool deterministic,
             const camera_model &params,
             xrt_pose init_pose,
             t_blob *blobs,
             uint32_t num_blobs,
             t_constellation_tracker_led_model *leds_model,
             t_constellation_device_id_t device_id,
             xrt_pose &out_pose,
             RawPoseCovarianceMatrix out_covariance,
             const xrt_vec3 *camera_gravity,
             const xrt_vec3 *device_gravity)
{
	std::vector<Eigen::Vector2f> points2d;
	std::vector<Eigen::Vector3f> points3d;
	std::vector<t_constellation_led_id_it> led_ids;
	std::vector<Eigen::Vector3f> normals3d;
	std::vector<t_blob *> inlier_blobs;

	uint32_t num_leds = pickLabelledBlobs(blobs,         //
	                                      num_blobs,     //
	                                      leds_model,    //
	                                      device_id,     //
	                                      points2d,      //
	                                      points3d,      //
	                                      led_ids,       //
	                                      normals3d,     //
	                                      inlier_blobs); //

	// We can't do an optimization with so few LEDs.
	if (num_leds < 4) {
		U_LOG_IFL_D(log_level, "Not enough LEDs for PnP optimization: %d", num_leds);
		return false;
	}

	// We only should run RANSAC if we have more than four labelled LEDs (and it's enabled)
	bool run_ransac = num_leds > 4 && kRunRansac;

	const xrt_pose given_pose = init_pose;

	if (run_ransac) {
		std::vector<size_t> ransac_inlier_indices;
		xrt_pose computed_ransac_pose;

		bool success = ransacPose(deterministic,         //
		                          params,                //
		                          inlier_blobs,          //
		                          points3d,              //
		                          normals3d,             //
		                          ransac_inlier_indices, //
		                          computed_ransac_pose); //
		if (success) {
			init_pose = computed_ransac_pose;

			uint32_t original_num_leds = num_leds;

			// Iterate over all input inlier blobs and mark them as unlabelled (outliers)
			for (auto blob : inlier_blobs) {
				blob->matched_device_id = XRT_CONSTELLATION_INVALID_DEVICE_ID;
				blob->matched_device_led_id = XRT_CONSTELLATION_INVALID_LED_ID;
			}

			// Update the points2d and points3d to only include the inliers from RANSAC.
			std::vector<Eigen::Vector2f> inlier_points2d;
			inlier_points2d.reserve(ransac_inlier_indices.size());
			std::vector<Eigen::Vector3f> inlier_points3d;
			inlier_points3d.reserve(ransac_inlier_indices.size());

			for (size_t index : ransac_inlier_indices) {
				inlier_points2d.push_back(points2d[index]);
				inlier_points3d.push_back(points3d[index]);

				auto inlier_blob = inlier_blobs[index];

				// Mark the blob as an inlier
				inlier_blob->matched_device_id = device_id;
				inlier_blob->matched_device_led_id = led_ids[index];
			}

			points2d = std::move(inlier_points2d);
			points3d = std::move(inlier_points3d);
			num_leds = static_cast<uint32_t>(points2d.size());

			U_LOG_IFL_D(log_level, "RANSAC found a valid pose for device %d with %d LEDs (originally %d)",
			            device_id, num_leds, original_num_leds);
		} else {
			U_LOG_IFL_D(log_level, "RANSAC failed for device %d with %d LEDs", device_id, num_leds);
		}
	}

	// Assert that RANSAC hasn't put us below 4 LEDs
	assert(num_leds >= 4);

	auto optimize_cost_functor = PnPOptimizeCostFunctor(num_leds, points2d, points3d, params);

	// Initialize our solver with the pose
	Eigen::Vector<double, kPoseStateSize> solver_params;
	Pose<double>(init_pose).pack(solver_params);

	CostFunction cost_function = {
	    &optimize_cost_functor,
	    optimize_cost_functor.numResiduals(),
	    ceres::DO_NOT_TAKE_OWNERSHIP,
	};
	PoseManifold pose_manifold;

	// The IMU's down, for a pose from few LEDs, see constellation_tracker_gravity_prior_deg.
	const float gravity_sigma_rad =
	    static_cast<float>(DEG_TO_RAD(debug_get_float_option_constellation_tracker_gravity_prior_deg()));
	const bool use_gravity =
	    camera_gravity != nullptr && device_gravity != nullptr && gravity_sigma_rad > 0.0f &&
	    num_leds <= static_cast<uint32_t>(debug_get_num_option_constellation_tracker_gravity_prior_max_leds());
	GravityPriorCostFunctor gravity_functor = {};
	if (use_gravity) {
		gravity_functor.camera_gravity = map_vec3(*camera_gravity).cast<double>().normalized();
		gravity_functor.device_gravity = map_vec3(*device_gravity).cast<double>().normalized();
		gravity_functor.weight = kBlobPositionSigmaPixels / gravity_sigma_rad;
	}
	GravityCostFunction gravity_cost_function(&gravity_functor, ceres::DO_NOT_TAKE_OWNERSHIP);

	ceres::Solver::Options options;
	ceres::Problem problem;
	setupProblem(solver_params, pose_manifold, cost_function, options, problem);
	if (use_gravity) {
		problem.AddResidualBlock(&gravity_cost_function, nullptr, solver_params.data());
	}

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	bool solved = true;
	switch (summary.termination_type) {
	case ceres::CONVERGENCE:
	case ceres::USER_SUCCESS:
		U_LOG_IFL_D(log_level, "PnP optimization converged for device %d with %d LEDs (%s)", device_id,
		            num_leds, summary.message.c_str());
		break;
	case ceres::NO_CONVERGENCE:
		U_LOG_IFL_D(log_level, "PnP optimization hit max iterations (%d) for device %d with %d LEDs",
		            options.max_num_iterations, device_id, num_leds);
		break;
	default:
		U_LOG_IFL_E(log_level, "PnP optimization failed for device %d with %d LEDs: %s", device_id, num_leds,
		            summary.message.c_str());
		solved = false;
		break;
	}
	U_LOG_IFL_D(log_level, "Initial cost: %f\tFinal cost: %f", summary.initial_cost, summary.final_cost);

	solved = solved && solver_params.allFinite() && summary.IsSolutionUsable();

	// The given pose's side of a mirror pair, see kGivenBranchSlackPx.
	Eigen::Vector<double, kPoseStateSize> given_params;
	ceres::Solver::Options given_options;
	ceres::Problem given_problem;
	ceres::Solver::Summary given_summary;
	bool use_given = false;
	if (debug_get_bool_option_constellation_tracker_keep_given_branch() &&
	    map_quat(given_pose.orientation).angularDistance(map_quat(init_pose.orientation)) >
	        kGivenBranchMinAngleRad) {
		Pose<double>(given_pose).pack(given_params);
		setupProblem(given_params, pose_manifold, cost_function, given_options, given_problem);
		if (use_gravity) {
			given_problem.AddResidualBlock(&gravity_cost_function, nullptr, given_params.data());
		}
		ceres::Solve(given_options, &given_problem, &given_summary);

		const bool given_solved = (given_summary.termination_type == ceres::CONVERGENCE ||
		                           given_summary.termination_type == ceres::USER_SUCCESS ||
		                           given_summary.termination_type == ceres::NO_CONVERGENCE) &&
		                          given_params.allFinite() && given_summary.IsSolutionUsable();

		// Ceres' cost is half the sum of squares over both axes of every LED.
		const double ransac_rms_px = std::sqrt(2.0 * summary.final_cost / num_leds);
		const double given_rms_px = std::sqrt(2.0 * given_summary.final_cost / num_leds);
		// With the IMU's down weighed in, the cost tells the mirror poses apart by itself.
		use_given =
		    given_solved && (!solved || (use_gravity ? given_summary.final_cost <= summary.final_cost
		                                             : given_rms_px <= ransac_rms_px + kGivenBranchSlackPx));

		U_LOG_IFL_D(log_level,
		            "Device %d: RANSAC's pose is %.1f degrees from the given one, fits %.2f px against %.2f px "
		            "from the given one, keeping %s",
		            device_id,
		            map_quat(given_pose.orientation).angularDistance(map_quat(init_pose.orientation)) * 180.0 /
		                M_PI,
		            ransac_rms_px, given_rms_px, use_given ? "the given one" : "RANSAC's");
	}

	if (!solved && !use_given) {
		return false;
	}

	ceres::Problem &chosen_problem = use_given ? given_problem : problem;

	// Unpack the parameters
	out_pose = Pose<double>(use_given ? given_params : solver_params).toXrtPose();

	if (out_covariance != nullptr) {
		uint32_t num_residuals;
		double rss;
		PoseStateCovarianceMatrix H;

		Eigen::Map<PoseStateCovarianceMatrix> covariance(out_covariance);
		if (!computeProblemHessian(chosen_problem, num_residuals, rss, H) || //
		    !computeCovariance(num_residuals, rss, H, covariance)) {         //
			covariance = BAD_COVARIANCE_MATRIX;
		}
	}

	return true;
}

void
computePoseCovariance(u_logging_level log_level,
                      const camera_model &params,
                      xrt_pose init_pose,
                      t_blob *blobs,
                      uint32_t num_blobs,
                      t_constellation_tracker_led_model *leds_model,
                      t_constellation_device_id_t device_id,
                      RawPoseCovarianceMatrix out_covariance)
{
	Eigen::Map<PoseStateCovarianceMatrix> covariance(out_covariance);

	std::vector<Eigen::Vector2f> points2d;
	std::vector<Eigen::Vector3f> points3d;
	std::vector<t_constellation_led_id_it> led_ids;
	std::vector<Eigen::Vector3f> normals3d;
	std::vector<t_blob *> inlier_blobs;

	uint32_t num_leds = pickLabelledBlobs(blobs,         //
	                                      num_blobs,     //
	                                      leds_model,    //
	                                      device_id,     //
	                                      points2d,      //
	                                      points3d,      //
	                                      led_ids,       //
	                                      normals3d,     //
	                                      inlier_blobs); //

	// sigma2 = rss / (num_residuals - kPoseCovarianceSize), if num_residuals is less than 8 (num_leds * 2), then we
	// can't compute a covariance matrix, so we return a bad covariance matrix.
	if (num_leds < 4) {
		covariance = BAD_COVARIANCE_MATRIX;
		return;
	}

	auto optimize_cost_functor = PnPOptimizeCostFunctor(num_leds, points2d, points3d, params);

	Eigen::Vector<double, kPoseStateSize> solver_params;
	Pose<double>(init_pose).pack(solver_params);

	CostFunction cost_function = {
	    &optimize_cost_functor,
	    optimize_cost_functor.numResiduals(),
	    ceres::DO_NOT_TAKE_OWNERSHIP,
	};
	PoseManifold pose_manifold;

	ceres::Solver::Options options;
	ceres::Problem problem;
	setupProblem(solver_params, pose_manifold, cost_function, options, problem);

	uint32_t num_residuals;
	double rss;
	PoseStateCovarianceMatrix H;

	if (!computeProblemHessian(problem, num_residuals, rss, H) || //
	    !computeCovariance(num_residuals, rss, H, covariance)) {  //
		covariance = BAD_COVARIANCE_MATRIX;
	}
}

void
covariancePositionRadius(const RawPoseCovarianceMatrix &raw_covariance, xrt_vec3 &radii, xrt_quat &Q_cam_cov)
{
	Eigen::Map<const PoseStateCovarianceMatrix> covariance(raw_covariance);

	const Eigen::Matrix3d &pos_covariance =
	    covariance.block<3, 3>(static_cast<int>(CovarianceIndex::PosX), static_cast<int>(CovarianceIndex::PosX));

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(pos_covariance);

	if (solver.info() != Eigen::Success) {
		// @todo handle failure better, if possible
		U_LOG_E("Failed to compute eigenvalues for position covariance matrix.");
		radii = {0.0f, 0.0f, 0.0f};
		Q_cam_cov = {0.0f, 0.0f, 0.0f, 1.0f};
		return;
	}

	const Eigen::Vector3d &eigenvalues = solver.eigenvalues();
	Eigen::Matrix3d eigenvectors = solver.eigenvectors();

	if (eigenvectors.determinant() < 0.0) {
		eigenvectors.col(2) *= -1.0;
	}

	Eigen::Quaterniond q(eigenvectors);
	map_quat(Q_cam_cov) = q.cast<float>();

	Eigen::Vector3d half_sizes = 3.0 * eigenvalues.cwiseMax(0.0).cwiseSqrt();
	map_vec3(radii) = half_sizes.cast<float>();
}

}; // namespace xrt::tracking::constellation::optimizer
