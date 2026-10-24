// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal ures for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "util/u_debug.h"
#include "util/u_logging.h"
#include "util/u_var.h"
#include "util/u_threading.h"
#include "util/u_weak_ptr.hpp"
#include "util/u_time.h"
#include "util/u_sink.h"
#include "util/u_frame.h"
#include "util/u_frame_scribble.h"

#include "tracking/t_constellation.h"

#include "math/m_api.h"

#include <atomic>
#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <stdexcept>
#include <array>
#include <deque>
#include <map>
#include <unordered_map>
#include <tuple>
#include <fstream>

#include "correspondence_search.h"
#include "led_search_model.h"
#include "pose_metrics.h"
#include "t_constellation_tracker.h"
#include "optimizer/pose_optimize.hpp"
#include "optimizer/rig_calibration.hpp"
#include "optimizer/sensor_fusion.hpp"
#include "loose_part.hpp"
#include "level_watch.hpp"


#define CT_TRACE(ct, ...) U_LOG_IFL_T(ct->log_level, __VA_ARGS__)
#define CT_DEBUG(ct, ...) U_LOG_IFL_D(ct->log_level, __VA_ARGS__)
#define CT_INFO(ct, ...) U_LOG_IFL_I(ct->log_level, __VA_ARGS__)
#define CT_WARN(ct, ...) U_LOG_IFL_W(ct->log_level, __VA_ARGS__)
#define CT_ERROR(ct, ...) U_LOG_IFL_E(ct->log_level, __VA_ARGS__)

#define MIN_ROT_ERROR DEG_TO_RAD(30)
#define MIN_POS_ERROR 0.10

/*
 *
 * Forward declares for C callback functions
 *
 */

extern "C" void
constellation_tracker_camera_push_blobs(t_blob_sink *tbs, t_blob_observation *tbo);

extern "C" void
constellation_tracker_camera_destroy(t_blob_sink *tbs);

extern "C" void *
constellation_tracker_camera_slow_thread(void *ptr);

extern "C" void *
constellation_tracker_camera_fast_thread(void *ptr);

extern "C" void
constellation_tracker_node_break_apart(xrt_frame_node *node);

extern "C" void
constellation_tracker_node_destroy(xrt_frame_node *node);


namespace xrt::tracking::constellation {

using namespace xrt::tracking::constellation::optimizer;

namespace os = xrt::auxiliary::os;

// Forward-declares
struct Camera;
struct CameraMosaic;
struct ConstellationTracker;
struct DataRecorder;
struct Device;

//! Which path through the tracker produced a device pose. Recorded with every solve, see @ref DataRecorder.
enum class SolveSource : uint8_t
{
	//! Fast thread, the pose predicted by the device's own tracking, checked against the blobs.
	FastPredicted = 0,
	//! Fast thread, RANSAC over blobs that kept their labels from earlier frames.
	FastBlobRecovery = 1,
	//! Fast thread, the last pose the tracker solved, checked against the blobs.
	FastLastKnown = 2,
	//! Slow thread, shallow correspondence search.
	SlowShallowSearch = 3,
	//! Slow thread, deep correspondence search.
	SlowDeepSearch = 4,
	//! Fast thread, the predicted pose moved across the image to where its LEDs' blobs are, then checked.
	FastPredictedShifted = 5,
	//! Fast thread, the prior's orientation kept and only its position solved, from as few as 2 LEDs.
	FastFewLeds = 6,
	//! Fast thread, a headset no sensor sees: where it was last seen, turned as it is held, see tryDevicePoseHeldStill.
	FastHeldStill = 7,
};

struct FoundDevicePose
{
	xrt_pose Tcv_cam_device XRT_POSE_IDENTITY;
	float average_blob_brightness;
	RawPoseCovarianceMatrix covariance;

	/*!
	 * Whether a found pose was already handled, this is present so that if a single sample is torn between some
	 * devices fast recovering and some devices slow recovering, we push the fast recovery poses the moment they're
	 * found rather than at the end of the slow recovery where the sample is "terminated".
	 */
	bool handled{false};
};

struct DeviceState
{
	//! The ID of the device.
	t_constellation_device_id_t device_id{XRT_CONSTELLATION_INVALID_DEVICE_ID};

	//! The predicted pose of the device in the world at the time of the sample.
	std::optional<xrt_pose> Txr_world_device_prior{std::nullopt};
	//! The predicted pose of the device relative to the camera at the time of the sample.
	std::optional<xrt_space_relation> Tcv_cam_device_predicted{std::nullopt};

	/*!
	 * Whether the prior's position is tracked. A device that lost the cameras may still hand over a prior with only
	 * its orientation tracked, by its IMU, and its position held where it was last seen, see
	 * @ref Camera::bindingPrior.
	 */
	bool prior_position_tracked{false};

	//! The final found pose of the device in this specific sample.
	std::optional<FoundDevicePose> found_pose{std::nullopt};

	//! Whether the device needs to have the slow processing thread run over it
	bool needs_slow_processing{false};
};

struct CameraSample
{
public: // Fields
	t_blobwatch *source{nullptr};
	uint64_t id{};
	uint64_t sequence_id;
	int64_t timestamp_ns{};
	t_blob blobs[XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME]{};
	uint32_t blob_count{};
	//! Per blob: a spot that has stayed put and unexplained for a while, see Camera::classifyStatic.
	std::array<bool, XRT_CONSTELLATION_MAX_BLOBS_PER_FRAME> blob_static{};

	//! The camera's position in the constellation tracker's tracking origin
	std::optional<xrt_pose> Txr_world_cam{std::nullopt};

	std::array<DeviceState, XRT_CONSTELLATION_MAX_DEVICES> device_states{};
	uint32_t device_count{};

	uint32_t mosaic_index;
	uint32_t camera_index;

public: // Methods
	std::optional<DeviceState *>
	getDeviceState(t_constellation_device_id_t device_id);

	DeviceState &
	putDeviceState(t_constellation_device_id_t device_id);

	CameraSample(t_blob_observation &blobservation, Camera *camera);

	CameraSample() = default;

	void
	markMatchingBlobs(ConstellationTracker *ct,
	                  t_constellation_tracker_led_model &led_model,
	                  t_constellation_device_id_t device_id,
	                  pose_metrics_blob_match_info &blob_match_info);

	t_blob_observation
	toBlobObservation() const &
	{
		t_blob_observation obs = {
		    .source = this->source,
		    .id = this->id,
		    .sequence_id = this->sequence_id,
		    .timestamp_ns = this->timestamp_ns,
		    .blobs = const_cast<t_blob *>(this->blobs),
		    .num_blobs = this->blob_count,
		};

		return obs;
	}

	t_blob_observation
	toBlobObservation() && = delete;
	t_blob_observation
	toBlobObservation() const && = delete;
};

struct CameraScribbleSettings
{
public: // Fields
	//! Whether to draw the blobs in the sample
	bool draw_blobs{true};
	//! Whether to draw the raw blob IDs
	bool draw_blob_ids{false};
	//! Whether to draw the device IDs on the blobs where possible
	bool draw_blob_device_ids{true};
	//! Whether to draw the LED IDs on the blobs where possible
	bool draw_blob_led_ids{false};

	//! Whether to draw the prior device pose in the sample, if it exists.
	bool draw_prior{false};
	//! Whether to draw the final device pose in the sample, if it exists.
	bool draw_found{false};

public: // Methods
	void
	setupDebugTracking(void *root);
};

/*!
 * Counts an event and hands back how often it happened at most once a second, so a warning that can fire every frame
 * becomes one line a second. Logging writes to stderr, which can block on the disk for tens of milliseconds, and the
 * tracking threads must not: writing a warning per frame dropped frames on both sensors of a CV1 every 5 seconds,
 * when the filesystem committed.
 */
struct RateLimitedCount
{
	std::atomic<uint64_t> count{0};
	std::atomic<int64_t> last_report_ns{0};

	//! Counts one event, returns how many to report, or 0 when it is not time to report yet.
	uint64_t
	tick(int64_t now_ns)
	{
		this->count.fetch_add(1);
		int64_t last_ns = this->last_report_ns.load();
		if (now_ns - last_ns < U_TIME_1S_IN_NS ||
		    !this->last_report_ns.compare_exchange_strong(last_ns, now_ns)) {
			return 0;
		}
		return this->count.exchange(0);
	}
};

struct OldOptimizationData
{
	RawPoseCovarianceMatrix covariance;
};

struct Camera
{
public: // Fields
	t_blob_sink base = {
	    .push_blobs = constellation_tracker_camera_push_blobs,
	    .destroy = constellation_tracker_camera_destroy,
	};

	//! The owner tracker, so we can retrieve it from the blob sink callback
	ConstellationTracker *tracker;
	std::weak_ptr<CameraMosaic> mosaic;

	t_camera_calibration calibration;

	camera_model model;

	CameraScribbleSettings scribble_settings{};

	//! The index in the mosaic
	size_t index;

	//! Identifies the physical camera across runs, empty if it has nothing to identify it by.
	std::string serial;

	//! Does "slow" processing for this camera when fast recovery paths fail.
	os_thread_helper slow_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		//! The search over the sample's blobs less the static spots, for searches with no prior.
		correspondence_search *cs_blind{nullptr};
		std::vector<t_blob> blind_blobs;
		//! Per device, when a search with no prior last took every blob, static spots too.
		std::array<timepoint_ns, XRT_CONSTELLATION_MAX_DEVICES> last_full_blind_ns{};
		uint32_t blind_searches{0};
		uint32_t blind_spots_left_out{0};

		u_sink_debug debug_sink{};
	} slow_processing_thread_data;

	/*!
	 * Spots in this camera's image that have stayed put and that no device explains: daylight and what it lights
	 * up, 36-87 per sensor in recorded morning sessions against none at night. Kept per blob of
	 * the blobwatch, which follows a blob from frame to frame by its ID. As Meta does for the Quest's "holiday
	 * lights", they are a label on the blob and not a mask: only searches with no prior leave them out, see
	 * classifyStatic. Only touched by whoever feeds the camera its samples, in order.
	 */
	struct StaticSpot
	{
		float x{0.0f};
		float y{0.0f};
		timepoint_ns since_ns{0};
		timepoint_ns last_ns{0};
	};
	std::unordered_map<uint32_t, StaticSpot> static_spots;
	timepoint_ns static_spots_pruned_ns{0};
	uint32_t static_last{0};

	/*!
	 * Does "fast" processing for this camera, trying to recover a pose quickly. It's valid for this to happen at
	 * the same time as a slow process.
	 */
	os_thread_helper fast_processing_thread{};
	struct
	{
		std::optional<CameraSample> sample{std::nullopt};

		correspondence_search *cs{nullptr};

		u_sink_debug debug_sink{};
	} fast_processing_thread_data;

	//! Locks all processing data
	mutable os::Mutex processing_lock;
	//! All data protected by the processing lock
	struct
	{
		xrt_pose Txr_origin_cam;
		bool has_concrete_pose;
	} locked_data;

	std::atomic<uint32_t> shallow_searches_failed{0};

	//! When this camera last had an accepted solve of each device, for the moved-camera watch's probe.
	std::array<std::atomic<int64_t>, XRT_CONSTELLATION_MAX_DEVICES> rig_watch_seen_ns{};
	//! When the probe last looked, slow thread only.
	timepoint_ns rig_watch_probe_ns{0};
	//! The probe's last find of the device, in this camera, as where the next look starts; slow thread only.
	std::optional<xrt_pose> rig_watch_probe_found{};
	timepoint_ns rig_watch_probe_found_ns{0};

	//! When the last blob observation arrived, monotonic, to warn about gaps in delivery.
	std::atomic<int64_t> last_push_ns{0};
	//! When this camera's first frame arrived, and whether its one gap at the start (see push_blobs) was seen.
	std::atomic<int64_t> first_push_ns{0};
	std::atomic<bool> start_gap_seen{false};

	//! Samples dropped because the fast or the slow thread was still busy with the previous one.
	RateLimitedCount dropped_fast_samples{};
	RateLimitedCount dropped_slow_samples{};

	//! Frames the slow thread never got to, it was still searching an earlier one; reported with @ref slow_stats.
	std::atomic<uint64_t> slow_skipped{0};
	//! How long the slow searches take, reported once a second by the slow thread.
	struct
	{
		int64_t window_start_ns{0};
		uint32_t runs{0};
		double total_ms{0.0};
		double max_ms{0.0};
	} slow_stats;

public: // Methods (t_constellation_tracker.cpp)
	static Camera *
	Get(t_blob_sink *tbs)
	{
		return container_of(tbs, Camera, base);
	}

	Camera(ConstellationTracker *tracker,
	       std::weak_ptr<CameraMosaic> mosaic,
	       const t_constellation_tracker_camera &camera_params,
	       enum u_logging_level *log_level_ptr,
	       size_t index);

	~Camera();

	// Delete all copy/move ctors, since the pointers for `base` need to be stable
	Camera(const Camera &) = delete;
	Camera(Camera &&) = delete;
	Camera &
	operator=(const Camera &) = delete;
	Camera &
	operator=(Camera &&) = delete;

	std::optional<xrt_pose>
	getWorldPose(timepoint_ns when_ns);

	void
	deferSampleToSlowThread(CameraSample &sample);

	//! Fast matching based on prior pose
	bool
	tryDevicePose(Device *device,
	              CameraSample &sample,
	              DeviceState &device_state,
	              const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
	              const xrt_pose &Tcv_cam_device_candidate,
	              SolveSource source);

	/*!
	 * Fast matching on a prior that is off by a few pixels: the prior moved across the image by the offset its
	 * projected LEDs have to the nearest blobs, when enough of them agree on it, then checked as @ref
	 * tryDevicePose.
	 */
	bool
	tryDevicePoseShifted(Device *device,
	                     CameraSample &sample,
	                     DeviceState &device_state,
	                     const std::optional<xrt_space_relation> &Tcv_cam_device_prior,
	                     SolveSource source = SolveSource::FastPredictedShifted);

	/*!
	 * A headset no sensor sees, held turning about the neck: the place it was last seen, with the held orientation,
	 * tried as a prior of its own, as is and shifted, see the implementation.
	 */
	bool
	tryDevicePoseHeldStill(Device *device,
	                       CameraSample &sample,
	                       DeviceState &device_state,
	                       const std::optional<xrt_pose> &Tcv_cam_world);

	//! Keeps a fresh prior's orientation and solves only its position, from 2 or 3 LEDs, see the implementation.
	bool
	tryDevicePoseFewLeds(Device *device,
	                     CameraSample &sample,
	                     DeviceState &device_state,
	                     const std::optional<xrt_space_relation> &Tcv_cam_device_prior);

	bool
	tryDeviceBlobRecovery(Device *device,
	                      CameraSample &sample,
	                      DeviceState &device_state,
	                      const std::optional<xrt_space_relation> &Tcv_cam_device_prior);

	void
	handleProcessingComplete(CameraSample &sample);

	//! Hands a solve that fits its blobs to the moved-camera watch, see ConstellationTracker::watchRig.
	void
	feedRigWatch(Device *device,
	             const CameraSample &sample,
	             const xrt_pose &Tcv_cam_device,
	             uint32_t matched_blobs,
	             float rms_px);

	//! The moved-camera watch's own look at a device this camera has lost, see the implementation.
	void
	probeForRigWatch(CameraSample &sample, Device *device);

	void
	processSampleSlow(CameraSample &sample);

	//! Returns whether a slow search is needed
	bool
	processSampleFast(CameraSample &sample);

	//! Marks the sample's blobs that are static spots, see @ref static_spots. Call for every sample, in order.
	void
	classifyStatic(CameraSample &sample);

	/*!
	 * Refines a found pose against the blobs and hands it on.
	 *
	 * @return false when the refined pose was rejected, see @ref poseMatchesGravity and @ref poseMatchesPrior.
	 */
	bool
	pushPose(CameraSample &camera_sample,
	         DeviceState &device_state,
	         Device *device,
	         pose_metrics &score,
	         const xrt_pose &Tcv_cam_device,
	         std::optional<OldOptimizationData *> was_optimized,
	         SolveSource source);

	/*!
	 * Whether the sample has a prior for the device that a solve must agree with, and how closely: a tracked one
	 * within @ref ConstellationTracker::prior_fresh_ns of the device's last solve, to @ref
	 * ConstellationTracker::prior_tolerance_pos; one with only its orientation tracked and its position held,
	 * within
	 * @ref ConstellationTracker::held_prior_ns (a headset's within @ref ConstellationTracker::prior_fresh_ns), to
	 * @ref ConstellationTracker::held_prior_tolerance_pos. Both to @ref ConstellationTracker::prior_tolerance_rot.
	 */
	bool
	bindingPrior(Device *device,
	             const CameraSample &sample,
	             const DeviceState &device_state,
	             const xrt_vec3 *&out_pos_tolerance,
	             const xrt_vec3 *&out_rot_tolerance) const;

	//! Whether a pose agrees with a binding prior, true when there is none.
	bool
	poseMatchesPrior(Device *device,
	                 const CameraSample &sample,
	                 const DeviceState &device_state,
	                 const xrt_pose &Tcv_cam_device,
	                 SolveSource source);

	//! Starts the device's fusion over when accepted solves keep disagreeing with it.
	void
	checkFusionAgrees(Device *device, timepoint_ns timestamp_ns, const xrt_pose &Txr_world_device);

	/*!
	 * The direction of gravity in this camera's frame and in the device's frame for a sample, OpenCV coordinates.
	 *
	 * @return false when either is unknown: the camera is not placed yet, the device has no IMU estimate, or the
	 *         check is turned off.
	 */
	bool
	gravityForSample(const Device *device,
	                 const CameraSample &sample,
	                 xrt_vec3 &out_camera_gravity,
	                 xrt_vec3 &out_device_gravity) const;

	//! Whether a candidate pose agrees with the device's IMU on which way is down, true when that is unknown.
	bool
	poseMatchesGravity(Device *device,
	                   const CameraSample &sample,
	                   const xrt_pose &Tcv_cam_device,
	                   SolveSource source);

public: // Methods (constellation_debug_scribble.cpp)
	void
	debugScribbleSample(CameraSample &sample, bool fast);
};

struct CameraMosaic
{
public: // Fields
	std::vector<std::unique_ptr<Camera>> cameras;
	//! The index of this mosaic in the tracker.
	size_t index;

	t_constellation_tracker_tracking_source *tracking_origin;

public: // Methods
	CameraMosaic(ConstellationTracker *tracker,
	             const t_constellation_tracker_camera_mosaic &mosaic_params,
	             size_t index);

	~CameraMosaic() = default;

	std::optional<xrt_pose>
	getTrackingOriginPose(timepoint_ns when_ns);
};

struct DeviceLastPose
{
public: // Fields
	xrt_pose Txr_world_device;
	xrt_pose Tcv_cam_device;
	timepoint_ns timestamp_ns;

public: // Methods
	DeviceLastPose(xrt_pose Txr_world_device, xrt_pose Tcv_cam_device, timepoint_ns timestamp_ns);
};

/*!
 * Which way is down for a device, from its own IMU alone: the gyro carries the direction along, the accelerometer pulls
 * it back while it reads about one g. It knows nothing of the solves, which is what lets it veto them: the CV1 Touch
 * ring is nearly symmetric, and solves turned 25-180 degrees still fit its blobs.
 *
 * A CV1 Touch left at rest stops reporting its IMU, so an estimate that ended at rest stays good until the device moves
 * again, which also brings its IMU back.
 */
struct ImuTilt
{
public: // Methods
	//! Folds in one IMU sample, OpenXR coordinates in the device frame.
	void
	push(const xrt_imu_sample &sample);

	/*!
	 * The direction of gravity in the device frame, OpenXR coordinates, at about @p when_ns. Nothing when there is
	 * no estimate to trust then.
	 */
	std::optional<xrt_vec3>
	gravityAt(timepoint_ns when_ns) const;

private: // Fields
	struct Entry
	{
		timepoint_ns timestamp_ns;
		xrt_vec3 up;
	};

	//! History spacing and length, so it reaches back about a quarter second.
	static constexpr timepoint_ns kHistoryStepNs = 4 * U_TIME_1MS_IN_NS;
	static constexpr size_t kHistorySize = 64;

	mutable os::Mutex lock;

	bool have_up{false};
	//! World up in the device frame, unit length.
	xrt_vec3 up{0.0f, 1.0f, 0.0f};
	timepoint_ns start_ns{0};
	timepoint_ns last_ns{0};
	//! The last sample that turned faster than resting.
	timepoint_ns last_moving_ns{0};
	//! What the accelerometer reads at rest, learned: a CV1 Touch reads 10.1-10.5 m/s².
	float rest_accel_len{static_cast<float>(MATH_GRAVITY_M_S2)};

	std::array<Entry, kHistorySize> history{};
	size_t history_count{0};
	//! The slot the next entry goes in.
	size_t history_head{0};

private: // Methods
	void
	resetLocked();

	void
	recordLocked(timepoint_ns timestamp_ns);
};

struct DeviceBase
{
	xrt_imu_sink imu_sink;
};

struct Device : public DeviceBase
{
public: // Fields
	t_constellation_tracker_device_params params;
	t_constellation_tracker_device *device;

	t_constellation_device_id_t id;

	//! Devices are disabled by default, and the constellation tracker must choose to enable them.
	std::atomic<bool> enabled{false};

	//! The owner tracker, so we can retrieve it from the IMU sink callback
	ConstellationTracker *tracker;

	std::unique_ptr<sensor_fusion::SensorFusion> sensor_fusion{};

	// @todo remove when clang-format is updated in CI
	// clang-format off
	t_constellation_search_model *search_model{nullptr};

	// @todo These need to be pulled from the device and put into the sample. 
	//       Right now we just hardcode them since we don't have any real sensor fusion.
	xrt_vec3 prior_pos_error{MIN_POS_ERROR, MIN_POS_ERROR, MIN_POS_ERROR};
	xrt_vec3 prior_rot_error{MIN_ROT_ERROR, MIN_ROT_ERROR, MIN_ROT_ERROR};

	//! Which way is down from the device's IMU, to reject solves that disagree.
	ImuTilt imu_tilt;
	//! Candidate poses rejected for disagreeing with @ref imu_tilt, for the log, and in total.
	RateLimitedCount gravity_rejections{};
	//! Slow-search solves turned away for fitting their blobs too loosely.
	std::atomic<uint64_t> rms_rejections_total{0};
	std::atomic<uint64_t> gravity_rejections_total{0};
	//! Solves rejected for leaving a binding prior, see @ref Camera::bindingPrior.
	RateLimitedCount prior_rejections{};
	std::atomic<uint64_t> prior_rejections_total{0};
	/*!
	 * Solves that left a binding prior since the last accepted solve: when the first was, and how many. When they
	 * keep coming and none agrees, the prior is what is wrong, see @ref Camera::poseMatchesPrior.
	 */
	std::atomic<int64_t> prior_disagree_since_ns{0};
	std::atomic<uint32_t> prior_disagree_count{0};
	RateLimitedCount prior_distrusts{};
	/*!
	 * Accepted solves in a row that the device's fusion disagrees with, see @ref Camera::checkFusionAgrees.
	 */
	struct
	{
		os::Mutex lock;
		uint32_t count{0};
		timepoint_ns since_ns{0};
		/*!
		 * The fusion's orientation to the first solve's, in the device's frame and in the world's: a fusion that
		 * is off keeps showing the same in one of them, a turn of the wrong axis in the first, a drift in heading
		 * in the second.
		 */
		xrt_quat rotation_offset = XRT_QUAT_IDENTITY;
		xrt_quat world_rotation_offset = XRT_QUAT_IDENTITY;
		xrt_vec3 position_offset{};
	} fusion_disagree;
	std::atomic<uint64_t> fusion_resets_total{0};
	//! The camera sample time of the newest pose from 4 or more LEDs, see Camera::tryDevicePoseFewLeds.
	std::atomic<int64_t> last_full_solve_ns{0};
	RateLimitedCount fusion_resets{};

	mutable os::Mutex data_lock;
	struct
	{
		std::optional<DeviceLastPose> last_known_pose;
	} locked_data;
	// clang-format on

	/*!
	 * The device's adjustable part (the CV1's rear headband), see t_constellation_tracker_device_params::loose_part.
	 * Its LEDs join @ref params led_model once @ref loose_part has measured where it sits; the model and the search
	 * model are only changed with ConstellationTracker::device_lock held exclusively (ConstellationTracker::
	 * applyLooseParts).
	 */
	struct LoosePartState
	{
		//! The rigid LEDs and the part's at their factory place, OpenCV axes.
		std::vector<t_constellation_tracker_led> rigid_leds;
		std::vector<t_constellation_tracker_led> part_leds;
		//! By LED id: one of the part's.
		std::array<bool, 128> is_part_led{};
		std::string serial;
		std::unique_ptr<LoosePartEstimator> estimator;
		//! Where the part is in the model now, if it is.
		std::optional<xrt_pose> in_model;

		//! A measurement to apply: an offset, or none (take the part out of the model); with why.
		std::mutex result_lock;
		bool result_pending{false};
		std::optional<LoosePartFit> result;
		std::string result_why;

		/*!
		 * Frames are made from a camera that sees at most a couple of the rigid LEDs (the part's side) and a solve
		 * of the same exposure from another camera, from its rigid LEDs; either may come first.
		 */
		struct View
		{
			timepoint_ns ts;
			uint32_t camera;
			xrt_pose Txr_world_cam;
			float fx, fy, cx, cy;
			std::vector<xrt_vec2> blobs;
		};
		struct Solve
		{
			timepoint_ns ts;
			uint32_t camera;
			xrt_pose Txr_world_device;
		};
		std::mutex pair_lock;
		std::deque<View> views;
		std::deque<Solve> solves;
	};
	std::unique_ptr<LoosePartState> loose_part{};
	//! From the headset's wear sensor (t_constellation_tracker_set_device_worn); unknown counts as worn.
	std::atomic<bool> worn{true};

public: // Methods
	Device(ConstellationTracker *tracker,
	       t_constellation_tracker_device_params *params,
	       t_constellation_tracker_device *device,
	       t_constellation_device_id_t id);

	~Device();

	// Delete move constructors
	Device(const Device &) = delete;
	Device(Device &&) = delete;
	Device &
	operator=(const Device &) = delete;
	Device &
	operator=(Device &&) = delete;

	static Device *
	fromXrtImuSink(xrt_imu_sink *sink)
	{
		// Go through DeviceBase* to Device* so that `Device` doesn't have to be a standard layout type and we
		// can still use `container_of` safely.
		return static_cast<Device *>(container_of(sink, DeviceBase, imu_sink));
	}

	void
	pushImuSample(const xrt_imu_sample &sample);

	//! The direction of gravity in the device frame, OpenCV coordinates, see @ref ImuTilt::gravityAt.
	std::optional<xrt_vec3>
	gravityAt(timepoint_ns when_ns) const;

	/*!
	 * Rebuilds the LED model and the search model from the rigid LEDs and, with @p part_offset, the loose part's at
	 * that place. Only with ConstellationTracker::device_lock held exclusively, or before the device is added.
	 */
	void
	buildLedModel(const std::optional<xrt_pose> &part_offset);

	//! Blobs this camera sample has on the device's rigid LEDs.
	uint32_t
	rigidBlobs(const CameraSample &sample) const;

	//! Whether @p led_id is one of the loose part's LEDs.
	bool
	isPartLed(t_constellation_led_id_it led_id) const;

	//! Blobs these blobs have on the loose part's LEDs.
	uint32_t
	partBlobs(const t_blob *blobs, uint32_t blob_count) const;

	/*!
	 * For a solve that used the loose part's LEDs: the pose from the rigid LEDs alone, with how many of them it keeps
	 * and how well it fits them. The part's LEDs sit where the part was last measured, and a solve bends by as much as
	 * that is off. False when the rigid LEDs alone give no pose.
	 */
	bool
	rigidPose(const ConstellationTracker &tracker,
	          const camera_model &model,
	          const t_blob *blobs,
	          uint32_t blob_count,
	          const xrt_pose &Tcv_cam_device,
	          xrt_pose &out_pose,
	          uint32_t &out_matched,
	          float &out_rms_px);

	/*!
	 * For the loose part: a camera sample that sees at most a couple of the rigid LEDs, its other blobs (unclaimed,
	 * or claimed by the part's LEDs). Paired with a solve of the same exposure from another camera.
	 */
	void
	loosePartView(uint32_t camera_key, const CameraSample &sample, const camera_model &model);

	/*!
	 * For the loose part: an accepted solve from the rigid LEDs. Paired with views of the same exposure, and makes
	 * a frame of its own camera's other blobs (the part seen from the side together with the rigid LEDs).
	 */
	void
	loosePartSolve(uint32_t camera_key,
	               const CameraSample &sample,
	               const camera_model &model,
	               const xrt_pose &Tcv_cam_device);
};

// Separate base struct with our interface implementations so that `ConstellationTrackerBase` remains a standard layout
// type and we can safely use `container_of` on it.
struct ConstellationTrackerBase
{
	xrt_frame_node node = {
	    .next = nullptr,
	    .break_apart = constellation_tracker_node_break_apart,
	    .destroy = constellation_tracker_node_destroy,
	};
	xrt_tracking_origin tracking_origin = {
	    .name = "Constellation Tracker",
	    .type = XRT_TRACKING_TYPE_CONSTELLATION,
	    .initial_offset =
	        {
	            .orientation = XRT_QUAT_IDENTITY,
	            .position = {.x = 0, .y = 1.2, .z = 0},
	        },
	};
};

struct ConstellationTracker : public ConstellationTrackerBase
{
public: // Fields
	//! Whether the constellation tracker is running
	bool running = true;

	enum u_logging_level log_level;

	t_constellation_tracker_params params;

	bool single_threaded{false};
	bool deterministic{false};

	//! How far a solve may tilt a device from what its IMU says, 0 turns the check off.
	float gravity_tolerance_rad{0.0f};

	//! How far a solve may be from a fresh tracked prior, per axis, see @ref Camera::bindingPrior.
	xrt_vec3 prior_tolerance_pos{};
	xrt_vec3 prior_tolerance_rot{};
	//! How recent a device's last solve must be for its prior to be binding, 0 turns the check off.
	timepoint_ns prior_fresh_ns{0};

	//! How far a solve may be from a held prior's position, and for how long after the last solve it binds.
	xrt_vec3 held_prior_tolerance_pos{};
	timepoint_ns held_prior_ns{0};

	//! After how long of solves leaving the prior, with none agreeing, the prior stops binding.
	timepoint_ns prior_distrust_ns{0};

	std::vector<std::shared_ptr<CameraMosaic>> mosaics;

	//! A device's loose part has a measurement to apply, see @ref applyLooseParts.
	std::atomic<bool> loose_part_pending{false};

	std::shared_mutex device_lock;
	/*!
	 * Set while a device is added or removed: the camera threads take device_lock shared back to back, and with
	 * every search busy the exclusive lock never got its turn, so the service hung on shutdown. They
	 * skip their work while this is set.
	 */
	std::atomic<bool> devices_changing{false};
	std::vector<std::unique_ptr<Device>> devices;
	t_constellation_device_id_t next_device_id{0};

	std::unique_ptr<DataRecorder> data_recorder{};

	mutable os::Mutex rig_calibration_lock{};
	os_thread_helper rig_calibration_thread{};
	std::atomic<bool> running_calibration{false};
	//! Keeps the room's measuring from starting at all (replays of recorded placements); nothing else waits on it.
	std::atomic<bool> calibration_held{false};
	//! When the sensors' state was last published for status views (publishStatus), and each camera's placement as
	//! last seen there (kept when its lock was busy).
	std::atomic<int64_t> status_published_ns{0};
	std::vector<bool> status_placed;
	/*!
	 * Cameras placed and in use (as the status has them), at once when measuring every sensor again starts or a
	 * solve places some, else from the status about once a second; -1 before anything is known.
	 */
	std::atomic<int> cameras_placed{-1};
	//! When the collection progress was last logged, so it is logged now and then instead of every sample.
	timepoint_ns last_calibration_progress_log_ns{0};

	/*!
	 * Rig consistency, see @ref checkRigConsistency: the latest world position each camera solved each device at,
	 * and how far apart cameras put a device in the same frame.
	 */
	struct
	{
		os::Mutex lock{};
		std::map<std::tuple<t_constellation_device_id_t, size_t, size_t>, std::pair<timepoint_ns, xrt_vec3>>
		    last;
		//! From the lower indexed camera of each pair to the other, in the world.
		std::vector<xrt_vec3> differences;
		uint32_t strikes{0};
	} rig_check;

	//! A solve as the moved-camera watch keeps it, see @ref watchRig.
	struct RigWatchSolve
	{
		timepoint_ns timestamp_ns{0};
		xrt_pose Txr_world_cam{};
		xrt_pose Txr_cam_device{};
	};
	//! A camera as (mosaic index, camera index).
	using CameraKey = std::pair<size_t, size_t>;
	//! How far a device moved over a window of the moved-camera watch: from its first view, metres and degrees.
	struct RigWatchReach
	{
		xrt_pose first{};
		float m{0.0f};
		float deg{0.0f};
	};
	/*!
	 * Moved-camera watch, see @ref watchRig: per pair of cameras the directions to a device each disagrees on, and
	 * the camera left out while it is placed again.
	 */
	struct
	{
		os::Mutex lock{};
		//! The latest solve of each device by each camera.
		std::map<std::tuple<t_constellation_device_id_t, size_t, size_t>, RigWatchSolve> last;
		//! Per device, the frame the watch last compared and the device's fused pose in it, see
		//! CT_RIG_WATCH_NEW_VIEW_M: only new views of a device are compared.
		std::map<t_constellation_device_id_t, std::pair<timepoint_ns, xrt_pose>> view;
		//! Per pair of cameras (lower first): the current window of disagreements, degrees, and windows over.
		std::map<std::pair<CameraKey, CameraKey>, std::vector<float>> window;
		//! Per pair, how far the device moved in its current window and apart window, see CT_RIG_WATCH_REACH_M.
		std::map<std::pair<CameraKey, CameraKey>, RigWatchReach> reach;
		std::map<std::pair<CameraKey, CameraKey>, RigWatchReach> apart_reach;
		std::map<std::pair<CameraKey, CameraKey>, uint32_t> strikes;
		//! Per pair, the last window's median and when: with three or more cameras, the vote (see watchRig).
		std::map<std::pair<CameraKey, CameraKey>, std::pair<timepoint_ns, float>> last_median;
		//! Per pair, since when its disagreement waits for a third camera's view to tell which one moved.
		std::map<std::pair<CameraKey, CameraKey>, timepoint_ns> suspect_since;
		//! The same for how far apart they put the device, from the lower camera's to the other's, metres.
		std::map<std::pair<CameraKey, CameraKey>, std::vector<xrt_vec3>> apart;
		std::map<std::pair<CameraKey, CameraKey>, uint32_t> apart_strikes;
		//! Per device and camera, recent solves' tilt against the device's IMU and direction against its fusion,
		//! degrees: compared for the device two cameras disagree on, a Touch's are noisier than the HMD's.
		std::map<std::tuple<t_constellation_device_id_t, size_t, size_t>, std::vector<float>> tilt;
		std::map<std::tuple<t_constellation_device_id_t, size_t, size_t>, std::vector<float>> fusion;
		//! The camera left out, the camera it is placed against, and its world poses from their pairs so far.
		std::optional<CameraKey> quarantined;
		//! A camera placed again and in use, placed once more from more pairs, see CT_RIG_WATCH_REFINE_PAIRS.
		std::optional<CameraKey> refining;
		CameraKey anchor{};
		std::vector<xrt_pose> candidates;
		//! Its pose when it was found moved, to report how far it moved.
		xrt_pose quarantined_from{};
	} rig_watch;

	/*!
	 * Quick placement (@ref quickPlace), while cameras are not placed (the first start, or every sensor measured
	 * again): as the Oculus runtime places its sensors, from a few seconds of the headset in view, where the room's
	 * solve needed one to two minutes of movement. The first camera is placed level by the headset's own down (its
	 * IMU) as that camera sees it; every other one by the headset seen by it and a placed camera in the same frame,
	 * as the moved-camera watch places a camera again.
	 */
	struct QuickSolve
	{
		timepoint_ns timestamp_ns{0};
		//! The device in the camera's frame.
		xrt_pose Txr_cam_device{};
		//! The camera, if it is placed already (an anchor for the others).
		std::optional<xrt_pose> Txr_world_cam{};
		//! LEDs the solve matched.
		uint32_t leds{0};
	};
	struct
	{
		os::Mutex lock;
		//! Collecting: cameras are waiting to be placed.
		bool active{false};
		timepoint_ns started_ns{0};
		//! The first and the latest solve's frame times: how long it took in the sensors' time (replays too).
		timepoint_ns first_frame_ns{0};
		timepoint_ns last_frame_ns{0};
		//! The cameras to place.
		std::vector<CameraKey> unplaced;
		//! Each camera's solves of the headset in the last moments, for pairs of the same frame.
		std::map<CameraKey, std::deque<QuickSolve>> recent;
		//! Down in each camera's own frame (OpenCV), from its solves and the headset's IMU: the first one placed.
		std::map<CameraKey, std::vector<xrt_vec3>> downs;
		//! Per camera to place, where pairs with a placed one put it.
		std::map<CameraKey, std::vector<xrt_pose>> candidates;
		/*!
		 * How far the headset turned over each camera's samples (degrees from the first one's orientation): a
		 * headset lying still gave every pair the same view, and its LEDs' depth error (cm at a slant) placed a
		 * camera 8 cm off. The pairs need CONSTELLATION_TRACKER_QUICK_TURN_DEG (20) of it; down (the first camera)
		 * does not.
		 */
		std::map<CameraKey, std::optional<xrt_quat>> first_turn;
		std::map<CameraKey, float> turned_deg;
		//! Placements decided (OpenXR); all of @ref unplaced in it: ready to apply.
		std::map<CameraKey, xrt_pose> placed;
		bool ready{false};
		//! Said once a measuring: no down correction learned yet, so the room's solve places the cameras.
		bool told_no_correction{false};
	} quick;

	/*!
	 * The headset's down as its IMU has it against the room's down as the cameras see it, learned from every good
	 * solve while the cameras are placed: the IMU sits a little turned in the headset (2.6 degrees in one CV1,
	 * which the room's solve estimates and the gravity check's 8 degrees hide). Quick placement levels the first
	 * camera by it; without it, the room's solve places them.
	 */
	struct
	{
		os::Mutex lock;
		//! Running means of the two unit vectors, in the headset's frame (OpenCV), and how many went in.
		xrt_vec3 imu_down{0.0f, 0.0f, 0.0f};
		xrt_vec3 room_down{0.0f, 0.0f, 0.0f};
		size_t count{0};
		//! Read from the headset's saved one (by serial, once), and saved this session.
		bool loaded{false};
		bool saved{false};
	} down_learn;

	//! The saved down correction of @p device (down_learn), once; from the config dir by its serial.
	void
	loadDownLearn(const Device *device);
	//! Saves the down correction learned this session for @p serial, kept apart from other headsets'.
	void
	saveDownLearn(const std::string &serial, const xrt_vec3 &imu_down, const xrt_vec3 &room_down, size_t count);

	/*!
	 * Whether the room is level (LevelWatch): the headset's gravity against the down its solves give through the
	 * sensors' placements. A room tilted by sensors moved while the driver was off (both alike, or the one the
	 * others are placed against) is something the moved-sensor watch, which compares sensors with each other, cannot
	 * see; its tilt rejects the controllers' solves and pulls the headset's orientation (a user saw the world spin).
	 */
	struct
	{
		os::Mutex lock;
		LevelWatch watch;
		//! The last window's level rotation, when it asked for one: two in a row must agree.
		std::optional<xrt_vec3> pending;
		uint32_t levelled{0};
		//! The last window's common tilt (degrees) and when: the room's down correction is saved only when level.
		float last_tilt_deg{-1.0f};
		timepoint_ns last_ns{0};
	} level;

	/*!
	 * The room placed tilted beyond the gravity checks (sensors tilted alike while the driver was off, more than
	 * the room's level can take in): every camera that sees the headset tilts it from its IMU's down by more than
	 * the checks allow, so none of its solves is taken. Then every sensor is measured again, as with the Measure
	 * button, as the Oculus runtime recovers from moved sensors by itself.
	 */
	struct
	{
		os::Mutex lock;
		//! The recent tilts of the headset's solves (time, degrees), by camera key.
		std::map<uint32_t, std::deque<std::pair<timepoint_ns, float>>> tilts;
		//! Since when every camera has disagreed (0: not now).
		timepoint_ns since_ns{0};
	} remeasure;
	std::atomic<bool> remeasure_requested{false};

	std::unique_ptr<rig_calibration::RigCalibration> rig_calibration{};

	/*!
	 * A recalibration asked for (@ref recalibrate): each camera's placement before it, by camera serial, so the
	 * solve's placements are put back where the room was by the cameras that kept their place (rl_room.hpp), and
	 * whoever keeps a floor learns how far each camera moved up or down (u_notice_set_recalibration).
	 */
	struct
	{
		bool pending{false};
		std::map<std::string, xrt_pose> pose_before;
		uint32_t sequence{0};
	} recalibration;

#ifdef XRT_FEATURE_RERUN
	std::unique_ptr<struct RerunContext> rerun_stream{};
#endif

public: // Methods
	/*!
	 * Publishes the sensors' state for status views (u_notice_set_sensors: the Rift tab): how many cameras, how many
	 * send frames, how many are placed, whether one is being placed or the room measured. Only reads; takes no lock
	 * that is busy (a busy camera keeps its last seen placement), so a camera's frames never wait for it. About once a
	 * second, from the first camera whose frame comes after that.
	 */
	void
	publishStatus(timepoint_ns now_ns);

	/*!
	 * Restores the placement of cameras saved by an earlier rig calibration, matched by serial. Only for cameras
	 * that don't already have a concrete pose. Called before the rig calibration learns which cameras are placed.
	 */
	void
	loadStoredRig();

	//! Saves the placement of every placed camera with a serial, all from the current calibration.
	void
	saveRig(double camera_rms_sigma);

	/*!
	 * For the room's level: a good solve of the headset (orientation in the room, OpenCV) and its IMU's down; when
	 * two windows in a row find the room tilted, every sensor's place is turned to level it (applyLevel).
	 */
	void
	feedLevel(timepoint_ns timestamp_ns,
	          uint32_t camera_key,
	          const xrt_quat &Rcv_world_device,
	          const xrt_vec3 &imu_down_cv);

	//! Turns every placed sensor about the room's origin by @p Rcv (OpenCV axes): the room levelled.
	void
	applyLevel(timepoint_ns timestamp_ns, const xrt_vec3 &level_rotvec_cv, const LevelResult &result);

	/*!
	 * Puts the devices' loose parts into their LED models, or takes them out, as their estimators decided. Takes
	 * @ref device_lock exclusively: only call it with no lock held, at the start of a camera's processing.
	 */
	void
	applyLooseParts();

	//! Stops every camera's processing threads (the frame node's break apart, and the destructor); repeatable.
	void
	stopCameraThreads();

	//! Where a device's loose part was measured to sit, saved by serial; and saving it.
	std::optional<xrt_pose>
	loadLoosePart(const std::string &serial);
	void
	saveLoosePart(const std::string &serial, const LoosePartFit &fit);

	/*!
	 * Watches whether the placed cameras still agree on where a device is. Called with every solve that has a
	 * world pose. When two cameras disagree persistently, one of them has moved since it was placed: every camera
	 * but one is unplaced, so that the rig calibration places them again against it.
	 */
	void
	checkRigConsistency(t_constellation_device_id_t device_id,
	                    size_t mosaic_idx,
	                    size_t camera_idx,
	                    timepoint_ns timestamp_ns,
	                    uint32_t matched_blobs,
	                    const xrt_vec3 &world_position);

	/*!
	 * The moved-camera watch: fed every solve of a device that fits its blobs, before the gravity and prior checks
	 * turn any away (a moved camera's solves are what they turn away). When a camera is found moved it is left out,
	 * see @ref rigWatchLeavesOut, until it is placed again from its solves against another camera's, and saved.
	 *
	 * @param tilt_deg how far the solve tilts the device from its IMU's down, negative when unknown.
	 * @param fusion_deg how far the solve's direction to the device is from the device's fused position, negative
	 *                   when unknown.
	 * @param Txr_world_device_fused the device's fused pose at the solve's frame, when it has one.
	 */
	void
	watchRig(t_constellation_device_id_t device_id,
	         size_t mosaic_idx,
	         size_t camera_idx,
	         timepoint_ns timestamp_ns,
	         const xrt_pose &Txr_world_cam,
	         const xrt_pose &Txr_cam_device,
	         float tilt_deg,
	         float fusion_deg,
	         const std::optional<xrt_pose> &Txr_world_device_fused);

	//! Whether the moved-camera watch leaves this camera's solves out, until it has been placed again.
	bool
	rigWatchLeavesOut(size_t mosaic_idx, size_t camera_idx);

	static ConstellationTracker *
	Get(xrt_frame_node *node)
	{
		return static_cast<ConstellationTracker *>(container_of(node, ConstellationTrackerBase, node));
	}

	static ConstellationTracker *
	Get(t_constellation_tracker *tracker)
	{
		return reinterpret_cast<ConstellationTracker *>(tracker);
	}

	ConstellationTracker(t_constellation_tracker_params *params);

	~ConstellationTracker();

	// Delete all copy/move ctors, since the pointers for `node` need to be stable
	ConstellationTracker(const ConstellationTracker &) = delete;
	ConstellationTracker(ConstellationTracker &&) = delete;
	ConstellationTracker &
	operator=(const ConstellationTracker &) = delete;
	ConstellationTracker &
	operator=(ConstellationTracker &&) = delete;

	bool
	needSensorCalibration();

	bool
	checkSensorCalibration();

	/*!
	 * Measures every camera again from the devices' movement, as on a first start: every placement dropped (here,
	 * in the solve's job and the moved-camera watch), the saved rig set aside, and the collection begun anew by
	 * checkSensorCalibration, which says so. Whichever way the cameras were moved, none is kept as a reference.
	 * False while a solve is running.
	 */
	bool
	recalibrate(bool automatic = false);

	/*!
	 * For the automatic measuring (see remeasure): a good solve of the headset through a camera, and how far it
	 * tilts the headset from its IMU's down. Camera threads, no tracker lock held.
	 */
	void
	feedRemeasure(timepoint_ns timestamp_ns, Device *device, uint32_t camera_key, float tilt_deg);

	//! Measures every camera again when feedRemeasure asked for it. Only call it with no lock held.
	void
	applyRemeasure();

	//! Starts quick placement for the cameras not placed (from checkSensorCalibration, its lock held).
	void
	startQuickPlace();

	//! Runs a placement's last step (@p func) on rig_calibration_thread, the last one's joined first.
	void
	startCalibrationThread(void *(*func)(void *), const char *what);

	//! A solve of the headset by a camera while cameras wait to be placed (see quick).
	void
	quickPlace(const Device *device,
	           size_t mosaic_idx,
	           size_t camera_idx,
	           const CameraSample &sample,
	           const xrt_pose &Tcv_cam_device,
	           uint32_t leds);

	//! A camera's new placement, in the tracking space (OpenXR).
	struct Placement
	{
		size_t mosaic_idx;
		size_t camera_idx;
		xrt_pose Txr_origin_cam;
	};

	/*!
	 * New placements in use, from the room's solve or quick placement: put back where the room was when every
	 * sensor was measured again on request (rl_room.hpp), set in the cameras and the solve's job, saved, said
	 * ("calibration-done", with the recalibration's result), and every device tracked again.
	 */
	void
	applyPlacements(const std::vector<Placement> &placements, double camera_rms_sigma, const char *how);

	void
	setupVariableTracking();

	t_constellation_device_id_t
	addDevice(t_constellation_tracker_device_params *params, t_constellation_tracker_device *device);

	void
	removeDevice(t_constellation_device_id_t device_id);

	void
	getTrackedPose(t_constellation_device_id_t device_id,
	               timepoint_ns when_ns,
	               xrt_space_relation &out_relation,
	               bool blended = true);
};

}; // namespace xrt::tracking::constellation
