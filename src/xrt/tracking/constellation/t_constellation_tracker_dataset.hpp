// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the data recorder logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "t_constellation_tracker_internal.hpp"

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>


namespace xrt::tracking::constellation {

namespace rig_calibration = optimizer::rig_calibration;

//! One IMU sample as the tracker received it from a device, in OpenXR space.
struct DatasetImuSample
{
	t_constellation_device_id_t device_id;
	xrt_imu_sample sample;
};

//! A device pose the tracker solved from one camera sample, from either the fast or the slow thread.
struct DatasetSolve
{
	t_constellation_device_id_t device_id;
	//! The @ref CameraSample::id this solve came from.
	uint64_t sample_id;
	timepoint_ns timestamp_ns;
	uint32_t mosaic_index;
	uint32_t camera_index;

	SolveSource source;
	//! The @ref pose_match_flags of the final score.
	uint32_t match_flags;

	//! The solved pose, in OpenCV space.
	xrt_pose Tcv_cam_device;
	//! Only present when the camera had been placed in the world when the sample was taken.
	std::optional<xrt_pose> Txr_world_device;

	uint32_t matched_blobs;
	uint32_t unmatched_blobs;
	uint32_t visible_leds;
	//! RMS reprojection error over the matched blobs, in pixels.
	double reprojection_error_px;

	//! The LEDs that had a blob matched to them.
	std::vector<uint8_t> led_ids;
};

//! A pose the tracker handed out for a device, what an application ends up seeing.
struct DatasetOutputPose
{
	t_constellation_device_id_t device_id;
	//! When the pose was asked for, monotonic.
	timepoint_ns query_ns;
	//! The time the pose was asked for, usually a predicted display time.
	timepoint_ns when_ns;
	//! The time of the fused state the pose was predicted from.
	timepoint_ns state_ns;
	//! The relation as handed out, in OpenXR space.
	xrt_space_relation relation;
};

enum class CalibrationEventKind : uint8_t
{
	BeginCollecting = 0,
	EndCollecting = 1,
	Solved = 2,
	Failed = 3,
};

//! A step of the rig calibration, so a recording shows when it ran and what it produced.
struct DatasetCalibrationEvent
{
	CalibrationEventKind kind;
	//! Monotonic time the event happened at, the same clock as the samples.
	timepoint_ns timestamp_ns;

	//! @ref CalibrationEventKind::EndCollecting only, the sample count of the device the solve will use.
	uint64_t num_samples{0};

	//! @ref CalibrationEventKind::Solved only. The keyframes' camera samples are not recorded.
	std::optional<rig_calibration::CalibrationResult> result{std::nullopt};

	static DatasetCalibrationEvent
	begin(timepoint_ns timestamp_ns)
	{
		return {CalibrationEventKind::BeginCollecting, timestamp_ns};
	}

	static DatasetCalibrationEvent
	end(timepoint_ns timestamp_ns, uint64_t num_samples)
	{
		return {CalibrationEventKind::EndCollecting, timestamp_ns, num_samples};
	}

	static DatasetCalibrationEvent
	solved(timepoint_ns timestamp_ns, const rig_calibration::CalibrationResult &result)
	{
		DatasetCalibrationEvent event = {CalibrationEventKind::Solved, timestamp_ns, 0, result};
		// The observations are already in the recording as camera samples, don't hold a second copy.
		for (auto &keyframe : event.result->keyframes) {
			keyframe.camera_samples.clear();
		}
		return event;
	}

	static DatasetCalibrationEvent
	failed(timepoint_ns timestamp_ns)
	{
		return {CalibrationEventKind::Failed, timestamp_ns};
	}
};

struct DataSerializer
{
private: // Fields
	//! A file, or memory for a serializer that builds packets to be written elsewhere.
	std::unique_ptr<std::iostream> stream;

	//! Where the length of the packet started by @ref beginSizedPacket goes.
	std::streampos sized_packet_length_pos{};

public: // Methods
	DataSerializer(std::string file, bool write);

	//! Serializes into memory, see @ref takeBuffer.
	DataSerializer();

	//! What a memory serializer holds, leaving it empty.
	std::string
	takeBuffer();

	~DataSerializer();

	void
	write(uint8_t value);
	void
	read(uint8_t &value);

	void
	write(uint32_t value);
	void
	read(uint32_t &value);

	void
	write(float value);
	void
	read(float &value);

	void
	write(uint64_t value);
	void
	read(uint64_t &value);

	void
	write(double value);
	void
	read(double &value);

	void
	write(const xrt_pose &value);
	void
	read(xrt_pose &value);

	void
	write(const FoundDevicePose &value);
	void
	read(FoundDevicePose &value);

	void
	write(const DeviceState &value);
	void
	read(DeviceState &value);

	void
	write(const t_blob &value);
	void
	read(t_blob &value);

	void
	write(const CameraSample &value);
	void
	read(CameraSample &value);

	void
	write(const t_camera_calibration &value);
	void
	read(t_camera_calibration &value);

	void
	write(t_constellation_tracker_led &value);
	void
	read(t_constellation_tracker_led &value);

	void
	write(const t_constellation_tracker_led_model &value);
	void
	read(std::vector<t_constellation_tracker_led> &led_storage, t_constellation_tracker_led_model &value);

	template <typename T>
	void
	write(const std::optional<T> &opt_value)
	{
		this->write(static_cast<uint8_t>(opt_value.has_value() ? 1 : 0));
		if (opt_value.has_value()) {
			this->write(opt_value.value());
		}
	}

	template <typename T>
	void
	read(std::optional<T> &opt_value)
	{
		uint8_t has_value;
		this->read(has_value);
		if (has_value) {
			T value;
			this->read(value);
			opt_value = value;
		} else {
			opt_value = std::nullopt;
		}
	}

	void
	write(const xrt_vec3 &value);
	void
	read(xrt_vec3 &value);

	void
	write(const xrt_vec3_f64 &value);
	void
	read(xrt_vec3_f64 &value);

	void
	write(const xrt_quat &value);
	void
	read(xrt_quat &value);

	void
	write(const DatasetImuSample &value);
	void
	read(DatasetImuSample &value);

	void
	write(const DatasetSolve &value);
	void
	read(DatasetSolve &value);

	void
	write(const rig_calibration::CalibrationResult &value);
	void
	read(rig_calibration::CalibrationResult &value);

	void
	write(const DatasetCalibrationEvent &value);
	void
	read(DatasetCalibrationEvent &value);

	void
	write(const DatasetOutputPose &value);
	void
	read(DatasetOutputPose &value);

	/*!
	 * Starts a packet that has its payload length written after the type, so readers can skip packet types
	 * they don't know and fields added after the ones they read. Every packet type after the device info
	 * packet is written this way. Not reentrant.
	 */
	void
	beginSizedPacket(uint8_t packet_type);
	//! Writes the payload length of the packet started by @ref beginSizedPacket.
	void
	endSizedPacket();

	std::streampos
	tell();
	void
	seek(std::streampos pos);

	void
	flush();
};

/*!
 * Writes a recording without ever making the caller wait on the disk.
 *
 * The callers are the tracking threads and the compositor asking for poses. Each record call builds its packet in
 * memory and queues it, and a writer thread of its own does the writing. Writing from the callers stalled them for
 * tens of milliseconds whenever the filesystem committed, every 5 seconds, which dropped frames on every sensor.
 */
struct DataRecorder
{
private: // Fields
	std::ofstream out;

	std::mutex queue_lock;
	std::condition_variable queue_cv;
	std::deque<std::string> queue;
	bool stopping{false};

	std::thread writer;

private: // Methods
	//! Queues a finished packet for the writer thread.
	void
	push(std::string &&packet);

	void
	writerLoop();

public: // Methods
	DataRecorder(ConstellationTracker *tracker, std::string out_file);

	//! Writes out everything queued before returning.
	~DataRecorder();

	void
	recordSample(const CameraSample &sample);

	void
	recordDeviceInfo(const Device &device);

	void
	recordImuSample(t_constellation_device_id_t device_id, const xrt_imu_sample &sample);

	void
	recordSolve(const DatasetSolve &solve);

	void
	recordCalibrationEvent(const DatasetCalibrationEvent &event);

	void
	recordOutputPose(const DatasetOutputPose &pose);
};

struct DatasetMosaic
{
	std::vector<t_camera_calibration> camera_calibrations;
};

struct DatasetDevice
{
	t_constellation_device_id_t id;

	std::vector<t_constellation_tracker_led> leds;
	//! In OpenXR space, as @ref ConstellationTracker::addDevice takes it.
	t_constellation_tracker_led_model led_model;
};

struct DatasetReader
{
private: // Fields
	DataSerializer serializer;

public: // Fields
	std::vector<DatasetMosaic> mosaics;
	std::vector<DatasetDevice> devices;

	std::vector<CameraSample> samples;

	std::vector<DatasetImuSample> imu_samples;
	std::vector<DatasetSolve> solves;
	std::vector<DatasetCalibrationEvent> calibration_events;
	std::vector<DatasetOutputPose> output_poses;

public: // Methods
	DatasetReader(std::string filename);
};

}; // namespace xrt::tracking::constellation
