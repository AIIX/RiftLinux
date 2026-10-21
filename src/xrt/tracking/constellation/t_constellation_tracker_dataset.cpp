// Copyright 2026, Beyley Cardellio
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the data recorder logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "xrt/xrt_byte_order.h"

#include "t_constellation_tracker_dataset.hpp"

#include <string>


namespace xrt::tracking::constellation {

#define CT_DATA_MAGIC_VALUE 0x066A4C50

enum PacketType
{
	PACKET_TYPE_CAMERA_SAMPLE = 1,
	PACKET_TYPE_DEVICE_INFO = 2,

	// Everything from here on is written with DataSerializer::beginSizedPacket.
	PACKET_TYPE_FIRST_SIZED = 3,
	PACKET_TYPE_IMU_SAMPLE = 3,
	PACKET_TYPE_DEVICE_SOLVE = 4,
	PACKET_TYPE_CALIBRATION_EVENT = 5,
	PACKET_TYPE_OUTPUT_POSE = 6,
};

namespace {

	void
	throw_if_stream_failed(std::ios &stream, const char *operation)
	{
		if (!stream) {
			throw std::runtime_error(operation);
		}
	}

} // namespace

/*
 *
 * DataSerializer implementations
 *
 */

DataSerializer::DataSerializer(std::string file, bool write)
{
	auto file_stream =
	    std::make_unique<std::fstream>(file, std::ios::binary | (write ? std::ios::out : std::ios::in));
	if (!file_stream->is_open()) {
		throw std::runtime_error("Failed to open file for data serialization.");
	}
	this->stream = std::move(file_stream);
}

DataSerializer::DataSerializer()
    : stream(std::make_unique<std::stringstream>(std::ios::binary | std::ios::in | std::ios::out))
{}

DataSerializer::~DataSerializer() = default;

std::string
DataSerializer::takeBuffer()
{
	auto *memory = dynamic_cast<std::stringstream *>(this->stream.get());
	if (memory == nullptr) {
		throw std::runtime_error("Only a memory serializer has a buffer to take.");
	}

	std::string buffer = memory->str();
	memory->str({});
	memory->clear();
	return buffer;
}

// uint8_t

void
DataSerializer::write(uint8_t value)
{
	this->stream->write(reinterpret_cast<const char *>(&value), sizeof(value));
	throw_if_stream_failed(*this->stream, "Failed to write dataset file.");
}

void
DataSerializer::read(uint8_t &value)
{
	this->stream->read(reinterpret_cast<char *>(&value), sizeof(value));
	throw_if_stream_failed(*this->stream, "Failed to read dataset file.");
}

// uint32_t

void
DataSerializer::write(uint32_t value)
{
	__le32 le_value = __cpu_to_le32(value);
	this->stream->write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to write dataset file.");
}

void
DataSerializer::read(uint32_t &value)
{
	__le32 le_value;
	this->stream->read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to read dataset file.");
	value = __le32_to_cpu(le_value);
}

// float

void
DataSerializer::write(float value)
{
	__lef32 le_value = __cpu_to_lef32(value);
	this->stream->write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to write dataset file.");
}

void
DataSerializer::read(float &value)
{
	__lef32 le_value;
	this->stream->read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to read dataset file.");
	value = __lef32_to_cpu(le_value);
}

// uint64_t

void
DataSerializer::write(uint64_t value)
{
	__le64 le_value = __cpu_to_le64(value);
	this->stream->write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to write dataset file.");
}

void
DataSerializer::read(uint64_t &value)
{
	__le64 le_value;
	this->stream->read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to read dataset file.");
	value = __le64_to_cpu(le_value);
}

// double

void
DataSerializer::write(double value)
{
	__lef64 le_value = __cpu_to_lef64(value);
	this->stream->write(reinterpret_cast<const char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to write dataset file.");
}

void
DataSerializer::read(double &value)
{
	__lef64 le_value;
	this->stream->read(reinterpret_cast<char *>(&le_value), sizeof(le_value));
	throw_if_stream_failed(*this->stream, "Failed to read dataset file.");
	value = __lef64_to_cpu(le_value);
}

// xrt_pose

void
DataSerializer::write(const xrt_pose &value)
{
	this->write(value.position.x);
	this->write(value.position.y);
	this->write(value.position.z);
	this->write(value.orientation.x);
	this->write(value.orientation.y);
	this->write(value.orientation.z);
	this->write(value.orientation.w);
}

void
DataSerializer::read(xrt_pose &value)
{
	this->read(value.position.x);
	this->read(value.position.y);
	this->read(value.position.z);
	this->read(value.orientation.x);
	this->read(value.orientation.y);
	this->read(value.orientation.z);
	this->read(value.orientation.w);
}

// FoundDevicePose

void
DataSerializer::write(const FoundDevicePose &value)
{
	this->write(value.Tcv_cam_device);
}

void
DataSerializer::read(FoundDevicePose &value)
{
	this->read(value.Tcv_cam_device);
}

// DeviceState

void
DataSerializer::write(const DeviceState &value)
{
	this->write(static_cast<uint8_t>(value.device_id));

	this->write(value.Txr_world_device_prior);
	this->write(value.found_pose);
}

void
DataSerializer::read(DeviceState &value)
{
	uint8_t device_id;
	this->read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	this->read(value.Txr_world_device_prior);
	this->read(value.found_pose);
}

// t_blob

void
DataSerializer::write(const t_blob &value)
{
	this->write(value.blob_id);
	this->write(static_cast<uint8_t>(value.matched_device_id));
	this->write(static_cast<uint8_t>(value.matched_device_led_id));
	this->write(value.center_distorted.x);
	this->write(value.center_distorted.y);
	this->write(value.center_undistorted.x);
	this->write(value.center_undistorted.y);
	this->write(value.motion_vector.x);
	this->write(value.motion_vector.y);
	this->write(static_cast<uint32_t>(value.bounding_box.offset.w));
	this->write(static_cast<uint32_t>(value.bounding_box.offset.h));
	this->write(static_cast<uint32_t>(value.bounding_box.extent.w));
	this->write(static_cast<uint32_t>(value.bounding_box.extent.h));
	this->write(value.size.x);
	this->write(value.size.y);
	this->write(value.brightness);
}

void
DataSerializer::read(t_blob &value)
{
	this->read(value.blob_id);

	uint8_t matched_device_id;
	this->read(matched_device_id);
	value.matched_device_id = static_cast<t_constellation_device_id_t>(matched_device_id);

	uint8_t matched_device_led_id;
	this->read(matched_device_led_id);
	value.matched_device_led_id = static_cast<t_constellation_device_id_t>(matched_device_led_id);

	this->read(value.center_distorted.x);
	this->read(value.center_distorted.y);
	this->read(value.center_undistorted.x);
	this->read(value.center_undistorted.y);
	this->read(value.motion_vector.x);
	this->read(value.motion_vector.y);

	uint32_t bounding_box_offset_w;
	this->read(bounding_box_offset_w);
	value.bounding_box.offset.w = static_cast<int>(bounding_box_offset_w);

	uint32_t bounding_box_offset_h;
	this->read(bounding_box_offset_h);
	value.bounding_box.offset.h = static_cast<int>(bounding_box_offset_h);

	uint32_t bounding_box_extent_w;
	this->read(bounding_box_extent_w);
	value.bounding_box.extent.w = static_cast<int>(bounding_box_extent_w);

	uint32_t bounding_box_extent_h;
	this->read(bounding_box_extent_h);
	value.bounding_box.extent.h = static_cast<int>(bounding_box_extent_h);

	this->read(value.size.x);
	this->read(value.size.y);
	this->read(value.brightness);
}

// CameraSample

void
DataSerializer::write(const CameraSample &value)
{
	this->write(static_cast<uint64_t>(value.id));
	this->write(static_cast<uint64_t>(value.timestamp_ns));

	this->write(static_cast<uint32_t>(value.blob_count));
	for (size_t i = 0; i < value.blob_count; i++) {
		this->write(value.blobs[i]);
	}

	this->write(value.Txr_world_cam);

	this->write(value.device_count);
	for (size_t i = 0; i < value.device_count; i++) {
		this->write(value.device_states[i]);
	}

	this->write(value.mosaic_index);
	this->write(value.camera_index);
}

void
DataSerializer::read(CameraSample &value)
{
	this->read(value.id);

	uint64_t timestamp_ns;
	this->read(timestamp_ns);
	value.timestamp_ns = timestamp_ns;

	this->read(value.blob_count);
	if (value.blob_count > ARRAY_SIZE(value.blobs)) {
		throw std::runtime_error("Dataset sample blob_count exceeds CameraSample storage.");
	}

	for (size_t i = 0; i < value.blob_count; i++) {
		this->read(value.blobs[i]);
	}

	this->read(value.Txr_world_cam);

	uint32_t device_count;
	this->read(device_count);
	value.device_count = device_count;

	if (device_count > value.device_states.max_size()) {
		throw std::runtime_error("Dataset sample device_count exceeds CameraSample storage.");
	}

	for (size_t i = 0; i < value.device_count; i++) {
		this->read(value.device_states[i]);
	}

	this->read(value.mosaic_index);
	this->read(value.camera_index);
}

// t_camera_calibration

void
DataSerializer::write(const t_camera_calibration &value)
{
	this->write(static_cast<uint32_t>(value.image_size_pixels.w));
	this->write(static_cast<uint32_t>(value.image_size_pixels.h));

	// @todo remove when clang-format is updated in CI
	// clang-format off
	double *intrinsics_as_array = reinterpret_cast<double *>(const_cast<double (*)[3][3]>(&value.intrinsics));
	// clang-format on

	for (size_t i = 0; i < 9; i++) {
		this->write(intrinsics_as_array[i]);
	}

	for (size_t i = 0; i < ARRAY_SIZE(value.distortion_parameters_as_array); i++) {
		this->write(value.distortion_parameters_as_array[i]);
	}

	this->write(static_cast<uint8_t>(value.distortion_model));
}

void
DataSerializer::read(t_camera_calibration &value)
{
	uint32_t image_size_w;
	this->read(image_size_w);
	value.image_size_pixels.w = static_cast<int>(image_size_w);

	uint32_t image_size_h;
	this->read(image_size_h);
	value.image_size_pixels.h = static_cast<int>(image_size_h);

	// @todo remove when clang-format is updated in CI
	// clang-format off
	double *intrinsics_as_array = reinterpret_cast<double *>(const_cast<double (*)[3][3]>(&value.intrinsics));
	// clang-format on

	for (size_t i = 0; i < 9; i++) {
		this->read(intrinsics_as_array[i]);
	}

	for (size_t i = 0; i < ARRAY_SIZE(value.distortion_parameters_as_array); i++) {
		this->read(value.distortion_parameters_as_array[i]);
	}

	uint8_t distortion_model;
	this->read(distortion_model);
	value.distortion_model = static_cast<t_camera_distortion_model>(distortion_model);
}

// t_constellation_tracker_led

void
DataSerializer::write(t_constellation_tracker_led &value)
{
	this->write(value.position.x);
	this->write(value.position.y);
	this->write(value.position.z);

	this->write(value.normal.x);
	this->write(value.normal.y);
	this->write(value.normal.z);

	this->write(value.radius_m);
	this->write(value.visibility_angle);

	this->write(static_cast<uint8_t>(value.id));
}

void
DataSerializer::read(t_constellation_tracker_led &value)
{
	this->read(value.position.x);
	this->read(value.position.y);
	this->read(value.position.z);

	this->read(value.normal.x);
	this->read(value.normal.y);
	this->read(value.normal.z);

	this->read(value.radius_m);
	this->read(value.visibility_angle);

	uint8_t id;
	this->read(id);
	value.id = static_cast<t_constellation_led_id_it>(id);
}

// t_constellation_tracker_led_model

void
DataSerializer::write(const t_constellation_tracker_led_model &value)
{
	this->write(static_cast<uint32_t>(value.led_count));
	for (size_t i = 0; i < value.led_count; i++) {
		this->write(value.leds[i]);
	}
}

void
DataSerializer::read(std::vector<t_constellation_tracker_led> &led_storage, t_constellation_tracker_led_model &value)
{
	uint32_t led_count;
	this->read(led_count);
	value.led_count = led_count;

	led_storage.resize(led_count);
	value.leds = led_storage.data();

	for (size_t i = 0; i < value.led_count; i++) {
		this->read(value.leds[i]);
	}
}

// xrt_vec3

void
DataSerializer::write(const xrt_vec3 &value)
{
	this->write(value.x);
	this->write(value.y);
	this->write(value.z);
}

void
DataSerializer::read(xrt_vec3 &value)
{
	this->read(value.x);
	this->read(value.y);
	this->read(value.z);
}

// xrt_vec3_f64

void
DataSerializer::write(const xrt_vec3_f64 &value)
{
	this->write(value.x);
	this->write(value.y);
	this->write(value.z);
}

void
DataSerializer::read(xrt_vec3_f64 &value)
{
	this->read(value.x);
	this->read(value.y);
	this->read(value.z);
}

// xrt_quat

void
DataSerializer::write(const xrt_quat &value)
{
	this->write(value.x);
	this->write(value.y);
	this->write(value.z);
	this->write(value.w);
}

void
DataSerializer::read(xrt_quat &value)
{
	this->read(value.x);
	this->read(value.y);
	this->read(value.z);
	this->read(value.w);
}

// DatasetImuSample

void
DataSerializer::write(const DatasetImuSample &value)
{
	this->write(static_cast<uint8_t>(value.device_id));
	this->write(static_cast<uint64_t>(value.sample.timestamp_ns));
	this->write(value.sample.accel_m_s2);
	this->write(value.sample.gyro_rad_secs);
}

void
DataSerializer::read(DatasetImuSample &value)
{
	uint8_t device_id;
	this->read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	uint64_t timestamp_ns;
	this->read(timestamp_ns);
	value.sample.timestamp_ns = static_cast<timepoint_ns>(timestamp_ns);

	this->read(value.sample.accel_m_s2);
	this->read(value.sample.gyro_rad_secs);
}

// DatasetSolve

void
DataSerializer::write(const DatasetSolve &value)
{
	this->write(static_cast<uint8_t>(value.device_id));
	this->write(value.sample_id);
	this->write(static_cast<uint64_t>(value.timestamp_ns));
	this->write(value.mosaic_index);
	this->write(value.camera_index);

	this->write(static_cast<uint8_t>(value.source));
	this->write(value.match_flags);

	this->write(value.Tcv_cam_device);
	this->write(value.Txr_world_device);

	this->write(value.matched_blobs);
	this->write(value.unmatched_blobs);
	this->write(value.visible_leds);
	this->write(value.reprojection_error_px);

	this->write(static_cast<uint8_t>(value.led_ids.size()));
	for (uint8_t led_id : value.led_ids) {
		this->write(led_id);
	}
}

void
DataSerializer::read(DatasetSolve &value)
{
	uint8_t device_id;
	this->read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	this->read(value.sample_id);

	uint64_t timestamp_ns;
	this->read(timestamp_ns);
	value.timestamp_ns = static_cast<timepoint_ns>(timestamp_ns);

	this->read(value.mosaic_index);
	this->read(value.camera_index);

	uint8_t source;
	this->read(source);
	value.source = static_cast<SolveSource>(source);
	this->read(value.match_flags);

	this->read(value.Tcv_cam_device);
	this->read(value.Txr_world_device);

	this->read(value.matched_blobs);
	this->read(value.unmatched_blobs);
	this->read(value.visible_leds);
	this->read(value.reprojection_error_px);

	uint8_t led_count;
	this->read(led_count);
	value.led_ids.resize(led_count);
	for (uint8_t &led_id : value.led_ids) {
		this->read(led_id);
	}
}

// rig_calibration::CalibrationResult

void
DataSerializer::write(const rig_calibration::CalibrationResult &value)
{
	this->write(static_cast<uint8_t>(value.device_id));

	this->write(value.imu.accel_bias);
	this->write(value.imu.gyro_bias);
	this->write(value.imu.accel_scale);
	this->write(value.imu.Qcv_imu_model);
	this->write(value.gravity_mag);

	this->write(static_cast<uint64_t>(value.imu_fitness.num_residuals));
	this->write(value.imu_fitness.chi_squared);
	this->write(static_cast<uint64_t>(value.camera_fitness.num_residuals));
	this->write(value.camera_fitness.chi_squared);

	this->write(value.initial_cost);
	this->write(value.final_cost);
	this->write(static_cast<uint32_t>(value.iterations));

	this->write(static_cast<uint32_t>(value.cameras.size()));
	for (const auto &camera : value.cameras) {
		this->write(static_cast<uint32_t>(camera.mosaic_idx));
		this->write(static_cast<uint32_t>(camera.camera_idx));
		this->write(camera.Tcv_world_cam);
		this->write(static_cast<uint8_t>(camera.optimized ? 1 : 0));
		this->write(static_cast<uint8_t>(camera.is_2dof ? 1 : 0));
	}

	this->write(static_cast<uint32_t>(value.keyframes.size()));
	for (const auto &keyframe : value.keyframes) {
		this->write(static_cast<uint64_t>(keyframe.timestamp_ns));
		this->write(keyframe.Tcv_world_device);
		this->write(keyframe.velocity_m_s);
	}
}

void
DataSerializer::read(rig_calibration::CalibrationResult &value)
{
	uint8_t device_id;
	this->read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	this->read(value.imu.accel_bias);
	this->read(value.imu.gyro_bias);
	this->read(value.imu.accel_scale);
	this->read(value.imu.Qcv_imu_model);
	this->read(value.gravity_mag);

	uint64_t num_residuals;
	this->read(num_residuals);
	value.imu_fitness.num_residuals = static_cast<size_t>(num_residuals);
	this->read(value.imu_fitness.chi_squared);
	this->read(num_residuals);
	value.camera_fitness.num_residuals = static_cast<size_t>(num_residuals);
	this->read(value.camera_fitness.chi_squared);

	this->read(value.initial_cost);
	this->read(value.final_cost);
	uint32_t iterations;
	this->read(iterations);
	value.iterations = static_cast<int>(iterations);

	uint32_t camera_count;
	this->read(camera_count);
	value.cameras.resize(camera_count);
	for (auto &camera : value.cameras) {
		uint32_t index;
		this->read(index);
		camera.mosaic_idx = index;
		this->read(index);
		camera.camera_idx = index;
		this->read(camera.Tcv_world_cam);

		uint8_t flag;
		this->read(flag);
		camera.optimized = flag != 0;
		this->read(flag);
		camera.is_2dof = flag != 0;
	}

	uint32_t keyframe_count;
	this->read(keyframe_count);
	value.keyframes.resize(keyframe_count);
	for (auto &keyframe : value.keyframes) {
		uint64_t timestamp_ns;
		this->read(timestamp_ns);
		keyframe.timestamp_ns = static_cast<timepoint_ns>(timestamp_ns);
		this->read(keyframe.Tcv_world_device);
		this->read(keyframe.velocity_m_s);
	}
}

// DatasetCalibrationEvent

void
DataSerializer::write(const DatasetCalibrationEvent &value)
{
	this->write(static_cast<uint8_t>(value.kind));
	this->write(static_cast<uint64_t>(value.timestamp_ns));

	switch (value.kind) {
	case CalibrationEventKind::EndCollecting: this->write(value.num_samples); break;
	case CalibrationEventKind::Solved: this->write(value.result.value()); break;
	default: break;
	}
}

void
DataSerializer::read(DatasetCalibrationEvent &value)
{
	uint8_t kind;
	this->read(kind);
	value.kind = static_cast<CalibrationEventKind>(kind);

	uint64_t timestamp_ns;
	this->read(timestamp_ns);
	value.timestamp_ns = static_cast<timepoint_ns>(timestamp_ns);

	switch (value.kind) {
	case CalibrationEventKind::EndCollecting: this->read(value.num_samples); break;
	case CalibrationEventKind::Solved: {
		rig_calibration::CalibrationResult result{};
		this->read(result);
		value.result = std::move(result);
		break;
	}
	default: break;
	}
}

// DatasetOutputPose

void
DataSerializer::write(const DatasetOutputPose &value)
{
	this->write(static_cast<uint8_t>(value.device_id));
	this->write(static_cast<uint64_t>(value.query_ns));
	this->write(static_cast<uint64_t>(value.when_ns));
	this->write(static_cast<uint64_t>(value.state_ns));
	this->write(static_cast<uint32_t>(value.relation.relation_flags));
	this->write(value.relation.pose);
	this->write(value.relation.linear_velocity);
	this->write(value.relation.angular_velocity);
}

void
DataSerializer::read(DatasetOutputPose &value)
{
	uint8_t device_id;
	this->read(device_id);
	value.device_id = static_cast<t_constellation_device_id_t>(device_id);

	uint64_t ns;
	this->read(ns);
	value.query_ns = static_cast<timepoint_ns>(ns);
	this->read(ns);
	value.when_ns = static_cast<timepoint_ns>(ns);
	this->read(ns);
	value.state_ns = static_cast<timepoint_ns>(ns);

	uint32_t flags;
	this->read(flags);
	value.relation.relation_flags = static_cast<xrt_space_relation_flags>(flags);
	this->read(value.relation.pose);
	this->read(value.relation.linear_velocity);
	this->read(value.relation.angular_velocity);
}

// Sized packets

void
DataSerializer::beginSizedPacket(uint8_t packet_type)
{
	this->write(packet_type);
	this->sized_packet_length_pos = this->stream->tellp();
	// Placeholder, filled in by endSizedPacket.
	this->write(static_cast<uint32_t>(0));
}

void
DataSerializer::endSizedPacket()
{
	std::streampos end_pos = this->stream->tellp();
	std::streamoff payload_length = end_pos - this->sized_packet_length_pos - std::streamoff(sizeof(uint32_t));

	this->stream->seekp(this->sized_packet_length_pos);
	this->write(static_cast<uint32_t>(payload_length));
	this->stream->seekp(end_pos);
	throw_if_stream_failed(*this->stream, "Failed to seek in dataset file.");
}

std::streampos
DataSerializer::tell()
{
	return this->stream->tellg();
}

void
DataSerializer::seek(std::streampos pos)
{
	this->stream->seekg(pos);
	throw_if_stream_failed(*this->stream, "Failed to seek in dataset file.");
}

// Flush

void
DataSerializer::flush()
{
	this->stream->flush();
}

/*
 *
 * DataRecorder implementations
 *
 */

DataRecorder::DataRecorder(ConstellationTracker *tracker, std::string out_file)
    : out(out_file, std::ios::binary | std::ios::out)
{
	if (!this->out.is_open()) {
		throw std::runtime_error("Failed to open file for data recording.");
	}

	DataSerializer header;

	// Magic value to identify the file so we can prevent loading invalid files. Written in BE to preserve the
	// match with the source code.
	header.write(static_cast<uint32_t>(__be32_to_cpu(CT_DATA_MAGIC_VALUE)));

	header.write(static_cast<uint32_t>(tracker->mosaics.size()));
	for (auto &mosaic : tracker->mosaics) {
		header.write(static_cast<uint32_t>(mosaic->cameras.size()));
		for (auto &camera : mosaic->cameras) {
			header.write(camera->calibration);
		}
	}

	// Once, before any tracking thread runs, so it can be written here.
	const std::string bytes = header.takeBuffer();
	this->out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
	this->out.flush();

	this->writer = std::thread([this] { this->writerLoop(); });
}

DataRecorder::~DataRecorder()
{
	{
		std::lock_guard<std::mutex> guard(this->queue_lock);
		this->stopping = true;
	}
	this->queue_cv.notify_one();

	if (this->writer.joinable()) {
		this->writer.join();
	}
}

void
DataRecorder::push(std::string &&packet)
{
	{
		std::lock_guard<std::mutex> guard(this->queue_lock);
		this->queue.push_back(std::move(packet));
	}
	this->queue_cv.notify_one();
}

void
DataRecorder::writerLoop()
{
	std::unique_lock<std::mutex> guard(this->queue_lock);
	while (true) {
		this->queue_cv.wait(guard, [this] { return this->stopping || !this->queue.empty(); });

		if (this->queue.empty() && this->stopping) {
			break;
		}

		std::deque<std::string> batch;
		batch.swap(this->queue);
		guard.unlock();

		for (const std::string &packet : batch) {
			this->out.write(packet.data(), static_cast<std::streamsize>(packet.size()));
		}
		// Only flush once caught up, a recording that is cut off is still readable up to there.
		this->out.flush();

		guard.lock();
	}
}

void
DataRecorder::recordSample(const CameraSample &sample)
{
	DataSerializer packet;
	packet.write(static_cast<uint8_t>(PACKET_TYPE_CAMERA_SAMPLE));
	packet.write(sample);
	this->push(packet.takeBuffer());
}

void
DataRecorder::recordDeviceInfo(const Device &device)
{
	DataSerializer packet;
	packet.write(static_cast<uint8_t>(PACKET_TYPE_DEVICE_INFO));
	packet.write(static_cast<uint8_t>(device.id));
	packet.write(device.params.led_model);
	this->push(packet.takeBuffer());
}

void
DataRecorder::recordImuSample(t_constellation_device_id_t device_id, const xrt_imu_sample &sample)
{
	DataSerializer packet;
	packet.beginSizedPacket(PACKET_TYPE_IMU_SAMPLE);
	packet.write(DatasetImuSample{device_id, sample});
	packet.endSizedPacket();
	this->push(packet.takeBuffer());
}

void
DataRecorder::recordSolve(const DatasetSolve &solve)
{
	DataSerializer packet;
	packet.beginSizedPacket(PACKET_TYPE_DEVICE_SOLVE);
	packet.write(solve);
	packet.endSizedPacket();
	this->push(packet.takeBuffer());
}

void
DataRecorder::recordOutputPose(const DatasetOutputPose &pose)
{
	DataSerializer packet;
	packet.beginSizedPacket(PACKET_TYPE_OUTPUT_POSE);
	packet.write(pose);
	packet.endSizedPacket();
	this->push(packet.takeBuffer());
}

void
DataRecorder::recordCalibrationEvent(const DatasetCalibrationEvent &event)
{
	DataSerializer packet;
	packet.beginSizedPacket(PACKET_TYPE_CALIBRATION_EVENT);
	packet.write(event);
	packet.endSizedPacket();
	this->push(packet.takeBuffer());
}

/*
 *
 * DatasetReader implementations
 *
 */

DatasetReader::DatasetReader(std::string filename) : serializer(filename, false)
{
	uint32_t magic_value;
	this->serializer.read(magic_value);
	if (magic_value != static_cast<uint32_t>(__be32_to_cpu(CT_DATA_MAGIC_VALUE))) {
		throw std::runtime_error(
		    "Invalid magic value in dataset file, not a valid constellation tracker dataset.");
	}

	uint32_t mosaic_count;
	this->serializer.read(mosaic_count);

	for (size_t i = 0; i < mosaic_count; i++) {
		DatasetMosaic mosaic;

		uint32_t camera_count;
		this->serializer.read(camera_count);

		for (size_t j = 0; j < camera_count; j++) {
			t_camera_calibration calibration;
			this->serializer.read(calibration);
			mosaic.camera_calibrations.push_back(calibration);
		}

		this->mosaics.push_back(mosaic);
	}

	while (true) {
		try {
			uint8_t packet_type;
			this->serializer.read(packet_type);
			switch (packet_type) {
			case PACKET_TYPE_CAMERA_SAMPLE: {
				CameraSample sample;
				this->serializer.read(sample);

				// Not recorded, the blobwatch derives it from the undistorted center and the
				// intrinsics. The correspondence search and RANSAC use it, so restore it the same way.
				if (sample.mosaic_index < this->mosaics.size() &&
				    sample.camera_index <
				        this->mosaics[sample.mosaic_index].camera_calibrations.size()) {
					const t_camera_calibration &calib =
					    this->mosaics[sample.mosaic_index].camera_calibrations[sample.camera_index];
					const double fx = calib.intrinsics[0][0];
					const double fy = calib.intrinsics[1][1];
					const double cx = calib.intrinsics[0][2];
					const double cy = calib.intrinsics[1][2];

					for (uint32_t i = 0; i < sample.blob_count; i++) {
						t_blob &b = sample.blobs[i];
						b.center_homogenized.x =
						    static_cast<float>((b.center_undistorted.x - cx) / fx);
						b.center_homogenized.y =
						    static_cast<float>((b.center_undistorted.y - cy) / fy);
						b.center_homogenized.z = 1.0f;
					}
				}

				this->samples.push_back(sample);
				break;
			}
			case PACKET_TYPE_DEVICE_INFO: {
				DatasetDevice &device = this->devices.emplace_back();

				uint8_t device_id;
				this->serializer.read(device_id);
				device.id = static_cast<t_constellation_device_id_t>(device_id);

				this->serializer.read(device.leds, device.led_model);

				// The recorder writes the model the tracker holds, which it flipped into OpenCV space.
				// Hand back what was given to the tracker, so that adding the device again doesn't flip
				// it a second time. The conversion is its own inverse.
				for (auto &led : device.leds) {
					math_vec3_convert_from_opencv(&led.position, &led.position);
					math_vec3_convert_from_opencv(&led.normal, &led.normal);
				}

				break;
			}
			default: {
				if (packet_type < PACKET_TYPE_FIRST_SIZED) {
					throw std::runtime_error("Invalid packet type in dataset file. Got ID " +
					                         std::to_string(packet_type));
				}

				uint32_t payload_length;
				this->serializer.read(payload_length);
				std::streampos payload_start = this->serializer.tell();

				// Read into a local first, a recording cut off mid-packet must not leave a half-read
				// entry.
				switch (packet_type) {
				case PACKET_TYPE_IMU_SAMPLE: {
					DatasetImuSample imu_sample{};
					this->serializer.read(imu_sample);
					this->imu_samples.push_back(imu_sample);
					break;
				}
				case PACKET_TYPE_DEVICE_SOLVE: {
					DatasetSolve solve{};
					this->serializer.read(solve);
					this->solves.push_back(std::move(solve));
					break;
				}
				case PACKET_TYPE_CALIBRATION_EVENT: {
					DatasetCalibrationEvent event{};
					this->serializer.read(event);
					this->calibration_events.push_back(std::move(event));
					break;
				}
				case PACKET_TYPE_OUTPUT_POSE: {
					DatasetOutputPose pose{};
					this->serializer.read(pose);
					this->output_poses.push_back(pose);
					break;
				}
				default:
					// From a newer recorder, skip it.
					break;
				}

				// Skips unknown packets, and any fields a newer recorder appended to known ones.
				this->serializer.seek(payload_start + std::streamoff(payload_length));
				break;
			}
			}
		} catch (const std::exception &e) {
			break;
		}
	}
}

}; // namespace xrt::tracking::constellation
