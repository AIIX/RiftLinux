// Copyright 2025, Beyley Cardellio
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to Oculus Rift driver code.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#pragma once

#include "xrt/xrt_byte_order.h"

#include "util/u_device.h"
#include "util/u_logging.h"
#include "util/u_distortion_mesh.h"

#include "math/m_imu_3dof.h"
#include "math/m_api.h"
#include "math/m_mathinclude.h"
#include "math/m_clock_tracking.h"
#include "math/m_filter_fifo.h"
#include "math/m_gyro_rest_bias.h"

#include "tracking/t_imu.h"
#include "tracking/t_constellation.h"

#include "os/os_hid.h"
#include "os/os_threading.h"

#include "tracking/t_time_sync.h"

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "rift_interface.h"


#define HMD_TRACE(hmd, ...) U_LOG_XDEV_IFL_T(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_DEBUG(hmd, ...) U_LOG_XDEV_IFL_D(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_WARN(hmd, ...) U_LOG_XDEV_IFL_W(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

#define REPORT_MAX_SIZE 69                // max size of a feature report (FEATURE_REPORT_CALIBRATE)
#define KEEPALIVE_INTERVAL_NS 10000000000 // 10 seconds
// give a 5% breathing room (at 10 seconds, this is 500 milliseconds of breathing room)
#define KEEPALIVE_SEND_RATE_NS ((KEEPALIVE_INTERVAL_NS * 19) / 20)
#define IMU_SAMPLE_RATE (1000)      // 1000hz
#define NS_PER_SAMPLE (1000 * 1000) // 1ms (1,000,000 ns) per sample
#define SERIAL_NUMBER_LENGTH 14
#define RIFT_USB_LATENCY_BIAS (U_TIME_1US_IN_NS * 200LL) // 200us latency bias over USB
#define RIFT_RADIO_LATENCY_BIAS (U_TIME_1MS_IN_NS * 4LL) // 4ms bias over radio

#define CALIBRATION_HASH_BYTE_OFFSET 0x1bf0
#define CALIBRATION_HASH_BYTE_LENGTH 0x10

#define RIFT_CONFIG_SUBDIR "rift"

#define CALIBRATION_HEADER_BYTE_OFFSET 0x0
#define CALIBRATION_HEADER_BYTE_LENGTH 0x4

#define CALIBRATION_BODY_BYTE_OFFSET 0x4
#define CALIBRATION_BODY_BYTE_CHUNK_LENGTH 0x14

#define MICROMETERS_TO_METERS(microns) ((float)microns / 1000000.0f)

// value taken from LibOVR 0.4.4
#define DEFAULT_EXTRA_EYE_ROTATION DEG_TO_RAD(30.0f)

#define IN_REPORT_DK2 11                 // sent on the HMD HID interface
#define IN_REPORT_RADIO_DATA 12          // sent on the radio HID interface
#define IN_REPORT_CV1_RADIO_KEEPALIVE 13 // sent on the HMD HID interface when no devices are connected

#define IN_REPORT_RADIO_DATA_SIZE 64
#define RADIO_STATE_TOUCH_CONTROLLERS_LEN 3

#define RIFT_LED_SIZE_M 0.0035f                   // 3.5mm
#define RIFT_LED_VISIBILITY_RAD DEG_TO_RAD(90.0f) // TODO: tune this value properly

#ifdef __cplusplus
extern "C" {
#endif

// asserts the size of a type is equal to the byte size provided
#define SIZE_ASSERT(type, size)                                                                                        \
	static_assert(sizeof(type) == (size), "Size of " #type " is not " #size " bytes as was expected")

enum rift_feature_reports
{
	// DK1
	FEATURE_REPORT_CONFIG = 2,         // get + set
	FEATURE_REPORT_CALIBRATE = 3,      // get + set
	FEATURE_REPORT_RANGE = 4,          // get + set
	FEATURE_REPORT_REGISTER = 5,       // get + set
	FEATURE_REPORT_DFU = 6,            // get + set
	FEATURE_REPORT_DK1_KEEP_ALIVE = 8, // get + set
	FEATURE_REPORT_DISPLAY_INFO = 9,   // get + set
	FEATURE_REPORT_SERIAL = 10,        // get + set

	// DK2
	FEATURE_REPORT_TRACKING = 12,        // get + set
	FEATURE_REPORT_DISPLAY = 13,         // get + set
	FEATURE_REPORT_MAG_CALIBRATION = 14, // get + set
	FEATURE_REPORT_POS_CALIBRATION = 15, // get + set
	FEATURE_REPORT_CUSTOM_PATTERN = 16,  // get + set
	FEATURE_REPORT_KEEPALIVE_MUX = 17,   // get + set
	FEATURE_REPORT_MANUFACTURING = 18,   // get + set
	FEATURE_REPORT_UUID = 19,            // get + set
	FEATURE_REPORT_TEMPERATURE = 20,     // get + set
	FEATURE_REPORT_GYROOFFSET = 21,      // get only
	FEATURE_REPORT_LENS_DISTORTION = 22, // get + set

	// CV1
	FEATURE_REPORT_RADIO_CONTROL = 26,       // get + set
	FEATURE_REPORT_RADIO_READ_DATA_CMD = 27, // @todo: get + ???
	FEATURE_REPORT_ENABLE_COMPONENTS = 29,   // @todo: ??? + set
};

enum rift_config_report_flags
{
	// output the sample data raw from the sensors without converting them to known units
	RIFT_CONFIG_REPORT_USE_RAW = 1,
	// internal test mode for calibrating zero rate drift on gyro
	RIFT_CONFIG_REPORT_INTERNAL_CALIBRATION = 1 << 1,
	// use the calibration parameters stored on the device
	RIFT_CONFIG_REPORT_USE_CALIBRATION = 1 << 2,
	// recalibrate the gyro zero rate offset when the device is stationary
	RIFT_CONFIG_REPORT_AUTO_CALIBRATION = 1 << 3,
	// stop sending IN reports when the device has stopped moving for Interval milliseconds
	RIFT_CONFIG_REPORT_MOTION_KEEP_ALIVE = 1 << 4,
	// stop sending IN reports when the device has stopped receiving feature reports for Interval milliseconds
	RIFT_CONFIG_REPORT_COMMAND_KEEP_ALIVE = 1 << 5,
	// output the IN report data in the coordinate system used by LibOVR relative to the tracker, otherwise, report
	// in the coordinate system of the device
	RIFT_CONFIG_REPORT_USE_SENSOR_COORDINATES = 1 << 6,
	// override the power state of the USB hub, forcing it to act as if the external power source is connected (DK2
	// only, does nothing on DK1)
	RIFT_CONFIG_REPORT_OVERRIDE_POWER = 1 << 7,
};

enum rift_distortion_type
{
	RIFT_DISTORTION_TYPE_DIMS = 1,
	RIFT_DISTORTION_TYPE_K = 2,
};

enum rift_lens_type
{
	// firmware indirectly states lens type A is 0
	RIFT_LENS_TYPE_A = 0,
	// firmware does not state what lens type B is, 1 is an educated guess
	RIFT_LENS_TYPE_B = 1,
};

enum rift_lens_distortion_version
{
	// no distortion data is stored
	RIFT_LENS_DISTORTION_NONE = 0,
	// standard distortion matrix
	RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1 = 1,
};

enum rift_component_flags
{
	RIFT_COMPONENT_DISPLAY = 1 << 0,
	RIFT_COMPONENT_AUDIO = 1 << 1,
	RIFT_COMPONENT_LEDS = 1 << 2,
};

/*
 *
 * Packed structs for USB communication
 *
 */

#pragma pack(push, 1)

struct rift_config_report
{
	uint16_t command_id;
	uint8_t config_flags;
	// the IN report rate of the headset, rate is calculated as `sample_rate / (1 + interval)`
	uint8_t interval;
	// sample rate of the IMU, always 1000hz on DK1/DK2, read-only
	uint16_t sample_rate;
};

SIZE_ASSERT(struct rift_config_report, 6);

struct rift_display_info_report
{
	uint16_t command_id;
	uint8_t distortion_type;
	// the horizontal resolution of the display, in pixels
	uint16_t resolution_x;
	// the vertical resolution of the display, in pixels
	uint16_t resolution_y;
	// width in micrometers
	uint32_t display_width;
	// height in micrometers
	uint32_t display_height;
	// the vertical center of the display, in micrometers
	uint32_t center_v;
	// the separation between the two lenses, in micrometers
	uint32_t lens_separation;
	uint32_t lens_distance[2];
	float distortion[6];
};

SIZE_ASSERT(struct rift_display_info_report, 55);

#define CATMULL_COEFFICIENTS 11

struct rift_catmull_rom_distortion_report_data
{
	// eye relief setting, in micrometers from front surface of lens
	uint16_t eye_relief;
	// the k coeffecients of the distortion
	uint16_t k[CATMULL_COEFFICIENTS];
	uint16_t max_r;
	uint16_t meters_per_tan_angle_at_center;
	uint16_t chromatic_abberation[U_RIFT_CHROMATIC_ABBERATION_COUNT];
	uint8_t unused[14];
};

SIZE_ASSERT(struct rift_catmull_rom_distortion_report_data, 50);

struct rift_lens_distortion_report
{
	uint16_t command_id;
	// the amount of distortions on this device
	uint8_t num_distortions;
	// the index of this distortion in the devices array
	uint8_t distortion_idx;
	// unused bitmask field
	uint8_t bitmask;
	// the type of the lenses
	uint16_t lens_type;
	// the version of the lens distortion data
	uint16_t distortion_version;

	union {
		struct rift_catmull_rom_distortion_report_data lcsv_catmull_rom_10;
	} data;
};

SIZE_ASSERT(struct rift_lens_distortion_report, 9 + sizeof(struct rift_catmull_rom_distortion_report_data));

enum rift_position_calibration_version
{
	// no data stored
	RIFT_POSITION_CALIBRATION_VERSION_NONE = 0,
	// hard-coded default positions
	RIFT_POSITION_CALIBRATION_VERSION_DEFAULT = 1,
	// factory calibrated
	RIFT_POSITION_CALIBRATION_VERSION_FACTORY = 2,
	// user calibrated
	RIFT_POSITION_CALIBRATION_VERSION_USER = 3,
};

enum rift_position_calibration_type
{
	RIFT_POSITION_CALIBRATION_TYPE_LED = 0,
	RIFT_POSITION_CALIBRATION_TYPE_INERTIAL_SENSOR = 1,
};

struct rift_position_calibration_report
{
	uint16_t command_id;
	// the version/type of calibration, see rift_position_calibration_version
	uint8_t version;
	// the x/y/z position of the object, this is a signed integer in micrometers, position is relative to the center
	// of the emitter plane of the display at nominal focus.
	int32_t position[3];
	// the x/y/z axis normal of the object, this is a signed integer in micrometers, normal is relative to the
	// position
	int16_t normal[3];
	// rotation around the normal, in units of 10^-4 radians
	uint16_t rotation;
	// the current position in the array of LEDs, increments on reads, gets set to the value on writes
	uint16_t position_index;
	// read-only value of the number of LEDs
	uint16_t position_count;
	// the type of the object being described, see rift_position_calibration_type
	uint16_t position_type;
};

SIZE_ASSERT(struct rift_position_calibration_report, 29);

enum rift_custom_pattern_state
{
	RIFT_CUSTOM_PATTERN_STAT_OFF = 0,
	RIFT_CUSTOM_PATTERN_STAT_LOW = 1,
	RIFT_CUSTOM_PATTERN_STAT_HIGH = 3,
};

struct rift_custom_pattern_report
{
	uint16_t command_id;
	// the length of the sequence that each LED goes through
	uint8_t sequence_length;
	// the sequence the specific LED goes through, 2 bits per state, 0 (off), 1 (low), and 3 (high), ordered from
	// LSB to MSB
	uint32_t sequence;
	// the current LED being described, increments on reads, gets set to the value on writes
	uint16_t led_index;
	// the number of tracking LEDs present on the device
	uint16_t led_count;
};

SIZE_ASSERT(struct rift_custom_pattern_report, 11);

struct rift_dk2_keepalive_mux_report
{
	uint16_t command;
	uint8_t in_report;
	uint16_t interval;
};

SIZE_ASSERT(struct rift_dk2_keepalive_mux_report, 5);

enum rift_display_mode
{
	RIFT_DISPLAY_MODE_GLOBAL,
	RIFT_DISPLAY_MODE_ROLLING_TOP_BOTTOM,
	RIFT_DISPLAY_MODE_ROLLING_LEFT_RIGHT,
	RIFT_DISPLAY_MODE_ROLLING_RIGHT_LEFT,
};

enum rift_display_limit
{
	RIFT_DISPLAY_LIMIT_ACL_OFF = 0,
	RIFT_DISPLAY_LIMIT_ACL_30 = 1,
	RIFT_DISPLAY_LIMIT_ACL_25 = 2,
	RIFT_DISPLAY_LIMIT_ACL_50 = 3,
};

enum rift_display_flags
{
	RIFT_DISPLAY_USE_ROLLING = 1 << 6,
	RIFT_DISPLAY_REVERSE_ROLLING = 1 << 7,
	RIFT_DISPLAY_HIGH_BRIGHTNESS = 1 << 8,
	RIFT_DISPLAY_SELF_REFRESH = 1 << 9,
	RIFT_DISPLAY_READ_PIXEL = 1 << 10,
	RIFT_DISPLAY_DIRECT_PENTILE = 1 << 11,
};

struct rift_display_report
{
	uint16_t command_id;
	// relative brightness setting independent of pixel persistence, only effective when high brightness is disabled
	uint8_t brightness;
	// a set of flags, ordered from LSB -> MSB
	// - panel mode/shutter type (4 bits), read only, see rift_display_mode
	// - current limit (2 bits), see rift_display_limit
	// - use rolling (1 bit)
	// - reverse rolling (1 bit), unavailable on released DK2 firmware for unknown reason
	// - high brightness (1 bit), unavailable on released DK2 firmware for unpublished reason
	// - self refresh (1 bit)
	// - read pixel (1 bit)
	// - direct pentile (1 bit)
	uint32_t flags;
	// the length of time in rows that the display is lit each frame, defaults to the full size of the display, full
	// persistence
	uint16_t persistence;
	// the offset in rows from vsync that the panel is lit when using global shutter, no effect in rolling shutter,
	// disabled on released DK2 firmware for unknown reason
	uint16_t lighting_offset;
	// the time in microseconds it is estimated for a pixel to settle to one value after it is set, read only
	uint16_t pixel_settle;
	// the number of rows including active area and blanking period used with persistence and lightingoffset, read
	// only
	uint16_t total_rows;
};

SIZE_ASSERT(struct rift_display_report, 15);

struct rift_dk2_sensor_sample
{
	uint8_t data[8];
};

SIZE_ASSERT(struct rift_dk2_sensor_sample, 8);

struct rift_dk2_sample_pack
{
	struct rift_dk2_sensor_sample accel;
	struct rift_dk2_sensor_sample gyro;
};

SIZE_ASSERT(struct rift_dk2_sample_pack, sizeof(struct rift_dk2_sensor_sample) * 2);

struct rift_dk2_version_data
{
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
};

SIZE_ASSERT(struct rift_dk2_version_data, 6);

struct rift_cv1_version_data
{
	uint16_t presence_sensor;
	uint16_t iad_adc_value;
	uint16_t unk;
};

SIZE_ASSERT(struct rift_cv1_version_data, 6);
static_assert(sizeof(struct rift_cv1_version_data) == sizeof(struct rift_dk2_version_data),
              "Incorrect version data size");

#define DK2_MAX_SAMPLES 2
struct dk2_in_report
{
	uint16_t command_id;
	uint8_t num_samples;
	uint16_t sample_count;
	uint16_t temperature;
	uint32_t sample_timestamp;
	struct rift_dk2_sample_pack samples[DK2_MAX_SAMPLES];
	union {
		struct rift_dk2_version_data dk2;
		struct rift_cv1_version_data cv1;
	};
	uint16_t frame_count;
	uint32_t frame_timestamp;
	uint8_t frame_id;
	uint8_t tracking_pattern;
	uint16_t tracking_count;
	uint32_t tracking_timestamp;
};

SIZE_ASSERT(struct dk2_in_report, 63);

struct rift_enable_components_report
{
	uint16_t command_id;
	// which components to enable, see rift_component_flags
	uint8_t flags;
};

SIZE_ASSERT(struct rift_enable_components_report, 3);

struct rift_imu_calibration_report
{
	uint16_t command_id;
	struct rift_dk2_sample_pack offset;
	struct rift_dk2_sample_pack matrix_samples[3];
	uint16_t temperature;
};

SIZE_ASSERT(struct rift_imu_calibration_report, sizeof(struct rift_dk2_sample_pack) * 4 + 4);

enum rift_radio_read_cmd
{
	RIFT_RADIO_READ_CMD_FLASH_CONTROL = 0x0a,
	RIFT_RADIO_READ_CMD_SERIAL = 0x88,
};

struct rift_radio_cmd_report
{
	uint16_t command_id;
	uint8_t a;
	uint8_t b;
	uint8_t c;
};

SIZE_ASSERT(struct rift_radio_cmd_report, 5);

struct rift_radio_data_read_cmd
{
	uint16_t command_id;
	uint16_t offset;
	uint16_t length;
	uint8_t unk[28];
};

SIZE_ASSERT(struct rift_radio_data_read_cmd, 34);

struct rift_radio_flash_read_response_header
{
	uint8_t unk[5];
	__le16 data_length;
};
SIZE_ASSERT(struct rift_radio_flash_read_response_header, 7);

struct rift_radio_address_radio_report
{
	uint16_t command_id;
	uint8_t radio_address[5];
};

SIZE_ASSERT(struct rift_radio_address_radio_report, 7);

enum rift_radio_report_remote_button_masks
{
	RIFT_REMOTE_BUTTON_MASK_DPAD_UP = 0x001,
	RIFT_REMOTE_BUTTON_MASK_DPAD_DOWN = 0x002,
	RIFT_REMOTE_BUTTON_MASK_DPAD_LEFT = 0x004,
	RIFT_REMOTE_BUTTON_MASK_DPAD_RIGHT = 0x008,
	RIFT_REMOTE_BUTTON_MASK_SELECT = 0x010,
	RIFT_REMOTE_BUTTON_MASK_VOLUME_UP = 0x020,
	RIFT_REMOTE_BUTTON_MASK_VOLUME_DOWN = 0x040,
	RIFT_REMOTE_BUTTON_MASK_OCULUS = 0x080,
	RIFT_REMOTE_BUTTON_MASK_BACK = 0x100,
};

struct rift_radio_report_remote_message
{
	// the button state of the controller, see rift_radio_report_remote_button_masks
	uint16_t buttons;
};

SIZE_ASSERT(struct rift_radio_report_remote_message, 2);

enum rift_radio_report_touch_buttons
{
	RIFT_TOUCH_CONTROLLER_BUTTON_A = 0x01,
	RIFT_TOUCH_CONTROLLER_BUTTON_X = 0x01,
	RIFT_TOUCH_CONTROLLER_BUTTON_B = 0x02,
	RIFT_TOUCH_CONTROLLER_BUTTON_Y = 0x02,
	RIFT_TOUCH_CONTROLLER_BUTTON_MENU = 0x04,
	RIFT_TOUCH_CONTROLLER_BUTTON_OCULUS = 0x04,
	RIFT_TOUCH_CONTROLLER_BUTTON_STICK = 0x08,
};

enum rift_radio_report_adc_channel
{
	RIFT_TOUCH_CONTROLLER_ADC_STICK = 0x01,
	RIFT_TOUCH_CONTROLLER_ADC_B_Y = 0x02,
	RIFT_TOUCH_CONTROLLER_ADC_TRIGGER = 0x03,
	RIFT_TOUCH_CONTROLLER_ADC_A_X = 0x04,
	RIFT_TOUCH_CONTROLLER_ADC_THUMBREST = 0x08,
	// seen with values varying per controller, maybe power draw? temperature? my left controller while powered on
	// had the value slowly rise from 2800 to 3000 over the span of a couple minutes, dunno what that could be tbh
	RIFT_TOUCH_CONTROLLER_ADC_UNK1 = 0x20,
	RIFT_TOUCH_CONTROLLER_ADC_BATTERY = 0x21,
	RIFT_TOUCH_CONTROLLER_ADC_HAPTIC_COUNTER = 0x23,
};

struct rift_radio_report_touch_message
{
	uint32_t timestamp;
	int16_t accel[3];
	int16_t gyro[3];
	uint8_t buttons;
	uint8_t touch_grip_stick_state[5];
	// see rift_radio_report_adc_channel
	uint8_t adc_channel;
	uint16_t adc_value;
};

SIZE_ASSERT(struct rift_radio_report_touch_message, 25);

enum rift_radio_device_type
{
	RIFT_RADIO_DEVICE_REMOTE = 1,
	RIFT_RADIO_DEVICE_LEFT_TOUCH = 2,
	RIFT_RADIO_DEVICE_RIGHT_TOUCH = 3,
	RIFT_RADIO_DEVICE_TRACKED_OBJECT = 6,
};

struct rift_radio_report_message
{
	uint16_t flags;
	// the type of device sending the message, see rift_radio_device_type
	uint8_t device_type;
	union {
		struct rift_radio_report_remote_message remote;
		struct rift_radio_report_touch_message touch;
	};
};

SIZE_ASSERT(struct rift_radio_report_message, 3 + sizeof(struct rift_radio_report_touch_message));

struct rift_radio_report
{
	uint16_t command_id;
	struct rift_radio_report_message messages[2];
};

enum rift_tracking_flags
{
	// enable the tracking LED exposure and updating
	RIFT_TRACKING_ENABLE = 1 << 0,
	// automatically increment the pattern index after each exposure
	RIFT_TRACKING_AUTO_INCREMENT = 1 << 1,
	// modulate the tracking LEDs at 85kHz to allow wireless sync, defaults to on
	RIFT_TRACKING_USE_CARRIER = 1 << 2,
	// trigger LED exposure using a rising edge of GPIO1, else triggered on a timer
	RIFT_TRACKING_SYNC_INPUT = 1 << 3,
	// trigger LED exposure on each vsync rather than an internal or external timer
	RIFT_TRACKING_VSYNC_LOCK = 1 << 4,
	// use the custom pattern given to the headset
	RIFT_TRACKING_CUSTOM_PATTERN = 1 << 5,
};

struct rift_tracking_report
{
	uint16_t command_id;
	// the index of the current pattern being flashed, pattern 255 is reserved for "all high"
	uint8_t pattern_idx;
	// the enabled tracking flags, see rift_tracking_flags
	uint16_t flags;
	// the amount of time to enable the LEDs for during an exposure, sync output also follows this length, cannot be
	// longer than frame_interval, and has a minimum of 10 microseconds
	uint16_t exposure_length;
	// when SYNCINPUT and VSYNC_LOCK are false, the tracking LEDs are exposed on the interval set here, in
	// microseconds
	uint16_t frame_interval;
	// when VSYNC_LOCK is true, this gives a fixed microsecond offset from the vsync to when the LEDs are triggered
	uint16_t vsync_offset;
	// the duty cycle of the 85kHz modulation, defaults to 128, resulting in a 50% duty cycle
	uint8_t duty_cycle;
};

SIZE_ASSERT(struct rift_tracking_report, 12);

#pragma pack(pop)

/*
 *
 * Parsed structs for internal use
 *
 */

struct rift_catmull_rom_distortion_data
{
	// the k coeffecients of the distortion
	float k[CATMULL_COEFFICIENTS];
	float max_r;
	float meters_per_tan_angle_at_center;
	float chromatic_abberation[U_RIFT_CHROMATIC_ABBERATION_COUNT];
};

struct rift_lens_distortion
{
	// the version of the lens distortion data
	uint16_t distortion_version;
	// eye relief setting, in meters from surface of lens
	float eye_relief;

	union {
		struct rift_catmull_rom_distortion_data lcsv_catmull_rom_10;
	} data;
};

struct rift_imu_calibration
{
	struct xrt_vec3 gyro_offset;
	struct xrt_vec3 accel_offset;
	struct xrt_matrix_3x3 gyro_matrix;
	struct xrt_matrix_3x3 accel_matrix;
	float temperature;
};

enum rift_touch_controller_input
{
	// left
	RIFT_TOUCH_CONTROLLER_INPUT_X_CLICK = 0,
	RIFT_TOUCH_CONTROLLER_INPUT_X_TOUCH = 1,
	RIFT_TOUCH_CONTROLLER_INPUT_Y_CLICK = 2,
	RIFT_TOUCH_CONTROLLER_INPUT_Y_TOUCH = 3,
	RIFT_TOUCH_CONTROLLER_INPUT_SYSTEM_CLICK = 4,
	// right
	RIFT_TOUCH_CONTROLLER_INPUT_A_CLICK = 0,
	RIFT_TOUCH_CONTROLLER_INPUT_A_TOUCH = 1,
	RIFT_TOUCH_CONTROLLER_INPUT_B_CLICK = 2,
	RIFT_TOUCH_CONTROLLER_INPUT_B_TOUCH = 3,
	RIFT_TOUCH_CONTROLLER_INPUT_MENU_CLICK = 4,
	// both
	RIFT_TOUCH_CONTROLLER_INPUT_SQUEEZE_VALUE = 5,
	RIFT_TOUCH_CONTROLLER_INPUT_TRIGGER_TOUCH = 6,
	RIFT_TOUCH_CONTROLLER_INPUT_TRIGGER_VALUE = 7,
	RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK_CLICK = 8,
	RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK_TOUCH = 9,
	RIFT_TOUCH_CONTROLLER_INPUT_THUMBSTICK = 10,
	RIFT_TOUCH_CONTROLLER_INPUT_THUMBREST_TOUCH = 11,
	RIFT_TOUCH_CONTROLLER_INPUT_GRIP_POSE = 12,
	RIFT_TOUCH_CONTROLLER_INPUT_AIM_POSE = 13,
	RIFT_TOUCH_CONTROLLER_INPUT_TRIGGER_PROXIMITY = 14,
	RIFT_TOUCH_CONTROLLER_INPUT_THUMB_PROXIMITY = 15,
	//! Finger poses from what the controller senses, see rift_touch_controller_get_hand_tracking.
	RIFT_TOUCH_CONTROLLER_INPUT_HAND_TRACKING = 16,
	RIFT_TOUCH_CONTROLLER_INPUT_COUNT = 17,
};

struct rift_touch_controller_calibration
{
	uint16_t joy_x_range[2];
	uint16_t joy_x_dead[2];
	uint16_t joy_y_range[2];
	uint16_t joy_y_dead[2];

	// min - mid - max
	uint16_t trigger_range[3];

	// min - mid - max
	uint16_t middle_range[3];
	bool middle_flipped;

	uint16_t cap_sense_min[8];
	uint16_t cap_sense_touch[8];

	float gyro_calibration[3][3];
	struct xrt_vec3 gyro_offset;
	float accel_calibration[3][3];
	struct xrt_vec3 accel_offset;

	struct xrt_vec3 imu_position;

	struct t_constellation_tracker_led_model led_model;
};

struct rift_touch_controller_input_state
{
	uint8_t buttons;
	float trigger;
	float grip;
	struct xrt_vec2 stick;
	uint8_t haptic_counter;
	float cap_stick;
	float cap_b_y;
	float cap_a_x;
	float cap_trigger;
	float cap_thumbrest;
};

/*!
 * A Rift Touch controller device.
 *
 * @implements xrt_device
 */
//! IMU orientations kept for the tracker's prior, about 128 ms at the Touch's ~500 Hz: camera frames arrive that late.
#define RIFT_TOUCH_IMU_HISTORY 64

struct rift_touch_controller
{
	struct xrt_device base;
	struct xrt_frame_node node;

	struct rift_hmd *hmd;

	enum rift_radio_device_type device_type;

	bool use_constellation;

	bool constellation_mutex_created;
	struct os_mutex constellation_mutex;
	// The constellation related fields below should not be touched unless a lock is held on the above mutex.
	struct t_constellation_tracker *constellation_tracker;
	struct t_constellation_tracker_device constellation_device;
	struct t_constellation_tracker_tracking_source constellation_tracking_source;
	t_constellation_device_id_t constellation_device_id;
	struct xrt_imu_sink *constellation_imu_sink;

	/*!
	 * Set once the tracker and device ID above are filled in, after which they don't change. The pose functions
	 * read them through this, without the mutex: the tracker asks for the prior while holding its device lock,
	 * which adding the controller waits for with the mutex held.
	 */
	xrt_atomic_s32_t constellation_ready;
	//! The camera sample time of the newest constellation solve of the controller, 0 before the first.
	xrt_atomic_s64_t last_solve_ns;

	/*!
	 * Where the controller was last tracked, held while no camera sees it, see
	 * rift_touch_controller_get_tracked_pose. The mutex is a leaf: nothing else is locked while it is held, so the
	 * tracker's prior can read it under the tracker's own locks.
	 */
	struct
	{
		bool mutex_created;
		struct os_mutex mutex;
		bool have;
		struct xrt_vec3 position;
		//! Turns the orientation of the controller's own IMU fusion into the tracker's world.
		struct xrt_quat imu_to_world;
		//! The camera sample time of the solve the hold is anchored on.
		int64_t solve_ns;

		//! The IMU fusion's orientation over the last RIFT_TOUCH_IMU_HISTORY samples, oldest overwritten first.
		struct
		{
			int64_t timestamp_ns;
			struct xrt_quat orientation;
		} imu_history[RIFT_TOUCH_IMU_HISTORY];
		uint32_t imu_history_count;

		//! The app's last pose was the held one; the next tracked one slides in from it, see
		//! touch_get_app_relation.
		bool app_was_held;
		struct xrt_pose app_held_pose;
		int64_t blend_start_ns;
		struct xrt_pose blend_from;

		//! Where the app was last shown the controller tracked, for the held position to start from.
		bool have_app_tracked;
		struct xrt_vec3 app_tracked_position;
		//! Where the app sees the controller held this time, chosen when the hold starts.
		struct xrt_vec3 app_hold_position;
	} pose_hold;

	struct
	{
		bool mutex_created;
		struct os_mutex mutex;

		struct rift_touch_controller_input_state state;

		xrt_atomic_s32_t battery_status;

		uint32_t last_device_remote_us;
		timepoint_ns device_remote_ns;
		timepoint_ns device_local_ns;
		//! The controller is at rest and sends no IMU, see rift_touch_controller_handle_radio_input_report.
		bool imu_idle;

		struct imu_fusion *imu_fusion;
		struct xrt_imu_sample last_imu_sample;
		//! What the factory calibration leaves of the gyro's bias, measured whenever the controller lies still.
		struct m_gyro_rest_bias gyro_rest_bias;
		//! Whether the bias saved last session was looked for, and when and what was saved since.
		bool gyro_bias_loaded;
		timepoint_ns gyro_bias_saved_ns;
		struct xrt_vec3 gyro_bias_saved;

		struct m_clock_windowed_skew_tracker *clock_tracker;

		bool calibration_read;
		struct rift_touch_controller_calibration calibration;

		struct
		{
			timepoint_ns end_time_ns;
			bool high_freq;
			float amplitude;

			bool set_enabled;
			float set_amplitude;
			bool set_high_freq;
		} haptic;
	} input;

	//! Locked by radio_state.thread
	struct
	{
		bool serial_valid;

		uint8_t calibration_hash[CALIBRATION_HASH_BYTE_LENGTH];

		uint8_t calibration_data_buffer[CALIBRATION_BODY_BYTE_CHUNK_LENGTH];

		uint8_t *calibration_body_json;
		uint16_t calibration_body_json_length;

		//! When the controller's last report arrived, monotonic, 0 before the first.
		timepoint_ns last_report_ns;
		//! Whether it was given its setup since it last turned up, see rift_radio_handle_config.
		bool configured;
		//! When its active timeout was last sent.
		timepoint_ns keepalive_ns;
		//! No report for over 1.5 s (logged on change, a diagnostic).
		bool silent;
	} radio_data;
};

enum rift_remote_inputs
{
	RIFT_REMOTE_INPUT_DPAD_UP,
	RIFT_REMOTE_INPUT_DPAD_DOWN,
	RIFT_REMOTE_INPUT_DPAD_LEFT,
	RIFT_REMOTE_INPUT_DPAD_RIGHT,
	RIFT_REMOTE_INPUT_SELECT,
	RIFT_REMOTE_INPUT_VOLUME_UP,
	RIFT_REMOTE_INPUT_VOLUME_DOWN,
	RIFT_REMOTE_INPUT_BACK,
	RIFT_REMOTE_INPUT_OCULUS,
	RIFT_REMOTE_INPUT_COUNT,
};

/*!
 * A Rift Remote device.
 *
 * @implements xrt_device
 */
struct rift_remote
{
	struct xrt_device base;
	struct xrt_frame_node node;

	//! The button state of the remote, stored as an atomic to avoid needing a mutex.
	xrt_atomic_s32_t buttons;

	//! Locked by radio_state.thread
	bool serial_valid;
};

enum rift_radio_command
{
	RIFT_RADIO_COMMAND_NONE = 0,
	RIFT_RADIO_COMMAND_READ_SERIAL,
	RIFT_RADIO_COMMAND_READ_FLASH,
	RIFT_RADIO_COMMAND_SEND_HAPTICS,
	RIFT_RADIO_COMMAND_CONFIG,
};

//! A setting written to the radio or a device through it, see rift_radio_handle_config.
struct rift_radio_config_write
{
	uint8_t a;
	uint8_t b;
	uint8_t c;
	uint8_t length;
	uint8_t data[4];
};

#define RIFT_RADIO_CONFIG_QUEUE_LEN 32

struct rift_radio_command_data_read_serial
{
	//! A pointer to store the serial string. Must contain at least SERIAL_NUMBER_LENGTH bytes.
	char *serial;
	//! A pointer to store when reading the serial was successful.
	bool *serial_valid;
};

typedef int (*flash_read_callback_t)(void *user_data, uint16_t address, uint16_t length);

struct rift_radio_command_data_read_flash
{
	void *user_data;

	uint16_t address;
	uint16_t length;

	uint8_t *buffer;

	flash_read_callback_t read_callback;
};

union rift_radio_command_data {
	struct rift_radio_command_data_read_serial read_serial;
	struct rift_radio_command_data_read_flash read_flash;
};

//! How many past exposures a frame can be matched against. At ~60 Hz this is a little over a quarter second.
#define RIFT_EXPOSURE_HISTORY_SIZE 16

//! One camera exposure the HMD told us about, held so that late frames can still find the exposure they belong to.
struct rift_exposure_event
{
	//! The value of rift_hmd::exposure_counter at this exposure.
	uint32_t sequence_id;
	//! When the exposure started, in local monotonic time. This is what frames matched to it are timestamped with.
	timepoint_ns timestamp_ns;
	/*!
	 * When the IN report announcing this exposure arrived, in local monotonic time.
	 *
	 * Frames are matched against this rather than against @ref timestamp_ns. Both the report and the frame have
	 * travelled over USB before we see them, so their arrival times share most of that delay and land near each
	 * other; the exposure instant on the HMD's own clock is a good deal earlier than either.
	 */
	timepoint_ns recv_timestamp_ns;
};

/*!
 * A rift HMD device.
 *
 * @implements xrt_device
 * @implements t_constellation_tracker_device
 * @implements t_constellation_tracker_tracking_source
 */
//! The orientation guard's look back at its steering (see guard_orientation): slots and their spacing.
#define RIFT_GUARD_NET_SLOTS 41
#define RIFT_GUARD_NET_SLOT_NS (250 * U_TIME_1MS_IN_NS)

struct rift_hmd
{
	struct xrt_device base;
	struct xrt_frame_node node;

	struct xrt_frame_context *xfctx;

	enum u_logging_level log_level;

	// has built-in mutex so thread safe
	struct m_relation_history *relation_hist;

	bool use_constellation_poses;

	/*!
	 * While no sensor is placed (the room measured: the first start, or every sensor measured again), the
	 * tracker's orientation is its gyro's alone, unconstrained, and rolled the view over 60 degrees in two minutes.
	 * Then the headset's own fusion (@ref fusion, levelled by gravity) gives it, turned to the tracker's heading,
	 * and hands back over a second once the sensors give a pose again (see get_raw_pose).
	 */
	struct
	{
		struct os_mutex mutex;
		bool mutex_ready;
		bool active;
		//! The tracker's world from the own fusion's, as they were when it took over.
		struct xrt_quat yaw_offset;
		//! Placed again since: the hand-back waits for the tracker's tilt to agree, at most a few seconds.
		int64_t placed_since_ns;
		//! Handing back since; 0 not yet.
		int64_t blend_start_ns;
		int64_t last_log_ns;
		//! The placed cameras' count last seen, for the log when it changes.
		int last_placed;
	} level;

	/*!
	 * The orientation guard (see guard_orientation): the view turns as the headset's own gyro says, and the
	 * tracker's orientation only steers it within the rate its normal corrections have; a tracker that runs off
	 * (wrong sensor placements, wrong solves) can never turn the world around the user.
	 */
	struct
	{
		struct os_mutex mutex;
		bool mutex_ready;
		bool have;
		//! The time of the last step, and the orientation shown then.
		int64_t last_ns;
		struct xrt_quat out;
		//! The gyro's own orientation (@ref gyro) at the last step: the turn since then is the gyro's.
		struct xrt_quat gyro;
		bool have_gyro;
		//! What turns the tracker's orientation into the shown one: out = correction * tracker.
		struct xrt_quat correction;
		/*!
		 * All the steering so far (world frame), and its value every RIFT_GUARD_NET_SLOT_NS over the last
		 * RIFT_GUARD_NET_WINDOW_NS: how far the tracker turned the view beyond the gyro lately.
		 */
		struct xrt_quat steered;
		struct xrt_quat steered_ring[RIFT_GUARD_NET_SLOTS];
		int64_t steered_ring_ns;
		uint32_t steered_ring_head;
		uint32_t steered_ring_count;
		//! Holding on the gyro: the tracker ran too far off. While holding: since when the two agree again.
		bool holding;
		int64_t agree_since_ns;
		int64_t last_log_ns;
		float largest_gap_deg;
		//! RIFT_GUARD_TEST_SPIN_DEG_S: when the test turn started, and its last log line.
		int64_t test_start_ns;
		int64_t test_log_ns;
	} guard;

	/*!
	 * The headset's orientation from its gyro alone (no gravity levelling, unlike @ref fusion, which turns itself
	 * level at up to tens of degrees a second), for the orientation guard's steps. Its own frame; only its turns
	 * between two moments count. Written by the IMU thread.
	 */
	struct
	{
		struct os_mutex mutex;
		bool mutex_ready;
		bool valid;
		struct xrt_quat rot;
		//! The newest sample's time and gyro (the headset's frame).
		int64_t ns;
		struct xrt_vec3 rate;
	} gyro;

	struct os_hid_device *hmd_dev;
	struct os_hid_device *radio_dev;

	struct os_thread_helper sensor_thread;

	uint32_t last_remote_sample_time_us;
	timepoint_ns last_remote_sample_time_ns;
	timepoint_ns last_sample_local_timestamp_ns;

	uint32_t last_remote_exposure_time_us;
	//! The time of the last exposure in remote time, only accessed from the sensor thread, not locked.
	timepoint_ns last_remote_exposure_time_ns;
	//! The time of the last exposure, locked by sensor_thread.
	timepoint_ns last_local_exposure_time_ns;
	//! A total counter for how many exposures have occurred
	uint32_t exposure_counter;
	uint16_t last_tracking_count;

	//! The most recent exposures, newest at `(exposure_history_pushed - 1) % RIFT_EXPOSURE_HISTORY_SIZE`, locked by
	//! sensor_thread.
	struct rift_exposure_event exposure_history[RIFT_EXPOSURE_HISTORY_SIZE];
	//! How many exposures have ever been pushed into the history, locked by sensor_thread.
	uint64_t exposure_history_pushed;

	struct m_imu_3dof fusion;
	struct m_clock_windowed_skew_tracker *clock_tracker;

	timepoint_ns last_keepalive_time;
	enum rift_variant variant;
	struct rift_config_report config;
	struct rift_display_info_report display_info;

	struct t_timing_event_sink *timing_event_sink;
	struct t_timing_event_source *timing_event_source;

	struct rift_tracking_report tracking;

	const struct rift_lens_distortion *lens_distortions;
	uint16_t num_lens_distortions;
	uint16_t distortion_in_use;

	struct u_rift_panel panel;
	struct u_rift_eye_profile eye_profile;

	//! Interpupillary distance of the headset, in meters.
	float default_icd;
	float icd_override_m;

	bool presence;

	//! CV1: whether the panel is on, see rift_cv1_update_display_power.
	bool display_on;
	//! CV1: since when the wear sensor has read "not worn", 0 while worn.
	timepoint_ns unworn_since_ns;
	//! CV1: when switching the panel last failed, so a failing headset isn't asked 500 times a second.
	timepoint_ns display_power_failed_ns;
	//! CV1: the panel stays on until then, worn or not, so a compositor starting up finds the display.
	timepoint_ns display_on_until_ns;

	bool imu_needs_calibration;
	struct rift_imu_calibration imu_calibration;

	uint8_t radio_address[5];

	//! Mutex to protect access to the device array, device count == -1 means uninitialized
	struct os_mutex device_mutex;

	int device_count;
	int added_devices;
	struct xrt_device *devices[4]; // left touch, right touch, tracked object, remote

	struct t_constellation_tracker *constellation_tracker;
	struct t_constellation_tracker_device constellation_device;
	struct t_constellation_tracker_tracking_source constellation_tracking_source;
	t_constellation_device_id_t constellation_device_id;

	struct m_ff_vec3_f32 *gyro_ff;
	struct m_ff_vec3_f32 *accel_ff;
	struct m_relation_history *raw_constellation_relation_hist;
	timepoint_ns last_ff_timestamp_ns;
	struct m_ff_f64 *gravity_correction;
	timepoint_ns latest_constellation_ts;

	struct xrt_imu_sink *constellation_imu_sink;

	struct t_constellation_tracker_led_model led_model;
	/*!
	 * The back strap's LEDs, lit, for the constellation tracker as the headset's adjustable part: it measures where
	 * the strap sits and then tracks them with the front's. Empty unless RIFT_BACK_STRAP_LEDS_TRACKED.
	 */
	struct t_constellation_tracker_led_model back_strap_model;
	//! The wear state last told to the constellation tracker: -1 not yet, else 0 / 1.
	int presence_told;
	struct xrt_pose T_imu_device;
	struct xrt_pose T_device_imu;

	//! Generic state for the radio state machine
	struct
	{
		struct os_thread_helper thread;

		struct rift_touch_controller *touch_controllers[RADIO_STATE_TOUCH_CONTROLLERS_LEN];
		struct rift_remote *remote;

		enum rift_radio_command current_command;
		union rift_radio_command_data command_data;

		//! Settings waiting for the radio to be free, sent one at a time.
		struct rift_radio_config_write config_queue[RIFT_RADIO_CONFIG_QUEUE_LEN];
		size_t config_queue_head;
		size_t config_queue_count;
		//! Whether the radio itself was given its setup.
		bool radio_configured;
	} radio_state;
};

//! Casting helper function from xrt_device->rift_hmd
static inline struct rift_hmd *
rift_hmd(struct xrt_device *xdev)
{
	return (struct rift_hmd *)xdev;
}

static inline struct rift_hmd *
rift_hmd_from_node(struct xrt_frame_node *node)
{
	return (struct rift_hmd *)container_of(node, struct rift_hmd, node);
}

/*!
 * Adds a Touch controller to the HMD's constellation tracker, once the HMD is in one and the controller's calibration,
 * which holds its LED model, has been read. Does nothing for a controller already added, so it is called both when
 * the HMD joins the tracker and whenever a controller's calibration finishes reading: over the radio that can take
 * longer than startup does.
 */
void
rift_touch_controller_add_to_constellation(struct rift_hmd *hmd, struct rift_touch_controller *controller);

static inline struct rift_touch_controller *
rift_touch_controller(struct xrt_device *xdev)
{
	return (struct rift_touch_controller *)xdev;
}

static inline struct rift_touch_controller *
rift_touch_controller_from_node(struct xrt_frame_node *node)
{
	return (struct rift_touch_controller *)container_of(node, struct rift_touch_controller, node);
}

static inline struct rift_remote *
rift_remote(struct xrt_device *xdev)
{
	return (struct rift_remote *)xdev;
}

static inline struct rift_remote *
rift_remote_from_node(struct xrt_frame_node *node)
{
	return (struct rift_remote *)container_of(node, struct rift_remote, node);
}

static inline size_t
rift_radio_device_type_to_touch_index(enum rift_radio_device_type device_type)
{
	switch (device_type) {
	case RIFT_RADIO_DEVICE_LEFT_TOUCH: return 0;
	case RIFT_RADIO_DEVICE_RIGHT_TOUCH: return 1;
	case RIFT_RADIO_DEVICE_TRACKED_OBJECT: return 2;
	default: assert(false);
	}

	return -1;
}

static inline enum rift_radio_device_type
rift_radio_touch_index_to_device_type(size_t index)
{
	switch (index) {
	case 0: return RIFT_RADIO_DEVICE_LEFT_TOUCH;
	case 1: return RIFT_RADIO_DEVICE_RIGHT_TOUCH;
	case 2: return RIFT_RADIO_DEVICE_TRACKED_OBJECT;
	default: assert(false);
	}

	return (enum rift_radio_device_type)0;
}

static inline float
rift_min_mid_max_cap(struct rift_touch_controller_calibration *calibration, size_t index, float value)
{
	return (value - calibration->cap_sense_min[index]) /
	       (calibration->cap_sense_touch[index] - calibration->cap_sense_min[index]);
}

static inline float
rift_min_mid_max_range_to_float(uint16_t range[3], uint16_t value)
{
	if (value < range[1]) {
		return 1.0f - ((float)value - range[0]) / (range[1] - range[0]) * 0.5f;
	} else {
		return 0.5f - ((float)value - range[1]) / (range[2] - range[1]) * 0.5f;
	}
}

bool
rift_touch_calibration_parse(const char *calibration_data,
                             size_t calibration_size,
                             struct rift_touch_controller_calibration *out_calibration);

#ifdef __cplusplus
} // extern "C"
#endif
