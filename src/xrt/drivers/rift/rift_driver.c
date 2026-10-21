// Copyright 2020-2025, Collabora, Ltd.
// Copyright 2025-2026, Beyley Cardellio
// Copyright 2026, NVIDIA CORPORATION.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Driver for the Oculus Rift.
 *
 * Based largely on simulated_hmd.c, with reference to the DK2 firmware and OpenHMD's rift driver.
 *
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup drv_rift
 */

#include "os/os_time.h"

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_results.h"
#include "xrt/xrt_compiler.h"

#include "math/m_relation_history.h"
#include "math/m_clock_tracking.h"
#include "math/m_api.h"
#include "math/m_space.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h" // IWYU pragma: keep

#include "tracking/t_dead_reckoning.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"
#include "util/u_trace_marker.h"
#include "util/u_linux.h"
#include "util/u_truncate_printf.h"

#include "b_timing_source.h"

#include <stdio.h>
#include <assert.h>
#include <inttypes.h>

#include "rift_internal.h"
#include "rift_usb.h"
#include "rift_radio.h"


/*
 *
 * Structs and defines.
 *
 */

DEBUG_GET_ONCE_LOG_OPTION(rift_log, "RIFT_LOG", U_LOGGING_WARN)
DEBUG_GET_ONCE_FLOAT_OPTION(rift_override_icd_mm, "RIFT_OVERRIDE_ICD", -1.0f)
DEBUG_GET_ONCE_BOOL_OPTION(rift_distortion_render_fov, "RIFT_DISTORTION_RENDER_FOV", true)
DEBUG_GET_ONCE_BOOL_OPTION(rift_use_firmware_distortion, "RIFT_USE_FIRMWARE_DISTORTION", false)
DEBUG_GET_ONCE_BOOL_OPTION(rift_power_override, "RIFT_POWER_OVERRIDE", false)
/*
 * A CV1's display report as the Oculus runtime sets it on every start (1.208.0, USB capture): brightness 255 and 264
 * rows lit a frame of 1472, about 2 ms. Monado never sent it, and the headset ran at brightness 0: SteamVR Home
 * looked "bland, like an LCD, not an OLED". Negative leaves the report alone.
 */
DEBUG_GET_ONCE_NUM_OPTION(rift_display_brightness, "RIFT_DISPLAY_BRIGHTNESS", 255)
#define RIFT_CV1_PERSISTENCE_ROWS 264
//! How long the CV1 has to lie unworn before its panel goes off, see rift_cv1_update_display_power; negative: never.
DEBUG_GET_ONCE_NUM_OPTION(rift_display_off_unworn_ms, "RIFT_DISPLAY_OFF_UNWORN_MS", 5000)
//! The panel stays on this long after start, worn or not, so the compositor finds the display.
#define RIFT_DISPLAY_ON_AT_START_NS (30LL * U_TIME_1S_IN_NS)
//! Apply the HMD accelerometer's factory matrix as well as its offset, see where it's applied for why it's off.
DEBUG_GET_ONCE_BOOL_OPTION(rift_accel_factory_matrix, "RIFT_ACCEL_FACTORY_MATRIX", false)
DEBUG_GET_ONCE_FLOAT_OPTION(rift_startup_wait_time, "RIFT_STARTUP_WAIT_TIME", 5.0f)
DEBUG_GET_ONCE_BOOL_OPTION(rift_enable_back_strap_leds, "RIFT_ENABLE_BACK_STRAP_LEDS", false)
//! Keeps the back strap LEDs lit but still out of the LED model, so recordings contain them as unmatched blobs.
DEBUG_GET_ONCE_BOOL_OPTION(rift_back_strap_leds_lit_untracked, "RIFT_BACK_STRAP_LEDS_LIT_UNTRACKED", false)
/*!
 * Keeps the back strap LEDs lit and hands them to the constellation tracker as the headset's adjustable part: it
 * measures where the strap sits from frames in which the front is tracked (a sensor behind the user, or the front ones
 * from the side), then tracks them with the front's, so the headset stays tracked with its back to every sensor. On
 * by default (as the Oculus runtime keeps them lit); 0 holds them off and out of the model, as before.
 */
DEBUG_GET_ONCE_BOOL_OPTION(rift_back_strap_leds_tracked, "RIFT_BACK_STRAP_LEDS_TRACKED", true)

//! LEDs further behind the origin than this, along the device's backward (+Z) axis, are considered part of the back
//! strap and disabled by @ref rift_upload_back_strap_disable_pattern.
#define RIFT_BACK_STRAP_THRESHOLD_M 0.1f // 100mm

// The headset doesn't like it when you give a sequence of length 1 for some reason, so let's give it what it wants.
#define RIFT_CUSTOM_PATTERN_SEQUENCE_LENGTH 10

//! Set to 1 to dump IMU data to @ref RIFT_IMU_DUMP_PATH
#define RIFT_IMU_DUMP 0
//! The path where IMU data is dumped to.
#define RIFT_IMU_DUMP_PATH "/tmp/cv1_imu_dump.csv"

/*
 *
 * Headset functions
 *
 */

//! Repeats a 2-bit LED state across all @ref RIFT_CUSTOM_PATTERN_SEQUENCE_LENGTH phases of a custom pattern sequence.
static uint32_t
rift_custom_pattern_sequence_repeat(enum rift_custom_pattern_state state)
{
	uint32_t sequence = 0;
	for (int phase = 0; phase < RIFT_CUSTOM_PATTERN_SEQUENCE_LENGTH; phase++) {
		sequence |= ((uint32_t)state & 0x3) << (phase * 2);
	}
	return sequence;
}

/*
 * The CV1's panel stays lit once switched on, worn or not, and an OLED showing a still picture on a desk burns in: in
 * SteamVR the maintainer found it on with the headset off, and nothing in Monado or OpenHMD ever turned it off.
 * The Oculus runtime does; so does this, once the wear sensor has read "not worn" for a few seconds (lifting the headset
 * to adjust it doesn't blank it), and back on the moment it is worn. Audio and the LEDs (tracking) stay on. The HDMI link
 * stays up, SteamVR 2.17 in direct mode carried on through it.
 */
static void
rift_cv1_update_display_power(struct rift_hmd *hmd, timepoint_ns now)
{
	const int64_t off_ms = debug_get_num_option_rift_display_off_unworn_ms();
	if (off_ms < 0) {
		return;
	}

	bool want_on = true;
	if (hmd->presence) {
		hmd->unworn_since_ns = 0;
	} else {
		if (hmd->unworn_since_ns == 0) {
			hmd->unworn_since_ns = now;
		}
		/*
		 * Not until a compositor has the display: with the panel off the headset drops off HDMI, and SteamVR
		 * started with the headset on the desk found no display ("direct display through Vulkan WSI: (nil)").
		 * Once a compositor has it, the panel going off doesn't take it away.
		 */
		want_on = now - hmd->unworn_since_ns < off_ms * U_TIME_1MS_IN_NS || now < hmd->display_on_until_ns;
	}

	if (want_on == hmd->display_on || now - hmd->display_power_failed_ns < U_TIME_1S_IN_NS) {
		return;
	}

	enum rift_component_flags flags = RIFT_COMPONENT_AUDIO | RIFT_COMPONENT_LEDS;
	if (want_on) {
		flags |= RIFT_COMPONENT_DISPLAY;
	}
	int result = rift_enable_components(hmd, &(struct rift_enable_components_report){.flags = flags});
	if (result < 0) {
		HMD_WARN(hmd, "Failed to turn the display %s, reason %d", want_on ? "on" : "off", result);
		hmd->display_power_failed_ns = now;
		return;
	}
	hmd->display_on = want_on;
	HMD_INFO(hmd, "Display %s: headset %s.", want_on ? "on" : "off", want_on ? "worn" : "not worn");
}


/*!
 * The gyro's own orientation (rift_hmd::gyro), one sample on: the turn in the headset's frame, nothing else. A gap of
 * more than 100 ms between samples is not integrated (the orientation guard's steering takes it up).
 */
static void
rift_gyro_update(struct rift_hmd *hmd, int64_t timestamp_ns, const struct xrt_vec3 *gyro)
{
	if (!hmd->gyro.mutex_ready) {
		return;
	}
	os_mutex_lock(&hmd->gyro.mutex);
	if (!hmd->gyro.valid) {
		hmd->gyro.rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
		hmd->gyro.valid = true;
	} else {
		const int64_t dt_ns = timestamp_ns - hmd->gyro.ns;
		if (dt_ns > 0 && dt_ns < 100 * U_TIME_1MS_IN_NS) {
			math_quat_integrate_velocity(&hmd->gyro.rot, gyro, (float)time_ns_to_s(dt_ns), &hmd->gyro.rot);
			math_quat_normalize(&hmd->gyro.rot);
		}
	}
	hmd->gyro.ns = timestamp_ns;
	hmd->gyro.rate = *gyro;
	os_mutex_unlock(&hmd->gyro.mutex);
}

static int
rift_sensor_thread_tick(struct rift_hmd *hmd)
{
	uint8_t buf[REPORT_MAX_SIZE];
	int result;

	int64_t now = os_monotonic_get_ns();

	if (now - hmd->last_keepalive_time > KEEPALIVE_SEND_RATE_NS) {
		result = rift_send_keepalive(hmd);

		if (result < 0) {
			HMD_ERROR(hmd, "Got error sending keepalive, assuming fatal, reason %d", result);
			return result;
		}
	}

	result = os_hid_read(hmd->hmd_dev, buf, sizeof(buf), IMU_SAMPLE_RATE);
	timepoint_ns recv_time_ns = os_monotonic_get_ns() - RIFT_USB_LATENCY_BIAS;

	if (result < 0) {
		HMD_ERROR(hmd, "Got error reading from device, assuming fatal, reason %d", result);
		return result;
	}

	if (result == 0) {
		HMD_WARN(hmd, "Timed out waiting for packet from headset, packets should come in at %dhz",
		         IMU_SAMPLE_RATE);
		return 0;
	}

	switch (hmd->variant) {
	case RIFT_VARIANT_CV1:
	case RIFT_VARIANT_DK2: {
		// CV1 sends this every couple seconds when there's nothing connected to the radio
		if (buf[0] == IN_REPORT_CV1_RADIO_KEEPALIVE && hmd->variant == RIFT_VARIANT_CV1) {
			HMD_DEBUG(hmd, "Radio keepalive at %" PRIi64 " ns", recv_time_ns);
			break;
		}

		// skip unknown commands
		if (buf[0] != IN_REPORT_DK2) {
			HMD_WARN(hmd, "Skipping unknown IN command %d", buf[0]);
			break;
		}

		struct dk2_in_report report;

		// don't treat invalid IN reports as fatal, just ignore them
		if (result < (int)sizeof(report)) {
			HMD_WARN(hmd, "Got malformed DK2 IN report with size %d", result);
			break;
		}

		// TODO: handle endianness
		memcpy(&report, buf + 1, sizeof(report));

		if (hmd->variant == RIFT_VARIANT_CV1) {
			/*
			 * The wear sensor: 84-90 worn (on Windows, 77000 reports, Oculus' own capture), 2-6 lying
			 * unworn (6 pointing at the ceiling, which the old threshold of 3 took as worn and kept the panel
			 * lit: burn-in risk). Worn above 40, unworn below 20, unchanged in between.
			 */
			const uint16_t wear = report.cv1.presence_sensor;
			if (wear > 40) {
				hmd->presence = true;
			} else if (wear < 20) {
				hmd->presence = false;
			}
			rift_cv1_update_display_power(hmd, recv_time_ns);

			// The back strap sits elsewhere off the head: the tracker measures it only while worn.
			if (hmd->use_constellation_poses && hmd->constellation_tracker != NULL &&
			    hmd->presence_told != (hmd->presence ? 1 : 0)) {
				hmd->presence_told = hmd->presence ? 1 : 0;
				t_constellation_tracker_set_device_worn(hmd->constellation_tracker,
				                                        hmd->constellation_device_id, hmd->presence);
			}

			// TODO: figure out the actual algorithm for this, this one is too silly
			float iad_mm = (float)(report.cv1.iad_adc_value - 400) / 300.0f + 59.0f;

			// round to nearest half mm
			iad_mm = roundf(iad_mm * 2.0f) / 2.0f;

			// convert to meters
			hmd->default_icd = iad_mm / 1000.0f;
		}

		// if there's no samples, just do nothing.
		if (report.num_samples == 0) {
			break;
		}

		// wrap-around intentional and A-OK, given these are unsigned
		uint32_t remote_sample_delta_us = report.sample_timestamp - hmd->last_remote_sample_time_us;
		hmd->last_remote_sample_time_us = report.sample_timestamp;
		hmd->last_remote_sample_time_ns += (timepoint_ns)remote_sample_delta_us * OS_NS_PER_USEC;

		m_clock_windowed_skew_tracker_push(   //
		    hmd->clock_tracker,               //
		    recv_time_ns,                     //
		    hmd->last_remote_sample_time_ns); //

		int64_t local_timestamp_ns;
		// if we haven't synchronized our clocks, just do nothing
		if (!m_clock_windowed_skew_tracker_to_local(hmd->clock_tracker, hmd->last_remote_sample_time_ns,
		                                            &local_timestamp_ns)) {
			break;
		}

		timepoint_ns last_remote_exposure_time_ns = hmd->last_remote_exposure_time_ns;

		// Update the exposure time
		uint32_t remote_exposure_delta_us = report.tracking_timestamp - hmd->last_remote_exposure_time_us;
		hmd->last_remote_exposure_time_us = report.tracking_timestamp;
		hmd->last_remote_exposure_time_ns += (timepoint_ns)remote_exposure_delta_us * OS_NS_PER_USEC;

		uint16_t tracking_count_delta = report.tracking_count - hmd->last_tracking_count;
		hmd->last_tracking_count = report.tracking_count;
		hmd->exposure_counter += tracking_count_delta;

		if (remote_exposure_delta_us > 0) {
			os_thread_helper_lock(&hmd->sensor_thread);
			bool have_local_exposure_time = m_clock_windowed_skew_tracker_to_local(
			    hmd->clock_tracker, hmd->last_remote_exposure_time_ns, &hmd->last_local_exposure_time_ns);

			// Record the exposure so that frames which arrive after the next exposure has already been
			// reported can still find the exposure they were taken during. Skipped until the clocks are
			// synchronized, since before that there is no local time to record.
			if (have_local_exposure_time) {
				hmd->exposure_history[hmd->exposure_history_pushed % RIFT_EXPOSURE_HISTORY_SIZE] =
				    (struct rift_exposure_event){
				        .sequence_id = hmd->exposure_counter,
				        .timestamp_ns = hmd->last_local_exposure_time_ns,
				        .recv_timestamp_ns = recv_time_ns,
				    };
				hmd->exposure_history_pushed++;
			}
			os_thread_helper_unlock(&hmd->sensor_thread);

			if (have_local_exposure_time && hmd->timing_event_sink) {
				struct t_timing_event event = {
				    .type = T_TIMING_EVENT_TYPE_CAMERA_EXPOSURE_START,
				    .camera_exposure_start =
				        {
				            .sequence_id = hmd->exposure_counter,
				            .timestamp_ns = hmd->last_local_exposure_time_ns,
				            .frame_period_ns = (time_duration_ns)(hmd->last_remote_exposure_time_ns -
				                                                  last_remote_exposure_time_ns),
				            .exposure_time_ns =
				                (time_duration_ns)(hmd->tracking.exposure_length * OS_NS_PER_USEC),
				        },
				};

				t_timing_event_sink_push_timing_event(hmd->timing_event_sink, &event);
			}
		}

		if (report.num_samples > 1)
			HMD_TRACE(hmd,
			          "Had more than one sample queued! We aren't receiving IN reports fast enough, HMD "
			          "had %d samples in the queue! Having to work back that first sample...",
			          report.num_samples);

		const int num_samples = MIN(DK2_MAX_SAMPLES, report.num_samples);

		for (int i = 0; i < num_samples; i++) {
			struct rift_dk2_sample_pack latest_sample_pack = report.samples[i];

			struct xrt_vec3 accel, gyro;
			rift_unpack_float_sample(latest_sample_pack.accel.data, 0.0001f, &accel);
			rift_unpack_float_sample(latest_sample_pack.gyro.data, 0.0001f, &gyro);

			// If the HMD is not doing it's own calibration, we need to apply it now
			if (hmd->imu_needs_calibration) {
				math_matrix_3x3_transform_vec3(&hmd->imu_calibration.gyro_matrix, &gyro, &gyro);
				math_vec3_subtract(&hmd->imu_calibration.gyro_offset, &gyro);

				/*
				 * The accelerometer matrix as decoded makes things worse: on a CV1 HMD, recorded still
				 * in many orientations, the length of gravity varied 0.17 m/s^2 with the matrix and
				 * offset applied, 0.075 with the offset alone, and 0.033 for the best free per-axis
				 * scale and bias, whose scales were within 1.1 % of one while the matrix applies up to
				 * 2.5 %. Gravity read that far off tilts the IMU's "down" against the cameras'.
				 */
				if (debug_get_bool_option_rift_accel_factory_matrix()) {
					math_matrix_3x3_transform_vec3(&hmd->imu_calibration.accel_matrix, &accel,
					                               &accel);
				}
				math_vec3_subtract(&hmd->imu_calibration.accel_offset, &accel);
			}

			// work back the likely timestamp of the current sample
			// if there's only one sample, then this will always be zero, if there's two or more samples,
			// the previous samples will be offset by the sample rate of the IMU
			int64_t sample_local_timestamp_ns =
			    local_timestamp_ns - ((num_samples - 1 - i) * NS_PER_SAMPLE);

			// drop packets which are in the past (TODO: figure out why these happen..)
			if (sample_local_timestamp_ns < hmd->last_sample_local_timestamp_ns) {
				break;
			}

			os_thread_helper_lock(&hmd->sensor_thread);
			{
				// update the IMU for that sample
				m_imu_3dof_update(&hmd->fusion, sample_local_timestamp_ns, &accel, &gyro);
				rift_gyro_update(hmd, sample_local_timestamp_ns, &gyro);

				// Ensure these are monotonic. Sometimes due to the super fast IMU and clock jitter
				// we might estimate samples out of order a bit.
				if (sample_local_timestamp_ns > hmd->last_ff_timestamp_ns) {
					m_ff_vec3_f32_push(hmd->gyro_ff, &gyro, sample_local_timestamp_ns);
					m_ff_vec3_f32_push(hmd->accel_ff, &accel, sample_local_timestamp_ns);

					if (!hmd->fusion.grav.is_accel) {
						double grav_length = m_vec3_len(accel);
						m_ff_f64_push(hmd->gravity_correction, &grav_length,
						              sample_local_timestamp_ns);
					}

					hmd->last_ff_timestamp_ns = sample_local_timestamp_ns;
				}
			}
			os_thread_helper_unlock(&hmd->sensor_thread);

			if (hmd->constellation_imu_sink) {
				xrt_sink_push_imu(hmd->constellation_imu_sink,
				                  &(struct xrt_imu_sample){
				                      .timestamp_ns = sample_local_timestamp_ns,
				                      .accel_m_s2 = {accel.x, accel.y, accel.z},
				                      .gyro_rad_secs = {gyro.x, gyro.y, gyro.z},
				                  });
			}

#if RIFT_IMU_DUMP
			static FILE *imu_dump_file = NULL;
			if (imu_dump_file == NULL) {
				imu_dump_file = fopen(RIFT_IMU_DUMP_PATH, "wb");
				assert(imu_dump_file != NULL);
				fprintf(imu_dump_file, "ts,ax,ay,az,gx,gy,gz\n");
			}

			fprintf(imu_dump_file, "%" PRIi64 ",%f,%f,%f,%f,%f,%f\n", sample_local_timestamp_ns, accel.x,
			        accel.y, accel.z, gyro.x, gyro.y, gyro.z);
			fflush(imu_dump_file);
#endif

			hmd->last_sample_local_timestamp_ns = sample_local_timestamp_ns;

			// push the pose of the IMU for that sample, doing so per sample
			struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
			relation.relation_flags = (enum xrt_space_relation_flags)(
			    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT | XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
			    XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
			relation.pose.orientation = hmd->fusion.rot;
			relation.angular_velocity = gyro;

			m_relation_history_push(hmd->relation_hist, &relation, sample_local_timestamp_ns);
		}

		break;
	}
	}

	return 0;
}

static void *
rift_sensor_thread(void *ptr)
{
	U_TRACE_SET_THREAD_NAME("Rift sensor thread");

	struct rift_hmd *hmd = (struct rift_hmd *)ptr;

	os_thread_helper_lock(&hmd->sensor_thread);

	// uncomment this to be able to see if things are actually progressing as expected in a debugger, without having
	// to count yourself
	// #define TICK_DEBUG

	int result = 0;
#ifdef TICK_DEBUG
	int ticks = 0;
#endif

	while (os_thread_helper_is_running_locked(&hmd->sensor_thread) && result >= 0) {
		os_thread_helper_unlock(&hmd->sensor_thread);

		result = rift_sensor_thread_tick(hmd);

		os_thread_helper_lock(&hmd->sensor_thread);
#ifdef TICK_DEBUG
		ticks += 1;
#endif
	}

	os_thread_helper_unlock(&hmd->sensor_thread);

	if (result < 0) {
		HMD_ERROR(hmd, "Sensor thread exiting due to error %d", result);
	} else {
		HMD_DEBUG(hmd, "Sensor thread exiting cleanly");
	}

	return NULL;
}

static int
rift_usb_thread_radio_tick(struct rift_hmd *hmd)
{
	int result;

	result = rift_radio_handle_read(hmd);
	if (result < 0) {
		return result;
	}

	result = rift_radio_handle_command(hmd);
	if (result < 0) {
		return result;
	}

	result = rift_radio_handle_config(hmd);
	if (result < 0) {
		return result;
	}

	result = rift_radio_handle_haptics(hmd);
	if (result < 0) {
		return result;
	}

	return 0;
}

static void *
rift_radio_thread(void *ptr)
{
	struct rift_hmd *hmd = (struct rift_hmd *)ptr;

	const char *thread_name = "Rift Radio";

	U_TRACE_SET_THREAD_NAME(thread_name);
	os_thread_name_self(thread_name);

#ifdef XRT_OS_LINUX
	// Try to raise priority of this thread.
	u_linux_try_to_set_realtime_priority_on_thread(hmd->log_level, thread_name);
#endif

	os_thread_helper_lock(&hmd->radio_state.thread);

	int result = 0;
#if 0
	int ticks = 0;
#endif

	while (os_thread_helper_is_running_locked(&hmd->radio_state.thread) && result >= 0) {
		os_thread_helper_unlock(&hmd->radio_state.thread);

		result = rift_usb_thread_radio_tick(hmd);

		os_thread_helper_lock(&hmd->radio_state.thread);
#if 0
		ticks += 1;
#endif
	}

	os_thread_helper_unlock(&hmd->radio_state.thread);

	if (result < 0) {
		HMD_ERROR(hmd, "Radio thread stopped on error %d, no Touch input until a restart", result);
	}

	return NULL;
}

#define PARSE_MICROMETER_TRIPLET(out, values)                                                                          \
	do {                                                                                                           \
		out.x = MICROMETERS_TO_METERS(values[0]);                                                              \
		out.y = MICROMETERS_TO_METERS(values[1]);                                                              \
		out.z = MICROMETERS_TO_METERS(values[2]);                                                              \
	} while (0)

static int
rift_read_led_model(struct rift_hmd *hmd)
{
	int result;

	struct rift_position_calibration_report position_report;

	result = rift_get_position_calibration_report(hmd, &position_report);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to read position calibration report, reason %d", result);
		return result;
	}

	hmd->led_model.match_parameters = (struct t_constellation_tracker_led_model_match_parameters){
	    // Require a real good one if we don't have a prior, avoid ambiguities. 7 rather than 9: the pose scoring
	    // rejects ambiguous low-count solves on its own, and a CV1 HMD often shows only 7 or 8 LEDs to a sensor.
	    // Replaying a worn session, 5 and 6 LED solves were all wrong, 7 LED solves 96 % right.
	    .min_leds_for_correspondence_search_without_prior = 7,
	    .min_leds_for_correspondence_search_with_prior = 5,
	};

	// minus one to get rid of the IMU, which is also in here
	hmd->led_model.leds = U_TYPED_ARRAY_CALLOC(struct t_constellation_tracker_led, position_report.position_count);

	for (uint32_t i = 0; i < position_report.position_count; i++) {
		result = rift_get_position_calibration_report(hmd, &position_report);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to read position calibration report, reason %d", result);
			goto read_led_model_err;
		}

		if (position_report.position_type == RIFT_POSITION_CALIBRATION_TYPE_INERTIAL_SENSOR) {
			PARSE_MICROMETER_TRIPLET(hmd->T_device_imu.position, position_report.position);
			hmd->T_device_imu.position.x = -hmd->T_device_imu.position.x;
			hmd->T_device_imu.position.y = hmd->T_device_imu.position.y;
			hmd->T_device_imu.position.z = -hmd->T_device_imu.position.z;

			hmd->T_device_imu.orientation = (struct xrt_quat)XRT_QUAT_IDENTITY;

			// NOTE: we ignore the IMU orientation since it's always zeroed out..

			// Invert the pose for device relative to IMU
			math_pose_invert(&hmd->T_device_imu, &hmd->T_imu_device);
		} else {
			struct xrt_vec3 pos;
			PARSE_MICROMETER_TRIPLET(pos, position_report.position);

			struct xrt_vec3 normal;
			PARSE_MICROMETER_TRIPLET(normal, position_report.normal);
			math_vec3_normalize(&normal); // normalize the direction

			pos.x = -pos.x;
			pos.y = pos.y;
			pos.z = -pos.z;
			normal.x = -normal.x;
			normal.y = normal.y;
			normal.z = -normal.z;

			hmd->led_model.leds[hmd->led_model.led_count++] = (struct t_constellation_tracker_led){
			    .position = pos,
			    .normal = normal,
			    .radius_m = RIFT_LED_SIZE_M,                 // 3.5mm
			    .visibility_angle = RIFT_LED_VISIBILITY_RAD, // TODO: tune this value properly
			    .id = (t_constellation_led_id_it)position_report.position_index,
			};
		}
	}

	// Make the LEDs relative to the IMU
	for (size_t i = 0; i < hmd->led_model.led_count; i++) {
		struct xrt_vec3 led_pos = hmd->led_model.leds[i].position;
		hmd->led_model.leds[i].position = m_vec3_add(hmd->T_imu_device.position, led_pos);
	}

	return 0;

read_led_model_err:
	free(hmd->led_model.leds);
	hmd->led_model.leds = NULL;

	return result;
}

#undef PARSE_MICROMETER_TRIPLET

/*!
 * Uploads a custom pattern that holds every LED high except the back strap LEDs, which are held off, then drops
 * the back strap LEDs from @ref rift_hmd::led_model so the constellation tracker never tries to match blobs
 * against LEDs that are physically dark. The caller must have already set RIFT_TRACKING_CUSTOM_PATTERN on the
 * tracking report: writes to the CustomPattern report are no-ops until that bit is set.
 *
 * With @p keep_lit the back strap LEDs are held high like the rest and only dropped from the model. The strap is
 * adjustable, so they can't be tracked as part of the rigid front, but recordings then contain them. With
 * @p tracked they are also held high, and kept in @ref rift_hmd::back_strap_model for the constellation tracker, which
 * measures where the strap sits before it tracks them.
 */
static int
rift_upload_back_strap_disable_pattern(struct rift_hmd *hmd, bool keep_lit, bool tracked)
{
	int result;
	const size_t total_count = hmd->led_model.led_count;
	size_t kept_count = 0;

	if (tracked) {
		keep_lit = true;
		hmd->back_strap_model.leds = U_TYPED_ARRAY_CALLOC(struct t_constellation_tracker_led, total_count);
		hmd->back_strap_model.led_count = 0;
	}

	const uint32_t sequence_high = rift_custom_pattern_sequence_repeat(RIFT_CUSTOM_PATTERN_STAT_HIGH);
	const uint32_t sequence_off = rift_custom_pattern_sequence_repeat(RIFT_CUSTOM_PATTERN_STAT_OFF);

	for (size_t i = 0; i < total_count; i++) {
		// Copied out since the compaction below writes to index `kept_count`, which trails `i`.
		struct t_constellation_tracker_led led = hmd->led_model.leds[i];

		// Monado device space is -Z forward, so a LED further than the threshold along +Z sits behind the
		// origin by more than the threshold, i.e. on the back strap.
		bool is_back_strap = led.position.z > RIFT_BACK_STRAP_THRESHOLD_M;

		struct rift_custom_pattern_report pattern = {
		    .command_id = 0,
		    .sequence_length = RIFT_CUSTOM_PATTERN_SEQUENCE_LENGTH,
		    .sequence = (is_back_strap && !keep_lit) ? sequence_off : sequence_high,
		    .led_index = led.id,
		    .led_count = (uint16_t)total_count,
		};

		result = rift_set_custom_pattern(hmd, &pattern);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to upload custom pattern for LED %d, reason %d", led.id, result);
			return result;
		}

		if (is_back_strap) {
			if (tracked && hmd->back_strap_model.leds != NULL) {
				hmd->back_strap_model.leds[hmd->back_strap_model.led_count++] = led;
			}
			if (keep_lit) {
				// They are not in the recorded LED model, so log the firmware geometry for offline use.
				HMD_INFO(hmd,
				         "Back strap LED %d kept lit, untracked: position %f %f %f normal %f %f %f",
				         led.id, (double)led.position.x, (double)led.position.y, (double)led.position.z,
				         (double)led.normal.x, (double)led.normal.y, (double)led.normal.z);
			} else {
				HMD_DEBUG(hmd, "Disabling LED %d as back strap, %.1fmm behind origin", led.id,
				          (double)(led.position.z * 1000.0f));
			}
			continue;
		}

		// Dropped from the model as well as the hardware.
		hmd->led_model.leds[kept_count++] = led;
	}

	hmd->led_model.led_count = kept_count;

	HMD_INFO(hmd, "%s %zu back strap LED(s), %zu of %zu LEDs remain in the model.",
	         tracked ? "Lit for the tracker to measure and track:"
	         : keep_lit ? "Untracked (kept lit)"
	                    : "Disabled",
	         total_count - kept_count, kept_count, total_count);

	return 0;
}

//! The angle between two orientations' up, degrees: how differently they are tilted.
static float
tilt_between_deg(const struct xrt_quat *a, const struct xrt_quat *b)
{
	const struct xrt_vec3 y = {0.0f, 1.0f, 0.0f};
	struct xrt_quat ia, ib;
	struct xrt_vec3 ua, ub;
	// Where each has the world's up, in the headset's frame.
	math_quat_invert(a, &ia);
	math_quat_invert(b, &ib);
	math_quat_rotate_vec3(&ia, &y, &ua);
	math_quat_rotate_vec3(&ib, &y, &ub);
	return RAD_TO_DEG(acosf(CLAMP(m_vec3_dot(ua, ub), -1.0f, 1.0f)));
}

/*!
 * The orientation for apps while no sensor is placed (see rift_hmd::level): the own fusion's, levelled by gravity,
 * turned to the tracker's heading when it took over; handed back to the tracker's over a second once the cameras give
 * the headset a pose again and the two agree on which way is down (or after 5 s regardless).
 */
static void
level_orientation(struct rift_hmd *hmd, timepoint_ns when_ns, struct xrt_space_relation *relation)
{
	if (!hmd->level.mutex_ready || hmd->constellation_tracker == NULL) {
		return;
	}
	const int placed = t_constellation_tracker_cameras_placed(hmd->constellation_tracker);
	struct xrt_space_relation own = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(hmd->relation_hist, when_ns, &own);
	if (placed != hmd->level.last_placed) {
		HMD_INFO(hmd, "Sensors placed: %d (was %d); headset pose from the tracker 0x%x, its own fusion 0x%x.", placed,
		         hmd->level.last_placed, relation->relation_flags, own.relation_flags);
		hmd->level.last_placed = placed;
	}
	if ((own.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return;
	}
	const bool tracker_has_orientation = (relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0;

	os_mutex_lock(&hmd->level.mutex);
	if (placed == 0 && !hmd->level.active) {
		/*
		 * Taking over: turned to the tracker's orientation now, all of it, so the view does not move. Its
		 * heading alone tipped it 2.5 degrees: the own fusion's down is the IMU's, which sits a little turned
		 * in the headset (the tracker's own placements level by the cameras).
		 */
		hmd->level.yaw_offset = (struct xrt_quat)XRT_QUAT_IDENTITY;
		if (tracker_has_orientation) {
			struct xrt_quat own_inv;
			math_quat_invert(&own.pose.orientation, &own_inv);
			math_quat_rotate(&relation->pose.orientation, &own_inv, &hmd->level.yaw_offset);
			math_quat_normalize(&hmd->level.yaw_offset);
		}
		hmd->level.active = true;
		hmd->level.placed_since_ns = 0;
		hmd->level.blend_start_ns = 0;
		hmd->level.last_log_ns = when_ns;
		HMD_WARN(hmd, "No sensor placed: the headset's orientation comes from its own gyro, levelled by gravity, "
		              "until the sensors are placed again.");
	}
	if (!hmd->level.active) {
		os_mutex_unlock(&hmd->level.mutex);
		return;
	}

	struct xrt_quat levelled;
	math_quat_rotate(&hmd->level.yaw_offset, &own.pose.orientation, &levelled);
	math_quat_normalize(&levelled);
	const float tilt_deg = tracker_has_orientation ? tilt_between_deg(&levelled, &relation->pose.orientation) : 0.0f;
	if (when_ns - hmd->level.last_log_ns > 10LL * U_TIME_1S_IN_NS) {
		hmd->level.last_log_ns = when_ns;
		HMD_WARN(hmd, "Levelled orientation while no sensor is placed: the tracker's own is %.1f degrees tilted from it.",
		         tilt_deg);
	}

	struct xrt_quat out = levelled;
	if (placed > 0) {
		if (hmd->level.placed_since_ns == 0) {
			hmd->level.placed_since_ns = when_ns;
		}
		const bool tracked = (relation->relation_flags & XRT_SPACE_RELATION_POSITION_TRACKED_BIT) != 0;
		if (hmd->level.blend_start_ns == 0 && tracker_has_orientation &&
		    ((tracked && tilt_deg <= 5.0f) || when_ns - hmd->level.placed_since_ns > 5LL * U_TIME_1S_IN_NS)) {
			hmd->level.blend_start_ns = when_ns;
			HMD_WARN(hmd, "Sensors placed: the orientation goes back to the tracker's (%.1f degrees tilted apart).",
			         tilt_deg);
		}
		if (hmd->level.blend_start_ns != 0) {
			const float t = (float)(when_ns - hmd->level.blend_start_ns) / (float)U_TIME_1S_IN_NS;
			if (t >= 1.0f) {
				hmd->level.active = false;
				os_mutex_unlock(&hmd->level.mutex);
				return;
			}
			math_quat_slerp(&levelled, &relation->pose.orientation, CLAMP(t, 0.0f, 1.0f), &out);
		}
	}
	os_mutex_unlock(&hmd->level.mutex);

	relation->pose.orientation = out;
	relation->relation_flags |= XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT;
}

/*!
 * The orientation guard. The Oculus runtime's view never turns on its own, whatever its sensors do: the headset's
 * orientation is its IMU's, the sensors correct it. The tracker's fused orientation is IMU-driven between solves, but
 * a solve through a wrongly placed sensor pulls it, and two such sensors pull it every frame: a user saw the world
 * spin. So the shown orientation is stepped by the headset's gyro alone (rift_hmd::gyro: nothing the tracker does
 * reaches it, not its solves, not its estimate of the gyro's bias) and steered toward the tracker's orientation at
 * RIFT_GUARD_RATE_DEG_S at most. It holds (RIFT_GUARD_HOLD_RATE_DEG_S) once the two are RIFT_GUARD_HOLD_DEG apart, or
 * once the steering turned the view RIFT_GUARD_NET_DEG beyond the gyro within RIFT_GUARD_NET_WINDOW_NS (a tracker
 * drifting slower than the steering), until they agree within RIFT_GUARD_RESUME_DEG for RIFT_GUARD_RESUME_NS.
 *
 * The steering is per step on the gap built since the last step, so callers that ask often (SteamVR asks about every
 * millisecond) get no more than callers that ask once a frame. The gyro's axes are the tracker's output axes within a
 * degree (replayed sessions), so a turn carries over as it is. Replayed sessions: the tracker's orientation turned
 * against the gyro by 0.1 to 0.3 degrees in a typical second, 3.5 at most, all followed.
 * RIFT_ORIENTATION_GUARD=0 turns it off.
 */
DEBUG_GET_ONCE_BOOL_OPTION(rift_orientation_guard, "RIFT_ORIENTATION_GUARD", true)
DEBUG_GET_ONCE_FLOAT_OPTION(rift_guard_test_spin_deg_s, "RIFT_GUARD_TEST_SPIN_DEG_S", 0.0f)
#define RIFT_GUARD_RATE_DEG_S 6.0f
#define RIFT_GUARD_HOLD_DEG 10.0f
#define RIFT_GUARD_NET_DEG 8.0f
#define RIFT_GUARD_NET_WINDOW_NS ((int64_t)(RIFT_GUARD_NET_SLOTS - 1) * RIFT_GUARD_NET_SLOT_NS)
#define RIFT_GUARD_RESUME_DEG 3.0f
#define RIFT_GUARD_RESUME_NS (1 * U_TIME_1S_IN_NS)
#define RIFT_GUARD_HOLD_RATE_DEG_S 0.5f
//! A step at most this often: queries in between, or for earlier moments, take the current correction.
#define RIFT_GUARD_STEP_NS (2 * U_TIME_1MS_IN_NS)
//! The gyro carried on by its newest rate this far at most (apps ask a frame or two ahead).
#define RIFT_GUARD_GYRO_AHEAD_NS (200 * U_TIME_1MS_IN_NS)
//! A step's gyro turn above this rate is no head's (a glitch): that step takes the tracker's rate instead.
#define RIFT_GUARD_MAX_TURN_DEG_S 2000.0f

static float
quat_angle_deg(const struct xrt_quat *a, const struct xrt_quat *b)
{
	const float d = fabsf(a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w);
	return RAD_TO_DEG(2.0f * acosf(CLAMP(d, 0.0f, 1.0f)));
}

//! The gyro's own orientation at @p when_ns (carried on by its newest rate, in the headset's frame), and that rate.
static bool
gyro_orientation_at(struct rift_hmd *hmd, timepoint_ns when_ns, struct xrt_quat *out_orientation, struct xrt_vec3 *out_rate)
{
	if (!hmd->gyro.mutex_ready) {
		return false;
	}
	os_mutex_lock(&hmd->gyro.mutex);
	const bool valid = hmd->gyro.valid;
	const struct xrt_quat rot = hmd->gyro.rot;
	const struct xrt_vec3 rate = hmd->gyro.rate;
	const int64_t ns = hmd->gyro.ns;
	os_mutex_unlock(&hmd->gyro.mutex);

	// The IMU stopped, or the asked time is far off: carried on by an old rate it would turn the view on its own.
	if (!valid || when_ns - ns > RIFT_GUARD_GYRO_AHEAD_NS || ns - when_ns > RIFT_GUARD_GYRO_AHEAD_NS) {
		return false;
	}
	*out_orientation = rot;
	if (when_ns != ns) {
		// math_quat_integrate_velocity turns in the orientation's own (the headset's) frame.
		math_quat_integrate_velocity(&rot, &rate, (float)time_ns_to_s(when_ns - ns), out_orientation);
	}
	math_quat_normalize(out_orientation);
	*out_rate = rate;
	return true;
}

static void
guard_orientation(struct rift_hmd *hmd, timepoint_ns when_ns, struct xrt_space_relation *relation)
{
	if (!hmd->guard.mutex_ready || !debug_get_bool_option_rift_orientation_guard() ||
	    (relation->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return;
	}

	/*
	 * Test (desk, headset lying still): RIFT_GUARD_TEST_SPIN_DEG_S turns the tracker's orientation (and its
	 * world-frame rate, as a tracker that is off would have it) about the vertical at that rate, as a runaway
	 * tracker; the shown orientation must stay still. Logged once a second.
	 */
	const float test_spin = debug_get_float_option_rift_guard_test_spin_deg_s();
	if (test_spin != 0.0f) {
		if (hmd->guard.test_start_ns == 0) {
			hmd->guard.test_start_ns = when_ns;
		}
		const struct xrt_vec3 up = {0.0f, 1.0f, 0.0f};
		struct xrt_quat spin;
		math_quat_from_angle_vector(DEG_TO_RAD(test_spin) * (float)(when_ns - hmd->guard.test_start_ns) /
		                                (float)U_TIME_1S_IN_NS,
		                            &up, &spin);
		math_quat_rotate(&spin, &relation->pose.orientation, &relation->pose.orientation);
		struct xrt_vec3 w = relation->angular_velocity;
		math_quat_rotate_vec3(&spin, &w, &relation->angular_velocity);
	}
	const struct xrt_quat tracker = relation->pose.orientation;

	struct xrt_quat gyro = XRT_QUAT_IDENTITY;
	struct xrt_vec3 gyro_rate = XRT_VEC3_ZERO;
	const bool gyro_ok = gyro_orientation_at(hmd, when_ns, &gyro, &gyro_rate);

	os_mutex_lock(&hmd->guard.mutex);
	if (!hmd->guard.have) {
		// Starting: the tracker's orientation as it is.
		hmd->guard.have = true;
		hmd->guard.out = tracker;
		hmd->guard.gyro = gyro;
		hmd->guard.have_gyro = gyro_ok;
		hmd->guard.correction = (struct xrt_quat)XRT_QUAT_IDENTITY;
		hmd->guard.steered = (struct xrt_quat)XRT_QUAT_IDENTITY;
		hmd->guard.steered_ring_ns = when_ns;
		hmd->guard.steered_ring[0] = hmd->guard.steered;
		hmd->guard.steered_ring_head = 0;
		hmd->guard.steered_ring_count = 1;
		hmd->guard.holding = false;
		hmd->guard.agree_since_ns = 0;
		hmd->guard.last_ns = when_ns;
	}

	// Earlier moments, or within a step of the last: the correction as it is.
	const int64_t since_ns = when_ns - hmd->guard.last_ns;
	if (since_ns >= RIFT_GUARD_STEP_NS) {
		const float dt = (float)since_ns / (float)U_TIME_1S_IN_NS;

		// The gyro's turn over dt, in the headset's frame, onto the orientation shown last.
		struct xrt_quat predicted = hmd->guard.out;
		bool stepped = false;
		if (gyro_ok && hmd->guard.have_gyro) {
			struct xrt_quat gyro_inv, turn;
			math_quat_invert(&hmd->guard.gyro, &gyro_inv);
			math_quat_rotate(&gyro_inv, &gyro, &turn);
			const struct xrt_quat none = XRT_QUAT_IDENTITY;
			if (quat_angle_deg(&turn, &none) <= RIFT_GUARD_MAX_TURN_DEG_S * dt + 1.0f) {
				math_quat_rotate(&hmd->guard.out, &turn, &predicted);
				stepped = true;
			}
		}
		if (!stepped && (relation->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) != 0) {
			// No gyro of its own: the tracker's rate, in the headset's frame (through the tracker's orientation).
			struct xrt_quat inv;
			struct xrt_vec3 w_dev;
			math_quat_invert(&tracker, &inv);
			math_quat_rotate_vec3(&inv, &relation->angular_velocity, &w_dev);
			math_quat_integrate_velocity(&hmd->guard.out, &w_dev, dt, &predicted);
		}
		math_quat_normalize(&predicted);
		hmd->guard.gyro = gyro;
		hmd->guard.have_gyro = gyro_ok;

		const float gap_deg = quat_angle_deg(&predicted, &tracker);
		if (gap_deg > hmd->guard.largest_gap_deg) {
			hmd->guard.largest_gap_deg = gap_deg;
		}

		// How far the steering turned the view beyond the gyro over the last window.
		while (when_ns - hmd->guard.steered_ring_ns >= RIFT_GUARD_NET_SLOT_NS) {
			hmd->guard.steered_ring_ns += RIFT_GUARD_NET_SLOT_NS;
			if (when_ns - hmd->guard.steered_ring_ns > RIFT_GUARD_NET_WINDOW_NS) {
				// A long pause: the window starts again.
				hmd->guard.steered_ring_ns = when_ns;
				hmd->guard.steered_ring_count = 0;
			}
			hmd->guard.steered_ring_head = (hmd->guard.steered_ring_head + 1) % RIFT_GUARD_NET_SLOTS;
			hmd->guard.steered_ring[hmd->guard.steered_ring_head] = hmd->guard.steered;
			if (hmd->guard.steered_ring_count < RIFT_GUARD_NET_SLOTS) {
				hmd->guard.steered_ring_count++;
			}
		}
		const uint32_t oldest = (hmd->guard.steered_ring_head + RIFT_GUARD_NET_SLOTS + 1 -
		                         hmd->guard.steered_ring_count) %
		                        RIFT_GUARD_NET_SLOTS;
		const float net_deg = quat_angle_deg(&hmd->guard.steered, &hmd->guard.steered_ring[oldest]);

		if (!hmd->guard.holding && (gap_deg > RIFT_GUARD_HOLD_DEG || net_deg > RIFT_GUARD_NET_DEG)) {
			hmd->guard.holding = true;
			hmd->guard.agree_since_ns = 0;
			HMD_WARN(hmd,
			         "Orientation guard: the tracker's orientation is %.1f degrees from what the gyro says (it turned "
			         "the view %.1f degrees beyond the gyro lately); the view holds to the gyro until they agree "
			         "again.",
			         gap_deg, net_deg);
		} else if (hmd->guard.holding) {
			// Agreeing again, and the window's steering back under its limit (a drift that went on is not done).
			if (gap_deg >= RIFT_GUARD_RESUME_DEG || net_deg > RIFT_GUARD_NET_DEG) {
				hmd->guard.agree_since_ns = 0;
			} else if (hmd->guard.agree_since_ns == 0) {
				hmd->guard.agree_since_ns = when_ns;
			} else if (when_ns - hmd->guard.agree_since_ns >= RIFT_GUARD_RESUME_NS) {
				hmd->guard.holding = false;
				HMD_WARN(hmd, "Orientation guard: the tracker's orientation agrees with the gyro again (%.1f degrees).",
				         gap_deg);
			}
		}

		// Toward the tracker by what the rate allows over dt, all of it when the gap is smaller.
		const float rate = hmd->guard.holding ? RIFT_GUARD_HOLD_RATE_DEG_S : RIFT_GUARD_RATE_DEG_S;
		const float t = gap_deg > 1e-4f ? CLAMP(rate * dt / gap_deg, 0.0f, 1.0f) : 1.0f;
		math_quat_slerp(&predicted, &tracker, t, &hmd->guard.out);
		math_quat_normalize(&hmd->guard.out);

		// The steering of this step (world frame), added to all of it.
		struct xrt_quat predicted_inv, step;
		math_quat_invert(&predicted, &predicted_inv);
		math_quat_rotate(&hmd->guard.out, &predicted_inv, &step);
		math_quat_rotate(&step, &hmd->guard.steered, &hmd->guard.steered);
		math_quat_normalize(&hmd->guard.steered);

		struct xrt_quat tracker_inv;
		math_quat_invert(&tracker, &tracker_inv);
		math_quat_rotate(&hmd->guard.out, &tracker_inv, &hmd->guard.correction);
		math_quat_normalize(&hmd->guard.correction);
		hmd->guard.last_ns = when_ns;

		if (gap_deg > 1.0f && when_ns - hmd->guard.last_log_ns > 10LL * U_TIME_1S_IN_NS) {
			hmd->guard.last_log_ns = when_ns;
			HMD_WARN(hmd,
			         "Orientation guard: the tracker's orientation is %.1f degrees from the gyro's (largest %.1f, "
			         "steered %.1f degrees lately); the view follows it at most %.1f degrees a second.",
			         gap_deg, hmd->guard.largest_gap_deg, net_deg, rate);
			hmd->guard.largest_gap_deg = 0.0f;
		}
	}

	struct xrt_quat shown;
	math_quat_rotate(&hmd->guard.correction, &tracker, &shown);
	math_quat_normalize(&shown);
	const bool holding = hmd->guard.holding;
	os_mutex_unlock(&hmd->guard.mutex);

	if (test_spin != 0.0f && when_ns - hmd->guard.test_log_ns > U_TIME_1S_IN_NS) {
		hmd->guard.test_log_ns = when_ns;
		struct xrt_vec3 e_tracker, e_shown;
		math_quat_to_euler_angles(&tracker, &e_tracker);
		math_quat_to_euler_angles(&shown, &e_shown);
		HMD_WARN(hmd, "Guard test: tracker yaw %.1f, shown yaw %.1f, pitch %.1f, roll %.1f degrees%s.",
		         RAD_TO_DEG(e_tracker.y), RAD_TO_DEG(e_shown.y), RAD_TO_DEG(e_shown.x), RAD_TO_DEG(e_shown.z),
		         holding ? " (holding)" : "");
	}
	relation->pose.orientation = shown;
	// The rate: the gyro's, turned into the world as shown (without a gyro of its own: the tracker's, turned alike).
	if (gyro_ok) {
		math_quat_rotate_vec3(&shown, &gyro_rate, &relation->angular_velocity);
		relation->relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	} else {
		struct xrt_vec3 w = relation->angular_velocity;
		math_quat_rotate_vec3(&hmd->guard.correction, &w, &relation->angular_velocity);
	}
}

//! @p for_prior: the solved pose to search for the LEDs with, rather than the one with corrections blended in.
static void
get_raw_pose(struct rift_hmd *hmd, timepoint_ns when_ns, bool for_prior, struct xrt_relation_chain *xrc)
{
	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	if (hmd->use_constellation_poses) {
		const int ret = for_prior
		                    ? t_constellation_tracker_get_solved_pose(
		                          hmd->constellation_tracker, hmd->constellation_device_id, when_ns, &relation)
		                    : t_constellation_tracker_get_tracked_pose(
		                          hmd->constellation_tracker, hmd->constellation_device_id, when_ns, &relation);
		if (ret < 0) {
			relation = XRT_C11_COMPOUND(struct xrt_space_relation) XRT_SPACE_RELATION_ZERO;
		}
	}

	if (relation.relation_flags == 0) {
		m_relation_history_get(hmd->relation_hist, when_ns, &relation);
	} else if (!for_prior) {
		level_orientation(hmd, when_ns, &relation);
		guard_orientation(hmd, when_ns, &relation);
	}

	(*m_relation_chain_reserve(xrc)) = relation;
}

/*
 *
 * Distortion
 *
 */

// @todo remove clang-format off when CI is updated
// clang-format off
static const struct rift_lens_distortion DK2_DISTORTIONS[] = {
    {
        .distortion_version = RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1,
        .eye_relief = 0.008f,
        .data =
            {
                .lcsv_catmull_rom_10 =
                    {
                        .meters_per_tan_angle_at_center = 0.036f,
                        .max_r = 1.0f,
                        .chromatic_abberation = {-0.0112f, -0.015f, 0.0187f, 0.015f},
                        .k =
                            {
                                1.003f,
                                1.02f,
                                1.042f,
                                1.066f,
                                1.094f,
                                1.126f,
                                1.162f,
                                1.203f,
                                1.25f,
                                1.31f,
                                1.38f,
                            },
                    },
            },
    },
    {
        .distortion_version = RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1,
        .eye_relief = 0.018f,
        .data =
            {
                .lcsv_catmull_rom_10 =
                    {
                        .meters_per_tan_angle_at_center = 0.036f,
                        .max_r = 1.0f,
                        .chromatic_abberation = {-0.015f, -0.02f, 0.025f, 0.02f},
                        .k =
                            {
                                1.003f,
                                1.02f,
                                1.042f,
                                1.066f,
                                1.094f,
                                1.126f,
                                1.162f,
                                1.203f,
                                1.25f,
                                1.31f,
                                1.38f,
                            },
                    },
            },
    }};

static const struct rift_lens_distortion CV1_DISTORTIONS[] = {{
    .distortion_version = RIFT_LENS_DISTORTION_LCSV_CATMULL_ROM_10_VERSION_1,
    .eye_relief = 0.015f,
    .data =
        {
            .lcsv_catmull_rom_10 =
                {
                    .meters_per_tan_angle_at_center = 0.0438f,
                    .max_r = 1.0f,
                    .chromatic_abberation = {-0.008f, -0.005f, 0.015f, 0.005f},
                    .k =
                        {
                            1.000f,
                            1.0312999f,
                            1.0698f,
                            1.1155f,
                            1.173f,
                            1.2460001f,
                            1.336f,
                            1.457f,
                            1.630f,
                            1.900f,
                            2.3599999f,
                        },
                },
        },
}};
// clang-format on

static void
rift_fill_in_default_distortions(struct rift_hmd *hmd)
{
	switch (hmd->variant) {
	case RIFT_VARIANT_DK2: {
		hmd->num_lens_distortions = ARRAY_SIZE(DK2_DISTORTIONS);
		hmd->lens_distortions = DK2_DISTORTIONS;

		// TODO: let the user specify which distortion is in use with an env var,
		//       and interpolate the distortions for the user's specific eye relief setting
		hmd->distortion_in_use = 1;

		break;
	}
	case RIFT_VARIANT_CV1: {
		hmd->num_lens_distortions = ARRAY_SIZE(CV1_DISTORTIONS);
		hmd->lens_distortions = CV1_DISTORTIONS;

		// TODO: let the user specify which distortion is in use with an env var,
		//       and interpolate the distortions for the user's specific eye relief setting
		hmd->distortion_in_use = 0;

		break;
	}
	}
}

static void
rift_fill_in_distortion_inputs(struct rift_hmd *hmd)
{
	assert(hmd->lens_distortions != NULL);
	assert(hmd->distortion_in_use < hmd->num_lens_distortions);

	float lens_diameter_m = 0.0f;
	switch (hmd->variant) {
	// TODO: figure out the *real* value for CV1 by dumping it from LibOVR somehow
	case RIFT_VARIANT_CV1: lens_diameter_m = 0.05f; break;
	case RIFT_VARIANT_DK2: lens_diameter_m = 0.04f; break;
	}

	hmd->panel = XRT_C11_COMPOUND(struct u_rift_panel){
	    .size_m =
	        {
	            MICROMETERS_TO_METERS(hmd->display_info.display_width),
	            MICROMETERS_TO_METERS(hmd->display_info.display_height),
	        },
	    // No headset seen so far reports a nonzero gap.
	    .eye_gap_m = 0.0f,
	    .size_px =
	        {
	            (float)hmd->display_info.resolution_x,
	            (float)hmd->display_info.resolution_y,
	        },
	    .lens_center_separation_m = MICROMETERS_TO_METERS(hmd->display_info.lens_separation),
	    .lens_center_from_panel_top = MICROMETERS_TO_METERS(hmd->display_info.center_v),
	    .lens_diameter_m = lens_diameter_m,
	};

	const struct rift_lens_distortion *distortion = &hmd->lens_distortions[hmd->distortion_in_use];
	const struct rift_catmull_rom_distortion_data *data = &distortion->data.lcsv_catmull_rom_10;

	hmd->eye_profile = XRT_C11_COMPOUND(struct u_rift_eye_profile){
	    .eye_relief_m = distortion->eye_relief,
	    .k = data->k,
	    .k_count = CATMULL_COEFFICIENTS,
	    .chromatic_abberation = {data->chromatic_abberation[0], data->chromatic_abberation[1],
	                             data->chromatic_abberation[2], data->chromatic_abberation[3]},
	    .max_r = data->max_r,
	    .meters_per_tan_angle = data->meters_per_tan_angle_at_center,
	};
}

/*
 *
 * Driver functions
 *
 */

static void
rift_hmd_destroy(struct xrt_device *xdev)
{
	struct rift_hmd *hmd = rift_hmd(xdev);

	// Don't do anything but break apart the device
	hmd->node.break_apart(&hmd->node);
}

static xrt_result_t
rift_hmd_update_inputs(struct xrt_device *xdev)
{
	struct rift_hmd *hmd = rift_hmd(xdev);

	// Only the CV1 has a presence sensor
	if (hmd->variant != RIFT_VARIANT_CV1) {
		return XRT_SUCCESS;
	}

	hmd->base.inputs[1].value = (union xrt_input_value){.boolean = hmd->presence};
	hmd->base.inputs[1].timestamp = os_monotonic_get_ns();
	return XRT_SUCCESS;
}

static xrt_result_t
rift_hmd_get_tracked_pose(struct xrt_device *xdev,
                          enum xrt_input_name name,
                          int64_t at_timestamp_ns,
                          struct xrt_space_relation *out_relation)
{
	struct rift_hmd *hmd = rift_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_relation_chain xrc = {0};

	if (hmd->use_constellation_poses) {
		// Constellation poses are in IMU space
		m_relation_chain_push_pose(&xrc, &hmd->T_imu_device);
	}

	get_raw_pose(hmd, at_timestamp_ns, false, &xrc);

	struct xrt_space_relation relation;
	m_relation_chain_resolve(&xrc, &relation);

	if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		// If we provide an orientation, make sure that it is normalized.
		math_quat_normalize(&relation.pose.orientation);
	}

	*out_relation = relation;
	return XRT_SUCCESS;
}

static xrt_result_t
rift_hmd_get_view_poses(struct xrt_device *xdev,
                        const struct xrt_vec3 *default_eye_relation,
                        int64_t at_timestamp_ns,
                        enum xrt_view_type view_type,
                        uint32_t view_count,
                        struct xrt_space_relation *out_head_relation,
                        struct xrt_fov *out_fovs,
                        struct xrt_pose *out_poses)
{
	struct rift_hmd *hmd = rift_hmd(xdev);

	struct xrt_vec3 eye_relation = XRT_VEC3_ZERO;
	if (hmd->icd_override_m >= 0.0f) {
		eye_relation.x = hmd->icd_override_m;
	} else {
		eye_relation.x = hmd->default_icd;
	}

	return u_device_get_view_poses( //
	    xdev,                       //
	    &eye_relation,              //
	    at_timestamp_ns,            //
	    view_type,                  //
	    view_count,                 //
	    out_head_relation,          //
	    out_fovs,                   //
	    out_poses);
}

static xrt_result_t
rift_hmd_get_visibility_mask(struct xrt_device *xdev,
                             enum xrt_visibility_mask_type type,
                             uint32_t view_index,
                             struct xrt_visibility_mask **out_mask)
{
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

static xrt_result_t
rift_hmd_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *out_result)
{
	struct rift_hmd *hmd = rift_hmd(xdev);

	// RIFT_DISTORTION_RENDER_FOV=0 maps into the lens' own extents as before, for comparing.
	const bool render_fov = debug_get_bool_option_rift_distortion_render_fov();
	u_compute_distortion_rift(&hmd->panel, &hmd->eye_profile, view, u, v,
	                          render_fov ? &hmd->base.hmd->distortion.fov[view] : NULL, out_result);

	return XRT_SUCCESS;
}

static void
rift_hmd_constellation_tracking_source_get_tracked_pose(struct t_constellation_tracker_tracking_source *tracking_source,
                                                        timepoint_ns when_ns,
                                                        struct xrt_space_relation *out_relation)
{
	struct rift_hmd *hmd = container_of(tracking_source, struct rift_hmd, constellation_tracking_source);

	struct xrt_relation_chain xrc = {0};
	get_raw_pose(hmd, when_ns, true, &xrc);
	m_relation_chain_resolve(&xrc, out_relation);
}

static void
rift_hmd_constellation_device_push_constellation_tracker_sample(struct t_constellation_tracker_device *connection,
                                                                struct t_constellation_tracker_sample *sample)
{
	struct rift_hmd *hmd = container_of(connection, struct rift_hmd, constellation_device);

	if (!sample->has_world_pose) {
		return;
	}

	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	relation.pose = sample->world_pose;
	relation.relation_flags = XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                          XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT |
	                          XRT_SPACE_RELATION_POSITION_VALID_BIT;

	struct xrt_space_relation relation_3dof = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(hmd->relation_hist, sample->timestamp_ns, &relation_3dof);

	if (relation_3dof.relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) {
		relation.angular_velocity = relation_3dof.angular_velocity;
		relation.relation_flags |= XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT;
	}

	m_relation_history_push_with_motion_estimation(hmd->raw_constellation_relation_hist, &relation,
	                                               sample->timestamp_ns);
}

static void
rift_frame_node_break_apart(struct xrt_frame_node *node)
{
	struct rift_hmd *hmd = rift_hmd_from_node(node);

	if (hmd->sensor_thread.initialized) {
		os_thread_helper_stop_and_wait(&hmd->sensor_thread);
	}

	// The panel off when the driver stops (RiftLinux): nothing turned it off before, so a worn headset's stayed lit
	// until it slept by itself.
	if (hmd->variant == RIFT_VARIANT_CV1 && hmd->display_on) {
		rift_enable_components(
		    hmd, &(struct rift_enable_components_report){.flags = RIFT_COMPONENT_AUDIO | RIFT_COMPONENT_LEDS});
		hmd->display_on = false;
		HMD_INFO(hmd, "Display off: the driver stops.");
	}

	if (hmd->radio_state.thread.initialized) {
		os_thread_helper_stop_and_wait(&hmd->radio_state.thread);
	}
}

static void
rift_frame_node_destroy(struct xrt_frame_node *node)
{
	struct rift_hmd *hmd = rift_hmd_from_node(node);

	if (hmd->constellation_tracker) {
		t_constellation_tracker_remove_device(hmd->constellation_tracker, hmd->constellation_device_id);
	}

	// Remove the variable tracking.
	u_var_remove_root(hmd);

	if (hmd->sensor_thread.initialized) {
		os_thread_helper_destroy(&hmd->sensor_thread);
	}

	if (hmd->clock_tracker) {
		m_clock_windowed_skew_tracker_destroy(hmd->clock_tracker);
	}

	m_relation_history_destroy(&hmd->relation_hist);
	m_relation_history_destroy(&hmd->raw_constellation_relation_hist);
	if (hmd->level.mutex_ready) {
		os_mutex_destroy(&hmd->level.mutex);
		hmd->level.mutex_ready = false;
	}
	if (hmd->guard.mutex_ready) {
		os_mutex_destroy(&hmd->guard.mutex);
		hmd->guard.mutex_ready = false;
	}
	if (hmd->gyro.mutex_ready) {
		os_mutex_destroy(&hmd->gyro.mutex);
		hmd->gyro.mutex_ready = false;
	}

	m_ff_vec3_f32_free(&hmd->gyro_ff);
	m_ff_vec3_f32_free(&hmd->accel_ff);
	m_ff_f64_free(&hmd->gravity_correction);

	// Only free if we are definitely the ones that allocated it
	if (hmd->lens_distortions && debug_get_bool_option_rift_use_firmware_distortion())
		free((void *)hmd->lens_distortions);

	os_hid_destroy(hmd->hmd_dev);
	if (hmd->radio_dev != NULL) {
		if (hmd->radio_state.thread.initialized) {
			os_thread_helper_destroy(&hmd->radio_state.thread);
		}

		os_hid_destroy(hmd->radio_dev);
	}

	if (hmd->device_count >= 0) {
		os_mutex_destroy(&hmd->device_mutex);

		// Free any sub-devices we created that weren't returned to the caller
		if (hmd->added_devices > 1) {
			for (int i = (hmd->added_devices - 1); i < hmd->device_count; i++) {
				u_device_free(hmd->devices[i]);
			}
		}
	}

	if (hmd->led_model.leds != NULL) {
		free(hmd->led_model.leds);

		hmd->led_model.leds = NULL;
		hmd->led_model.led_count = 0;
	}

	if (hmd->back_strap_model.leds != NULL) {
		free(hmd->back_strap_model.leds);
		hmd->back_strap_model.leds = NULL;
		hmd->back_strap_model.led_count = 0;
	}

	u_device_free(&hmd->base);
}

static const char *
variant_name(enum rift_variant variant)
{
	switch (variant) {
	case RIFT_VARIANT_DK2: return RIFT_DK2_PRODUCT_STRING;
	case RIFT_VARIANT_CV1: return RIFT_CV1_PRODUCT_STRING;
	}

	return "UNKNOWN";
}

/*
 *
 * Exported functions
 *
 */

int
rift_devices_create(struct os_hid_device *hmd_dev,
                    struct os_hid_device *radio_dev,
                    enum rift_variant variant,
                    const char *serial_number,
                    struct xrt_frame_context *xfctx,
                    struct rift_hmd **out_hmd,
                    struct xrt_device **out_xdevs)
{
	int result;

	// This indicates you won't be using Monado's built-in tracking algorithms.
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);

	bool has_presence = variant == RIFT_VARIANT_CV1;
	struct rift_hmd *hmd = U_DEVICE_ALLOCATE(struct rift_hmd, flags, has_presence ? 2 : 1, 0);
	// The tracker hears the wear state from the first report on.
	hmd->presence_told = -1;

	hmd->xfctx = xfctx;

	hmd->node.break_apart = rift_frame_node_break_apart;
	hmd->node.destroy = rift_frame_node_destroy;

	// Mark mutex as not initialized yet
	hmd->device_count = -1;

	hmd->variant = variant;
	hmd->hmd_dev = hmd_dev;
	hmd->radio_dev = radio_dev;

	// 4 seconds of IMU samples at 1000hz, should be more than enough
	m_ff_vec3_f32_alloc(&hmd->gyro_ff, 4096);
	m_ff_vec3_f32_alloc(&hmd->accel_ff, 4096);
	m_ff_f64_alloc(&hmd->gravity_correction, 4096);

	xrt_result_t xret = b_timing_source_create(xfctx, &hmd->timing_event_sink, &hmd->timing_event_source);
	U_LOG_CHK_ONLY_PRINT(hmd->log_level, xret, "b_timing_source_create");

	result = rift_send_keepalive(hmd);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to send keepalive to spin up headset, reason %d", result);
		goto error;
	}

	if (variant == RIFT_VARIANT_CV1) {
		// On CV1, we need to send a command to enable headset features
		rift_enable_components(hmd, &(struct rift_enable_components_report){.flags = RIFT_COMPONENT_DISPLAY |
		                                                                             RIFT_COMPONENT_AUDIO |
		                                                                             RIFT_COMPONENT_LEDS});
		hmd->display_on = true;
		hmd->display_on_until_ns = os_monotonic_get_ns() + RIFT_DISPLAY_ON_AT_START_NS;

		// Read the radio address, this also enables the radio
		result = rift_get_radio_address(hmd, hmd->radio_address);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to get radio address, reason %d", result);
			goto error;
		}

		HMD_DEBUG(hmd, "Got radio address %02X:%02X:%02X:%02X:%02X", hmd->radio_address[0],
		          hmd->radio_address[1], hmd->radio_address[2], hmd->radio_address[3], hmd->radio_address[4]);

		const int64_t brightness = debug_get_num_option_rift_display_brightness();
		if (brightness >= 0) {
			result = rift_set_cv1_display(hmd, (uint8_t)MIN(brightness, 255), RIFT_CV1_PERSISTENCE_ROWS);
			if (result < 0) {
				HMD_WARN(hmd, "Failed to set the display's brightness, reason %d", result);
			} else {
				HMD_INFO(hmd, "Display brightness %d, lit %d rows a frame.", (int)MIN(brightness, 255),
				         RIFT_CV1_PERSISTENCE_ROWS);
			}
		}
	}

	result = rift_get_display_info(hmd, &hmd->display_info);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to get device config, reason %d", result);
		goto error;
	}
	HMD_DEBUG(hmd, "Got display info from hmd, res: %dx%d", hmd->display_info.resolution_x,
	          hmd->display_info.resolution_y);

	result = rift_get_config(hmd, &hmd->config);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to get device config, reason %d", result);
		goto error;
	}
	HMD_DEBUG(hmd, "Got config from hmd, config flags: %X", hmd->config.config_flags);

	// Set a sane new config
	hmd->config.config_flags = RIFT_CONFIG_REPORT_USE_CALIBRATION | RIFT_CONFIG_REPORT_AUTO_CALIBRATION |
	                           RIFT_CONFIG_REPORT_COMMAND_KEEP_ALIVE | RIFT_CONFIG_REPORT_MOTION_KEEP_ALIVE;

	if (debug_get_bool_option_rift_power_override()) {
		hmd->config.config_flags |= RIFT_CONFIG_REPORT_OVERRIDE_POWER;
		HMD_INFO(hmd, "Enabling the override power config flag.");
	} else {
		hmd->config.config_flags &= ~RIFT_CONFIG_REPORT_OVERRIDE_POWER;
		HMD_DEBUG(hmd, "Disabling the override power config flag.");
	}

	// @todo figure out why we have to set these to zero
	hmd->config.interval = 0;
	hmd->config.command_id = 0;

	// update the config
	result = rift_set_config(hmd, &hmd->config);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to set the device config, reason %d", result);
		goto error;
	}

	// read it back
	result = rift_get_config(hmd, &hmd->config);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to set the device config, reason %d", result);
		goto error;
	}
	HMD_DEBUG(hmd, "After writing, HMD has config flags: %X", hmd->config.config_flags);

	if ((hmd->config.config_flags & RIFT_CONFIG_REPORT_USE_CALIBRATION) == 0) {
		HMD_INFO(hmd,
		         "Headset calibration enabling ignored by the headset, "
		         "reading out calibration to do ourselves");

		result = rift_get_imu_calibration(hmd, &hmd->imu_calibration);
		if (result < 0) {
			HMD_ERROR(hmd,
			          "failed to get IMU calibration, this is non-fatal, but headset IMU might drift more "
			          "than expected, reason %d",
			          result);
			hmd->imu_needs_calibration = false;
		} else {
			hmd->imu_needs_calibration = true;

			// Logged so that recordings, which carry the calibrated samples, can be turned back into raw
			// ones.
			const struct rift_imu_calibration *c = &hmd->imu_calibration;
			HMD_INFO(
			    hmd,
			    "IMU factory calibration at %.2f C: accel offset [%f %f %f], gyro offset [%f %f %f], "
			    "accel matrix [%f %f %f; %f %f %f; %f %f %f], gyro matrix [%f %f %f; %f %f %f; %f %f %f]",
			    (double)c->temperature, (double)c->accel_offset.x, (double)c->accel_offset.y,
			    (double)c->accel_offset.z, (double)c->gyro_offset.x, (double)c->gyro_offset.y,
			    (double)c->gyro_offset.z, (double)c->accel_matrix.v[0], (double)c->accel_matrix.v[1],
			    (double)c->accel_matrix.v[2], (double)c->accel_matrix.v[3], (double)c->accel_matrix.v[4],
			    (double)c->accel_matrix.v[5], (double)c->accel_matrix.v[6], (double)c->accel_matrix.v[7],
			    (double)c->accel_matrix.v[8], (double)c->gyro_matrix.v[0], (double)c->gyro_matrix.v[1],
			    (double)c->gyro_matrix.v[2], (double)c->gyro_matrix.v[3], (double)c->gyro_matrix.v[4],
			    (double)c->gyro_matrix.v[5], (double)c->gyro_matrix.v[6], (double)c->gyro_matrix.v[7],
			    (double)c->gyro_matrix.v[8]);
		}
	}

	if (debug_get_bool_option_rift_use_firmware_distortion()) {
		// get the lens distortions
		struct rift_lens_distortion_report lens_distortion_report;
		result = rift_get_lens_distortion(hmd, &lens_distortion_report);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to get lens distortion, reason %d", result);
			goto error;
		}

		hmd->num_lens_distortions = lens_distortion_report.num_distortions;
		struct rift_lens_distortion *lens_distortions =
		    calloc(lens_distortion_report.num_distortions, sizeof(struct rift_lens_distortion));

		rift_parse_distortion_report(&lens_distortion_report,
		                             &lens_distortions[lens_distortion_report.distortion_idx]);
		// TODO: actually verify we initialize all the distortions. if the headset is working correctly,
		// this should have happened, but you never know.
		for (uint16_t i = 1; i < hmd->num_lens_distortions; i++) {
			result = rift_get_lens_distortion(hmd, &lens_distortion_report);
			if (result < 0) {
				HMD_ERROR(hmd, "Failed to get lens distortion idx %d, reason %d", i, result);
				free(lens_distortions);
				goto error;
			}

			rift_parse_distortion_report(&lens_distortion_report,
			                             &lens_distortions[lens_distortion_report.distortion_idx]);
		}

		hmd->lens_distortions = lens_distortions;

		// TODO: pick the correct distortion for the eye relief setting the user has picked
		hmd->distortion_in_use = 0;
	} else {
		rift_fill_in_default_distortions(hmd);
	}

	result = rift_get_tracking_report(hmd, &hmd->tracking);
	if (result == 0) {
		// @note Intentionally we don't enable CUSTOM_PATTERN bit so that we don't inherit external custom
		//       patterns that may have been written by other processes or prior Monado runs
		hmd->tracking.flags = RIFT_TRACKING_ENABLE | RIFT_TRACKING_USE_CARRIER;
		hmd->tracking.pattern_idx = 0xff;

		hmd->tracking.vsync_offset = 0;
		hmd->tracking.duty_cycle = 0x7f;

		switch (hmd->variant) {
		case RIFT_VARIANT_DK2:
			hmd->tracking.exposure_length = 350;
			hmd->tracking.frame_interval = 16666;
			break;
		case RIFT_VARIANT_CV1:
			/*
			 * The LEDs light for 399 us of the sensors' 495 us exposure, as the Oculus runtime has them.
			 * The blob thresholds were set on its frames: lit longer, the LEDs come out
			 * brighter than that, and the Touch ring's, 2-4 px apart at a few metres, run together sooner.
			 */
			hmd->tracking.exposure_length = 399;
			hmd->tracking.frame_interval = 19200;
			break;
		}

		result = rift_set_tracking(hmd, &hmd->tracking);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to enable tracking.");
		}
	}

	result = rift_read_led_model(hmd);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to read LED model, reason %d", result);
		goto error;
	} else {
		HMD_DEBUG(hmd, "Read LED model from Rift.");
	}

	if (!debug_get_bool_option_rift_enable_back_strap_leds()) {
		// Writes to the CustomPattern report only work with CUSTOM_PATTERN set to true
		hmd->tracking.flags |= RIFT_TRACKING_CUSTOM_PATTERN | RIFT_TRACKING_AUTO_INCREMENT;
		hmd->tracking.pattern_idx = 0;

		result = rift_set_tracking(hmd, &hmd->tracking);
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to enable custom pattern mode, reason %d", result);
			goto error;
		}

		result = rift_upload_back_strap_disable_pattern(hmd, debug_get_bool_option_rift_back_strap_leds_lit_untracked(),
		                                                debug_get_bool_option_rift_back_strap_leds_tracked());
		if (result < 0) {
			HMD_ERROR(hmd, "Failed to upload back strap disable pattern, reason %d", result);
			goto error;
		}

		HMD_INFO(hmd, "Back strap LEDs %s via custom pattern.",
		         debug_get_bool_option_rift_back_strap_leds_tracked() ? "lit for tracking" : "set up");
	}

	hmd->constellation_tracking_source.get_tracked_pose = rift_hmd_constellation_tracking_source_get_tracked_pose;
	hmd->constellation_device.push_constellation_tracker_sample =
	    rift_hmd_constellation_device_push_constellation_tracker_sample;

	// fill in extra display info about the headset

	size_t idx = 0;
	hmd->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = idx;

	hmd->base.update_inputs = rift_hmd_update_inputs;
	hmd->base.get_tracked_pose = rift_hmd_get_tracked_pose;
	hmd->base.get_view_poses = rift_hmd_get_view_poses;
	hmd->base.get_visibility_mask = rift_hmd_get_visibility_mask;
	hmd->base.destroy = rift_hmd_destroy;

	// Has to happen before the mesh is filled in below, which calls straight into compute_distortion.
	rift_fill_in_distortion_inputs(hmd);

	hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.compute_distortion = rift_hmd_compute_distortion;

	hmd->log_level = debug_get_log_option_rift_log();

	// Print name.
	u_truncate_snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "%s", variant_name(hmd->variant));
	u_truncate_snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "%s", serial_number);

	m_relation_history_create(&hmd->relation_hist);
	m_relation_history_create(&hmd->raw_constellation_relation_hist);
	hmd->level.mutex_ready = os_mutex_init(&hmd->level.mutex) == 0;
	hmd->guard.mutex_ready = os_mutex_init(&hmd->guard.mutex) == 0;
	hmd->gyro.mutex_ready = os_mutex_init(&hmd->gyro.mutex) == 0;

	// Setup input.
	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	if (has_presence) {
		hmd->base.inputs[1].name = XRT_INPUT_GENERIC_HEAD_DETECT;
	}
	hmd->base.supported.orientation_tracking = true;
	// @todo Disable this when the user doesn't have sensors connected.
	hmd->base.supported.position_tracking = true;
	hmd->base.supported.presence = variant == RIFT_VARIANT_CV1;

	// Set up display details
	switch (hmd->variant) {
	case RIFT_VARIANT_DK2: hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 75.0f); break;
	case RIFT_VARIANT_CV1: hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / 90.0f); break;
	}

	hmd->default_icd = MICROMETERS_TO_METERS(hmd->display_info.lens_separation);

	hmd->icd_override_m = debug_get_float_option_rift_override_icd_mm() / 1000.0f;
	if (hmd->icd_override_m >= 0.0f) {
		HMD_INFO(hmd, "Applying ICD override of %f", hmd->icd_override_m);
	} else {
		HMD_DEBUG(hmd, "Using default ICD of %f", hmd->default_icd);
	}

	switch (hmd->variant) {
	case RIFT_VARIANT_CV1: {
		hmd->base.hmd->screens[0].w_pixels = hmd->display_info.resolution_x;
		hmd->base.hmd->screens[0].h_pixels = hmd->display_info.resolution_y;

		// TODO: properly apply using u_rift_panel::eye_gap_m, but this isn't necessary, as
		//       observed gap is always zero
		uint16_t view_width = hmd->display_info.resolution_x / 2;
		uint16_t view_height = hmd->display_info.resolution_y;

		for (uint32_t i = 0; i < 2; ++i) {
			hmd->base.hmd->views[i].display.w_pixels = view_width;
			hmd->base.hmd->views[i].display.h_pixels = view_height;

			hmd->base.hmd->views[i].viewport.x_pixels = i * (hmd->display_info.resolution_x / 2);
			hmd->base.hmd->views[i].viewport.y_pixels = 0;
			hmd->base.hmd->views[i].viewport.w_pixels = view_width;
			hmd->base.hmd->views[i].viewport.h_pixels = view_height;
			hmd->base.hmd->views[i].rot = u_device_rotation_ident;
		}

		// TODO: figure out how to calculate this programmatically, right now this is hardcoded with data dumped
		//       from oculus' OpenXR runtime, some of the math for this is in rift_distortion.c, used for
		//       calculating distortion
		hmd->base.hmd->distortion.fov[0].angle_up = 0.7269826;
		hmd->base.hmd->distortion.fov[0].angle_down = -0.8378981;
		hmd->base.hmd->distortion.fov[0].angle_left = -0.76754993;
		hmd->base.hmd->distortion.fov[0].angle_right = 0.6208969;

		hmd->base.hmd->distortion.fov[1].angle_up = 0.7269826;
		hmd->base.hmd->distortion.fov[1].angle_down = -0.8378981;
		hmd->base.hmd->distortion.fov[1].angle_left = -0.6208969;
		hmd->base.hmd->distortion.fov[1].angle_right = 0.76754993;

		/*
		 * One rendered pixel per panel pixel at the centre of the lens (Oculus' pixel density 1): the lens' metres
		 * per unit of tangent at its centre (from the headset) over the panel's pixel pitch, across each view's
		 * field of view. About 1322x1572 a view; Oculus' runtime asks 1332x1586. The size games are asked to
		 * render at 100 %; more (supersampling) is the user's choice.
		 */
		const float metres_per_pixel = hmd->panel.size_m.x / hmd->panel.size_px.x;
		const float px_per_tan = metres_per_pixel > 0.0f ? hmd->eye_profile.meters_per_tan_angle / metres_per_pixel
		                                                 : 0.0f;
		for (uint32_t i = 0; i < 2; ++i) {
			const struct xrt_fov *f = &hmd->base.hmd->distortion.fov[i];
			hmd->base.hmd->views[i].native.w_pixels =
			    (uint32_t)ceilf((tanf(f->angle_right) - tanf(f->angle_left)) * px_per_tan);
			hmd->base.hmd->views[i].native.h_pixels =
			    (uint32_t)ceilf((tanf(f->angle_up) - tanf(f->angle_down)) * px_per_tan);
		}
		HMD_INFO(hmd, "1:1 render size at the lens centre: %ux%u a view (%.1f pixels per tangent: lens %.5f m, "
		              "pixel %.2f um)",
		         hmd->base.hmd->views[0].native.w_pixels, hmd->base.hmd->views[0].native.h_pixels, px_per_tan,
		         hmd->eye_profile.meters_per_tan_angle, metres_per_pixel * 1e6f);

		break;
	}
	case RIFT_VARIANT_DK2: {
		// screen is rotated, so we need to undo that here
		hmd->base.hmd->screens[0].h_pixels = hmd->display_info.resolution_x;
		hmd->base.hmd->screens[0].w_pixels = hmd->display_info.resolution_y;

		// TODO: properly apply using u_rift_panel::eye_gap_m, but this isn't necessary, as
		//       observed gap is always zero
		uint16_t view_width = hmd->display_info.resolution_x / 2;
		uint16_t view_height = hmd->display_info.resolution_y;

		for (uint32_t i = 0; i < 2; ++i) {
			hmd->base.hmd->views[i].display.w_pixels = view_width;
			hmd->base.hmd->views[i].display.h_pixels = view_height;

			hmd->base.hmd->views[i].viewport.x_pixels = 0;
			hmd->base.hmd->views[i].viewport.y_pixels = (1 - i) * (hmd->display_info.resolution_x / 2);
			hmd->base.hmd->views[i].viewport.w_pixels = view_height; // screen is rotated, so swap w and h
			hmd->base.hmd->views[i].viewport.h_pixels = view_width;
			hmd->base.hmd->views[i].rot = u_device_rotation_left;
		}

		// TODO: figure out how to calculate this programmatically, right now this is hardcoded with data dumped
		//       from oculus' OpenXR runtime, some of the math for this is in rift_distortion.c, used for
		//       calculating distortion
		hmd->base.hmd->distortion.fov[0].angle_up = 0.92667186;
		hmd->base.hmd->distortion.fov[0].angle_down = -0.92667186;
		hmd->base.hmd->distortion.fov[0].angle_left = -0.8138836;
		hmd->base.hmd->distortion.fov[0].angle_right = 0.82951474;

		hmd->base.hmd->distortion.fov[1].angle_up = 0.92667186;
		hmd->base.hmd->distortion.fov[1].angle_down = -0.92667186;
		hmd->base.hmd->distortion.fov[1].angle_left = -0.82951474;
		hmd->base.hmd->distortion.fov[1].angle_right = 0.8138836;

		break;
	}
	}

	// After the views' field of view is set: the distortion's UVs index the image rendered with it.
	u_distortion_mesh_fill_in_compute(&hmd->base);

	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
	hmd->clock_tracker = m_clock_windowed_skew_tracker_alloc(2048);

	result = os_thread_helper_init(&hmd->sensor_thread);

	if (result < 0) {
		HMD_ERROR(hmd, "Failed to init os thread helper");
		goto error;
	}

	result = os_thread_helper_start(&hmd->sensor_thread, rift_sensor_thread, hmd);

	if (result < 0) {
		HMD_ERROR(hmd, "Failed to start sensor thread");
		goto error;
	}

	result = os_mutex_init(&hmd->device_mutex);
	if (result < 0) {
		HMD_ERROR(hmd, "Failed to init radio mutex");
		goto error;
	}
	hmd->device_count = 0;

	if (hmd->radio_dev != NULL) {
		// Before the radio thread, so the controllers exist for applications even if they connect later.
		result = rift_radio_create_touch_controllers(hmd);
		if (result < 0) {
			goto error;
		}

		result = os_thread_helper_init(&hmd->radio_state.thread);

		if (result < 0) {
			HMD_ERROR(hmd, "Failed to init os thread helper for radio");
			goto error;
		}

		result = os_thread_helper_start(&hmd->radio_state.thread, rift_radio_thread, hmd);

		if (result < 0) {
			HMD_ERROR(hmd, "Failed to start radio thread");
			goto error;
		}
	}

	// Setup variable tracker: Optional but useful for debugging
	u_var_add_root(hmd, "Rift HMD", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_ro_i32(hmd, (int32_t *)&hmd->variant, "variant");
	u_var_add_f32(hmd, &hmd->default_icd, "default_icd");
	u_var_add_ro_i64_ns(hmd, &hmd->last_remote_sample_time_ns, "last_remote_sample_time_ns");
	u_var_add_ro_i64_ns(hmd, &hmd->last_sample_local_timestamp_ns, "last_sample_local_timestamp_ns");
	u_var_add_ro_i64_ns(hmd, &hmd->last_remote_exposure_time_ns, "last_remote_exposure_time_ns");
	u_var_add_f32(hmd, &hmd->icd_override_m, "icd_override_m");
	u_var_add_bool(hmd, &hmd->presence, "presence");

	u_var_add_gui_header(hmd, NULL, "IMU Fusion");
	m_imu_3dof_add_vars(&hmd->fusion, hmd, "3dof_");

	u_var_add_gui_header(hmd, NULL, "IMU Calibration");
	u_var_add_bool(hmd, &hmd->imu_needs_calibration, "imu_needs_calibration");
	u_var_add_vec3_f32(hmd, &hmd->imu_calibration.gyro_offset, "gyro_offset");
	u_var_add_vec3_f32(hmd, &hmd->imu_calibration.accel_offset, "accel_offset");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.gyro_matrix.v[0], "gyro_matrix[0..3]");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.gyro_matrix.v[3], "gyro_matrix[3..6]");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.gyro_matrix.v[6], "gyro_matrix[6..9]");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.accel_matrix.v[0], "accel_matrix[0..3]");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.accel_matrix.v[3], "accel_matrix[3..6]");
	u_var_add_vec3_f32(hmd, (struct xrt_vec3 *)&hmd->imu_calibration.accel_matrix.v[6], "accel_matrix[6..9]");
	u_var_add_f32(hmd, &hmd->imu_calibration.temperature, "temperature");

	// wait for display/controller init
	// @note once monado *has* the capabilities to signal to us when a display is ready, we should use them!
	os_nanosleep(time_s_to_ns(debug_get_float_option_rift_startup_wait_time()));

	*out_hmd = hmd;

	os_mutex_lock(&hmd->device_mutex);
	out_xdevs[hmd->added_devices++] = &hmd->base;
	for (int i = 0; i < hmd->device_count; i++) {
		out_xdevs[hmd->added_devices++] = hmd->devices[i];
	}
	os_mutex_unlock(&hmd->device_mutex);

	return hmd->added_devices;
error:
	rift_frame_node_destroy(&hmd->node);
	return -1;
}

bool
rift_get_radio_id(struct rift_hmd *hmd, uint8_t out_radio_id[5])
{
	if (hmd->variant != RIFT_VARIANT_CV1) {
		return false;
	}

	memcpy(out_radio_id, hmd->radio_address, sizeof(hmd->radio_address));

	return true;
}

bool
rift_hmd_frame_timestamp_callback(void *user_data,             //
                                  timepoint_ns *out_timestamp, //
                                  uint64_t *out_sequence_id,   //
                                  timepoint_ns frame_start_ns, //
                                  uint32_t pts)                //
{
	struct rift_hmd *hmd = (struct rift_hmd *)user_data;

	/*
	 * A frame belongs to the exposure whose IN report arrives alongside its first packet, which is the match
	 * OpenHMD makes. Searching for that exposure directly does not work here: the gap between the two arrivals
	 * sits near zero, the ~1 ms quantisation of the 1 kHz report stream walks it either side, and whenever it goes
	 * negative the report has not been pushed into the history yet and the frame finds nothing.
	 *
	 * So the search is centred one frame interval earlier, where the exposure is always already in the history and
	 * has half an interval of margin each way, and one interval is added back to the timestamp it hands out. The
	 * exposure that lands on is the one the search cannot reliably see, extrapolated across a single interval of a
	 * clock that the HMD holds to well under a microsecond of that per frame.
	 *
	 * Frames with nothing in the window are left alone rather than guessed at.
	 */
	const time_duration_ns interval_ns = (time_duration_ns)hmd->tracking.frame_interval * OS_NS_PER_USEC;
	const time_duration_ns window_ns = interval_ns / 2;

	struct rift_exposure_event match = {0};
	bool have_match = false;

	os_thread_helper_lock(&hmd->sensor_thread);

	const uint64_t oldest_held = hmd->exposure_history_pushed > RIFT_EXPOSURE_HISTORY_SIZE
	                                 ? hmd->exposure_history_pushed - RIFT_EXPOSURE_HISTORY_SIZE
	                                 : 0;

	for (uint64_t i = oldest_held; i < hmd->exposure_history_pushed; i++) {
		const struct rift_exposure_event *event = &hmd->exposure_history[i % RIFT_EXPOSURE_HISTORY_SIZE];

		const time_duration_ns offset_ns = frame_start_ns - event->recv_timestamp_ns - interval_ns;
		if (offset_ns > -window_ns && offset_ns < window_ns) {
			match = *event;
			have_match = true;
		}
	}

	os_thread_helper_unlock(&hmd->sensor_thread);

	if (!have_match) {
		// Either the clocks have not synchronized yet, so there is nothing to match against, or the frame
		// landed between two exposures. Leaving the frame with the receive time the UVC driver already gave it
		// is far better than attributing it to an exposure it may not belong to.
		HMD_TRACE(hmd, "Frame with PTS %u matched no exposure", pts);
		return false;
	}

	*out_timestamp = match.timestamp_ns + interval_ns;
	*out_sequence_id = match.sequence_id + 1;

	HMD_TRACE(hmd,
	          "Frame with PTS %u matched exposure %u, taken at %" PRId64 " ns (sequence ID %" PRIu64
	          "), report %" PRId64 " ns before frame, %" PRId64 " ns from window centre",
	          pts, match.sequence_id + 1, *out_timestamp, *out_sequence_id,
	          frame_start_ns - match.recv_timestamp_ns, frame_start_ns - match.recv_timestamp_ns - interval_ns);

	return true;
}

int
rift_add_to_constellation_tracker(struct rift_hmd *hmd, struct t_constellation_tracker *tracker)
{
	struct t_constellation_tracker_device_params params = {
	    .led_model = hmd->led_model,
	    .tracking_source = &hmd->constellation_tracking_source,
	    .loose_part = hmd->back_strap_model,
	};
	// Where the strap sits is saved per headset.
	u_truncate_snprintf(params.serial, sizeof(params.serial), "%s", hmd->base.serial);

	hmd->constellation_tracker = tracker;

	int ret = t_constellation_tracker_add_device(tracker, &params, &hmd->constellation_device,
	                                             &hmd->constellation_device_id);
	if (ret < 0) {
		HMD_ERROR(hmd, "Failed to add Rift HMD to constellation tracker, reason %d", ret);
		return ret;
	}

	struct xrt_tracking_origin *tracking_origin = t_constellation_tracker_get_tracking_origin(tracker);
	hmd->base.tracking_origin = tracking_origin;
	hmd->constellation_imu_sink = params.imu_sink;

	// Mark that we're using constellation poses now
	hmd->use_constellation_poses = true;

	/*
	 * Every controller is in the tracker's space from now on, also one that is asleep and only joins the tracker
	 * once it wakes: the space overseer links a device to the tracking origin it has when the system is set up,
	 * which is right after this, and never looks again. A Touch picked up after the start kept the HMD's earlier
	 * origin there and was drawn off to one side of where it was tracked (two recorded sessions).
	 */
	os_mutex_lock(&hmd->device_mutex);
	for (int i = 0; i < RADIO_STATE_TOUCH_CONTROLLERS_LEN; ++i) {
		if (hmd->radio_state.touch_controllers[i] != NULL) {
			hmd->radio_state.touch_controllers[i]->base.tracking_origin = tracking_origin;
		}
	}
	if (hmd->radio_state.remote != NULL) {
		hmd->radio_state.remote->base.tracking_origin = tracking_origin;
	}

	// Controllers whose calibration is read already; the others are added once theirs is.
	for (int i = 0; i < RADIO_STATE_TOUCH_CONTROLLERS_LEN; ++i) {
		if (hmd->radio_state.touch_controllers[i] != NULL) {
			rift_touch_controller_add_to_constellation(hmd, hmd->radio_state.touch_controllers[i]);
		}
	}
	os_mutex_unlock(&hmd->device_mutex);

	return 0;
}

struct t_timing_event_source *
rift_hmd_get_timing_event_source(struct rift_hmd *hmd)
{
	return hmd->timing_event_source;
}
