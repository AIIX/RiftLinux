// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Routines to help decoding PS VR2 controller Gray codes + brightness curves
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "constellation/t_led_sync_refinement.h"
#include "util/u_time.h"

#include "math/m_api.h"

#include "led_sync_internal.h"
#include <float.h>


/*!
 * Whether to enable the "spew" for brightness-decoding related routines.
 *
 * @note This should never be upstreamed set to `1`!
 */
#define GRAY_SPEW 0

#if GRAY_SPEW
#define LOG_GRAY_SPEW(refinement, ...) LOG_DEBUG(refinement, __VA_ARGS__)
#else
#define LOG_GRAY_SPEW(refinement, ...)
#endif

#define TIME_BETWEEN_LED_SELECTIONS_NS (U_TIME_1MS_IN_NS * 2500LL)

//! The jump amount for an invalid code
#define INVALID_GRAY_CODE_JUMP INT64_MAX
//! This gray code isn't in the official table.
#define INVALID_GRAY_CODE (0x4)
//! How many identical gray codes do we need to decode in a row to accept it.
#define GRAY_CODE_MATCHES_NEEDED 3

//! Proportional gain for the BG clock-offset servo.
#define BG_OFFSET_GAIN 0.5f
//! BG measurements per skew least-squares window.
#define BG_SKEW_WINDOW_SAMPLES 20
//! Clamp on the estimated clock skew, in ns per ns.
#define BG_MAX_SKEW 1e-3

//! A table of how much time to jump for each gray code
static const time_duration_ns gray_code_jumps_ns[] = {
    U_TIME_1US_IN_NS * -500, // 000
    U_TIME_1US_IN_NS * -250, // 001
    U_TIME_1US_IN_NS * 250,  // 010
    0,                       // 011 <- Target code
    INVALID_GRAY_CODE_JUMP,  // 100 <- Invalid, never emit.
    U_TIME_1US_IN_NS * 750,  // 101
    U_TIME_1US_IN_NS * 500,  // 110
    U_TIME_1US_IN_NS * 750,  // 111
};

static void
handle_decode_gray_tick(struct t_led_sync_refinement *refinement)
{
	// If the final one hasn't been marked as applied, but it has been applied, send the new broad mode
	if (!refinement->decode_gray.applied_gray_start && refinement->sample_applied) {
		set_pssense_mode(refinement, T_LED_SYNC_SAMPLE_PSSENSE_MODE_BROAD, NULL);

		refinement->decode_gray.applied_gray_start = true;
	}
}

static void
handle_successful_gray_decode(struct t_led_sync_refinement *refinement, uint16_t gray_code)
{
	timepoint_ns jump_amount_ns = gray_code_jumps_ns[gray_code];
	assert(jump_amount_ns != INVALID_GRAY_CODE_JUMP);

	if (jump_amount_ns == 0) {
		LOG_DEBUG(refinement, "Got the \"done\" gray code (%d), moving to next phase.", gray_code);

		// We're done!
		set_phase_locked(refinement, T_LED_SYNC_PHASE_OPTIMIZE_BRIGHTNESS_CURVE);

		return;
	}

	LOG_DEBUG(refinement, "Accepting gray code %x after %d identical matches, moving %" PRIi64 "us", gray_code,
	          refinement->decode_gray.matching_decodes, (jump_amount_ns / 1000L));

	bool success = advance_clock_offset_pssense_locked(refinement, jump_amount_ns);
	// @todo should we have a "busy" recovery strategy here?
	(void)success;
}

static void
handle_decoded_gray_code(struct t_led_sync_refinement *refinement, uint16_t decoded_gray_code)
{
	LOG_GRAY_SPEW(refinement, "Decoded gray code! Got %x", decoded_gray_code);

	// The code is invalid if it doesn't match the current and isn't the invalid gray code
	bool invalid_code = decoded_gray_code != refinement->decode_gray.decoded_gray_code || //
	                    decoded_gray_code == INVALID_GRAY_CODE;

	if (invalid_code) {
		// Reset since the code didn't match.
		refinement->decode_gray.decoded_gray_code = decoded_gray_code;
		refinement->decode_gray.matching_decodes = 1;
	} else {
		refinement->decode_gray.matching_decodes += 1;

		if (refinement->decode_gray.matching_decodes == GRAY_CODE_MATCHES_NEEDED) {
			// Success!
			handle_successful_gray_decode(refinement, decoded_gray_code);

			// Re-arm the counter so the next run of identical codes can be accepted too.
			// Without this the counter just keeps incrementing past GRAY_CODE_MATCHES_NEEDED
			// and never equals it again, so a stable (noise-free) code is only ever accepted
			// once and the offset stops converging after a single jump.
			refinement->decode_gray.matching_decodes = 0;
		}
	}
}

static void
handle_leds_seen(struct t_led_sync_refinement *refinement, const struct t_constellation_tracker_sample *sample)
{
	// Whether the sequence ID is being incremented or not
	bool clear_slot = sample->sequence_id > refinement->leds_seen.cur_sequence_id;

	// Clear the brightnesses for all cameras in the old slot.
	if (clear_slot) {
		for (uint32_t camera_idx = 0; camera_idx < XRT_TRACKING_MAX_CAMS; camera_idx++) {
			for (uint32_t led_idx = 0; led_idx < T_LED_SYNC_SELECTED_LEDS; led_idx++) {
				size_t led_buffer_index =
				    T_LED_SYNC_GRAY_CODE_BUFFER_INDEX(camera_idx, sample->sequence_id, led_idx);

				refinement->decode_gray.led_brightnesses[led_buffer_index] = 0.0f;
				// 1.0 would be "facing the same direction as the camera", so the worst case, we
				// want facing *toward* the camera, which will be -1.
				refinement->decode_gray.led_facing_dots[led_buffer_index] = 1.0f;

				led_buffer_index =
				    T_LED_SYNC_BRIGHTNESS_CURVE_BUFFER_INDEX(camera_idx, sample->sequence_id, led_idx);

				refinement->optimize_brightness_curve.led_brightnesses[led_buffer_index] = 0.0f;
				refinement->optimize_brightness_curve.led_facing_dots[led_buffer_index] = 1.0f;
			}
		}

		refinement->leds_seen.num_samples += 1;
	}

	// Wait until we have enough samples
	if (refinement->leds_seen.num_samples < T_LED_SYNC_SEEN_LEDS_ACCUMULATION_BUFFER_SIZE) {
		return;
	}

	uint32_t seen_led_idx =
	    T_LED_SYNC_SEEN_LED_BUFFER_INDEX(refinement->leds_seen.cur_sequence_id = sample->sequence_id);

	struct t_led_sync_seen_led *led_buf =
	    refinement->leds_seen.buf + (seen_led_idx * XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE);

	static_assert(ARRAY_SIZE(sample->leds) == XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE,
	              "LED sync depends on these being equal.");

	for (uint32_t i = 0; i < ARRAY_SIZE(sample->leds); i++) {
		if (clear_slot) {
			led_buf[i] = (struct t_led_sync_seen_led){0};
		}

		if (!sample->leds[i].observed) {
			continue;
		}

		led_buf[i].accum_seen_count += 1;
		led_buf[i].accum_facing_dot += sample->leds[i].facing_dot;
	}

	bool need_new_led_selection =
	    (sample->timestamp_ns - refinement->leds_seen.last_selection_ns) > TIME_BETWEEN_LED_SELECTIONS_NS &&
	    refinement->sample_for_driver.pssense_mode != T_LED_SYNC_SAMPLE_PSSENSE_MODE_PRESCAN;

	// If we don't have a sample outgoing, then compute a new one for the new "most-seen" LEDs.
	if (!refinement->has_sample_for_driver && need_new_led_selection) {
		struct t_led_sync_seen_led leds_seen[XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE] = {0};

		for (uint32_t window_idx = 0; window_idx < T_LED_SYNC_SEEN_LEDS_ACCUMULATION_BUFFER_SIZE;
		     window_idx++) {
			struct t_led_sync_seen_led *window_led_buf =
			    refinement->leds_seen.buf + (window_idx * XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE);

			for (uint32_t led_idx = 0; led_idx < XRT_CONSTELLATION_MAX_LEDS_PER_DEVICE; led_idx++) {
				leds_seen[led_idx].accum_seen_count += window_led_buf[led_idx].accum_seen_count;
				leds_seen[led_idx].accum_facing_dot += window_led_buf[led_idx].accum_facing_dot;
			}
		}

		size_t most_seen_index = 0;
		float most_seen_value = -INFINITY;
		size_t second_most_seen_index = 0;
		float second_most_seen_value = -INFINITY;
		for (uint32_t i = 0; i < ARRAY_SIZE(leds_seen); i++) {
			struct t_led_sync_seen_led seen = leds_seen[i];

			// Seen more times == good, facing more towards camera == good
			float value = seen.accum_seen_count + -(seen.accum_facing_dot / seen.accum_seen_count);

			if (value > most_seen_value) {
				second_most_seen_index = most_seen_index;
				second_most_seen_value = most_seen_value;

				most_seen_index = i;
				most_seen_value = value;
			} else if (value > second_most_seen_value) {
				second_most_seen_index = i;
				second_most_seen_value = value;
			}
		}

		uint8_t leds[4] = {0xFF, 0xFF, 0xFF, 0xFF};
		if (most_seen_value > -INFINITY) {
			leds[0] = most_seen_index;
		}
		if (second_most_seen_value > -INFINITY) {
			leds[1] = second_most_seen_index;
		}

		LOG_GRAY_SPEW(refinement, "Selected LEDs %d,%d", leds[0], leds[1]);

		// Only set the new one if there isn't an existing sample ready
		if (!refinement->has_sample_for_driver) {
			set_pssense_mode(refinement, refinement->sample_for_driver.pssense_mode, leds);

			refinement->leds_seen.last_selection_ns = sample->timestamp_ns;
		}
	}
}

static void
handle_decode_gray_constellation_sample(struct t_led_sync_refinement *refinement,
                                        const struct t_constellation_tracker_sample *sample)
{
	XRT_MAYBE_UNUSED const uint16_t decoding_mask = 0x7FF; // 0b11111111111
	const uint16_t header_mask = 0x7F8;                    // 0b11111111000
	const uint16_t gray_code_mask = 0x7;                   // 0b00000000111
	const uint16_t expected_header = 0x38;                 // 0b00000111000

	// Update the ring buffer with the new values
	for (int i = 0; i < T_LED_SYNC_SELECTED_LEDS; i++) {
		uint8_t led = refinement->sample_for_driver.pssense.leds[i];
		if (led == 0xFF) {
			continue;
		}

		size_t led_buffer_index =
		    T_LED_SYNC_GRAY_CODE_BUFFER_INDEX(sample->camera_index, refinement->leds_seen.cur_sequence_id, i);

		refinement->decode_gray.led_brightnesses[led_buffer_index] = sample->leds[led].brightness;
		refinement->decode_gray.led_facing_dots[led_buffer_index] = sample->leds[led].facing_dot;
	}

	// We have enough info in theory
	// @todo this should be an actual "num samples" check
	if (refinement->leds_seen.cur_sequence_id > T_LED_SYNC_GRAY_CODE_SIZE_BITS) {
		uint16_t decoded_values[T_LED_SYNC_SELECTED_LEDS] = {0};

		for (int led = 0; led < T_LED_SYNC_SELECTED_LEDS; led++) {
			float min_observed_brightness[XRT_TRACKING_MAX_CAMS];
			float max_observed_brightness[XRT_TRACKING_MAX_CAMS] = {0};
			for (uint32_t i = 0; i < ARRAY_SIZE(min_observed_brightness); i++) {
				min_observed_brightness[i] = 1.0f;
			}

			float brightnesses[T_LED_SYNC_GRAY_CODE_SIZE_BITS] = {0};
			int best_camera_indices[T_LED_SYNC_GRAY_CODE_SIZE_BITS];
			for (size_t i = 0; i < ARRAY_SIZE(best_camera_indices); i++) {
				best_camera_indices[i] = -1;
			}

			// Collect the brightnesses and the thresholds
			for (int age = 0; age < T_LED_SYNC_GRAY_CODE_SIZE_BITS; age++) {
				float best_facing_dot = 1.0f;
				float best_brightness = 0.0f;

				for (int camera_index = 0; camera_index < XRT_TRACKING_MAX_CAMS; camera_index++) {
					// Pick which camera's data to use
					size_t led_buffer_index = T_LED_SYNC_GRAY_CODE_BUFFER_INDEX(
					    camera_index, (refinement->leds_seen.cur_sequence_id - age), led);

					float brightness = refinement->decode_gray.led_brightnesses[led_buffer_index];
					float facing_dot = refinement->decode_gray.led_facing_dots[led_buffer_index];

					if (brightness > 0.0f && facing_dot < 1) {
						// Update the min/max observed brightnesses, but only for valid
						// observations.
						min_observed_brightness[camera_index] =
						    MIN(min_observed_brightness[camera_index], brightness);
						max_observed_brightness[camera_index] =
						    MAX(max_observed_brightness[camera_index], brightness);
					}

					if (brightness > 0 && facing_dot < best_facing_dot) {
						best_brightness = brightness;
						best_facing_dot = facing_dot;
						best_camera_indices[age] = camera_index;
					}
				}

				brightnesses[age] = best_brightness;
			}

			// Decode the brightnesses
			for (int age = 0; age < T_LED_SYNC_GRAY_CODE_SIZE_BITS; age++) {
				if (age != 0) {
					decoded_values[led] >>= 1;
				}

				float threshold = 0.5f;
				if (best_camera_indices[age] != -1) {
					threshold = (min_observed_brightness[best_camera_indices[age]] +
					             max_observed_brightness[best_camera_indices[age]]) /
					            2.0f;
				}

				bool bit = brightnesses[age] > threshold;

				LOG_GRAY_SPEW(refinement, "brightness[%d][%d]: %f", led, age, brightnesses[age]);

				decoded_values[led] |= (bit ? 1 : 0) << (T_LED_SYNC_GRAY_CODE_SIZE_BITS - 1);
			}
		}

		if (decoded_values[0] == decoded_values[1]) {
			uint16_t decoded_value = decoded_values[0];

			// No bits outside the decoded value
			assert((decoded_value & (~decoding_mask)) == 0);

			uint16_t decoded_header = decoded_value & header_mask;
			uint16_t decoded_gray_code = decoded_value & gray_code_mask;

			if (decoded_header == expected_header) {
				handle_decoded_gray_code(refinement, decoded_gray_code);
			} else {
				LOG_GRAY_SPEW(refinement, "Decoding gray code failed! Got header %x and data %x",
				              decoded_header, decoded_gray_code);
			}
		} else {
			LOG_GRAY_SPEW(refinement, "Decoding gray code failed, the two LEDs didn't match, %d\t%d",
			              decoded_values[0], decoded_values[1]);
		}
	}
}

static bool
bg_marker_ok(const bool *pattern)
{
	return pattern[0] == true && pattern[1] == false && pattern[2] == false && pattern[3] == true &&
	       pattern[17] == true && pattern[18] == false && pattern[19] == true;
}

static void
handle_optimize_brightness_curve_sample(struct t_led_sync_refinement *refinement,
                                        const struct t_constellation_tracker_sample *sample)
{
	// Update the ring buffer with the new values
	for (int i = 0; i < T_LED_SYNC_SELECTED_LEDS; i++) {
		uint8_t led = refinement->sample_for_driver.pssense.leds[i];
		if (led == 0xFF) {
			continue;
		}

		size_t led_buffer_index = T_LED_SYNC_BRIGHTNESS_CURVE_BUFFER_INDEX(
		    sample->camera_index, refinement->leds_seen.cur_sequence_id, i);

		refinement->optimize_brightness_curve.led_brightnesses[led_buffer_index] = sample->leds[led].brightness;
		refinement->optimize_brightness_curve.led_facing_dots[led_buffer_index] = sample->leds[led].facing_dot;
	}
	refinement->optimize_brightness_curve.num_samples += 1;

	// Wait for enough samples
	if (refinement->optimize_brightness_curve.num_samples < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES) {
		return;
	}

	float brightnesses[T_LED_SYNC_SELECTED_LEDS][T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES];

	int best_camera_idx = -1;

	// Select the camera index that sees the LED the best across the total range
	for (int age = 0; age < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; age++) {
		float best_facing_dot = 1.0f;

		for (int camera_index = 0; camera_index < XRT_TRACKING_MAX_CAMS; camera_index++) {
			// Pick which camera's data to use
			size_t led_buffer_index = T_LED_SYNC_BRIGHTNESS_CURVE_BUFFER_INDEX(
			    camera_index, (refinement->leds_seen.cur_sequence_id - age), 0);

			float brightness = refinement->optimize_brightness_curve.led_brightnesses[led_buffer_index];
			float facing_dot = refinement->optimize_brightness_curve.led_facing_dots[led_buffer_index];

			if (brightness > 0 && facing_dot < best_facing_dot) {
				best_camera_idx = camera_index;
				best_facing_dot = facing_dot;
			}
		}
	}

	// no data for this camera :(
	if (best_camera_idx == -1) {
		return;
	}

	float min_observed_brightness[T_LED_SYNC_SELECTED_LEDS];
	for (size_t i = 0; i < ARRAY_SIZE(min_observed_brightness); i++) {
		min_observed_brightness[i] = 1.0f;
	}
	float max_observed_brightness[T_LED_SYNC_SELECTED_LEDS] = {0};

	// Read out the brightnesses for the LEDs
	for (int led = 0; led < T_LED_SYNC_SELECTED_LEDS; led++) {
		for (int age = 0; age < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; age++) {
			size_t led_buffer_index = T_LED_SYNC_BRIGHTNESS_CURVE_BUFFER_INDEX(
			    best_camera_idx, (refinement->leds_seen.cur_sequence_id - age), led);

			float brightness = refinement->optimize_brightness_curve.led_brightnesses[led_buffer_index];

			// Brightness is 0 at "not observed", we don't want that spoiling the actual real threshold
			if (brightness > 0) {
				// Update the min/max
				min_observed_brightness[led] = MIN(min_observed_brightness[led], brightness);
				max_observed_brightness[led] = MAX(max_observed_brightness[led], brightness);
			}

			brightnesses[led][T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES - 1 - age] = brightness;
		}
	}

	float thresholds[T_LED_SYNC_SELECTED_LEDS];
	for (int i = 0; i < T_LED_SYNC_SELECTED_LEDS; i++) {
		thresholds[i] = (min_observed_brightness[i] + max_observed_brightness[i]) / 2.0f;

		if (max_observed_brightness[i] - min_observed_brightness[i] < 0.1f) {
			// @todo toss it in this case
		}
	}

#if 1
	for (int led_i = 0; led_i < T_LED_SYNC_SELECTED_LEDS; led_i++) {
		LOG_GRAY_SPEW(refinement, "min/max: %f/%f", min_observed_brightness[led_i],
		              max_observed_brightness[led_i]);

		for (int sample_i = 0; sample_i < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; sample_i++) {
			LOG_GRAY_SPEW(refinement, "[%d,%d]\t%f", led_i, sample_i, brightnesses[led_i][sample_i]);
		}
	}
#endif

	for (int rotation = 0; rotation < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; rotation++) {
		bool patterns[T_LED_SYNC_SELECTED_LEDS][T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES];
		float rotated_raw[T_LED_SYNC_SELECTED_LEDS][T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES];

		for (int raw_i = 0; raw_i < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; raw_i++) {
			int i = (raw_i + rotation) % T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES;

			for (int led = 0; led < T_LED_SYNC_SELECTED_LEDS; led++) {
				patterns[led][raw_i] = brightnesses[led][i] > thresholds[led];
				rotated_raw[led][raw_i] = brightnesses[led][i];
			}
		}

		bool ok_0 = bg_marker_ok(patterns[0]);
		bool ok_1 = bg_marker_ok(patterns[1]);

		if (ok_0) {
			LOG_GRAY_SPEW(refinement, "[0] Found valid BG pattern at rotation %d!",
			              (refinement->leds_seen.cur_sequence_id + rotation) % 20);
		}
		if (ok_1) {
			LOG_GRAY_SPEW(refinement, "[1] Found valid BG pattern at rotation %d!",
			              (refinement->leds_seen.cur_sequence_id + rotation) % 20);
		}

		if (ok_0 && ok_1) {
			LOG_GRAY_SPEW(refinement, "LEDs agree on BG pattern, resetting loop");
			refinement->optimize_brightness_curve.num_samples = 0;

			for (int led_i = 0; led_i < T_LED_SYNC_SELECTED_LEDS; led_i++) {
				LOG_GRAY_SPEW(refinement, "min/max: %f/%f", min_observed_brightness[led_i],
				              max_observed_brightness[led_i]);

				for (int sample_i = 0; sample_i < T_LED_SYNC_BRIGHTNESS_CURVE_NUM_SAMPLES; sample_i++) {
					LOG_GRAY_SPEW(refinement, "[%d,%d]\t%f", led_i, sample_i,
					              brightnesses[led_i][sample_i]);
				}
			}

			const int estimator_sample_start = 4;
			float *estimator_values = rotated_raw[0] + estimator_sample_start;
			const int estimator_sample_count = 13;

			float min = FLT_MAX, max = FLT_MIN;
			for (int k = 0; k < estimator_sample_count; k++) {
				float brightness = estimator_values[k];

				min = MIN(min, brightness);
				max = MAX(max, brightness);
			}
			const float mid = (max + min) * 0.5f;

			float num = 0.0f, den = 0.0f;
			for (int k = 0; k < estimator_sample_count; k++) {
				float w = fmaxf(mid - estimator_values[k], 0.0f);
				num += (float)(k + estimator_sample_start) * w;
				den += w;
			}

			LOG_GRAY_SPEW(refinement, "mid:%f", mid);
			LOG_GRAY_SPEW(refinement, "num:%f\tden:%f", num, den);

			if (num == 0.0f || den == 0.0f) {
				// Bad sample, ignore.
				return;
			}

			float centroid = num / den;
			float error = 10.0f - centroid;
			time_duration_ns error_ns = (time_duration_ns)(U_TIME_1US_IN_NS * 50 * error);
			timepoint_ns now_ns = sample->timestamp_ns;

			if (!refinement->bg_sync.has_epoch) {
				refinement->bg_sync.has_epoch = true;
				refinement->bg_sync.epoch_ns = now_ns;
				refinement->bg_sync.last_correction_ns = now_ns;
				refinement->bg_sync.window_start_ns = now_ns;
				refinement->bg_sync.window_start_integral_ns = refinement->bg_sync.servo_integral_ns;
			}

			// Fit the residual deviation (servo output + current error) against time. Its slope is the
			// drift the current skew doesn't yet cover, so integrating it converges skew to the truth.
			double t_rel = (double)(now_ns - refinement->bg_sync.window_start_ns);
			double deviation = (double)((refinement->bg_sync.servo_integral_ns -
			                             refinement->bg_sync.window_start_integral_ns) +
			                            error_ns);
			refinement->bg_sync.num_samples++;
			refinement->bg_sync.sum_t += t_rel;
			refinement->bg_sync.sum_tt += t_rel * t_rel;
			refinement->bg_sync.sum_d += deviation;
			refinement->bg_sync.sum_td += t_rel * deviation;
			if (refinement->bg_sync.num_samples >= BG_SKEW_WINDOW_SAMPLES) {
				double n = refinement->bg_sync.num_samples;
				double denom = n * refinement->bg_sync.sum_tt -
				               refinement->bg_sync.sum_t * refinement->bg_sync.sum_t;
				if (denom != 0.0) {
					double slope = (n * refinement->bg_sync.sum_td -
					                refinement->bg_sync.sum_t * refinement->bg_sync.sum_d) /
					               denom;
					refinement->bg_sync.skew =
					    MAX(-BG_MAX_SKEW, MIN(BG_MAX_SKEW, refinement->bg_sync.skew + slope));
				}
				refinement->bg_sync.num_samples = 0;
				refinement->bg_sync.sum_t = refinement->bg_sync.sum_tt = 0.0;
				refinement->bg_sync.sum_d = refinement->bg_sync.sum_td = 0.0;
				refinement->bg_sync.window_start_ns = now_ns;
				refinement->bg_sync.window_start_integral_ns = refinement->bg_sync.servo_integral_ns;
			}

			LOG_GRAY_SPEW(refinement, "Got centroid %f, timing error %" PRIi64 "us, skew %f ppm", centroid,
			              error_ns / 1000, refinement->bg_sync.skew * 1e6);

			// Offset servo: only correct outside the +-1 sub-grid deadband, the skew handles the rest.
			time_duration_ns servo_correction_ns = 0;
			if (error > 1.0f || error < -1.0f) {
				servo_correction_ns = (time_duration_ns)(error_ns * BG_OFFSET_GAIN);
			}

			// Fold the skew-predicted drift since the last applied correction into this base-time
			// delta, so the controller tracks the device clock between measurements (BG is
			// delta-only, this is the only way to apply the skew). Only the servo portion is
			// counted in servo_integral_ns: the least-squares fit above must see the residual
			// (uncovered) skew as its slope, otherwise skew is double-counted and diverges.
			time_duration_ns skew_drift_ns =
			    (time_duration_ns)(refinement->bg_sync.skew *
			                       (double)(now_ns - refinement->bg_sync.last_correction_ns));

			time_duration_ns correction_ns = servo_correction_ns + skew_drift_ns;
			if (correction_ns != 0) {
				timepoint_ns prev_epoch_ns = refinement->bg_sync.epoch_ns;
				timepoint_ns prev_last_correction_ns = refinement->bg_sync.last_correction_ns;

				// The sample built by advance_clock_offset_pssense_locked publishes the current
				// host->device offset together with clock_skew/clock_epoch_ns. Advance the epoch
				// first so the sample's offset and epoch describe the same instant; otherwise the
				// offset includes this skew fold while the epoch still points to the old instant,
				// which makes IMU timestamp conversion jump by the folded amount.
				refinement->bg_sync.last_correction_ns = now_ns;
				refinement->bg_sync.epoch_ns = now_ns;

				if (advance_clock_offset_pssense_locked(refinement, correction_ns)) {
					refinement->bg_sync.servo_integral_ns += servo_correction_ns;
				} else {
					refinement->bg_sync.epoch_ns = prev_epoch_ns;
					refinement->bg_sync.last_correction_ns = prev_last_correction_ns;
				}
			}
		}
	}
}

static void
handle_optimize_brightness_curve(struct t_led_sync_refinement *refinement)
{
	(void)refinement; // nothing yet
}
