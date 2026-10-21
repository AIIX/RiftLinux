// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal routines for LED sync code
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "constellation/t_led_sync_refinement.h"
#include "os/os_time.h"
#include "util/u_time.h"

/*
 *
 * Defines
 *
 */

#define LOG_TRACE(refinement, ...) U_LOG_IFL_T(refinement->log_level, __VA_ARGS__)
#define LOG_DEBUG(refinement, ...) U_LOG_IFL_D(refinement->log_level, __VA_ARGS__)
#define LOG_INFO(refinement, ...) U_LOG_IFL_I(refinement->log_level, __VA_ARGS__)
#define LOG_WARN(refinement, ...) U_LOG_IFL_W(refinement->log_level, __VA_ARGS__)
#define LOG_ERROR(refinement, ...) U_LOG_IFL_E(refinement->log_level, __VA_ARGS__)

/*
 *
 * Helper functions
 *
 */

static bool
phase_uses_led_selection(enum t_led_sync_phase phase)
{
	switch (phase) {
	case T_LED_SYNC_PHASE_DECODE_GRAY:
	case T_LED_SYNC_PHASE_OPTIMIZE_BRIGHTNESS_CURVE: return true;
	default: return false;
	}
}

static time_duration_ns
max_right_edge(struct t_led_sync_refinement *refinement)
{
	if ((refinement->options.flags & T_LED_SYNC_REFINEMENT_FLAGS_HAS_LATENCY_CAP) != 0) {
		return refinement->options.latency_cap_ns;
	} else {
		return refinement->exposure_interval_ns - refinement->current_blink_duration_ns;
	}
}

/*
 *
 * State update functions
 *
 */

static void
set_offset_duration_pssense(struct t_led_sync_refinement *refinement,
                            time_duration_ns clock_offset_ns,
                            time_duration_ns fudge_offset_ns,
                            time_duration_ns blink_duration_ns,
                            enum t_led_sync_sample_pssense_mode pssense_mode,
                            const struct t_led_sync_sample_pssense_data pssense_data)
{
	refinement->current_clock_offset_ns = clock_offset_ns;
	refinement->current_blink_fudge_ns = fudge_offset_ns;
	refinement->current_blink_duration_ns = blink_duration_ns;

	refinement->has_sample_for_driver = true;
	refinement->sample_applied = false;
	refinement->sample_for_driver = XRT_C11_COMPOUND(struct t_led_sync_sample){
	    .timestamp.device_host_latency_ns = clock_offset_ns,
	    .timestamp_mode = T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_DEVICE_HOST_LATENCY,
	    .fudge_offset_ns = fudge_offset_ns,
	    .blink_duration_ns = blink_duration_ns,
	    .pssense = pssense_data,
	    .pssense_mode = pssense_mode,
	};

	// If we're doing optical driven offset and have a saved offset, use that mode.
	if ((refinement->options.flags & T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET) != 0 &&
	    refinement->optical_driven_offset.has_saved_offset) {
		refinement->sample_for_driver.timestamp_mode =
		    T_LED_SYNC_SAMPLE_TIMESTAMP_MODE_HOST_DEVICE_CLOCK_OFFSET;
		// In optical-driven mode the published offset is the host->device clock offset at the
		// published epoch, while the skew term extrapolates away from that epoch. These two values
		// must describe the same instant or IMU timestamp conversions will jump.
		refinement->sample_for_driver.timestamp.host_device_clock_offset_ns =
		    refinement->optical_driven_offset.saved_offset + clock_offset_ns;
		refinement->sample_for_driver.clock_skew = refinement->bg_sync.skew;
		refinement->sample_for_driver.clock_epoch_ns = refinement->bg_sync.epoch_ns;
	}

	LOG_TRACE(refinement,
	          "Setting offset to %" PRId64 "ns with fudge %" PRId64 "ns and blink duration to %" PRId64 "ns",
	          clock_offset_ns, fudge_offset_ns, blink_duration_ns);
}

static void
set_offset_duration(struct t_led_sync_refinement *refinement,
                    time_duration_ns clock_offset_ns,
                    time_duration_ns fudge_offset_ns,
                    time_duration_ns blink_duration_ns)
{
	// Set some safe dummy data
	struct t_led_sync_sample_pssense_data pssense_data = {
	    .base_time_offset_ns = 0,
	    .leds = {0xFF, 0xFF, 0xFF, 0xFF},
	};

	set_offset_duration_pssense(refinement,                             //
	                            clock_offset_ns,                        //
	                            fudge_offset_ns,                        //
	                            blink_duration_ns,                      //
	                            T_LED_SYNC_SAMPLE_PSSENSE_MODE_PRESCAN, //
	                            pssense_data);                          //
}

static void
set_centered_duration_from_binary_search(struct t_led_sync_refinement *refinement, time_duration_ns blink_duration_ns)
{
	// Find the clock offset, the offset needing to be applied to get blink start to line up with exposure
	// start, this is the clock offset from the controller local clock (as we view it) -> host local clock.
	time_duration_ns new_clock_offset_ns = refinement->found_left_edge_ns;

	// Set the offset so that the center of the blink is at the center of the exposure, since latency and
	// fudge offset are added together.
	time_duration_ns new_fudge_offset_ns = (refinement->exposure_time_ns / 2) - (blink_duration_ns / 2);

	set_offset_duration(refinement, new_clock_offset_ns, new_fudge_offset_ns, blink_duration_ns);
}

static void
set_pssense_mode(struct t_led_sync_refinement *refinement, enum t_led_sync_sample_pssense_mode mode, uint8_t leds[4])
{
	const time_duration_ns blink_duration_ns = refinement->current_blink_duration_ns;

	struct t_led_sync_sample_pssense_data data = {
	    .base_time_offset_ns = 0,
	    .leds =
	        {
	            0xFF,
	            0xFF,
	            0xFF,
	            0xFF,
	        },
	};
	if (leds != NULL) {
		memcpy(data.leds, leds, sizeof(data.leds));
	}

	set_offset_duration_pssense(refinement,                          //
	                            refinement->current_clock_offset_ns, //
	                            refinement->current_blink_fudge_ns,  //
	                            blink_duration_ns,                   //
	                            mode,                                //
	                            data);                               //
}

XRT_CHECK_RESULT
static bool
advance_clock_offset_pssense_locked(struct t_led_sync_refinement *refinement, time_duration_ns offset_ns)
{
	// Assert that this is a pssense refinement.
	assert(refinement->options.flags & T_LED_SYNC_REFINEMENT_FLAGS_PS_SENSE);
	// Assert that we're in the gray code (BROAD) phase.
	assert(refinement->phase == T_LED_SYNC_PHASE_DECODE_GRAY ||
	       refinement->phase == T_LED_SYNC_PHASE_OPTIMIZE_BRIGHTNESS_CURVE);

	if (refinement->has_sample_for_driver) {
		return false;
	}

	// Copy the latest data
	struct t_led_sync_sample_pssense_data data = refinement->sample_for_driver.pssense;
	// Push a new time offset
	data.base_time_offset_ns = (int32_t)offset_ns;

	set_offset_duration_pssense(refinement,                                      //
	                            refinement->current_clock_offset_ns + offset_ns, //
	                            refinement->current_blink_fudge_ns,              //
	                            refinement->current_blink_duration_ns,           //
	                            refinement->sample_for_driver.pssense_mode,      //
	                            data);                                           //

	return true;
}

static void
set_phase_locked(struct t_led_sync_refinement *refinement, enum t_led_sync_phase new_phase)
{
	enum t_led_sync_phase old_phase = refinement->phase;
	refinement->phase = new_phase;

	LOG_INFO(refinement, "Transitioning from search phase %d -> %d", old_phase, new_phase);

	switch (old_phase) {
	case T_LED_SYNC_PHASE_FIND_INITIAL_OFFSET: {
		// If we're doing clock driven offset, now's when we save the offset, since we're about to start our
		// binary search.
		if (refinement->options.flags & T_LED_SYNC_REFINEMENT_FLAGS_OPTICAL_DRIVEN_OFFSET) {
			refinement->optical_driven_offset.has_saved_offset = true;
			refinement->optical_driven_offset.saved_offset =
			    refinement->optical_driven_offset.driver_host_device_clock_offset_ns;
		}

		break;
	}
	case T_LED_SYNC_PHASE_FIND_LEFT_EDGE: {
		// We should have both edges by now
		assert(refinement->found_right_edge_ns > -1);
		assert(refinement->found_left_edge_ns > -1);

		set_centered_duration_from_binary_search(refinement, refinement->current_blink_duration_ns);
		break;
	}
	default: break;
	}

	if (phase_uses_led_selection(new_phase)) {
		refinement->leds_seen.last_selection_ns = 0; // reset
		refinement->leds_seen.num_samples = 0;       // reset
	}

	switch (new_phase) {
	case T_LED_SYNC_PHASE_FIND_INITIAL_OFFSET: {
		refinement->found_left_edge_ns = -1;
		refinement->found_right_edge_ns = -1;

		refinement->current_blink_duration_ns = refinement->options.initial_blink_duration_ns;

		refinement->optical_driven_offset.has_saved_offset = false;

		// Re-estimate the clock sync from scratch each BG session. These values were relative to the old saved
		// offset, which is now cleared.
		memset(&refinement->bg_sync, 0, sizeof(refinement->bg_sync));

		set_offset_duration(refinement, 0, 0, refinement->current_blink_duration_ns);

		break;
	}
	case T_LED_SYNC_PHASE_FIND_RIGHT_EDGE: {
		// Left bound is the current offset
		refinement->binary_search_state.left_bound_ns = refinement->current_blink_fudge_ns;
		// Right bound is the start of the next exposure minus the blink duration, we don't want to blink into
		// the next frame, rather start at zero, since latency will always be >0.
		refinement->binary_search_state.right_bound_ns = max_right_edge(refinement);

		LOG_DEBUG(refinement,
		          "Starting binary search for right edge, left bound at: %" PRId64
		          "ns, right bound at: %" PRId64 "ns",
		          refinement->binary_search_state.left_bound_ns,
		          refinement->binary_search_state.right_bound_ns);

		time_duration_ns new_offset_ns =
		    (refinement->binary_search_state.right_bound_ns + refinement->binary_search_state.left_bound_ns) /
		    2;
		set_offset_duration(refinement, 0, new_offset_ns, refinement->current_blink_duration_ns);

		break;
	}
	case T_LED_SYNC_PHASE_FIND_LEFT_EDGE: {
		// We should always have a right edge by now
		assert(refinement->found_right_edge_ns > -1);

		// Left bound is when the exposure *should* be given zero latency
		refinement->binary_search_state.left_bound_ns = 0;
		// Right bound is the found right edge
		refinement->binary_search_state.right_bound_ns = refinement->found_right_edge_ns;

		LOG_DEBUG(refinement, "Starting binary search for left edge, right edge at: %" PRId64 "ns",
		          refinement->found_right_edge_ns);

		time_duration_ns new_offset_ns =
		    (refinement->binary_search_state.right_bound_ns + refinement->binary_search_state.left_bound_ns) /
		    2;
		set_offset_duration(refinement, 0, new_offset_ns, refinement->current_blink_duration_ns);

		break;
	}
	case T_LED_SYNC_PHASE_DECODE_GRAY: {
		// PSSENSE requires a period id of 42 (2.1ms) for BROAD
		time_duration_ns new_blink_duration_ns = U_TIME_1US_IN_NS * 2100;

		time_duration_ns new_fudge_offset_ns = (refinement->exposure_time_ns / 2) - (new_blink_duration_ns / 2);
		// Pick the clock offset that will when added to the new blink fudge will produce the same value,
		// this is our rough clock estimate that we are going to refine.
		time_duration_ns new_clock_offset_ns = refinement->current_blink_fudge_ns - new_fudge_offset_ns;

		// Set the new offset/fudge so that our fudge is what we want, but we're about to refine the new clock
		// offset.
		set_offset_duration(refinement, new_clock_offset_ns, new_fudge_offset_ns, new_blink_duration_ns);

		refinement->decode_gray.applied_gray_start = false;
		refinement->decode_gray.matching_decodes = 0;
		refinement->decode_gray.decoded_gray_code = 0;

		break;
	}
	case T_LED_SYNC_PHASE_OPTIMIZE_BRIGHTNESS_CURVE: {
		// PSSENSE requires a period id of 30 (1.5ms) for BG
		time_duration_ns new_blink_duration_ns = U_TIME_1US_IN_NS * 1500;

		time_duration_ns new_fudge_offset_ns = (refinement->exposure_time_ns / 2) - (new_blink_duration_ns / 2);
		time_duration_ns new_clock_offset_ns = refinement->current_clock_offset_ns;

		// update us to the right offset when moving from broad to bg, keep blink centered with the new duration
		refinement->sample_for_driver.pssense.base_time_offset_ns =
		    new_fudge_offset_ns - refinement->current_blink_fudge_ns;

		set_offset_duration_pssense(refinement,                             //
		                            new_clock_offset_ns,                    //
		                            new_fudge_offset_ns,                    //
		                            new_blink_duration_ns,                  //
		                            T_LED_SYNC_SAMPLE_PSSENSE_MODE_BG,      //
		                            refinement->sample_for_driver.pssense); //

		refinement->optimize_brightness_curve.num_samples = 0;

		break;
	}
	case T_LED_SYNC_PHASE_REFINE_BLINK_DURATION: {
		refinement->blink_time_refinement_state.backing_off = false;
		refinement->blink_time_refinement_state.last_good_blink_duration_ns =
		    refinement->current_blink_duration_ns;

		break;
	}
	case T_LED_SYNC_PHASE_MAINTAIN_OFFSET: {
		LOG_DEBUG(refinement, "Maintaining offset at: %" PRId64 "ns", refinement->current_clock_offset_ns);

		break;
	}
	default: break;
	}
}
