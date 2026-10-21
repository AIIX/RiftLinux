// Copyright 2026, Aditya Mehra
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Measures a gyroscope's bias whenever its device lies still.
 * @author Aditya Mehra <aix.m@outlook.com>
 * @ingroup aux_math
 */

#pragma once

#include "xrt/xrt_defines.h"
#include "util/u_time.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * A gyroscope's bias drifts with temperature and age, and what is left of it after the factory calibration turns a
 * device's orientation by that much every second it is not seen. A CV1 Touch can be left with 0.02 rad/s, 1.2°/s,
 * more than its fusion learns in many seconds.
 *
 * Lying still on something is the one time the bias can be read straight off the gyro: its mean over a second is
 * the bias to a few 0.0001 rad/s. Held "still" in a hand is not still enough, the hand's tremor shows several times
 * over in the spread of both the gyro and the accelerometer, which is what tells the two apart.
 *
 * Feed it every sample, in the frame the bias is to be taken out in, and subtract @ref bias once @ref have_bias.
 */
struct m_gyro_rest_bias
{
	//! The bias, once a still second has been seen or one was set.
	struct xrt_vec3 bias;
	bool have_bias;

	//! Seconds of stillness measured so far, for logging.
	uint32_t still_windows;

	timepoint_ns window_start_ns;
	timepoint_ns last_ns;
	uint32_t count;
	double gyro_sum[3];
	double gyro_sq_sum[3];
	double accel_mag_sum;
	double accel_mag_sq_sum;
};

//! Starts with no bias.
void
m_gyro_rest_bias_init(struct m_gyro_rest_bias *rb);

//! Starts from a bias known from before, e.g. saved last session; still seconds then move it.
void
m_gyro_rest_bias_set(struct m_gyro_rest_bias *rb, const struct xrt_vec3 *bias);

/*!
 * Adds a sample. Returns true when it closed a still second and so changed @ref bias.
 */
bool
m_gyro_rest_bias_push(struct m_gyro_rest_bias *rb,
                      timepoint_ns timestamp_ns,
                      const struct xrt_vec3 *gyro_rad_secs,
                      const struct xrt_vec3 *accel_m_s2);

//! The gyro with the bias taken out, or as it is while there is none.
void
m_gyro_rest_bias_apply(const struct m_gyro_rest_bias *rb, const struct xrt_vec3 *gyro, struct xrt_vec3 *out_gyro);

#ifdef __cplusplus
}
#endif
