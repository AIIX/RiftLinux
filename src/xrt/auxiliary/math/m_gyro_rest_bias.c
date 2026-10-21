// Copyright 2026, Aditya Mehra
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Measures a gyroscope's bias whenever its device lies still.
 * @author Aditya Mehra <aix.m@outlook.com>
 * @ingroup aux_math
 */

#include "m_gyro_rest_bias.h"

#include <math.h>
#include <string.h>

/*
 * Measured on two CV1 Touch at about 470 Hz: over a second lying still, the gyro spreads by at most
 * 0.0054 rad/s on any axis and the accelerometer's magnitude by at most 0.036 m/s². Held as still as a hand can, at
 * least 0.013 and 0.055. The limits sit in between.
 */
#define WINDOW_NS (1 * U_TIME_1S_IN_NS)
#define MAX_GAP_NS (50 * U_TIME_1MS_IN_NS)
#define MIN_SAMPLES 100
#define MAX_GYRO_STDDEV 0.007
#define MAX_ACCEL_MAG_STDDEV 0.045
//! Anything bigger is no bias but the device turning slowly, e.g. on something that rocks.
#define MAX_BIAS 0.1
//! How much each still second moves the bias, after the first sets it.
#define AVERAGE_WEIGHT 0.25

static void
restart(struct m_gyro_rest_bias *rb, timepoint_ns timestamp_ns)
{
	rb->window_start_ns = timestamp_ns;
	rb->count = 0;
	memset(rb->gyro_sum, 0, sizeof(rb->gyro_sum));
	memset(rb->gyro_sq_sum, 0, sizeof(rb->gyro_sq_sum));
	rb->accel_mag_sum = 0.0;
	rb->accel_mag_sq_sum = 0.0;
}

static double
stddev(double sum, double sq_sum, uint32_t n)
{
	const double mean = sum / n;
	const double var = sq_sum / n - mean * mean;
	return var > 0.0 ? sqrt(var) : 0.0;
}

void
m_gyro_rest_bias_init(struct m_gyro_rest_bias *rb)
{
	memset(rb, 0, sizeof(*rb));
}

void
m_gyro_rest_bias_set(struct m_gyro_rest_bias *rb, const struct xrt_vec3 *bias)
{
	rb->bias = *bias;
	rb->have_bias = true;
}

bool
m_gyro_rest_bias_push(struct m_gyro_rest_bias *rb,
                      timepoint_ns timestamp_ns,
                      const struct xrt_vec3 *gyro_rad_secs,
                      const struct xrt_vec3 *accel_m_s2)
{
	if (rb->count == 0 || timestamp_ns < rb->last_ns || timestamp_ns - rb->last_ns > MAX_GAP_NS) {
		restart(rb, timestamp_ns);
	}
	rb->last_ns = timestamp_ns;

	const double g[3] = {gyro_rad_secs->x, gyro_rad_secs->y, gyro_rad_secs->z};
	for (int i = 0; i < 3; i++) {
		rb->gyro_sum[i] += g[i];
		rb->gyro_sq_sum[i] += g[i] * g[i];
	}
	const double a = sqrt((double)accel_m_s2->x * accel_m_s2->x + (double)accel_m_s2->y * accel_m_s2->y +
	                      (double)accel_m_s2->z * accel_m_s2->z);
	rb->accel_mag_sum += a;
	rb->accel_mag_sq_sum += a * a;
	rb->count++;

	if (timestamp_ns - rb->window_start_ns < WINDOW_NS) {
		return false;
	}

	const uint32_t n = rb->count;
	bool still = n >= MIN_SAMPLES && stddev(rb->accel_mag_sum, rb->accel_mag_sq_sum, n) <= MAX_ACCEL_MAG_STDDEV;
	double mean[3];
	for (int i = 0; i < 3; i++) {
		mean[i] = rb->gyro_sum[i] / n;
		still = still && stddev(rb->gyro_sum[i], rb->gyro_sq_sum[i], n) <= MAX_GYRO_STDDEV &&
		        fabs(mean[i]) <= MAX_BIAS;
	}

	// The next sample starts the next window, so a still second right after this one is measured on its own.
	restart(rb, timestamp_ns);

	if (!still) {
		return false;
	}

	// A second's mean is good to about 0.002 rad/s, the drift of the bias far slower: average them.
	const double w = rb->have_bias ? AVERAGE_WEIGHT : 1.0;
	rb->bias.x = (float)(rb->bias.x + w * (mean[0] - rb->bias.x));
	rb->bias.y = (float)(rb->bias.y + w * (mean[1] - rb->bias.y));
	rb->bias.z = (float)(rb->bias.z + w * (mean[2] - rb->bias.z));
	rb->have_bias = true;
	rb->still_windows++;
	return true;
}

void
m_gyro_rest_bias_apply(const struct m_gyro_rest_bias *rb, const struct xrt_vec3 *gyro, struct xrt_vec3 *out_gyro)
{
	if (!rb->have_bias) {
		*out_gyro = *gyro;
		return;
	}
	out_gyro->x = gyro->x - rb->bias.x;
	out_gyro->y = gyro->y - rb->bias.y;
	out_gyro->z = gyro->z - rb->bias.z;
}
