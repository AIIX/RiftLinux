// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Screenshots of what the headset shows, see u_screenshot.h.
 * @ingroup aux_util
 */

#include "util/u_screenshot.h"
#include "util/u_logging.h"

#include "xrt/xrt_config_os.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef XRT_OS_LINUX

#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#pragma GCC diagnostic pop

//! Only one screenshot at a time.
static pthread_mutex_t g_take = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_cond;
static pthread_once_t g_once = PTHREAD_ONCE_INIT;
static atomic_bool g_wanted;
static struct u_screenshot_provider g_provider;
static bool g_delivered;
static struct u_screenshot_view g_views[2];
static uint32_t g_count;
static float g_shoulder;

static void
init_cond(void)
{
	pthread_condattr_t attr;
	pthread_condattr_init(&attr);
	pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
	pthread_cond_init(&g_cond, &attr);
	pthread_condattr_destroy(&attr);
}

static float
half_to_float(uint16_t h)
{
	const uint32_t sign = (uint32_t)(h >> 15) << 31;
	uint32_t exp = (h >> 10) & 0x1f;
	uint32_t mant = h & 0x3ff;
	uint32_t bits;
	if (exp == 0) {
		if (mant == 0) {
			bits = sign;
		} else {
			// Subnormal: normalise.
			exp = 127 - 15 + 1;
			while ((mant & 0x400) == 0) {
				mant <<= 1;
				exp--;
			}
			mant &= 0x3ff;
			bits = sign | (exp << 23) | (mant << 13);
		}
	} else if (exp == 31) {
		bits = sign | 0x7f800000u | (mant << 13);
	} else {
		bits = sign | ((exp - 15 + 127) << 23) | (mant << 13);
	}
	float f;
	memcpy(&f, &bits, sizeof(f));
	return f;
}

static float
srgb_to_linear(float c)
{
	return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

static float
linear_to_srgb(float c)
{
	return c <= 0.0031308f ? c * 12.92f : 1.055f * powf(c, 1.0f / 2.4f) - 0.055f;
}

// distortion.comp's tone_map: values at or below @p s untouched, above eased into [s, 1); off unless 0 < s < 1.
static float
tone_map(float c, float s)
{
	if (s <= 0.0f || s >= 1.0f || c < s) {
		return c;
	}
	return s + (1.0f - s) * (1.0f - expf(-(c - s) / (1.0f - s)));
}

//! A view's pixel @p x, @p y (in range) as linear RGB.
static void
fetch(const struct u_screenshot_view *v, uint32_t x, uint32_t y, float out[3])
{
	const size_t i = (size_t)y * v->width + x;
	if (v->pixels == NULL) {
		out[0] = out[1] = out[2] = 0.0f;
	} else if (v->format == U_SCREENSHOT_RGBA16F) {
		const uint16_t *p = (const uint16_t *)v->pixels + i * 4;
		for (int k = 0; k < 3; k++) {
			out[k] = half_to_float(p[k]);
		}
	} else {
		const uint8_t *p = (const uint8_t *)v->pixels + i * 4;
		for (int k = 0; k < 3; k++) {
			out[k] = srgb_to_linear((float)p[k] / 255.0f);
		}
	}
}

/*!
 * The view @p v seen in the direction with tangents @p tx (right positive) and @p ty (up positive): bilinear, linear
 * light. False outside its field of view.
 */
static bool
sample(const struct u_screenshot_view *v, float tx, float ty, float out[3])
{
	const float l = tanf(v->fov.angle_left), r = tanf(v->fov.angle_right);
	const float u = tanf(v->fov.angle_up), d = tanf(v->fov.angle_down);
	const float fx = (tx - l) / (r - l) * (float)v->width - 0.5f;
	const float fy = (u - ty) / (u - d) * (float)v->height - 0.5f;
	if (fx < -0.5f || fy < -0.5f || fx > (float)v->width - 0.5f || fy > (float)v->height - 0.5f) {
		return false;
	}
	const float cx = fminf(fmaxf(fx, 0.0f), (float)v->width - 1.0f);
	const float cy = fminf(fmaxf(fy, 0.0f), (float)v->height - 1.0f);
	const uint32_t x0 = (uint32_t)cx, y0 = (uint32_t)cy;
	const uint32_t x1 = x0 + 1 < v->width ? x0 + 1 : x0, y1 = y0 + 1 < v->height ? y0 + 1 : y0;
	const float ax = cx - (float)x0, ay = cy - (float)y0;
	float c00[3], c10[3], c01[3], c11[3];
	fetch(v, x0, y0, c00);
	fetch(v, x1, y0, c10);
	fetch(v, x0, y1, c01);
	fetch(v, x1, y1, c11);
	for (int k = 0; k < 3; k++) {
		const float top = c00[k] + (c10[k] - c00[k]) * ax;
		const float bottom = c01[k] + (c11[k] - c01[k]) * ax;
		out[k] = top + (bottom - top) * ay;
	}
	return true;
}

static float
smoothstep(float e0, float e1, float x)
{
	const float t = fminf(fmaxf((x - e0) / (e1 - e0), 0.0f), 1.0f);
	return t * t * (3.0f - 2.0f * t);
}

/*!
 * One picture of the whole field of view: the left eye's view, widened on the right by what only the right eye sees,
 * the two blended over the last few degrees of the left eye's view (they differ a little there: the eyes are apart).
 * Same pixel density as the left eye's view.
 */
static bool
write_png(const char *path, const struct u_screenshot_view *views, uint32_t count, float shoulder)
{
	const struct u_screenshot_view *lv = &views[0];
	const struct u_screenshot_view *rv = count > 1 ? &views[1] : &views[0];
	const float left = tanf(lv->fov.angle_left), right = tanf(rv->fov.angle_right);
	const float up = tanf(fmaxf(lv->fov.angle_up, rv->fov.angle_up));
	const float down = tanf(fminf(lv->fov.angle_down, rv->fov.angle_down));
	// One density both ways, the view's finer one: its images are not as dense across as down, and the picture's
	// pixels are square, so that what is square in the view is square in it.
	const float per = fmaxf((float)lv->width / (tanf(lv->fov.angle_right) - tanf(lv->fov.angle_left)),
	                        (float)lv->height / (tanf(lv->fov.angle_up) - tanf(lv->fov.angle_down)));
	const uint32_t width = (uint32_t)lroundf((right - left) * per);
	const uint32_t height = (uint32_t)lroundf((up - down) * per);
	if (width == 0 || height == 0 || width > 16384 || height > 16384) {
		return false;
	}
	// The blend: from 4 degrees inside the left eye's right edge to that edge.
	const float edge = tanf(lv->fov.angle_right);
	const float blend_from = tanf(lv->fov.angle_right - 4.0f * 3.14159265f / 180.0f);

	uint8_t *rgb = malloc((size_t)width * height * 3);
	if (rgb == NULL) {
		return false;
	}
	for (uint32_t y = 0; y < height; y++) {
		const float ty = up - ((float)y + 0.5f) / per;
		for (uint32_t x = 0; x < width; x++) {
			const float tx = left + ((float)x + 0.5f) / per;
			float a[3] = {0, 0, 0}, b[3] = {0, 0, 0};
			const bool has_a = sample(lv, tx, ty, a);
			const bool has_b = count > 1 && sample(rv, tx, ty, b);
			float w = has_b ? smoothstep(blend_from, edge, tx) : 0.0f;
			if (!has_a) {
				w = has_b ? 1.0f : 0.0f;
			}
			uint8_t *o = rgb + ((size_t)y * width + x) * 3;
			for (int k = 0; k < 3; k++) {
				float c = a[k] + (b[k] - a[k]) * w;
				c = fminf(fmaxf(tone_map(c, shoulder), 0.0f), 1.0f);
				o[k] = (uint8_t)lroundf(linear_to_srgb(c) * 255.0f);
			}
		}
	}

	char part[1100];
	snprintf(part, sizeof(part), "%s.part", path);
	const int ok = stbi_write_png(part, (int)width, (int)height, 3, rgb, (int)width * 3);
	free(rgb);
	if (ok == 0 || rename(part, path) != 0) {
		remove(part);
		return false;
	}
	U_LOG_I("Screenshot: %s (%ux%u)", path, width, height);
	return true;
}

xrt_result_t
u_screenshot_take(const char *png_path, uint64_t timeout_ns)
{
	if (png_path == NULL || png_path[0] != '/') {
		return XRT_ERROR_INVALID_ARGUMENT;
	}
	pthread_once(&g_once, init_cond);
	pthread_mutex_lock(&g_take);

	// The read-back memory made here, on this thread, so the compositor's frame does not wait for it.
	pthread_mutex_lock(&g_lock);
	const struct u_screenshot_provider provider = g_provider;
	pthread_mutex_unlock(&g_lock);
	if (provider.prepare == NULL || !provider.prepare(provider.ctx)) {
		pthread_mutex_unlock(&g_take);
		U_LOG_W("Screenshot: the compositor offers none now");
		return XRT_ERROR_ALLOCATION;
	}

	struct timespec until;
	clock_gettime(CLOCK_MONOTONIC, &until);
	until.tv_sec += (time_t)(timeout_ns / 1000000000u);
	until.tv_nsec += (long)(timeout_ns % 1000000000u);
	if (until.tv_nsec >= 1000000000L) {
		until.tv_sec++;
		until.tv_nsec -= 1000000000L;
	}

	pthread_mutex_lock(&g_lock);
	g_delivered = false;
	g_count = 0;
	atomic_store(&g_wanted, true);
	int err = 0;
	while (!g_delivered && err != ETIMEDOUT) {
		err = pthread_cond_timedwait(&g_cond, &g_lock, &until);
	}
	atomic_store(&g_wanted, false);
	struct u_screenshot_view views[2];
	const uint32_t count = g_delivered ? g_count : 0;
	memcpy(views, g_views, sizeof(views));
	memset(g_views, 0, sizeof(g_views));
	g_count = 0;
	const float shoulder = g_shoulder;
	const bool delivered = g_delivered;
	pthread_mutex_unlock(&g_lock);

	xrt_result_t ret = XRT_SUCCESS;
	if (!delivered) {
		U_LOG_W("Screenshot: no frame came (nothing shown?)");
		ret = XRT_TIMEOUT;
	} else if (count == 0) {
		ret = XRT_ERROR_ALLOCATION;
	} else if (!write_png(png_path, views, count, shoulder)) {
		U_LOG_W("Screenshot: cannot write %s", png_path);
		ret = XRT_ERROR_ALLOCATION;
	}
	pthread_mutex_unlock(&g_take);
	return ret;
}

void
u_screenshot_set_provider(const struct u_screenshot_provider *provider)
{
	pthread_once(&g_once, init_cond);
	// Stopping waits for a screenshot being taken: it reads the provider's memory.
	pthread_mutex_lock(&g_take);
	pthread_mutex_lock(&g_lock);
	if (provider != NULL) {
		g_provider = *provider;
	} else {
		memset(&g_provider, 0, sizeof(g_provider));
	}
	pthread_mutex_unlock(&g_lock);
	pthread_mutex_unlock(&g_take);
}

bool
u_screenshot_wanted(void)
{
	return atomic_load(&g_wanted);
}

void
u_screenshot_deliver(struct u_screenshot_view *views, uint32_t count, float tonemap_shoulder)
{
	pthread_once(&g_once, init_cond);
	pthread_mutex_lock(&g_lock);
	if (!atomic_load(&g_wanted) || g_delivered) {
		pthread_mutex_unlock(&g_lock);
		return;
	}
	count = count > 2 ? 2 : count;
	memcpy(g_views, views, sizeof(*views) * count);
	g_count = count;
	g_shoulder = tonemap_shoulder;
	g_delivered = true;
	atomic_store(&g_wanted, false);
	pthread_cond_broadcast(&g_cond);
	pthread_mutex_unlock(&g_lock);
}

#else // XRT_OS_LINUX

xrt_result_t
u_screenshot_take(const char *png_path, uint64_t timeout_ns)
{
	(void)png_path;
	(void)timeout_ns;
	return XRT_ERROR_NOT_IMPLEMENTED;
}

void
u_screenshot_set_provider(const struct u_screenshot_provider *provider)
{
	(void)provider;
}

bool
u_screenshot_wanted(void)
{
	return false;
}

void
u_screenshot_deliver(struct u_screenshot_view *views, uint32_t count, float tonemap_shoulder)
{
	(void)views;
	(void)count;
	(void)tonemap_shoulder;
}

#endif // XRT_OS_LINUX
