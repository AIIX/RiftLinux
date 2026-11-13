// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Short notices for the user, see u_notice.h.
 * @ingroup aux_util
 */

#include "util/u_notice.h"

#include <atomic>
#include <cstring>
#include <mutex>

static std::mutex g_notice_lock;
static u_notice_sink_func_t g_notice_func = nullptr;
static void *g_notice_ptr = nullptr;

extern "C" void
u_notice_set_sink(u_notice_sink_func_t func, void *ptr)
{
	std::lock_guard<std::mutex> lock(g_notice_lock);
	g_notice_func = func;
	g_notice_ptr = ptr;
}

extern "C" void
u_notice_post(enum u_notice_kind kind, const char *key, int number)
{
	// Held while the sink runs, so a sink being unset never has a call still in it.
	std::lock_guard<std::mutex> lock(g_notice_lock);
	if (g_notice_func != nullptr) {
		g_notice_func(g_notice_ptr, kind, key, number);
	}
}

static std::mutex g_sensors_lock;
static bool g_sensors_set = false;
static struct u_notice_sensors g_sensors = {};

extern "C" void
u_notice_set_sensors(const struct u_notice_sensors *sensors)
{
	std::lock_guard<std::mutex> lock(g_sensors_lock);
	g_sensors = *sensors;
	g_sensors_set = true;
}

extern "C" bool
u_notice_get_sensors(struct u_notice_sensors *out_sensors)
{
	std::lock_guard<std::mutex> lock(g_sensors_lock);
	*out_sensors = g_sensors;
	return g_sensors_set;
}

static std::mutex g_recalibrate_lock;
static u_notice_recalibrate_func_t g_recalibrate_func = nullptr;
static void *g_recalibrate_ptr = nullptr;

extern "C" void
u_notice_set_recalibrator(u_notice_recalibrate_func_t func, void *ptr)
{
	std::lock_guard<std::mutex> lock(g_recalibrate_lock);
	g_recalibrate_func = func;
	g_recalibrate_ptr = ptr;
}

extern "C" bool
u_notice_request_recalibration(void)
{
	// Held while the tracker answers, so a tracker going away never has a call still in it.
	std::lock_guard<std::mutex> lock(g_recalibrate_lock);
	return g_recalibrate_func != nullptr && g_recalibrate_func(g_recalibrate_ptr);
}

static std::mutex g_recalibration_lock;
static bool g_recalibration_set = false;
static struct u_notice_recalibration g_recalibration = {};

extern "C" void
u_notice_set_recalibration(const struct u_notice_recalibration *result)
{
	std::lock_guard<std::mutex> lock(g_recalibration_lock);
	g_recalibration = *result;
	g_recalibration_set = true;
}

extern "C" bool
u_notice_get_recalibration(struct u_notice_recalibration *out_result)
{
	std::lock_guard<std::mutex> lock(g_recalibration_lock);
	*out_result = g_recalibration;
	return g_recalibration_set;
}

static std::atomic<bool> g_display_off_requested{false};

extern "C" void
u_notice_request_display_off(void)
{
	g_display_off_requested.store(true);
}

extern "C" bool
u_notice_display_off_requested(void)
{
	return g_display_off_requested.load();
}

static std::atomic<bool> g_display_black_requested{false};
static std::atomic<bool> g_display_black{false};

extern "C" void
u_notice_request_display_black(bool black)
{
	g_display_black_requested.store(black);
}

extern "C" bool
u_notice_display_black_requested(void)
{
	return g_display_black_requested.load();
}

extern "C" void
u_notice_set_display_black(bool black)
{
	g_display_black.store(black);
}

extern "C" bool
u_notice_display_black(void)
{
	return g_display_black.load();
}

static std::mutex g_ipd_lock;
static bool g_ipd_set = false;
static struct u_notice_ipd g_ipd = {};
static std::atomic<float> g_manual_ipd{0.0f};

extern "C" void
u_notice_set_ipd(const struct u_notice_ipd *ipd)
{
	std::lock_guard<std::mutex> lock(g_ipd_lock);
	g_ipd = *ipd;
	g_ipd_set = true;
}

extern "C" bool
u_notice_get_ipd(struct u_notice_ipd *out_ipd)
{
	std::lock_guard<std::mutex> lock(g_ipd_lock);
	*out_ipd = g_ipd;
	return g_ipd_set;
}

extern "C" void
u_notice_set_manual_ipd(float mm)
{
	g_manual_ipd.store(mm > 0.0f ? mm : 0.0f);
}

extern "C" float
u_notice_get_manual_ipd(void)
{
	return g_manual_ipd.load();
}

static std::mutex g_lens_lock;
static bool g_lens_pattern = false;
static int64_t g_lens_client = 0;
static bool g_lens_centres_set = false;
static int32_t g_lens_centres[2][2] = {};

extern "C" void
u_notice_set_lens_pattern(bool on, int64_t client)
{
	std::lock_guard<std::mutex> lock(g_lens_lock);
	g_lens_pattern = on;
	g_lens_client = on ? client : 0;
}

extern "C" void
u_notice_drop_lens_pattern(int64_t client)
{
	std::lock_guard<std::mutex> lock(g_lens_lock);
	if (g_lens_pattern && g_lens_client == client) {
		g_lens_pattern = false;
		g_lens_client = 0;
	}
}

extern "C" bool
u_notice_lens_pattern(void)
{
	std::lock_guard<std::mutex> lock(g_lens_lock);
	return g_lens_pattern;
}

extern "C" void
u_notice_set_lens_centres(int32_t left_x, int32_t left_y, int32_t right_x, int32_t right_y)
{
	std::lock_guard<std::mutex> lock(g_lens_lock);
	g_lens_centres[0][0] = left_x;
	g_lens_centres[0][1] = left_y;
	g_lens_centres[1][0] = right_x;
	g_lens_centres[1][1] = right_y;
	g_lens_centres_set = true;
}

extern "C" bool
u_notice_get_lens_centres(int32_t out_centres[2][2])
{
	std::lock_guard<std::mutex> lock(g_lens_lock);
	memcpy(out_centres, g_lens_centres, sizeof(g_lens_centres));
	return g_lens_centres_set;
}

