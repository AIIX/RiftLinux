// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Short notices for the user, see u_notice.h.
 * @ingroup aux_util
 */

#include "util/u_notice.h"

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
