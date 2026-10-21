// Copyright 2023, Collabora, Ltd.
// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Various helpers for doing Linux specific things.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 *
 * @ingroup aux_util
 */

#include "xrt/xrt_config_have.h"
#include "util/u_linux.h"
#include "util/u_notice.h"
#include "util/u_pretty_print.h"

#include <pthread.h>
#include <assert.h>
#include <unistd.h>
#include <sched.h>
#include <stdio.h>

#ifdef XRT_HAVE_DBUS
#include <dbus/dbus.h>
#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

#define LOG_D(...) U_LOG_IFL_D(log_level, __VA_ARGS__)
#define LOG_I(...) U_LOG_IFL_I(log_level, __VA_ARGS__)
#define LOG_W(...) U_LOG_IFL_W(log_level, __VA_ARGS__)
#define LOG_E(...) U_LOG_IFL_E(log_level, __VA_ARGS__)

#define NAME_LENGTH 32


/*
 *
 * Helper functions.
 *
 */

static const char *
policy_to_string(int policy)
{
	switch (policy) {
	case SCHED_FIFO: return "SCHED_FIFO";
	case SCHED_RR: return "SCHED_RR";
	case SCHED_OTHER: return "SCHED_OTHER(normal)";
	case SCHED_IDLE: return "SCHED_IDLE";
	case SCHED_BATCH: return "SCHED_BATCH";
	default: return "SCHED_<UNKNOWN>";
	}
}

static void
get_name(char *str, size_t count)
{
	assert(str != NULL);
	assert(count > 0);

	// First init.
	str[0] = '\0';

	// Get name of thread.
	pthread_t this_thread = pthread_self();
	pthread_getname_np(this_thread, str, count);

	if (str[0] == '\0') {
		snprintf(str, count, "tid(%i)", gettid());
	}
}

static void
print_thread_info(struct u_pp_delegate dg, enum u_logging_level log_level, pthread_t thread)
{
	struct sched_param params;
	int policy = 0;
	int ret = 0;

	// Get the policy and scheduling priority.
	ret = pthread_getschedparam(thread, &policy, &params);
	if (ret != 0) {
		LOG_E("pthread_getschedparam: %i", ret);
		return;
	}

	u_pp(dg, "policy: '%s', priority: '%i'", policy_to_string(policy), params.sched_priority);
}


#ifdef XRT_HAVE_DBUS
/*
 * RiftLinux: RealtimeKit fallback. A process without CAP_SYS_NICE and with RLIMIT_RTPRIO 0 (a normal desktop login,
 * and SteamVR's vrserver, which hosts the SteamVR driver) may not make its threads real-time itself; then the
 * headset's USB, radio, vsync and compositor threads compete with a CPU-heavy game on equal terms. RealtimeKit,
 * the system service PipeWire and PulseAudio use, grants real-time scheduling (up to its MaxRealtimePriority, 20 by
 * default) to a process whose RLIMIT_RTTIME is at most its RTTimeUSecMax (200 ms by default): a real-time thread
 * that runs that long without blocking gets SIGXCPU at the soft limit and SIGKILL at the hard one. The soft limit
 * is set to half: on SIGXCPU every thread made real-time here goes back to normal scheduling, so a runaway thread
 * costs real-time scheduling, never the process.
 */

#define RTKIT_MAX_THREADS 32

static _Atomic pid_t g_rt_tids[RTKIT_MAX_THREADS];
static atomic_int g_rt_demoted;

static void
rt_sigxcpu_handler(int sig)
{
	(void)sig;
	struct sched_param p = {.sched_priority = 0};
	for (int i = 0; i < RTKIT_MAX_THREADS; i++) {
		pid_t tid = atomic_load(&g_rt_tids[i]);
		if (tid > 0) {
			syscall(SYS_sched_setscheduler, tid, SCHED_OTHER, &p);
		}
	}
	atomic_store(&g_rt_demoted, 1);
}

static void
rt_remember_thread(pid_t tid)
{
	for (int i = 0; i < RTKIT_MAX_THREADS; i++) {
		pid_t expected = 0;
		if (atomic_compare_exchange_strong(&g_rt_tids[i], &expected, tid)) {
			return;
		}
	}
}

static bool
rtkit_get_int_property(DBusConnection *bus, const char *name, int64_t *out_value)
{
	const char *iface = "org.freedesktop.RealtimeKit1";
	DBusMessage *m = dbus_message_new_method_call("org.freedesktop.RealtimeKit1", "/org/freedesktop/RealtimeKit1",
	                                              "org.freedesktop.DBus.Properties", "Get");
	if (m == NULL) {
		return false;
	}
	dbus_message_append_args(m, DBUS_TYPE_STRING, &iface, DBUS_TYPE_STRING, &name, DBUS_TYPE_INVALID);

	DBusError err;
	dbus_error_init(&err);
	DBusMessage *r = dbus_connection_send_with_reply_and_block(bus, m, 1000, &err);
	dbus_message_unref(m);
	if (r == NULL) {
		dbus_error_free(&err);
		return false;
	}

	bool ok = false;
	DBusMessageIter it, var;
	if (dbus_message_iter_init(r, &it) && dbus_message_iter_get_arg_type(&it) == DBUS_TYPE_VARIANT) {
		dbus_message_iter_recurse(&it, &var);
		int type = dbus_message_iter_get_arg_type(&var);
		if (type == DBUS_TYPE_INT32) {
			dbus_int32_t v = 0;
			dbus_message_iter_get_basic(&var, &v);
			*out_value = v;
			ok = true;
		} else if (type == DBUS_TYPE_INT64) {
			dbus_int64_t v = 0;
			dbus_message_iter_get_basic(&var, &v);
			*out_value = v;
			ok = true;
		}
	}
	dbus_message_unref(r);
	return ok;
}

static bool
rtkit_make_thread_realtime(enum u_logging_level log_level, int *out_priority, bool *out_missing)
{
	*out_missing = false;
	DBusError err;
	dbus_error_init(&err);
	DBusConnection *bus = dbus_bus_get_private(DBUS_BUS_SYSTEM, &err);
	if (bus == NULL) {
		LOG_D("RealtimeKit: no system bus: %s", err.message ? err.message : "?");
		dbus_error_free(&err);
		return false;
	}
	dbus_connection_set_exit_on_disconnect(bus, FALSE);

	int64_t max_priority = 20;
	int64_t rttime_max_us = 200000;
	rtkit_get_int_property(bus, "MaxRealtimePriority", &max_priority);
	rtkit_get_int_property(bus, "RTTimeUSecMax", &rttime_max_us);

	// RealtimeKit refuses processes without a bounded RLIMIT_RTTIME.
	struct rlimit rl;
	if (getrlimit(RLIMIT_RTTIME, &rl) == 0 &&
	    (rl.rlim_max == RLIM_INFINITY || rl.rlim_max > (rlim_t)rttime_max_us)) {
		rl.rlim_max = (rlim_t)rttime_max_us;
		rl.rlim_cur = (rlim_t)rttime_max_us / 2;
		if (setrlimit(RLIMIT_RTTIME, &rl) != 0) {
			LOG_D("RealtimeKit: setrlimit(RLIMIT_RTTIME): %s", strerror(errno));
		}
	}

	// Demote instead of dying on SIGXCPU, unless the host process handles it already.
	static atomic_int handler_done;
	if (atomic_exchange(&handler_done, 1) == 0) {
		struct sigaction old;
		if (sigaction(SIGXCPU, NULL, &old) == 0 && old.sa_handler == SIG_DFL) {
			struct sigaction sa;
			memset(&sa, 0, sizeof(sa));
			sa.sa_handler = rt_sigxcpu_handler;
			sa.sa_flags = SA_RESTART;
			sigemptyset(&sa.sa_mask);
			sigaction(SIGXCPU, &sa, NULL);
		}
	}

	pid_t tid = (pid_t)syscall(SYS_gettid);
	dbus_uint64_t tid64 = (dbus_uint64_t)tid;
	dbus_uint32_t prio = (dbus_uint32_t)max_priority;
	bool ok = false;
	DBusMessage *m = dbus_message_new_method_call("org.freedesktop.RealtimeKit1", "/org/freedesktop/RealtimeKit1",
	                                              "org.freedesktop.RealtimeKit1", "MakeThreadRealtime");
	if (m != NULL) {
		dbus_message_append_args(m, DBUS_TYPE_UINT64, &tid64, DBUS_TYPE_UINT32, &prio, DBUS_TYPE_INVALID);
		DBusMessage *r = dbus_connection_send_with_reply_and_block(bus, m, 1000, &err);
		dbus_message_unref(m);
		if (r != NULL) {
			dbus_message_unref(r);
			ok = true;
			rt_remember_thread(tid);
			*out_priority = (int)prio;
		} else {
			LOG_D("RealtimeKit: MakeThreadRealtime: %s", err.message ? err.message : "?");
			// Not installed or not running, as opposed to refusing (polkit: a remote login, an inactive session).
			*out_missing = err.name != NULL && (strcmp(err.name, DBUS_ERROR_SERVICE_UNKNOWN) == 0 ||
			                                    strcmp(err.name, DBUS_ERROR_NAME_HAS_NO_OWNER) == 0);
			dbus_error_free(&err);
		}
	}

	dbus_connection_close(bus);
	dbus_connection_unref(bus);
	return ok;
}
#endif


/*
 *
 * 'Exported' functions.
 *
 */

bool
u_linux_try_to_set_realtime_priority_on_thread(enum u_logging_level log_level, const char *name)
{
	pthread_t this_thread = pthread_self();
	struct u_pp_sink_stack_only sink;
	struct sched_param params;
	char str[NAME_LENGTH];
	int ret;

	// Add printing delegate.
	struct u_pp_delegate dg = u_pp_sink_stack_only_init(&sink);

	// Always have some name.
	if (name == NULL) {
		get_name(str, ARRAY_SIZE(str));
		name = str;
	}

	if (log_level <= U_LOGGING_DEBUG) {
		u_pp(dg, "Trying to raise priority on thread '%s'\n\t", name);
		u_pp(dg, "before: ");
		print_thread_info(dg, log_level, this_thread);
	}

	// Get the maximum on this platform.
	params.sched_priority = sched_get_priority_max(SCHED_FIFO);

	// Here we try to set the realtime scheduling with the max priority available.
	ret = pthread_setschedparam(this_thread, SCHED_FIFO, &params);

#ifdef XRT_HAVE_DBUS
	// RiftLinux: not allowed ourselves -> ask RealtimeKit (see above).
	bool via_rtkit = false;
	if (ret != 0) {
		int rt_priority = 0;
		bool missing = false;
		if (rtkit_make_thread_realtime(log_level, &rt_priority, &missing)) {
			ret = 0;
			via_rtkit = true;
		} else {
			// Once a process, in the headset (RiftLinux): tracking and frames may stutter under load.
			static atomic_int said;
			if (atomic_exchange(&said, 1) == 0) {
				u_notice_post(U_NOTICE_SYSTEM, missing ? "rt-missing" : "rt-refused", 0);
			}
		}
	}
#endif

	// Print different amount depending on log level.
	if (log_level <= U_LOGGING_DEBUG) {
		u_pp(dg, "after: ");
		print_thread_info(dg, log_level, this_thread);
		u_pp(dg, "\n\tResult: %i", ret);
	} else {
		if (ret != 0) {
			u_pp(dg, "Could not raise priority for thread '%s'", name);
		} else {
			u_pp(dg, "Raised priority of thread '%s' to ", name);
#ifdef XRT_HAVE_DBUS
			if (via_rtkit) {
				// The C library's cached policy is stale after RealtimeKit's change: ask the kernel.
				struct sched_param kp = {0};
				sched_getparam(0, &kp);
				u_pp(dg, "policy: '%s', priority: '%i' (through RealtimeKit)",
				     policy_to_string(sched_getscheduler(0) & ~SCHED_RESET_ON_FORK), kp.sched_priority);
			} else
#endif
			{
				print_thread_info(dg, log_level, this_thread);
			}
		}
	}

	// Always print as warning or information.
	if (ret != 0) {
		LOG_W("%s", sink.buffer);
	} else {
		LOG_I("%s", sink.buffer);
	}

	return ret == 0;
}
