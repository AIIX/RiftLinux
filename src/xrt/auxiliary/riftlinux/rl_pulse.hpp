// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The default audio output's and input's (the microphone's) volume and mute through libpulse, loaded at run
 *         time (PipeWire answers the same protocol): read, followed and set, never waiting; shared by the Rift tab in
 *         SteamVR and the runtime's panel.
 */

#pragma once

#if __has_include(<pulse/pulseaudio.h>)
#include <pulse/pulseaudio.h>
#define RL_HAVE_PULSE 1

#include <dlfcn.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace rl {

//! Seconds on a steady clock.
inline double
pulse_now_s()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Pulse
{
	decltype(&pa_mainloop_new) mainloop_new = nullptr;
	decltype(&pa_mainloop_get_api) mainloop_get_api = nullptr;
	decltype(&pa_mainloop_iterate) mainloop_iterate = nullptr;
	decltype(&pa_mainloop_free) mainloop_free = nullptr;
	decltype(&pa_context_new) context_new = nullptr;
	decltype(&pa_context_connect) context_connect = nullptr;
	decltype(&pa_context_get_state) context_get_state = nullptr;
	decltype(&pa_context_disconnect) context_disconnect = nullptr;
	decltype(&pa_context_unref) context_unref = nullptr;
	decltype(&pa_context_set_subscribe_callback) context_set_subscribe_callback = nullptr;
	decltype(&pa_context_subscribe) context_subscribe = nullptr;
	decltype(&pa_context_get_sink_info_by_name) context_get_sink_info_by_name = nullptr;
	decltype(&pa_context_set_sink_volume_by_name) context_set_sink_volume_by_name = nullptr;
	decltype(&pa_context_set_sink_mute_by_name) context_set_sink_mute_by_name = nullptr;
	decltype(&pa_context_get_source_info_by_name) context_get_source_info_by_name = nullptr;
	decltype(&pa_context_set_source_volume_by_name) context_set_source_volume_by_name = nullptr;
	decltype(&pa_context_set_source_mute_by_name) context_set_source_mute_by_name = nullptr;
	decltype(&pa_operation_unref) operation_unref = nullptr;

	pa_mainloop *loop = nullptr;
	pa_context *ctx = nullptr;
	bool subscribed = false;
	bool query = false;
	bool query_mic = false;
	double next_query_s = 0.0;
	double retry_s = 0.0;

	//! The default output as last heard of.
	bool known = false;
	std::string sink;
	uint8_t channels = 0;
	int percent = 0;
	bool muted = false;
	//! Something changed since the tab was last drawn; changed from elsewhere (not by the tab).
	bool changed = false;
	bool changed_elsewhere = false;
	double own_change_until_s = 0.0;

	//! The default input, when it is a microphone (not another output's monitor), as last heard of.
	bool mic_known = false;
	std::string source;
	uint8_t mic_channels = 0;
	int mic_percent = 0;
	bool mic_muted = false;

	bool
	load()
	{
		void *lib = dlopen("libpulse.so.0", RTLD_NOW | RTLD_LOCAL);
		if (lib == nullptr) {
			fprintf(stderr, "riftlinux-notices: no libpulse.so.0, no volume in the dashboard\n");
			return false;
		}
		bool ok = true;
#define RLN_PA_SYM(field) ok = ok && (*reinterpret_cast<void **>(&field) = dlsym(lib, "pa_" #field)) != nullptr;
		RLN_PA_SYM(mainloop_new)
		RLN_PA_SYM(mainloop_get_api)
		RLN_PA_SYM(mainloop_iterate)
		RLN_PA_SYM(mainloop_free)
		RLN_PA_SYM(context_new)
		RLN_PA_SYM(context_connect)
		RLN_PA_SYM(context_get_state)
		RLN_PA_SYM(context_disconnect)
		RLN_PA_SYM(context_unref)
		RLN_PA_SYM(context_set_subscribe_callback)
		RLN_PA_SYM(context_subscribe)
		RLN_PA_SYM(context_get_sink_info_by_name)
		RLN_PA_SYM(context_set_sink_volume_by_name)
		RLN_PA_SYM(context_set_sink_mute_by_name)
		RLN_PA_SYM(context_get_source_info_by_name)
		RLN_PA_SYM(context_set_source_volume_by_name)
		RLN_PA_SYM(context_set_source_mute_by_name)
		RLN_PA_SYM(operation_unref)
#undef RLN_PA_SYM
		if (!ok) {
			fprintf(stderr, "riftlinux-notices: libpulse.so.0 lacks a function, no volume in the dashboard\n");
			return false;
		}
		loop = mainloop_new();
		return loop != nullptr;
	}

	void
	unref(pa_operation *op)
	{
		if (op != nullptr) {
			operation_unref(op);
		}
	}

	static void
	on_event(pa_context *, pa_subscription_event_type_t type, uint32_t, void *ud)
	{
		const unsigned facility = type & PA_SUBSCRIPTION_EVENT_FACILITY_MASK;
		auto *p = static_cast<Pulse *>(ud);
		if (facility == PA_SUBSCRIPTION_EVENT_SINK || facility == PA_SUBSCRIPTION_EVENT_SERVER) {
			p->query = true;
		}
		if (facility == PA_SUBSCRIPTION_EVENT_SOURCE || facility == PA_SUBSCRIPTION_EVENT_SERVER) {
			p->query_mic = true;
		}
	}

	//! The volume of all channels, % of normal.
	static int
	percent_of(const pa_cvolume &volume)
	{
		uint64_t sum = 0;
		for (uint8_t i = 0; i < volume.channels; i++) {
			sum += volume.values[i];
		}
		return static_cast<int>(std::lround(100.0 * sum / volume.channels / static_cast<double>(PA_VOLUME_NORM)));
	}

	static void
	on_source(pa_context *, const pa_source_info *info, int eol, void *ud)
	{
		auto *p = static_cast<Pulse *>(ud);
		if (eol != 0 || info == nullptr) {
			return;
		}
		// An output's monitor as the default input is no microphone: nothing to show.
		if (info->monitor_of_sink != PA_INVALID_INDEX || info->volume.channels == 0) {
			if (p->mic_known) {
				p->changed = true;
			}
			p->mic_known = false;
			return;
		}
		const int percent = percent_of(info->volume);
		const bool muted = info->mute != 0;
		if (!p->mic_known || p->source != info->name || percent != p->mic_percent || muted != p->mic_muted) {
			fprintf(stderr, "riftlinux-notices: microphone %s now %d%%%s\n", info->name, percent,
			        muted ? " muted" : "");
			p->changed = true;
		}
		p->mic_known = true;
		p->source = info->name;
		p->mic_channels = info->volume.channels;
		p->mic_percent = percent;
		p->mic_muted = muted;
	}

	static void
	on_sink(pa_context *, const pa_sink_info *info, int eol, void *ud)
	{
		auto *p = static_cast<Pulse *>(ud);
		if (eol != 0 || info == nullptr || info->volume.channels == 0) {
			return;
		}
		const int percent = percent_of(info->volume);
		const bool muted = info->mute != 0;
		// Another default output (the driver switching to the Rift as SteamVR starts) is taken quietly.
		const bool same_sink = p->known && p->sink == info->name;
		if (same_sink && (percent != p->percent || muted != p->muted)) {
			fprintf(stderr, "riftlinux-notices: output now %d%%%s (was %d%%%s)\n", percent, muted ? " muted" : "",
			        p->percent, p->muted ? " muted" : "");
			p->changed = true;
			p->changed_elsewhere = p->changed_elsewhere || pulse_now_s() > p->own_change_until_s;
		} else if (!same_sink) {
			p->changed = true;
		}
		p->known = true;
		p->sink = info->name;
		p->channels = info->volume.channels;
		p->percent = percent;
		p->muted = muted;
	}

	//! Runs the connection's work that is due, never waiting.
	void
	tick(double t)
	{
		if (loop == nullptr) {
			return;
		}
		if (ctx == nullptr && t >= retry_s) {
			ctx = context_new(mainloop_get_api(loop), "riftlinux-notices");
			if (ctx != nullptr && context_connect(ctx, nullptr, PA_CONTEXT_NOAUTOSPAWN, nullptr) < 0) {
				context_unref(ctx);
				ctx = nullptr;
			}
			subscribed = false;
			retry_s = t + 5.0;
		}
		for (int i = 0; i < 64 && mainloop_iterate(loop, 0, nullptr) > 0; i++) {
		}
		if (ctx == nullptr) {
			return;
		}
		const pa_context_state_t state = context_get_state(ctx);
		if (state == PA_CONTEXT_FAILED || state == PA_CONTEXT_TERMINATED) {
			context_disconnect(ctx);
			context_unref(ctx);
			ctx = nullptr;
			return;
		}
		if (state != PA_CONTEXT_READY) {
			return;
		}
		if (!subscribed) {
			context_set_subscribe_callback(ctx, on_event, this);
			unref(context_subscribe(ctx,
			                        static_cast<pa_subscription_mask_t>(PA_SUBSCRIPTION_MASK_SINK |
			                                                            PA_SUBSCRIPTION_MASK_SOURCE |
			                                                            PA_SUBSCRIPTION_MASK_SERVER),
			                        nullptr, nullptr));
			subscribed = true;
			query = true;
			query_mic = true;
		}
		if ((query || query_mic) && t >= next_query_s) {
			if (query) {
				unref(context_get_sink_info_by_name(ctx, "@DEFAULT_SINK@", on_sink, this));
			}
			if (query_mic) {
				unref(context_get_source_info_by_name(ctx, "@DEFAULT_SOURCE@", on_source, this));
			}
			query = false;
			query_mic = false;
			next_query_s = t + 0.05;
		}
	}

	bool
	ready()
	{
		return known && ctx != nullptr && context_get_state(ctx) == PA_CONTEXT_READY;
	}

	void
	set_percent(int value, double t)
	{
		if (!ready()) {
			return;
		}
		value = std::clamp(value, 0, 100);
		pa_cvolume volume;
		memset(&volume, 0, sizeof(volume));
		volume.channels = std::min<uint8_t>(channels, PA_CHANNELS_MAX);
		for (uint8_t i = 0; i < volume.channels; i++) {
			volume.values[i] = static_cast<pa_volume_t>(static_cast<uint64_t>(PA_VOLUME_NORM) * value / 100);
		}
		unref(context_set_sink_volume_by_name(ctx, sink.c_str(), &volume, nullptr, nullptr));
		own_change_until_s = t + 1.0;
		if (value != percent) {
			fprintf(stderr, "riftlinux-notices: volume %d%% (was %d%%)\n", value, percent);
		}
		percent = value;
		changed = true;
	}

	void
	set_muted(bool value, double t)
	{
		if (!ready()) {
			return;
		}
		unref(context_set_sink_mute_by_name(ctx, sink.c_str(), value ? 1 : 0, nullptr, nullptr));
		own_change_until_s = t + 1.0;
		fprintf(stderr, "riftlinux-notices: %s\n", value ? "muted" : "unmuted");
		muted = value;
		changed = true;
	}

	bool
	mic_ready()
	{
		return mic_known && ctx != nullptr && context_get_state(ctx) == PA_CONTEXT_READY;
	}

	void
	set_mic_percent(int value)
	{
		if (!mic_ready()) {
			return;
		}
		value = std::clamp(value, 0, 100);
		pa_cvolume volume;
		memset(&volume, 0, sizeof(volume));
		volume.channels = std::min<uint8_t>(mic_channels, PA_CHANNELS_MAX);
		for (uint8_t i = 0; i < volume.channels; i++) {
			volume.values[i] = static_cast<pa_volume_t>(static_cast<uint64_t>(PA_VOLUME_NORM) * value / 100);
		}
		unref(context_set_source_volume_by_name(ctx, source.c_str(), &volume, nullptr, nullptr));
		if (value != mic_percent) {
			fprintf(stderr, "riftlinux-notices: microphone %d%% (was %d%%)\n", value, mic_percent);
		}
		mic_percent = value;
		changed = true;
	}

	void
	set_mic_muted(bool value)
	{
		if (!mic_ready()) {
			return;
		}
		unref(context_set_source_mute_by_name(ctx, source.c_str(), value ? 1 : 0, nullptr, nullptr));
		fprintf(stderr, "riftlinux-notices: microphone %s\n", value ? "muted" : "unmuted");
		mic_muted = value;
		changed = true;
	}
};

} // namespace rl

#endif
