// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Rift CV1's headphones and microphone as the default audio output and input while VR runs, and the
 *         previous defaults back after, through libpulse loaded at run time (PipeWire answers the same protocol);
 *         shared by the SteamVR driver and RiftLinux's own runtime. Every call gives up after a deadline, so a sound
 *         server that hangs holds nothing up.
 */

#pragma once

#if __has_include(<pulse/pulseaudio.h>)
#include <pulse/pulseaudio.h>
#define RL_HAVE_AUDIO_SWITCH 1

#include <dlfcn.h>

#include <sys/stat.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace rl::audio {

//! libpulse's functions used here, from dlopen.
struct Api
{
	decltype(&pa_mainloop_new) mainloop_new;
	decltype(&pa_mainloop_get_api) mainloop_get_api;
	decltype(&pa_mainloop_iterate) mainloop_iterate;
	decltype(&pa_mainloop_free) mainloop_free;
	decltype(&pa_context_new) context_new;
	decltype(&pa_context_connect) context_connect;
	decltype(&pa_context_get_state) context_get_state;
	decltype(&pa_context_disconnect) context_disconnect;
	decltype(&pa_context_unref) context_unref;
	decltype(&pa_context_get_server_info) context_get_server_info;
	decltype(&pa_context_get_sink_info_by_name) context_get_sink_info_by_name;
	decltype(&pa_context_set_default_sink) context_set_default_sink;
	decltype(&pa_context_get_source_info_list) context_get_source_info_list;
	decltype(&pa_context_set_default_source) context_set_default_source;
	decltype(&pa_operation_get_state) operation_get_state;
	decltype(&pa_operation_unref) operation_unref;
};

inline Api g_pa{};

//! Loads libpulse once; false where there is none.
inline bool
load()
{
	static std::once_flag once;
	static bool loaded = false;
	std::call_once(once, [] {
		void *lib = dlopen("libpulse.so.0", RTLD_NOW | RTLD_LOCAL);
		if (lib == nullptr) {
			return;
		}
		bool ok = true;
#define RL_PA_SYM(field, name) ok = ok && (*(void **)&g_pa.field = dlsym(lib, #name)) != nullptr;
		RL_PA_SYM(mainloop_new, pa_mainloop_new)
		RL_PA_SYM(mainloop_get_api, pa_mainloop_get_api)
		RL_PA_SYM(mainloop_iterate, pa_mainloop_iterate)
		RL_PA_SYM(mainloop_free, pa_mainloop_free)
		RL_PA_SYM(context_new, pa_context_new)
		RL_PA_SYM(context_connect, pa_context_connect)
		RL_PA_SYM(context_get_state, pa_context_get_state)
		RL_PA_SYM(context_disconnect, pa_context_disconnect)
		RL_PA_SYM(context_unref, pa_context_unref)
		RL_PA_SYM(context_get_server_info, pa_context_get_server_info)
		RL_PA_SYM(context_get_sink_info_by_name, pa_context_get_sink_info_by_name)
		RL_PA_SYM(context_set_default_sink, pa_context_set_default_sink)
		RL_PA_SYM(context_get_source_info_list, pa_context_get_source_info_list)
		RL_PA_SYM(context_set_default_source, pa_context_set_default_source)
		RL_PA_SYM(operation_get_state, pa_operation_get_state)
		RL_PA_SYM(operation_unref, pa_operation_unref)
#undef RL_PA_SYM
		if (!ok) {
			dlclose(lib);
			return;
		}
		loaded = true; // Kept loaded: its threads and atexit handlers may still refer to it.
	});
	return loaded;
}

//! One short connection to the desktop's sound server, on the calling thread.
class Connection
{
public:
	Connection()
	{
		m_loop = g_pa.mainloop_new();
		if (m_loop == nullptr) {
			return;
		}
		m_ctx = g_pa.context_new(g_pa.mainloop_get_api(m_loop), "RiftLinux");
		if (m_ctx == nullptr || g_pa.context_connect(m_ctx, nullptr, PA_CONTEXT_NOAUTOSPAWN, nullptr) < 0) {
			return;
		}
		const auto deadline = std::chrono::steady_clock::now() + kTimeout;
		while (std::chrono::steady_clock::now() < deadline) {
			const pa_context_state_t state = g_pa.context_get_state(m_ctx);
			if (state == PA_CONTEXT_READY) {
				m_ready = true;
				return;
			}
			if (!PA_CONTEXT_IS_GOOD(state)) {
				return;
			}
			step();
		}
	}

	~Connection()
	{
		if (m_ctx != nullptr) {
			g_pa.context_disconnect(m_ctx);
			g_pa.context_unref(m_ctx);
		}
		if (m_loop != nullptr) {
			g_pa.mainloop_free(m_loop);
		}
	}

	Connection(const Connection &) = delete;
	Connection &
	operator=(const Connection &) = delete;

	bool
	ready() const
	{
		return m_ready;
	}

	//! The default output and input; false when the server did not say.
	bool
	defaults(std::string *out_sink, std::string *out_source)
	{
		struct Result
		{
			bool done = false;
			std::string sink, source;
		} r;
		auto cb = [](pa_context *, const pa_server_info *info, void *ud) {
			Result *res = static_cast<Result *>(ud);
			if (info != nullptr && info->default_sink_name != nullptr) {
				res->sink = info->default_sink_name;
			}
			if (info != nullptr && info->default_source_name != nullptr) {
				res->source = info->default_source_name;
			}
			res->done = true;
		};
		if (!m_ready || !wait(g_pa.context_get_server_info(m_ctx, cb, &r)) || !r.done) {
			return false;
		}
		*out_sink = r.sink;
		*out_source = r.source;
		return true;
	}

	bool
	default_sink(std::string *out)
	{
		std::string source;
		return defaults(out, &source);
	}

	/*!
	 * The first input (not an output's monitor) whose name starts with @p prefix: the Rift's microphone's profile part
	 * differs between systems ("mono-fallback", "analog-mono").
	 */
	bool
	find_source(const std::string &prefix, std::string *out)
	{
		struct Result
		{
			const std::string *prefix;
			std::string name;
		} r{&prefix, {}};
		auto cb = [](pa_context *, const pa_source_info *info, int eol, void *ud) {
			Result *res = static_cast<Result *>(ud);
			if (eol == 0 && info != nullptr && info->monitor_of_sink == PA_INVALID_INDEX && res->name.empty() &&
			    strncmp(info->name, res->prefix->c_str(), res->prefix->size()) == 0) {
				res->name = info->name;
			}
		};
		if (!m_ready || !wait(g_pa.context_get_source_info_list(m_ctx, cb, &r)) || r.name.empty()) {
			return false;
		}
		*out = r.name;
		return true;
	}

	bool
	has_source(const std::string &name)
	{
		std::string found;
		return find_source(name, &found) && found == name;
	}

	bool
	set_default_source(const std::string &name)
	{
		int success = 0;
		auto cb = [](pa_context *, int ok, void *ud) { *static_cast<int *>(ud) = ok; };
		return m_ready && wait(g_pa.context_set_default_source(m_ctx, name.c_str(), cb, &success)) && success;
	}

	bool
	has_sink(const std::string &name)
	{
		bool found = false;
		auto cb = [](pa_context *, const pa_sink_info *info, int eol, void *ud) {
			if (eol == 0 && info != nullptr) {
				*static_cast<bool *>(ud) = true;
			}
		};
		return m_ready && wait(g_pa.context_get_sink_info_by_name(m_ctx, name.c_str(), cb, &found)) && found;
	}

	bool
	set_default_sink(const std::string &name)
	{
		int success = 0;
		auto cb = [](pa_context *, int ok, void *ud) { *static_cast<int *>(ud) = ok; };
		return m_ready && wait(g_pa.context_set_default_sink(m_ctx, name.c_str(), cb, &success)) && success;
	}

private:
	static constexpr std::chrono::milliseconds kTimeout{2000};

	void
	step()
	{
		if (g_pa.mainloop_iterate(m_loop, 0, nullptr) == 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}

	bool
	wait(pa_operation *op)
	{
		if (op == nullptr) {
			return false;
		}
		const auto deadline = std::chrono::steady_clock::now() + kTimeout;
		bool done = false;
		while (std::chrono::steady_clock::now() < deadline) {
			if (g_pa.operation_get_state(op) != PA_OPERATION_RUNNING) {
				done = true;
				break;
			}
			if (!PA_CONTEXT_IS_GOOD(g_pa.context_get_state(m_ctx))) {
				break;
			}
			step();
		}
		g_pa.operation_unref(op);
		return done;
	}

	pa_mainloop *m_loop = nullptr;
	pa_context *m_ctx = nullptr;
	bool m_ready = false;
};

//! The CV1's headphones: a USB audio device named after the headset's serial (its first 13 characters).
inline std::string
rift_sink(const char *hmd_serial)
{
	char sink[256];
	snprintf(sink, sizeof(sink), "alsa_output.usb-Oculus_Rift_Audio_%.13s-00.analog-stereo", hmd_serial);
	return sink;
}

//! The CV1's microphone, on the same USB audio device: the start of its name (the rest is the profile's).
inline std::string
rift_source_prefix(const char *hmd_serial)
{
	char source[256];
	snprintf(source, sizeof(source), "alsa_input.usb-Oculus_Rift_Audio_%.13s-00.", hmd_serial);
	return source;
}

/*!
 * The defaults before the headset's, in a file while the headset's are set (~/.cache/riftlinux/audio-before, "sink
 * <name>" and "source <name>" lines): a session that ended without putting them back (a crash, the computer switched
 * off) left the headset's as the defaults, which the sound server keeps across restarts, so every later session found
 * them "the default already" and nothing ever put the desktop's own back.
 */
inline std::string
before_path()
{
	const char *xdg = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	const std::string dir =
	    (xdg != nullptr && xdg[0] != '\0' ? std::string(xdg) : std::string(home != nullptr ? home : "/tmp") + "/.cache");
	mkdir(dir.c_str(), 0700);
	mkdir((dir + "/riftlinux").c_str(), 0700);
	return dir + "/riftlinux/audio-before";
}

inline void
before_read(std::string *sink, std::string *source)
{
	FILE *file = fopen(before_path().c_str(), "r");
	if (file == nullptr) {
		return;
	}
	char buf[512];
	while (fgets(buf, sizeof(buf), file) != nullptr) {
		std::string line(buf);
		while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
			line.pop_back();
		}
		if (line.rfind("sink ", 0) == 0) {
			*sink = line.substr(5);
		} else if (line.rfind("source ", 0) == 0) {
			*source = line.substr(7);
		}
	}
	fclose(file);
}

//! Both empty: the file goes.
inline void
before_write(const std::string &sink, const std::string &source)
{
	const std::string path = before_path();
	if (sink.empty() && source.empty()) {
		remove(path.c_str());
		return;
	}
	FILE *file = fopen(path.c_str(), "w");
	if (file == nullptr) {
		return;
	}
	if (!sink.empty()) {
		fprintf(file, "sink %s\n", sink.c_str());
	}
	if (!source.empty()) {
		fprintf(file, "source %s\n", source.c_str());
	}
	fclose(file);
}

/*!
 * Makes the headset's headphones and microphone the default output and input, keeping the ones before; tries for a
 * few seconds (the headset's USB audio can come up after it). @p log gets a line on what happened to each. The
 * headset's already the default with the file of a session that did not put the defaults back: those are put back
 * after this one.
 */
class Switch
{
public:
	void
	to_headset(const std::string &sink,
	           const std::string &source_prefix,
	           const std::function<void(const std::string &)> &log)
	{
		if (!load()) {
			log("Audio: no libpulse.so.0, the default output and input stay as they are");
			return;
		}
		bool sink_done = false, source_done = source_prefix.empty();
		std::string left_sink, left_source;
		before_read(&left_sink, &left_source);
		for (int i = 0; i < 10 && !(sink_done && source_done); i++) {
			if (i > 0) {
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
			Connection pa;
			std::string current_sink, current_source;
			if (!pa.ready() || !pa.defaults(&current_sink, &current_source)) {
				continue;
			}
			if (!sink_done && current_sink == sink) {
				if (!left_sink.empty() && left_sink != sink) {
					m_previous = left_sink;
					m_sink = sink;
					log("Audio: " + sink + " is the default output already, left by a session that did not end; " +
					    left_sink + " back after this one");
				} else {
					log("Audio: " + sink + " is the default output already");
				}
				sink_done = true;
			} else if (!sink_done && pa.has_sink(sink) && pa.set_default_sink(sink)) {
				m_previous = current_sink;
				m_sink = sink;
				log("Audio: default output now " + sink + " (was " + current_sink + ")");
				sink_done = true;
			}
			std::string source;
			if (!source_done && pa.find_source(source_prefix, &source)) {
				std::lock_guard<std::mutex> lock(m_lock);
				m_found_source = source;
				if (current_source == source) {
					if (!left_source.empty() && left_source != source) {
						m_previous_source = left_source;
						m_source = source;
						log("Audio: " + source + " is the default input already, left by a session that did not " +
						    "end; " + left_source + " back after this one");
					} else {
						log("Audio: " + source + " is the default input already");
					}
					source_done = true;
				} else if (pa.set_default_source(source)) {
					m_previous_source = current_source;
					m_source = source;
					log("Audio: default input now " + source + " (was " + current_source + ")");
					source_done = true;
				}
			}
		}
		if (!m_sink.empty() || !m_source.empty()) {
			before_write(m_sink.empty() ? std::string() : m_previous, m_source.empty() ? std::string() : m_previous_source);
		}
		if (!sink_done) {
			log("Audio: could not make " + sink + " the default output");
		}
		if (!source_done) {
			log("Audio: no microphone " + source_prefix + "* to make the default input");
		}
	}

	//! The headset's microphone's full name once found, else empty.
	std::string
	found_source()
	{
		std::lock_guard<std::mutex> lock(m_lock);
		return m_found_source;
	}

	//! Each only if the headset's is still the default: the user may have picked another meanwhile.
	void
	restore(const std::function<void(const std::string &)> &log)
	{
		if ((m_sink.empty() && m_source.empty()) || !load()) {
			return;
		}
		Connection pa;
		std::string current_sink, current_source;
		if (pa.ready() && pa.defaults(&current_sink, &current_source)) {
			if (!m_sink.empty() && !m_previous.empty() && current_sink == m_sink && pa.has_sink(m_previous) &&
			    pa.set_default_sink(m_previous)) {
				log("Audio: default output back to " + m_previous);
			}
			if (!m_source.empty() && !m_previous_source.empty() && current_source == m_source &&
			    pa.has_source(m_previous_source) && pa.set_default_source(m_previous_source)) {
				log("Audio: default input back to " + m_previous_source);
			}
		}
		m_sink.clear();
		m_source.clear();
		before_write("", "");
	}

private:
	std::string m_sink;
	std::string m_previous;
	std::string m_source;
	std::string m_previous_source;
	std::mutex m_lock;
	std::string m_found_source;
};

} // namespace rl::audio

#endif
