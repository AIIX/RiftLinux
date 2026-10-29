// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  RiftLinux's settings file, ~/.config/riftlinux/settings.json: read and changed key by key, by the SteamVR
 *         driver, RiftLinux's own runtime and later the desktop app, so a setting made in one is the setting in all.
 */

#pragma once

#include "util/u_json.h"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

namespace rl::settings {

//! $XDG_CONFIG_HOME/riftlinux/settings.json, else ~/.config/riftlinux/settings.json.
inline std::string
path()
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	const std::string dir =
	    xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "") + "/.config";
	return dir + "/riftlinux/settings.json";
}

//! A small file's text (at most 64 KB and a block), or empty.
inline std::string
read_text(const std::string &file_path)
{
	FILE *file = fopen(file_path.c_str(), "r");
	if (file == nullptr) {
		return {};
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0 && text.size() < 65536) {
		text.append(buf, n);
	}
	fclose(file);
	return text;
}

//! @p text parsed when it is a JSON object, else NULL.
inline cJSON *
parse_object(const std::string &text)
{
	cJSON *root = cJSON_Parse(text.c_str());
	if (root != nullptr && !cJSON_IsObject(root)) {
		cJSON_Delete(root);
		root = nullptr;
	}
	return root;
}

/*!
 * The file parsed, or NULL (none, or empty); free with cJSON_Delete. Each file that could be used is kept as
 * settings.json.last-known; a file that cannot be (not JSON, not an object) is left as it is and the last-known one is
 * taken instead, or none: every setting at its default. That is said once on stderr (the driver's or the runtime's
 * log), and again when it changes, so a report shows why the settings did not apply. @p out_unusable: the file could
 * not be used (whatever was taken instead).
 */
inline cJSON *
load(bool *out_unusable = nullptr)
{
	static std::mutex lock;
	static int unusable = -1;
	static std::string kept;
	if (out_unusable != nullptr) {
		*out_unusable = false;
	}
	const std::string file_path = path();
	if (access(file_path.c_str(), F_OK) != 0) {
		return nullptr;
	}
	const std::string text = read_text(file_path);
	cJSON *root = parse_object(text);
	const bool empty = text.find_first_not_of(" \t\r\n") == std::string::npos;
	const int now = root == nullptr && !empty ? 1 : 0;
	if (out_unusable != nullptr) {
		*out_unusable = now == 1;
	}
	std::lock_guard<std::mutex> guard(lock);
	const std::string last_known = file_path + ".last-known";
	if (root != nullptr && text != kept) {
		// Kept whole and renamed into place: a copy is never half written.
		if (read_text(last_known) != text) {
			const std::string tmp = last_known + ".tmp";
			FILE *file = fopen(tmp.c_str(), "w");
			if (file != nullptr) {
				const bool ok = fwrite(text.data(), 1, text.size(), file) == text.size();
				if (fclose(file) == 0 && ok) {
					rename(tmp.c_str(), last_known.c_str());
				}
			}
		}
		kept = text;
	}
	cJSON *fallback = nullptr;
	if (now == 1) {
		fallback = parse_object(read_text(last_known));
	}
	if (now != unusable) {
		if (now == 1) {
			fprintf(stderr,
			        "RiftLinux: %s is not valid JSON, or not an object%s: it is left as it is, and %s "
			        "(riftlinux-setup check says where; riftlinux-setup fix last-known %s)\n",
			        file_path.c_str(), text.size() >= 65536 ? " (or larger than 64 KB)" : "",
			        fallback != nullptr ? "the last good settings are used (settings.json.last-known)"
			                            : "every RiftLinux setting is at its default",
			        fallback != nullptr ? "puts them back" : "starts over from the defaults");
		} else if (unusable == 1) {
			fprintf(stderr, "RiftLinux: %s can be read again\n", file_path.c_str());
		}
		unusable = now;
	}
	return root != nullptr ? root : fallback;
}

//! @p value as a whole number within [@p lo, @p hi]: settings.json may hold any number.
inline long
clamped(double value, double lo, double hi)
{
	return (long)(value < lo ? lo : value > hi ? hi : value);
}

//! @p section.@p key of a parsed file, or NULL.
inline const cJSON *
item(const cJSON *root, const char *section, const char *key)
{
	const cJSON *sec = root != nullptr ? cJSON_GetObjectItemCaseSensitive(root, section) : nullptr;
	return cJSON_IsObject(sec) ? cJSON_GetObjectItemCaseSensitive(sec, key) : nullptr;
}

//! @p section.@p key as a number, or @p fallback.
inline double
number(const cJSON *root, const char *section, const char *key, double fallback)
{
	const cJSON *it = item(root, section, key);
	return cJSON_IsNumber(it) ? it->valuedouble : fallback;
}

//! @p section.@p key as true or false (a number counts: 0 false), or @p fallback.
inline bool
boolean(const cJSON *root, const char *section, const char *key, bool fallback)
{
	const cJSON *it = item(root, section, key);
	if (cJSON_IsBool(it)) {
		return cJSON_IsTrue(it);
	}
	return cJSON_IsNumber(it) ? it->valuedouble != 0.0 : fallback;
}

/*!
 * Sets @p section.@p key to @p value (taken) in the file, keeping everything else in it; written whole, then renamed. A
 * file that is there but cannot be read is left as it is (written anew it would lose every other setting in it): the
 * change applies until the next start only, and stderr says so.
 */
inline void
set_item(const char *section, const char *key, cJSON *value)
{
	// Read, changed, written whole: one writer at a time in this process, or a change made at the same moment is lost.
	static std::mutex write_lock;
	std::lock_guard<std::mutex> lock(write_lock);
	bool unusable = false;
	cJSON *root = load(&unusable);
	if (unusable) {
		fprintf(stderr, "RiftLinux: %s.%s not saved: %s cannot be read, and is left as it is\n", section, key,
		        path().c_str());
		cJSON_Delete(root);
		cJSON_Delete(value);
		return;
	}
	if (root == nullptr) {
		root = cJSON_CreateObject();
	}
	cJSON *sec = cJSON_GetObjectItemCaseSensitive(root, section);
	if (!cJSON_IsObject(sec)) {
		cJSON_DeleteItemFromObjectCaseSensitive(root, section);
		sec = cJSON_AddObjectToObject(root, section);
	}
	cJSON_DeleteItemFromObjectCaseSensitive(sec, key);
	cJSON_AddItemToObject(sec, key, value);
	char *text = cJSON_Print(root);
	cJSON_Delete(root);
	if (text == nullptr) {
		return;
	}
	const std::string file_path = path();
	const std::string dir = file_path.substr(0, file_path.rfind('/'));
	mkdir(dir.c_str(), 0755);
	const std::string tmp = file_path + ".tmp";
	FILE *file = fopen(tmp.c_str(), "w");
	if (file != nullptr) {
		fputs(text, file);
		fputc('\n', file);
		fclose(file);
		rename(tmp.c_str(), file_path.c_str());
	}
	cJSON_free(text);
}

inline void
set_number(const char *section, const char *key, double value)
{
	set_item(section, key, cJSON_CreateNumber(value));
}

inline void
set_bool(const char *section, const char *key, bool value)
{
	set_item(section, key, cJSON_CreateBool(value));
}

/*!
 * The "headset" section as the Rift driver's options, set before the headset is opened, by the SteamVR driver and
 * RiftLinux's own runtime; an option already set in the environment wins. Keys:
 *  - wear_sensor (true/false, default true): false counts the headset as worn whenever it runs.
 *  - worn_above, unworn_below (wear sensor readings, default 30 and 15): worn above the first, not worn below the
 *    second.
 *  - worn_by_motion (true/false, default true): moving as on a head counts as worn.
 *  - worn_by_motion_after_s (seconds, default 20): after moving so that long (a headset held in the hands moves alike
 *    for a while).
 *  - display_off_after_s (seconds, default 5): how long not worn before the display switches off (or goes black, below);
 *    negative: never.
 *  - not_worn ("black" or "off", default "black"): the display kept on showing pure black (the headset stays connected:
 *    no monitor unplugged and plugged for the desktop), or switched off.
 *  - black_then_off_min (minutes, default 10): with "black", how long black before the display switches off after all;
 *    negative: never.
 * Returns what it set, for the log; empty when the section sets nothing.
 */
inline std::string
headset_to_env(const cJSON *root)
{
	std::string set;
	auto put = [&set](const char *env, const std::string &value, const char *key) {
		setenv(env, value.c_str(), 0);
		set += (set.empty() ? "" : ", ") + std::string(key) + " " + value;
	};
	const cJSON *it = item(root, "headset", "wear_sensor");
	if (cJSON_IsBool(it)) {
		put("RIFT_WEAR_SENSOR", cJSON_IsTrue(it) ? "1" : "0", "wear_sensor");
	}
	it = item(root, "headset", "worn_by_motion");
	if (cJSON_IsBool(it)) {
		put("RIFT_WEAR_BY_MOTION", cJSON_IsTrue(it) ? "1" : "0", "worn_by_motion");
	}
	it = item(root, "headset", "worn_by_motion_after_s");
	if (cJSON_IsNumber(it)) {
		put("RIFT_WEAR_BY_MOTION_AFTER_S", std::to_string(clamped(it->valuedouble, 1, 60)), "worn_by_motion_after_s");
	}
	it = item(root, "headset", "worn_above");
	if (cJSON_IsNumber(it)) {
		put("RIFT_WEAR_WORN_ABOVE", std::to_string(clamped(it->valuedouble, 0, 65535)), "worn_above");
	}
	it = item(root, "headset", "unworn_below");
	if (cJSON_IsNumber(it)) {
		put("RIFT_WEAR_UNWORN_BELOW", std::to_string(clamped(it->valuedouble, 0, 65535)), "unworn_below");
	}
	it = item(root, "headset", "display_off_after_s");
	if (cJSON_IsNumber(it)) {
		// At most a day.
		const long ms = it->valuedouble < 0.0 ? -1 : clamped(it->valuedouble * 1000.0, 0, 86400000);
		put("RIFT_DISPLAY_OFF_UNWORN_MS", std::to_string(ms), "display_off_after_s");
	}
	it = item(root, "headset", "not_worn");
	if (cJSON_IsString(it) && (strcmp(it->valuestring, "off") == 0 || strcmp(it->valuestring, "black") == 0)) {
		put("RIFT_DISPLAY_NOT_WORN", it->valuestring, "not_worn");
	}
	it = item(root, "headset", "black_then_off_min");
	if (cJSON_IsNumber(it)) {
		put("RIFT_DISPLAY_BLACK_THEN_OFF_MIN", std::to_string(it->valuedouble < 0.0 ? -1 : clamped(it->valuedouble, 0, 10080)),
		    "black_then_off_min");
	}
	// The Touch controllers' sleep: minutes after the headset is taken off; negative, kept awake all session.
	it = item(root, "controllers", "sleep_after_min");
	if (cJSON_IsNumber(it)) {
		put("RIFT_TOUCH_SLEEP_AFTER_MIN", std::to_string(it->valuedouble < 0.0 ? -1 : clamped(it->valuedouble, 0, 10080)),
		    "controllers.sleep_after_min");
	}
	return set;
}

} // namespace rl::settings
