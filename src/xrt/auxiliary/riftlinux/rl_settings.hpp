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

#include <cstdio>
#include <cstdlib>
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

//! The file parsed, or NULL (none, unreadable, not JSON); free with cJSON_Delete.
inline cJSON *
load()
{
	FILE *file = fopen(path().c_str(), "r");
	if (file == nullptr) {
		return nullptr;
	}
	std::string text;
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0 && text.size() < 65536) {
		text.append(buf, n);
	}
	fclose(file);
	return cJSON_Parse(text.c_str());
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

//! Sets @p section.@p key to @p value (taken) in the file, keeping everything else in it; written whole, then renamed.
inline void
set_item(const char *section, const char *key, cJSON *value)
{
	// Read, changed, written whole: one writer at a time in this process, or a change made at the same moment is lost.
	static std::mutex write_lock;
	std::lock_guard<std::mutex> lock(write_lock);
	cJSON *root = load();
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

} // namespace rl::settings
