// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The Rift CV1's SteamVR watchdog: it asks Steam to start SteamVR when the headset is put on or picked up,
 *         as the Oculus app starts when a Rift is put on.
 *
 * Steam loads a driver's watchdog while SteamVR is not running, in Steam's own process, which is 32-bit on Linux
 * (so the driver also ships a 32-bit build of just this, bin/linux32/driver_monado.so). It runs after SteamVR has
 * quit. Without a watchdog Steam has nothing that tells it a headset wants SteamVR, and a game started from Steam
 * runs on the desktop instead.
 *
 * The headset reports its wear sensor and gyro about a thousand times a second while it is sent the keep-alive the
 * driver sends; the display stays off. SteamVR is started when the wear sensor reads "worn" for a moment after it
 * read "not worn", or when the headset turns as it does in hands after it lay still. Neither counts right after
 * SteamVR quits with the headset still on (it has to be taken off first, or put down), and nothing is done while
 * SteamVR or RiftLinux's own runtime uses the headset. A SteamVR started this way quits again when the headset is not
 * put on (start_note_path).
 *
 * Off with "steamvr": {"start_when_put_on": false} in ~/.config/riftlinux/settings.json; "start_on_pick_up": false
 * leaves only putting it on (the wear sensor at the levels the driver uses, "headset"). A SteamVR started by a
 * pick-up that is not followed by putting it on quits again unless "quit_if_not_put_on" is false (the driver).
 *
 * Header-only and free of Monado so the 32-bit build needs nothing else.
 */

#pragma once

#include "openvr_driver.h"

#include "riftlinux/rl_steam.hpp"

#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/wait.h>

#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <limits.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>
#include <thread>
#include <vector>

extern char **environ;

namespace rl::watchdog {

//! Wear sensor above this reads "worn", below the second "not worn" (as the driver: 84-90 worn, 2-9 lying).
constexpr int kWornAbove = 30;
constexpr int kUnwornBelow = 15;
//! Worn this long, after "not worn" this long, starts SteamVR.
constexpr int64_t kWornForMs = 300;
constexpr int64_t kUnwornBeforeMs = 2000;
//! A 50 ms block "moves" when its mean turn rate is above this (lying still: about 1 deg/s, sample peaks 2.5).
constexpr double kMovingDegS = 20.0;
constexpr double kDegPerRad = 57.29577951308232;
constexpr int64_t kBlockMs = 50;
//! This many moving blocks within the last 20 (one second) start SteamVR, after this long without one.
constexpr int kMovingBlocks = 4;
constexpr int64_t kStillBeforeMs = 5000;
//! After asking, nothing more for this long (SteamVR starting, or Steam not starting it).
constexpr int64_t kAfterWakeMs = 60000;
//! The headset streams its reports for 10 s after each keep-alive; sent again after 9.5 s, as the driver does.
constexpr int64_t kKeepAliveMs = 9500;

inline int64_t
now_ms()
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
	    .count();
}

inline void
log(const char *fmt, ...)
{
	char buf[512];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf) - 1, fmt, args);
	va_end(args);
	strncat(buf, "\n", sizeof(buf) - strlen(buf) - 1);
	vr::IVRDriverLog *driver_log = vr::VRDriverLog();
	if (driver_log != nullptr) {
		driver_log->Log(buf);
	} else {
		fputs(buf, stderr);
	}
}

inline std::string
read_text(const std::string &path, size_t max = 65536)
{
	std::string text;
	FILE *file = fopen(path.c_str(), "r");
	if (file == nullptr) {
		return text;
	}
	char buf[4096];
	size_t n;
	while ((n = fread(buf, 1, sizeof(buf), file)) > 0 && text.size() < max) {
		text.append(buf, n);
	}
	fclose(file);
	return text;
}

/*!
 * The note that the headset started SteamVR ("<seconds since 1970> <what happened>"), for the driver as SteamVR
 * starts: a headset only picked up to be put away started SteamVR, Home and the headset's sound for nobody, so a
 * SteamVR the headset started quits again when it is not put on (ovrd_driver.cpp).
 */
inline std::string
start_note_path()
{
	const char *xdg = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	const std::string dir =
	    xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "/tmp") + "/.cache";
	mkdir(dir.c_str(), 0700);
	mkdir((dir + "/riftlinux").c_str(), 0700);
	return dir + "/riftlinux/started-by-headset";
}

inline void
note_start(const char *what)
{
	FILE *file = fopen(start_note_path().c_str(), "w");
	if (file != nullptr) {
		fprintf(file, "%lld %s\n", (long long)time(nullptr), what);
		fclose(file);
	}
}

/*!
 * The note that the driver closed SteamVR because its compositor stopped showing frames (its GPU device lost): the
 * watchdog starts SteamVR again once it is gone, instead of leaving the headset frozen until someone kills it. At most
 * once in ten minutes (a fault that comes back at every start is not restarted again and again).
 */
inline std::string
restart_note_path()
{
	const std::string start = start_note_path();
	return start.substr(0, start.rfind('/')) + "/restart-steamvr";
}

//! Leaves the note (driver); false if SteamVR was restarted this way in the last ten minutes.
inline bool
note_restart(const char *why)
{
	const std::string last = restart_note_path() + ".last";
	struct stat st = {};
	if (stat(last.c_str(), &st) == 0 && time(nullptr) - st.st_mtime < 600) {
		return false;
	}
	for (const std::string &path : {restart_note_path(), last}) {
		FILE *file = fopen(path.c_str(), "w");
		if (file != nullptr) {
			fprintf(file, "%lld %s\n", (long long)time(nullptr), why);
			fclose(file);
		}
	}
	return true;
}

//! Takes a note left in the last two minutes (watchdog); an older one is dropped.
inline bool
take_restart_note()
{
	const std::string path = restart_note_path();
	struct stat st = {};
	if (stat(path.c_str(), &st) != 0) {
		return false;
	}
	unlink(path.c_str());
	return time(nullptr) - st.st_mtime < 120;
}

/*!
 * The watchdog's settings, from ~/.config/riftlinux/settings.json once a second. A reader for just these keys (each
 * appears once in RiftLinux's own file): the 32-bit build has no JSON library.
 */
struct Settings
{
	//! "steamvr": {"start_when_put_on"}: off, nothing starts SteamVR.
	bool on = true;
	//! "steamvr": {"start_on_pick_up"}: off, only putting the headset on does (the wear sensor).
	bool pick_up = true;
	//! "headset": the wear sensor and its levels, as the driver reads them; with the sensor off, put-on cannot be seen
	//! and picking up starts SteamVR whatever start_on_pick_up says.
	bool wear_sensor = true;
	int worn_above = kWornAbove;
	int unworn_below = kUnwornBelow;

	bool
	operator!=(const Settings &o) const
	{
		return on != o.on || pick_up != o.pick_up || wear_sensor != o.wear_sensor || worn_above != o.worn_above ||
		       unworn_below != o.unworn_below;
	}
};

//! Where @p key's value starts in @p text, or npos.
inline size_t
setting_at(const std::string &text, const char *key)
{
	const std::string quoted = std::string("\"") + key + "\"";
	const size_t at = text.find(quoted);
	if (at == std::string::npos) {
		return std::string::npos;
	}
	size_t i = at + quoted.size();
	while (i < text.size() && (text[i] == ' ' || text[i] == '\t' || text[i] == ':' || text[i] == '\n' ||
	                           text[i] == '\r')) {
		i++;
	}
	return i;
}

inline bool
setting_bool(const std::string &text, const char *key, bool fallback)
{
	const size_t i = setting_at(text, key);
	if (i == std::string::npos) {
		return fallback;
	}
	return text.compare(i, 5, "false") != 0 && text.compare(i, 1, "0") != 0;
}

inline int
setting_int(const std::string &text, const char *key, int fallback)
{
	const size_t i = setting_at(text, key);
	if (i == std::string::npos || i >= text.size()) {
		return fallback;
	}
	char *end = nullptr;
	const long v = strtol(text.c_str() + i, &end, 10);
	return end != text.c_str() + i ? (int)v : fallback;
}

inline Settings
read_settings()
{
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	const std::string dir =
	    xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "") + "/.config";
	const std::string text = read_text(dir + "/riftlinux/settings.json");
	Settings s;
	s.on = setting_bool(text, "start_when_put_on", true);
	s.pick_up = setting_bool(text, "start_on_pick_up", true);
	s.wear_sensor = setting_bool(text, "wear_sensor", true);
	s.worn_above = setting_int(text, "worn_above", kWornAbove);
	s.unworn_below = setting_int(text, "unworn_below", kUnwornBelow);
	// Levels that cross each other fall back to the defaults, as in the driver.
	if (s.unworn_below > s.worn_above || s.unworn_below < 0) {
		s.worn_above = kWornAbove;
		s.unworn_below = kUnwornBelow;
	}
	return s;
}

//! SteamVR's server or RiftLinux's own runtime is running: the headset is theirs.
inline bool
headset_in_use()
{
	DIR *proc = opendir("/proc");
	if (proc == nullptr) {
		return false;
	}
	bool busy = false;
	struct dirent *entry;
	while (!busy && (entry = readdir(proc)) != nullptr) {
		if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
			continue;
		}
		const std::string link = std::string("/proc/") + entry->d_name + "/exe";
		char exe[PATH_MAX];
		const ssize_t len = readlink(link.c_str(), exe, sizeof(exe) - 1);
		if (len <= 0) {
			continue;
		}
		exe[len] = '\0';
		const char *base = strrchr(exe, '/');
		base = base != nullptr ? base + 1 : exe;
		busy = strcmp(base, "vrserver") == 0 || strcmp(base, "monado-service") == 0;
	}
	closedir(proc);
	return busy;
}

/*!
 * SteamVR's server that finished its shutdown but never ended: "[vrserver] Graceful exit" is the last line of its log
 * and one thread is left, waiting forever on a lock in SteamVR's own code (seen with SteamVR 2.18; a stack taken from
 * one showed only vrserver's code). It is a copy of the server that froze as it was made to start SteamVR's
 * compositor; it stays after the server ended too, also when the driver ended a server whose shutdown stuck (its log
 * then ends with the driver's line). It keeps SteamVR's namespace, so SteamVR cannot start again until it is gone.
 * Returns the process id of one that has stayed so while no running server is left, else 0.
 */
inline pid_t
finished_vrserver()
{
	DIR *proc = opendir("/proc");
	if (proc == nullptr) {
		return 0;
	}
	pid_t found = 0;
	bool running = false;
	struct dirent *entry;
	while (!running && (entry = readdir(proc)) != nullptr) {
		if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
			continue;
		}
		const std::string dir = std::string("/proc/") + entry->d_name;
		char exe[PATH_MAX];
		const ssize_t len = readlink((dir + "/exe").c_str(), exe, sizeof(exe) - 1);
		if (len <= 0) {
			continue;
		}
		exe[len] = '\0';
		const char *base = strrchr(exe, '/');
		if (base == nullptr || strcmp(base + 1, "vrserver") != 0) {
			continue;
		}
		int threads = -1;
		if (FILE *f = fopen((dir + "/status").c_str(), "r")) {
			char line[256];
			while (fgets(line, sizeof(line), f) != nullptr) {
				if (strncmp(line, "Threads:", 8) == 0) {
					threads = atoi(line + 8);
				}
			}
			fclose(f);
		}
		if (threads == 1) {
			found = (pid_t)atoi(entry->d_name);
		} else if (threads > 1) {
			running = true;
		}
	}
	closedir(proc);
	if (found == 0 || running) {
		return 0;
	}
	// Its log ends with the shutdown's last line, written a while ago.
	const std::string log_path = rl::steam::steamvr_log_dir() + "/vrserver.txt";
	struct stat st = {};
	if (stat(log_path.c_str(), &st) != 0 || time(nullptr) - st.st_mtime < 10) {
		return 0;
	}
	std::string tail;
	if (FILE *f = fopen(log_path.c_str(), "r")) {
		const long size = st.st_size;
		fseek(f, size > 4096 ? size - 4096 : 0, SEEK_SET);
		char buf[4096];
		const size_t n = fread(buf, 1, sizeof(buf), f);
		tail.assign(buf, n);
		fclose(f);
	}
	while (!tail.empty() && (tail.back() == '\n' || tail.back() == '\r' || tail.back() == ' ')) {
		tail.pop_back();
	}
	const size_t last = tail.rfind('\n');
	const std::string line = last == std::string::npos ? tail : tail.substr(last + 1);
	// The driver's line (ovrd_driver.cpp) can be followed by a few of SteamVR's in the half second before it ends.
	const bool ended_by_driver = tail.find("this driver ends SteamVR's server") != std::string::npos;
	return line.find("[vrserver] Graceful exit") != std::string::npos || ended_by_driver ? found : 0;
}

/*!
 * SteamVR's monitor (vrmonitor, beside the server running this code), ended once this server has gone. When the driver
 * ends a server whose shutdown is stuck (ovrd_driver.cpp), the monitor can go on waiting for it for good, and while it
 * runs Steam counts SteamVR as running: Steam loads no watchdog and starts no SteamVR. Ending the monitor while the
 * server still runs makes the server quit every client at once ("Lost master process"), so a child waits for the
 * server to go first; forked from a threaded process, it makes system calls only. Returns the monitor's process id,
 * 0 if none.
 */
inline pid_t
end_monitor_after_server()
{
	char self[PATH_MAX];
	const ssize_t len = readlink("/proc/self/exe", self, sizeof(self) - 1);
	if (len <= 0) {
		return 0;
	}
	self[len] = '\0';
	std::string monitor_exe(self);
	monitor_exe = monitor_exe.substr(0, monitor_exe.rfind('/') + 1) + "vrmonitor";
	pid_t monitor = 0;
	DIR *proc = opendir("/proc");
	if (proc == nullptr) {
		return 0;
	}
	struct dirent *entry;
	while (monitor == 0 && (entry = readdir(proc)) != nullptr) {
		if (entry->d_name[0] < '0' || entry->d_name[0] > '9') {
			continue;
		}
		const std::string link = std::string("/proc/") + entry->d_name + "/exe";
		char exe[PATH_MAX];
		const ssize_t n = readlink(link.c_str(), exe, sizeof(exe) - 1);
		if (n <= 0) {
			continue;
		}
		exe[n] = '\0';
		if (monitor_exe == exe) {
			monitor = (pid_t)atoi(entry->d_name);
		}
	}
	closedir(proc);
	if (monitor == 0) {
		return 0;
	}
	char server_stat[64];
	snprintf(server_stat, sizeof(server_stat), "/proc/%d/stat", (int)getpid());

	const pid_t child = fork();
	if (child != 0) {
		return child > 0 ? monitor : 0;
	}
	setsid();
#ifndef SYS_close_range
#define SYS_close_range 436
#endif
	if (syscall(SYS_close_range, 3u, ~0u, 0u) != 0) {
		for (int fd = 3; fd < 4096; fd++) {
			close(fd);
		}
	}
	// The server: gone, or ended and not yet reaped, within 10 s; else the monitor is left alone.
	const struct timespec step = {0, 100 * 1000 * 1000};
	bool server_gone = false;
	for (int i = 0; i < 100 && !server_gone; i++) {
		const int fd = open(server_stat, O_RDONLY | O_CLOEXEC);
		char buf[512];
		const ssize_t n = fd >= 0 ? read(fd, buf, sizeof(buf) - 1) : -1;
		if (fd >= 0) {
			close(fd);
		}
		if (n <= 0) {
			server_gone = true;
			break;
		}
		buf[n] = '\0';
		const char *end = strrchr(buf, ')');
		server_gone = end != nullptr && end[1] == ' ' && (end[2] == 'Z' || end[2] == 'X');
		if (!server_gone) {
			nanosleep(&step, nullptr);
		}
	}
	if (server_gone) {
		kill(monitor, SIGTERM);
		for (int i = 0; i < 30 && kill(monitor, 0) == 0; i++) {
			nanosleep(&step, nullptr);
		}
		kill(monitor, SIGKILL);
	}
	_exit(0);
}

//! The headset's sensor interface (USB 2833:0031, interface 0) as /dev/hidrawN, or empty.
inline std::string
find_headset()
{
	DIR *dir = opendir("/sys/class/hidraw");
	if (dir == nullptr) {
		return {};
	}
	std::string found;
	struct dirent *entry;
	while (found.empty() && (entry = readdir(dir)) != nullptr) {
		if (strncmp(entry->d_name, "hidraw", 6) != 0) {
			continue;
		}
		const std::string base = std::string("/sys/class/hidraw/") + entry->d_name + "/device";
		if (read_text(base + "/uevent").find("HID_ID=0003:00002833:00000031") == std::string::npos) {
			continue;
		}
		char real[PATH_MAX];
		if (realpath(base.c_str(), real) == nullptr) {
			continue;
		}
		std::string intf(real);
		intf = intf.substr(0, intf.rfind('/'));
		if (read_text(intf + "/bInterfaceNumber").compare(0, 2, "00") == 0) {
			found = std::string("/dev/") + entry->d_name;
		}
	}
	closedir(dir);
	return found;
}

//! The keep-alive the driver sends (feature report 0x11: sensor reports, 10 s).
inline bool
send_keep_alive(int fd)
{
	uint8_t report[6] = {0x11, 0x00, 0x00, 0x0b, 0x10, 0x27};
	return ioctl(fd, HIDIOCSFEATURE(sizeof(report)), report) >= 0;
}

/*!
 * The headset's panel switched off when nothing runs it: a SteamVR that crashed, was killed or whose shutdown was ended
 * by force skips its driver's own shutdown and leaves the panel lit (seen: the display on with nobody wearing it),
 * and an OLED showing a still picture burns in. Feature report 0x1d, components: 1 display, 2 audio, 4 LEDs; only
 * the display bit is cleared. True when it was on.
 */
inline bool
display_off_if_on(int fd)
{
	uint8_t report[8] = {0x1d};
	if (ioctl(fd, HIDIOCGFEATURE(sizeof(report)), report) < 0 || (report[3] & 0x01) == 0) {
		return false;
	}
	uint8_t off[4] = {0x1d, 0x00, 0x00, (uint8_t)(report[3] & ~0x01u)};
	return ioctl(fd, HIDIOCSFEATURE(sizeof(off)), off) >= 0;
}

/*!
 * Starts SteamVR as a click on it in Steam does: Steam's own tool (steam-runtime-steam-remote, what the steam command
 * uses when Steam runs) hands steam://rungameid/250820 to the running Steam, and Steam launches SteamVR through its
 * normal launch path. Steam's watchdog start instead runs SteamVR from inside Steam's process, with all Steam inherited
 * from the desktop: GNOME Shell on NVIDIA was seen holding 125000 leaked GPU fence handles without close-on-exec
 * after screen recordings, Steam started from the desktop carried 120000 of them, and SteamVR Home started that way
 * could open no more files and crashed. The tool runs as a fresh child with only standard input and output and
 * without Steam's library paths. Returns false when the tool is missing or fails (then Steam's own start is used).
 */
inline bool
start_steamvr_like_a_click()
{
	// Steam's own folder (this runs in Steam's process: .../ubuntu12_32/steam) first, then ~/.steam.
	std::vector<std::string> steam_dirs;
	char exe[PATH_MAX];
	const ssize_t len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
	if (len > 0) {
		exe[len] = '\0';
		std::string dir(exe);
		dir = dir.substr(0, dir.rfind('/'));                   // .../ubuntu12_32 (or ubuntu12_64)
		const std::string root = dir.substr(0, dir.rfind('/')); // Steam's folder
		steam_dirs.push_back(dir);
		steam_dirs.push_back(root + "/ubuntu12_32");
	}
	std::string problem;
	const bool ok = rl::steam::open_url("steam://rungameid/250820", problem, steam_dirs);
	if (!problem.empty()) {
		log("RiftLinux watchdog: %s", problem.c_str());
	}
	return ok;
}

//! One gyro sample of a sensor report: three 21-bit values, 1e-4 rad/s each.
inline void
unpack_gyro(const uint8_t *in, double out[3])
{
	const uint32_t x = (uint32_t(in[0]) << 24) | (uint32_t(in[1]) << 16) | (uint32_t(in[2] & 0xF8) << 8);
	const uint32_t y = (uint32_t(in[2] & 0x07) << 29) | (uint32_t(in[3]) << 21) | (uint32_t(in[4]) << 13) |
	                   (uint32_t(in[5] & 0xC0) << 5);
	const uint32_t z = (uint32_t(in[5] & 0x3F) << 26) | (uint32_t(in[6]) << 18) | (uint32_t(in[7]) << 10);
	out[0] = 1e-4 * double(int32_t(x) >> 11);
	out[1] = 1e-4 * double(int32_t(y) >> 11);
	out[2] = 1e-4 * double(int32_t(z) >> 11);
}

class Watchdog
{
public:
	void
	start()
	{
		stop();
		m_stop = false;
		m_thread = std::thread([this] { run(); });
	}

	void
	stop()
	{
		m_stop = true;
		if (m_thread.joinable()) {
			m_thread.join();
		}
	}

	~Watchdog()
	{
		stop();
	}

private:
	std::atomic<bool> m_stop{true};
	std::thread m_thread;

	int m_fd = -1;
	int64_t m_keep_alive_ms = 0;
	//! Whether SteamVR or the runtime ran at the last check, and when to check the panel (display_off_if_on).
	bool m_was_in_use = false;
	int64_t m_check_panel_ms = 0;
	//! A vrserver that finished its shutdown but did not end, and since when (finished_vrserver).
	pid_t m_finished_pid = 0;
	int64_t m_finished_since_ms = 0;
	int64_t m_unworn_since_ms = 0;
	int64_t m_worn_since_ms = 0;
	int64_t m_last_moving_ms = 0;
	int64_t m_block_start_ms = 0;
	double m_block_sum[3] = {0, 0, 0};
	int m_block_samples = 0;
	uint32_t m_moving_bits = 0;
	bool m_in_burst = false;
	//! As last read (run, once a second).
	Settings m_settings;

	void
	close_headset()
	{
		if (m_fd >= 0) {
			close(m_fd);
			m_fd = -1;
		}
	}

	//! Arming starts over: put-on needs "not worn" first, pick-up needs the headset still first.
	void
	reset(int64_t now)
	{
		m_unworn_since_ms = 0;
		m_worn_since_ms = 0;
		m_last_moving_ms = now;
		m_block_start_ms = now;
		m_block_sum[0] = m_block_sum[1] = m_block_sum[2] = 0;
		m_block_samples = 0;
		m_moving_bits = 0;
		m_in_burst = false;
	}

	void
	sleep_ms(int64_t ms)
	{
		for (int64_t t = 0; t < ms && !m_stop; t += 100) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	//! Returns the reason SteamVR should start, or nullptr.
	const char *
	handle_report(const uint8_t *buf, ssize_t len, int64_t now, int *wear_out)
	{
		if (len < 64 || buf[0] != 0x0b) {
			return nullptr;
		}
		const int wear = buf[44] | (buf[45] << 8);
		*wear_out = wear;
		if (!m_settings.wear_sensor) {
			// Put-on cannot be seen.
		} else if (wear < m_settings.unworn_below) {
			m_worn_since_ms = 0;
			if (m_unworn_since_ms == 0) {
				m_unworn_since_ms = now;
			}
		} else if (wear > m_settings.worn_above) {
			const bool armed = m_unworn_since_ms != 0 && now - m_unworn_since_ms >= kUnwornBeforeMs;
			if (!armed) {
				m_unworn_since_ms = 0;
			} else if (m_worn_since_ms == 0) {
				m_worn_since_ms = now;
			} else if (now - m_worn_since_ms >= kWornForMs) {
				return "put on";
			}
		}

		const int samples = buf[3] < 2 ? buf[3] : 2;
		for (int i = 0; i < samples; i++) {
			double gyro[3];
			unpack_gyro(buf + 12 + 16 * i + 8, gyro);
			for (int k = 0; k < 3; k++) {
				m_block_sum[k] += gyro[k];
			}
			m_block_samples++;
		}
		if (now - m_block_start_ms < kBlockMs) {
			return nullptr;
		}
		bool moving = false;
		if (m_block_samples > 0) {
			const double n = m_block_samples;
			const double rate = std::sqrt(m_block_sum[0] * m_block_sum[0] + m_block_sum[1] * m_block_sum[1] +
			                              m_block_sum[2] * m_block_sum[2]) /
			                    n;
			moving = rate * kDegPerRad > kMovingDegS;
		}
		m_block_start_ms = now;
		m_block_sum[0] = m_block_sum[1] = m_block_sum[2] = 0;
		m_block_samples = 0;
		m_moving_bits = ((m_moving_bits << 1) | (moving ? 1u : 0u)) & 0xFFFFFu;
		const bool armed = now - m_last_moving_ms >= kStillBeforeMs;
		if (moving && !armed) {
			// Still moving from before (taken off, put down): not a pick-up yet.
			m_last_moving_ms = now;
			m_moving_bits = 0;
			return nullptr;
		}
		if (__builtin_popcount(m_moving_bits) >= kMovingBlocks && (m_settings.pick_up || !m_settings.wear_sensor)) {
			return "picked up";
		}
		if (moving) {
			// A pick-up starting: the still time before it stands until the turn is over.
			m_in_burst = true;
		} else if (m_in_burst && m_moving_bits == 0) {
			// A knock that was no pick-up: the headset has to lie still again first.
			m_in_burst = false;
			m_last_moving_ms = now;
		}
		return nullptr;
	}

	void
	run()
	{
		log("RiftLinux watchdog: SteamVR starts when the Rift is put on or picked up, as a click on it in Steam does");
		m_settings = read_settings();
		bool told_settings = false;
		int64_t now = now_ms();
		int64_t next_check_ms = 0;
		int64_t next_find_ms = 0;
		bool told_missing = false;
		bool told_off = false;
		reset(now);
		m_check_panel_ms = now + 5000;

		while (!m_stop) {
			now = now_ms();
			if (now >= next_check_ms) {
				next_check_ms = now + 1000;
				const Settings read = read_settings();
				if (read != m_settings || !told_settings) {
					told_settings = true;
					if (read.on && !read.wear_sensor) {
						log("RiftLinux watchdog: the wear sensor is off (headset.wear_sensor): picking the Rift up "
						    "starts SteamVR");
					} else if (read.on && !read.pick_up) {
						log("RiftLinux watchdog: only putting the Rift on starts SteamVR (steamvr.start_on_pick_up)");
					} else if (read.on && (read.worn_above != kWornAbove || read.unworn_below != kUnwornBelow)) {
						log("RiftLinux watchdog: wear sensor levels %d and %d (headset.worn_above, unworn_below)",
						    read.worn_above, read.unworn_below);
					}
				}
				m_settings = read;
				const bool on = m_settings.on;
				// The panel left lit once SteamVR or the runtime has gone (display_off_if_on): checked 3 s after (the
				// driver's own shutdown switches it off first), and once at the watchdog's start.
				const bool in_use = headset_in_use();
				if (m_was_in_use && !in_use) {
					m_check_panel_ms = now + 3000;
				}
				m_was_in_use = in_use;
				if (m_check_panel_ms != 0 && now >= m_check_panel_ms && !in_use) {
					m_check_panel_ms = 0;
					const std::string path = m_fd >= 0 ? std::string() : find_headset();
					const int fd = m_fd >= 0 ? m_fd : (path.empty() ? -1 : open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC));
					if (fd >= 0) {
						if (display_off_if_on(fd)) {
							log("RiftLinux watchdog: the headset's display was left on with nothing running it: switched off");
						}
						if (fd != m_fd) {
							close(fd);
						}
					}
				}
				// SteamVR's server finished but did not end (see finished_vrserver): after 10 s more, ended.
				const pid_t finished = finished_vrserver();
				if (finished != 0 && finished == m_finished_pid && now - m_finished_since_ms >= 10000) {
					const bool ended = kill(finished, SIGKILL) == 0;
					log("RiftLinux watchdog: SteamVR's server (process %d) finished its shutdown but did not end; %s",
					    (int)finished, ended ? "ended it, so SteamVR can start again" : "could not end it");
					m_finished_pid = 0;
				} else if (finished != m_finished_pid) {
					m_finished_pid = finished;
					m_finished_since_ms = now;
				}
				// SteamVR the driver closed because its compositor stopped: started again once it is gone.
				if (on && !headset_in_use() && take_restart_note()) {
					log("RiftLinux watchdog: SteamVR was closed because its compositor stopped showing frames: "
					    "asking Steam to start it again");
					close_headset();
					note_start("restarted after its compositor stopped");
					if (!start_steamvr_like_a_click()) {
						vr::IVRWatchdogHost *host = vr::VRWatchdogHost();
						if (host != nullptr) {
							host->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
						}
					}
					sleep_ms(kAfterWakeMs);
					reset(now_ms());
					continue;
				}
				if (!on && !told_off) {
					log("RiftLinux watchdog: off in settings (steamvr.start_when_put_on)");
				}
				told_off = !on;
				if (!on || headset_in_use()) {
					close_headset();
					reset(now);
					sleep_ms(1000);
					continue;
				}
			}
			if (m_fd < 0) {
				if (now < next_find_ms) {
					sleep_ms(200);
					continue;
				}
				const std::string path = find_headset();
				m_fd = path.empty() ? -1 : open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
				if (m_fd < 0) {
					if (!told_missing) {
						log("RiftLinux watchdog: no Rift headset to watch (checked again every 2 s)");
						told_missing = true;
					}
					next_find_ms = now + 2000;
					continue;
				}
				told_missing = false;
				m_keep_alive_ms = 0;
				reset(now);
				log("RiftLinux watchdog: watching %s", path.c_str());
			}
			if (now - m_keep_alive_ms >= kKeepAliveMs) {
				if (!send_keep_alive(m_fd)) {
					log("RiftLinux watchdog: the headset took no keep-alive (%s); looking for it again",
					    strerror(errno));
					close_headset();
					next_find_ms = now + 2000;
					continue;
				}
				m_keep_alive_ms = now;
			}

			fd_set fds;
			FD_ZERO(&fds);
			FD_SET(m_fd, &fds);
			struct timeval tv = {0, 200000};
			const int ready = select(m_fd + 1, &fds, nullptr, nullptr, &tv);
			if (ready <= 0) {
				continue;
			}
			uint8_t buf[64];
			const ssize_t len = read(m_fd, buf, sizeof(buf));
			if (len < 0) {
				if (errno != EAGAIN && errno != EINTR) {
					log("RiftLinux watchdog: reading the headset failed (%s); looking for it again",
					    strerror(errno));
					close_headset();
					next_find_ms = now + 2000;
				}
				continue;
			}
			int wear = -1;
			const char *reason = handle_report(buf, len, now_ms(), &wear);
			if (reason == nullptr) {
				continue;
			}
			log("RiftLinux watchdog: the Rift was %s (wear sensor %d): asking Steam to start SteamVR", reason,
			    wear);
			close_headset();
			note_start(reason);
			if (!start_steamvr_like_a_click()) {
				vr::IVRWatchdogHost *host = vr::VRWatchdogHost();
				if (host != nullptr) {
					log("RiftLinux watchdog: using Steam's watchdog start instead");
					host->WatchdogWakeUp(vr::TrackedDeviceClass_HMD);
				}
			}
			sleep_ms(kAfterWakeMs);
			reset(now_ms());
		}
		close_headset();
		log("RiftLinux watchdog: stopped");
	}
};

} // namespace rl::watchdog
