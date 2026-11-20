// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Panel uniformity switched on from the Rift tab: the headset's calibration file, and the first time (no file
 *         yet) `riftlinux-setup uniformity on --yes` started in the background. It downloads the headset's factory
 *         measurement, builds the calibration and switches it on; the compositor takes the file as it appears
 *         (render_mura_reload_if_changed), so nothing waits for it.
 *
 * Used by the SteamVR driver (inside SteamVR's process) and by the runtime's own panel. The tool runs detached, in a
 * session of its own, without the caller's library paths and with the system's PATH, its output appended to
 * ~/.cache/riftlinux/uniformity-setup.log.
 */

#pragma once

#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <string>
#include <vector>

extern char **environ;

namespace rl::uniformity {

//! Headset @p serial's calibration file, where the compositor looks for it (render_mura.c): empty when unknown.
inline std::string
calibration_path(const std::string &serial)
{
	const char *file = getenv("XRT_COMPOSITOR_MURA_FILE");
	const char *xdg = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	if (file != nullptr && file[0] != '\0') {
		return file;
	}
	if (serial.empty()) {
		return "";
	}
	if (xdg != nullptr && xdg[0] != '\0') {
		return std::string(xdg) + "/riftlinux/spud/HMD_" + serial + ".rlspudc";
	}
	if (home != nullptr && home[0] != '\0') {
		return std::string(home) + "/.config/riftlinux/spud/HMD_" + serial + ".rlspudc";
	}
	return "";
}

//! Whether headset @p serial has a calibration file (false when its serial is not known).
inline bool
have_calibration(const std::string &serial)
{
	const std::string path = calibration_path(serial);
	struct stat st;
	return !path.empty() && stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

//! riftlinux-setup: RIFTLINUX_SETUP, else the package's, else a local install; empty when none is found.
inline std::string
setup_tool()
{
	std::vector<std::string> candidates;
	const char *own = getenv("RIFTLINUX_SETUP");
	if (own != nullptr && own[0] != '\0') {
		candidates.emplace_back(own);
	}
	candidates.emplace_back("/usr/bin/riftlinux-setup");
	candidates.emplace_back("/usr/local/bin/riftlinux-setup");
	const char *home = getenv("HOME");
	if (home != nullptr && home[0] != '\0') {
		candidates.push_back(std::string(home) + "/.local/bin/riftlinux-setup");
	}
	for (const std::string &c : candidates) {
		if (access(c.c_str(), X_OK) == 0) {
			return c;
		}
	}
	return "";
}

//! Where the background setup writes what it says.
inline std::string
log_path()
{
	const char *cache = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	std::string dir;
	if (cache != nullptr && cache[0] != '\0') {
		dir = cache;
	} else if (home != nullptr && home[0] != '\0') {
		dir = std::string(home) + "/.cache";
	} else {
		return "/dev/null";
	}
	mkdir(dir.c_str(), 0755);
	dir += "/riftlinux";
	mkdir(dir.c_str(), 0755);
	return dir + "/uniformity-setup.log";
}

/*!
 * Starts `riftlinux-setup uniformity on --yes` detached. Not again within a minute of the last start (a switch
 * flicked off and on while the download runs). False with @p problem when it did not start.
 */
inline bool
start_setup(std::string &problem)
{
	static std::atomic<int64_t> last_start_s{-1000};
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	const int64_t now_s = static_cast<int64_t>(now.tv_sec);
	if (now_s - last_start_s.load() < 60) {
		problem = "already started less than a minute ago";
		return false;
	}
	std::string tool = setup_tool();
	if (tool.empty()) {
		problem = "riftlinux-setup not found";
		return false;
	}

	// Everything the child needs is made before fork(): the caller has many threads, so the children only set up
	// their descriptors and execute. Steam's library paths and Python settings stay behind.
	std::vector<std::string> env_strings;
	for (char **e = environ; e != nullptr && *e != nullptr; e++) {
		if (strncmp(*e, "LD_PRELOAD=", 11) != 0 && strncmp(*e, "LD_LIBRARY_PATH=", 16) != 0 &&
		    strncmp(*e, "PYTHONHOME=", 11) != 0 && strncmp(*e, "PYTHONPATH=", 11) != 0 &&
		    strncmp(*e, "PATH=", 5) != 0) {
			env_strings.emplace_back(*e);
		}
	}
	env_strings.emplace_back("PATH=/usr/local/bin:/usr/bin:/bin");
	std::vector<char *> envp;
	for (std::string &s : env_strings) {
		envp.push_back(&s[0]);
	}
	envp.push_back(nullptr);
	std::string a1 = "uniformity", a2 = "on", a3 = "--yes";
	char *argv[] = {&tool[0], &a1[0], &a2[0], &a3[0], nullptr};
	const std::string log = log_path();
	const int in = open("/dev/null", O_RDONLY | O_CLOEXEC);
	const int out = open(log.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
	int highest_fd = 3;
	if (DIR *fds = opendir("/proc/self/fd")) {
		while (struct dirent *entry = readdir(fds)) {
			const int fd = atoi(entry->d_name);
			highest_fd = fd > highest_fd ? fd : highest_fd;
		}
		closedir(fds);
	}

	const pid_t pid = fork();
	if (pid == 0) {
		// A grandchild runs the tool: it is not the caller's child (nothing to wait for, it outlives SteamVR).
		if (fork() != 0) {
			_exit(0);
		}
		setsid();
		if (in >= 0) {
			dup2(in, 0);
		}
		if (out >= 0) {
			dup2(out, 1);
			dup2(out, 2);
		}
#ifndef SYS_close_range
#define SYS_close_range 436
#endif
		if (syscall(SYS_close_range, 3u, ~0u, 0u) != 0) {
			for (int fd = 3; fd <= highest_fd; fd++) {
				close(fd);
			}
		}
		execve(argv[0], argv, envp.data());
		_exit(127);
	}
	if (in >= 0) {
		close(in);
	}
	if (out >= 0) {
		close(out);
	}
	if (pid < 0) {
		problem = std::string("could not start riftlinux-setup (") + strerror(errno) + ")";
		return false;
	}
	int status = 0;
	while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {
	}
	last_start_s.store(now_s);
	problem = log;
	return true;
}

/*!
 * Panel uniformity was just switched on for headset @p serial: without a calibration file, the background setup.
 * Returns what to log (empty when the calibration is already there).
 */
inline std::string
switched_on(const std::string &serial)
{
	if (have_calibration(serial)) {
		return "";
	}
	std::string said;
	if (start_setup(said)) {
		return "no calibration for headset " + (serial.empty() ? std::string("(serial not known)") : serial) +
		       " yet: riftlinux-setup uniformity on started in the background (" + said + ")";
	}
	return "no calibration for headset " + (serial.empty() ? std::string("(serial not known)") : serial) +
	       " yet, setup not started: " + said;
}

} // namespace rl::uniformity
