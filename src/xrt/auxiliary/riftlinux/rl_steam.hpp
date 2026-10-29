// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Hands a steam:// link to the running Steam, as a click in Steam does: Steam's own tool
 *         (steam-runtime-steam-remote, what the steam command uses when Steam runs) writes it to Steam, and Steam
 *         starts the game or SteamVR through its normal launch path (its runtime, Proton for Windows games). Also
 *         where SteamVR writes its logs.
 *
 * Used by the watchdog inside Steam's process and by riftlinux-notices inside Valve's runtime container: the tool and
 * Steam's pipe are in the home directory, which both see. The tool runs as a fresh child with only standard input and
 * output and without the caller's library paths (Steam's process carries every descriptor it inherited from the
 * desktop, see ovrd_watchdog.hpp).
 *
 * Header-only and free of Monado so the 32-bit watchdog build needs nothing else.
 */

#pragma once

#include <sys/syscall.h>
#include <sys/wait.h>

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

extern char **environ;

namespace rl::steam {

/*!
 * Steam's tool, from Steam's folders (@p steam_dirs: Steam's own ubuntu12_32 directory first when known) and
 * ~/.steam; empty when none is found.
 */
inline std::string
remote_tool(const std::vector<std::string> &steam_dirs = {})
{
	std::vector<std::string> candidates;
	for (const std::string &dir : steam_dirs) {
		candidates.push_back(dir + "/steam-runtime/amd64/usr/bin/steam-runtime-steam-remote");
	}
	const char *home = getenv("HOME");
	if (home != nullptr) {
		for (const char *root : {"/.steam/root", "/.steam/steam"}) {
			candidates.push_back(std::string(home) + root +
			                     "/ubuntu12_32/steam-runtime/amd64/usr/bin/steam-runtime-steam-remote");
		}
	}
	for (const std::string &c : candidates) {
		if (access(c.c_str(), X_OK) == 0) {
			return c;
		}
	}
	return "";
}

/*!
 * Hands @p link to Steam. Waits up to @p wait_ms for the tool to finish; true when it took the link (or is still
 * busy after that). On false @p problem says why.
 */
inline bool
open_url(const std::string &link,
         std::string &problem,
         const std::vector<std::string> &steam_dirs = {},
         int wait_ms = 10000)
{
	std::string tool = remote_tool(steam_dirs);
	if (tool.empty()) {
		problem = "Steam's steam-runtime-steam-remote not found";
		return false;
	}

	// Everything the child needs is made before fork(): the caller may have many threads, so the child only closes
	// descriptors and executes.
	std::vector<std::string> env_strings;
	for (char **e = environ; e != nullptr && *e != nullptr; e++) {
		if (strncmp(*e, "LD_PRELOAD=", 11) != 0 && strncmp(*e, "LD_LIBRARY_PATH=", 16) != 0) {
			env_strings.emplace_back(*e);
		}
	}
	std::vector<char *> envp;
	for (std::string &s : env_strings) {
		envp.push_back(&s[0]);
	}
	envp.push_back(nullptr);
	std::string arg = link;
	char *argv[] = {&tool[0], &arg[0], nullptr};
	int highest_fd = 3;
	if (DIR *fds = opendir("/proc/self/fd")) {
		while (struct dirent *entry = readdir(fds)) {
			const int fd = atoi(entry->d_name);
			highest_fd = fd > highest_fd ? fd : highest_fd;
		}
		closedir(fds);
	}
	const int dev_null = open("/dev/null", O_RDWR | O_CLOEXEC);

	const pid_t pid = fork();
	if (pid < 0) {
		if (dev_null >= 0) {
			close(dev_null);
		}
		problem = std::string("could not start Steam's tool (") + strerror(errno) + ")";
		return false;
	}
	if (pid == 0) {
		if (dev_null >= 0) {
			dup2(dev_null, 0);
			dup2(dev_null, 1);
			dup2(dev_null, 2);
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
	if (dev_null >= 0) {
		close(dev_null);
	}
	int status = 0;
	for (int waited = 0; waited < wait_ms; waited += 50) {
		const pid_t done = waitpid(pid, &status, WNOHANG);
		if (done == pid) {
			if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
				return true;
			}
			problem = "Steam's tool did not take the link (status " + std::to_string(status) + ")";
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	problem = "Steam's tool did not finish in " + std::to_string(wait_ms / 1000) + " s; Steam may still take it";
	return true;
}

//! SteamVR's log folder: openvrpaths.vrpath's "log" entry, else Steam's usual one.
inline std::string
steamvr_log_dir()
{
	const char *config = getenv("XDG_CONFIG_HOME");
	const char *home = getenv("HOME");
	const std::string base = config != nullptr && config[0] == '/'
	                             ? std::string(config)
	                             : std::string(home != nullptr ? home : "") + "/.config";
	std::string text;
	if (FILE *f = fopen((base + "/openvr/openvrpaths.vrpath").c_str(), "r")) {
		char buf[4096];
		size_t n;
		while ((n = fread(buf, 1, sizeof(buf), f)) > 0 && text.size() < 65536) {
			text.append(buf, n);
		}
		fclose(f);
	}
	size_t at = text.find("\"log\"");
	at = at == std::string::npos ? at : text.find('[', at);
	at = at == std::string::npos ? at : text.find('"', at);
	if (at != std::string::npos) {
		const size_t end = text.find('"', at + 1);
		if (end != std::string::npos) {
			std::string dir;
			for (size_t i = at + 1; i < end; i++) {
				if (text[i] == '\\' && i + 1 < end) {
					i++;
				}
				dir += text[i];
			}
			return dir;
		}
	}
	return std::string(home != nullptr ? home : "") + "/.local/share/Steam/logs";
}

} // namespace rl::steam
