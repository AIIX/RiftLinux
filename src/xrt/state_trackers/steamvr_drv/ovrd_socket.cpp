// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The driver's socket: HDR descriptions from video players, status for tools (ovrd_socket.hpp).
 */

#include "ovrd_socket.hpp"
#include "ovrd_log.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <dlfcn.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {

struct Client
{
	int fd = -1;
	uint32_t pid = 0;
	std::string in;
	//! The processes this connection described: their descriptions go with it.
	std::vector<uint32_t> described;
};

struct State
{
	OvrdSocketHandlers handlers;
	std::thread thread;
	int listen_fd = -1;
	int wake[2] = {-1, -1};
	std::string path;
	//! Each described process's description as it stands (only the socket's thread touches it).
	std::map<uint32_t, struct xrt_layer_hdr_data> hdr;
};

State g_socket;

constexpr size_t kMaxClients = 16;
constexpr size_t kMaxLine = 64 * 1024;

std::string
socket_path()
{
	const char *xdg = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	std::string dir = xdg != nullptr && xdg[0] != '\0' ? xdg : std::string(home != nullptr ? home : "/tmp") + "/.cache";
	mkdir(dir.c_str(), 0700);
	dir += "/riftlinux";
	mkdir(dir.c_str(), 0700);
	return dir + "/driver.sock";
}

//! The package's VERSION next to the driver (bin/linux64/driver_monado.so -> ../../VERSION), or empty.
std::string
driver_version()
{
	Dl_info info = {};
	if (dladdr(reinterpret_cast<void *>(&driver_version), &info) == 0 || info.dli_fname == nullptr) {
		return "";
	}
	std::string dir = info.dli_fname;
	for (int i = 0; i < 3 && dir.find('/') != std::string::npos; i++) {
		dir.erase(dir.rfind('/'));
	}
	FILE *f = fopen((dir + "/VERSION").c_str(), "r");
	if (f == nullptr) {
		return "";
	}
	char line[128] = "";
	if (fgets(line, sizeof(line), f) == nullptr) {
		line[0] = '\0';
	}
	fclose(f);
	line[strcspn(line, "\r\n")] = '\0';
	return line;
}

void
reply(Client &c, cJSON *obj)
{
	char *text = cJSON_PrintUnformatted(obj);
	if (text != nullptr) {
		std::string line = std::string(text) + "\n";
		// Never waits: a client that does not read its replies loses them, the driver does not stall.
		(void)send(c.fd, line.data(), line.size(), MSG_DONTWAIT | MSG_NOSIGNAL);
		cJSON_free(text);
	}
	cJSON_Delete(obj);
}

void
reply_error(Client &c, const char *what)
{
	cJSON *obj = cJSON_CreateObject();
	cJSON_AddStringToObject(obj, "error", what);
	reply(c, obj);
}

void
log_description(uint32_t pid, const struct xrt_layer_hdr_data &d)
{
	const char *transfer = d.transfer == XRT_LAYER_TRANSFER_PQ       ? "PQ"
	                       : d.transfer == XRT_LAYER_TRANSFER_LINEAR ? "linear"
	                                                                 : "SDR";
	// Zero is "not given" (white: 203 nits, ITU-R BT.2408).
	ovrd_log("Socket: HDR description for pid %u: %s, %s, white %.0f nits, mastering %.4g-%.0f, MaxCLL %.0f, MaxFALL "
	         "%.0f nits (0: not given)%s\n",
	         pid, transfer, d.primaries == XRT_LAYER_PRIMARIES_BT2020 ? "BT.2020" : "BT.709",
	         d.reference_white_nits > 0.0f ? d.reference_white_nits : 203.0f, d.mastering_min_nits,
	         d.mastering_max_nits, d.max_cll_nits, d.max_fall_nits, d.scene_max_nits > 0.0f ? ", with per-scene values" : "");
}

//! A nits field: a number (0..10000) sets it, null resets it, absent leaves it. False if it is something else.
bool
read_nits(const cJSON *obj, const char *key, float &out)
{
	const cJSON *v = cJSON_GetObjectItemCaseSensitive(obj, key);
	if (v == nullptr) {
		return true;
	}
	if (cJSON_IsNull(v)) {
		out = 0.0f;
		return true;
	}
	if (!cJSON_IsNumber(v) || !std::isfinite(v->valuedouble)) {
		return false;
	}
	out = (float)std::clamp(v->valuedouble, 0.0, 10000.0);
	return true;
}

void
handle_hdr(Client &c, const cJSON *hdr, uint32_t pid)
{
	if (cJSON_IsNull(hdr)) {
		if (g_socket.hdr.erase(pid) > 0) {
			ovrd_log("Socket: HDR description for pid %u dropped\n", pid);
			if (g_socket.handlers.hdr) {
				g_socket.handlers.hdr(pid, nullptr);
			}
		}
		cJSON *ok = cJSON_CreateObject();
		cJSON_AddBoolToObject(ok, "ok", true);
		reply(c, ok);
		return;
	}
	if (!cJSON_IsObject(hdr)) {
		reply_error(c, "hdr must be an object or null");
		return;
	}
	const bool known = g_socket.hdr.count(pid) > 0;
	struct xrt_layer_hdr_data d = known ? g_socket.hdr[pid] : xrt_layer_hdr_data{};
	if (!known) {
		d.transfer = XRT_LAYER_TRANSFER_LINEAR;
	}
	// Taken as said, "sdr" included (a float image would otherwise be guessed to be HDR).
	d.flags |= XRT_LAYER_HDR_DESCRIBED;
	const struct xrt_layer_hdr_data before = d;

	const cJSON *transfer = cJSON_GetObjectItemCaseSensitive(hdr, "transfer");
	if (transfer != nullptr) {
		const char *t = cJSON_IsString(transfer) ? transfer->valuestring : "";
		if (strcmp(t, "pq") == 0) {
			d.transfer = XRT_LAYER_TRANSFER_PQ;
		} else if (strcmp(t, "linear") == 0) {
			d.transfer = XRT_LAYER_TRANSFER_LINEAR;
		} else if (strcmp(t, "sdr") == 0) {
			d.transfer = XRT_LAYER_TRANSFER_SDR;
		} else {
			reply_error(c, "transfer must be \"pq\", \"linear\" or \"sdr\"");
			return;
		}
	}
	const cJSON *primaries = cJSON_GetObjectItemCaseSensitive(hdr, "primaries");
	if (primaries != nullptr) {
		const char *p = cJSON_IsString(primaries) ? primaries->valuestring : "";
		if (strcmp(p, "bt2020") == 0) {
			d.primaries = XRT_LAYER_PRIMARIES_BT2020;
		} else if (strcmp(p, "bt709") == 0) {
			d.primaries = XRT_LAYER_PRIMARIES_BT709;
		} else {
			reply_error(c, "primaries must be \"bt709\" or \"bt2020\"");
			return;
		}
	}
	if (!read_nits(hdr, "white", d.reference_white_nits) || !read_nits(hdr, "mastering_max", d.mastering_max_nits) ||
	    !read_nits(hdr, "mastering_min", d.mastering_min_nits) || !read_nits(hdr, "max_cll", d.max_cll_nits) ||
	    !read_nits(hdr, "max_fall", d.max_fall_nits) || !read_nits(hdr, "scene_max", d.scene_max_nits) ||
	    !read_nits(hdr, "scene_avg", d.scene_avg_nits)) {
		reply_error(c, "nits values must be numbers (or null)");
		return;
	}

	g_socket.hdr[pid] = d;
	if (std::find(c.described.begin(), c.described.end(), pid) == c.described.end()) {
		c.described.push_back(pid);
	}
	// Logged when it is new or its kind changes, not for every scene's values.
	struct xrt_layer_hdr_data a = before, b = d;
	a.scene_max_nits = a.scene_avg_nits = b.scene_max_nits = b.scene_avg_nits = 0.0f;
	if (!known || memcmp(&a, &b, sizeof(a)) != 0 || (before.scene_max_nits > 0.0f) != (d.scene_max_nits > 0.0f)) {
		log_description(pid, d);
	}
	if (g_socket.handlers.hdr) {
		g_socket.handlers.hdr(pid, &d);
	}
	cJSON *ok = cJSON_CreateObject();
	cJSON_AddBoolToObject(ok, "ok", true);
	cJSON_AddNumberToObject(ok, "pid", pid);
	reply(c, ok);
}

void
handle_line(Client &c, const std::string &line)
{
	if (line.find_first_not_of(" \t\r") == std::string::npos) {
		return;
	}
	cJSON *req = cJSON_Parse(line.c_str());
	if (req == nullptr || !cJSON_IsObject(req)) {
		cJSON_Delete(req);
		reply_error(c, "not a JSON object");
		return;
	}
	const cJSON *hdr = cJSON_GetObjectItemCaseSensitive(req, "hdr");
	const cJSON *status = cJSON_GetObjectItemCaseSensitive(req, "status");
	const cJSON *recalibrate = cJSON_GetObjectItemCaseSensitive(req, "recalibrate");
	const cJSON *quit = cJSON_GetObjectItemCaseSensitive(req, "quit");
	if (cJSON_IsTrue(recalibrate)) {
		const bool started = g_socket.handlers.recalibrate && g_socket.handlers.recalibrate();
		cJSON *ok = cJSON_CreateObject();
		cJSON_AddBoolToObject(ok, "ok", true);
		cJSON_AddBoolToObject(ok, "started", started);
		reply(c, ok);
	} else if (cJSON_IsTrue(quit)) {
		if (g_socket.handlers.quit) {
			g_socket.handlers.quit();
		}
		cJSON *ok = cJSON_CreateObject();
		cJSON_AddBoolToObject(ok, "ok", true);
		reply(c, ok);
	} else if (hdr != nullptr) {
		uint32_t pid = c.pid;
		const cJSON *p = cJSON_GetObjectItemCaseSensitive(req, "pid");
		if (p != nullptr) {
			if (!cJSON_IsNumber(p) || p->valuedouble < 1 || p->valuedouble > 4194304) {
				cJSON_Delete(req);
				reply_error(c, "pid must be a process id");
				return;
			}
			pid = (uint32_t)p->valuedouble;
		}
		handle_hdr(c, hdr, pid);
	} else if (cJSON_IsTrue(status)) {
		cJSON *ok = cJSON_CreateObject();
		cJSON_AddBoolToObject(ok, "ok", true);
		cJSON *s = cJSON_AddObjectToObject(ok, "status");
		cJSON_AddStringToObject(s, "driver", "riftlinux");
		cJSON_AddStringToObject(s, "version", driver_version().c_str());
		if (g_socket.handlers.status) {
			g_socket.handlers.status(s);
		}
		cJSON *described = cJSON_AddArrayToObject(s, "hdr_described_pids");
		for (const auto &kv : g_socket.hdr) {
			cJSON_AddItemToArray(described, cJSON_CreateNumber(kv.first));
		}
		reply(c, ok);
	} else {
		reply_error(c, "unknown request (\"hdr\", \"status\", \"recalibrate\" or \"quit\")");
	}
	cJSON_Delete(req);
}

void
drop_client(Client &c)
{
	for (uint32_t pid : c.described) {
		if (g_socket.hdr.erase(pid) > 0) {
			ovrd_log("Socket: HDR description for pid %u dropped (its sender's connection closed)\n", pid);
			if (g_socket.handlers.hdr) {
				g_socket.handlers.hdr(pid, nullptr);
			}
		}
	}
	c.described.clear();
	close(c.fd);
	c.fd = -1;
}

void
accept_client(std::vector<Client> &clients)
{
	const int fd = accept4(g_socket.listen_fd, nullptr, nullptr, SOCK_CLOEXEC);
	if (fd < 0) {
		return;
	}
	struct ucred cred = {};
	socklen_t len = sizeof(cred);
	if (getsockopt(fd, SOL_SOCKET, SO_PEERCRED, &cred, &len) != 0 || cred.uid != getuid() ||
	    clients.size() >= kMaxClients) {
		close(fd);
		return;
	}
	Client c;
	c.fd = fd;
	c.pid = (uint32_t)cred.pid;
	clients.push_back(c);
}

void
serve()
{
	std::vector<Client> clients;
	while (true) {
		std::vector<struct pollfd> fds;
		fds.push_back({g_socket.wake[0], POLLIN, 0});
		fds.push_back({g_socket.listen_fd, POLLIN, 0});
		for (const Client &c : clients) {
			fds.push_back({c.fd, POLLIN, 0});
		}
		if (poll(fds.data(), fds.size(), -1) < 0) {
			if (errno == EINTR) {
				continue;
			}
			ovrd_log("Socket: poll failed (%s), closed\n", strerror(errno));
			break;
		}
		if (fds[0].revents != 0) {
			break;
		}
		// The clients polled are the first ones in the list; a new one only joins at its end.
		const size_t polled = fds.size() - 2;
		for (size_t i = 0; i < polled; i++) {
			if (fds[i + 2].revents == 0) {
				continue;
			}
			Client &c = clients[i];
			char buf[4096];
			const ssize_t n = recv(c.fd, buf, sizeof(buf), 0);
			if (n <= 0) {
				drop_client(c);
				continue;
			}
			c.in.append(buf, (size_t)n);
			size_t nl;
			while ((nl = c.in.find('\n')) != std::string::npos) {
				const std::string line = c.in.substr(0, nl);
				c.in.erase(0, nl + 1);
				handle_line(c, line);
			}
			if (c.in.size() > kMaxLine) {
				reply_error(c, "line too long");
				drop_client(c);
			}
		}
		if (fds[1].revents & POLLIN) {
			accept_client(clients);
		}
		clients.erase(std::remove_if(clients.begin(), clients.end(), [](const Client &c) { return c.fd < 0; }),
		              clients.end());
	}
	for (Client &c : clients) {
		drop_client(c);
	}
}

} // namespace

bool
ovrd_socket_start(const OvrdSocketHandlers &handlers)
{
	if (g_socket.thread.joinable()) {
		return true;
	}
	// The log is per source file (ovrd_log.hpp): this one's too.
	ovrd_log_init(vr::VRDriverLog());
	g_socket.handlers = handlers;
	g_socket.path = socket_path();
	struct sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	if (g_socket.path.size() >= sizeof(addr.sun_path)) {
		ovrd_log("Socket: path too long for a socket (%s), none\n", g_socket.path.c_str());
		return false;
	}
	memcpy(addr.sun_path, g_socket.path.c_str(), g_socket.path.size() + 1);

	// A socket file left by a driver that ended without closing it is replaced; one a running driver serves is not.
	struct stat st = {};
	if (lstat(g_socket.path.c_str(), &st) == 0) {
		if (!S_ISSOCK(st.st_mode)) {
			ovrd_log("Socket: %s is not a socket, left as it is; none\n", g_socket.path.c_str());
			return false;
		}
		const int probe = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
		const bool served = probe >= 0 && connect(probe, (struct sockaddr *)&addr, sizeof(addr)) == 0;
		if (probe >= 0) {
			close(probe);
		}
		if (served) {
			ovrd_log("Socket: %s is served by another driver; none here\n", g_socket.path.c_str());
			return false;
		}
		unlink(g_socket.path.c_str());
	}

	g_socket.listen_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
	if (g_socket.listen_fd < 0 || bind(g_socket.listen_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
	    chmod(g_socket.path.c_str(), 0600) != 0 || listen(g_socket.listen_fd, 8) != 0 ||
	    pipe2(g_socket.wake, O_CLOEXEC) != 0) {
		ovrd_log("Socket: can't serve %s (%s); none\n", g_socket.path.c_str(), strerror(errno));
		if (g_socket.listen_fd >= 0) {
			close(g_socket.listen_fd);
			g_socket.listen_fd = -1;
		}
		return false;
	}
	g_socket.thread = std::thread(serve);
	ovrd_log("Socket: serving %s (HDR descriptions from players, status)\n", g_socket.path.c_str());
	return true;
}

void
ovrd_socket_stop()
{
	if (!g_socket.thread.joinable()) {
		return;
	}
	const char byte = 1;
	(void)write(g_socket.wake[1], &byte, 1);
	g_socket.thread.join();
	close(g_socket.listen_fd);
	close(g_socket.wake[0]);
	close(g_socket.wake[1]);
	g_socket.listen_fd = g_socket.wake[0] = g_socket.wake[1] = -1;
	unlink(g_socket.path.c_str());
	g_socket.hdr.clear();
	ovrd_log("Socket: closed\n");
}
