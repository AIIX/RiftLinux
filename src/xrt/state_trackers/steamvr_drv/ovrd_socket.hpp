// SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The driver's socket for other programs: a video player hands over the HDR
 *         description of its images (HDR10, HDR10+, Dolby Vision values) for Auto-HDR's mapping, and tools ask for the
 *         driver's status.
 *
 * At $XDG_CACHE_HOME/riftlinux/driver.sock (by default ~/.cache): in the home folder, which SteamVR's container
 * shares with the desktop (its $XDG_RUNTIME_DIR is a private one). A Unix stream socket for processes of the same
 * user, one JSON object per line each way. Requests:
 *
 *   {"hdr": {"transfer": "pq" | "linear" | "sdr", "primaries": "bt709" | "bt2020", "white": nits,
 *            "mastering_max": nits, "mastering_min": nits, "max_cll": nits, "max_fall": nits,
 *            "scene_max": nits, "scene_avg": nits}, "pid": n}
 *       The images of process n (left out: the sender) are so described. Fields left out keep their value, so a
 *       scene's values (HDR10+, Dolby Vision L1) can come alone; null resets one. Without a transfer the images are
 *       linear light (float images, white at 1.0). The description goes when the sender's connection closes: a
 *       player keeps it open while it plays.
 *   {"hdr": null, "pid": n}   the description dropped.
 *   {"status": true}          the driver's status.
 *   {"recalibrate": true}     every sensor measured again, as on a first start; "started": false if refused.
 *   {"quit": true}            SteamVR quits, as its own Exit does.
 *
 * Each request gets one line back: {"ok": true, ...} or {"error": "..."}.
 */

#pragma once

#include "xrt/xrt_compositor.h"
#include "util/u_json.h"

#include <cstdint>
#include <functional>

struct OvrdSocketHandlers
{
	//! The images of a process described, or (nullptr) no longer.
	std::function<void(uint32_t pid, const struct xrt_layer_hdr_data *hdr)> hdr;
	//! Fills the status object with the driver's state.
	std::function<void(cJSON *status)> status;
	//! Every sensor measured again; true when it starts.
	std::function<bool()> recalibrate;
	//! SteamVR asked to quit.
	std::function<void()> quit;
};

//! Opens the socket and serves it from a thread of its own; false if it can't (logged), the driver runs on.
bool
ovrd_socket_start(const OvrdSocketHandlers &handlers);

//! Closes it; every description given through it goes too.
void
ovrd_socket_stop();
