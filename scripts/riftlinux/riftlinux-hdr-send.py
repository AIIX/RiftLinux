#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
"""Hands the RiftLinux SteamVR driver an HDR description for a process's images, as a video player does, or asks
for the driver's status. A test tool and a reference for players.

The driver's socket: $XDG_CACHE_HOME/riftlinux/driver.sock (~/.cache by default), one JSON object per line each
way; see src/xrt/state_trackers/steamvr_drv/ovrd_socket.hpp. A description lasts while its connection is open,
so this holds it open (--hold, seconds; 0 until interrupted).

  riftlinux-hdr-send.py --status
  riftlinux-hdr-send.py --pid 1234 --transfer linear --primaries bt709 --max-cll 4000 --hold 60
  riftlinux-hdr-send.py --pid 1234 --scene 1000:100,4000:400 --every 5 --hold 60   (per-scene values in turn)
"""
import argparse
import json
import os
import socket
import sys
import time


def socket_path():
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "riftlinux", "driver.sock")


def ask(sock, f, request):
    sock.sendall((json.dumps(request) + "\n").encode())
    line = f.readline()
    if not line:
        sys.exit("the driver closed the connection")
    answer = json.loads(line)
    print(json.dumps(answer), flush=True)
    if "error" in answer:
        sys.exit(1)
    return answer


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--status", action="store_true", help="print the driver's status and quit")
    ap.add_argument("--pid", type=int, help="the process whose images are described (default: this one)")
    ap.add_argument("--transfer", choices=("pq", "linear", "sdr"))
    ap.add_argument("--primaries", choices=("bt709", "bt2020"))
    ap.add_argument("--white", type=float, help="reference white, nits (default 203)")
    ap.add_argument("--mastering", help="mastering display min:max nits, e.g. 0.005:1000")
    ap.add_argument("--max-cll", type=float)
    ap.add_argument("--max-fall", type=float)
    ap.add_argument("--scene", help="per-scene max:avg nits; several separated by commas are sent in turn")
    ap.add_argument("--every", type=float, default=5.0, help="seconds between scenes (default 5)")
    ap.add_argument("--hold", type=float, default=0.0, help="seconds to keep the description (0: until interrupted)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(socket_path())
    except OSError as e:
        sys.exit(f"no driver socket at {socket_path()} ({e.strerror}): is SteamVR running with the RiftLinux driver?")
    f = sock.makefile("r")

    if args.status:
        ask(sock, f, {"status": True})
        return

    hdr = {}
    for key, value in (("transfer", args.transfer), ("primaries", args.primaries), ("white", args.white),
                       ("max_cll", args.max_cll), ("max_fall", args.max_fall)):
        if value is not None:
            hdr[key] = value
    if args.mastering:
        lo, hi = (float(v) for v in args.mastering.split(":"))
        hdr["mastering_min"], hdr["mastering_max"] = lo, hi
    scenes = []
    if args.scene:
        for part in args.scene.split(","):
            mx, avg = (float(v) for v in part.split(":"))
            scenes.append({"scene_max": mx, "scene_avg": avg})
    request = {"hdr": dict(hdr, **(scenes[0] if scenes else {}))}
    if args.pid is not None:
        request["pid"] = args.pid
    ask(sock, f, request)

    end = time.monotonic() + args.hold if args.hold > 0 else None
    i = 0
    try:
        while end is None or time.monotonic() < end:
            time.sleep(min(args.every, max(0.0, end - time.monotonic())) if end is not None else args.every)
            if len(scenes) > 1 and (end is None or time.monotonic() < end):
                i = (i + 1) % len(scenes)
                request = {"hdr": scenes[i]}
                if args.pid is not None:
                    request["pid"] = args.pid
                ask(sock, f, request)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
