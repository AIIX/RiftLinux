<!--
Copyright 2018-2021, Collabora, Ltd.
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>

SPDX-License-Identifier: CC-BY-4.0

Adapted from Monado's README.
-->

# RiftLinux: the Oculus Rift CV1 driver for Linux (SteamVR and OpenXR)

[![Platform: Linux](https://img.shields.io/badge/platform-Linux-blue)](#supported-hardware-and-systems)
[![Headset: Oculus Rift CV1](https://img.shields.io/badge/headset-Oculus%20Rift%20CV1-black)](#supported-hardware-and-systems)
[![SteamVR](https://img.shields.io/badge/SteamVR-supported-1b2838)](#install-steamvr)
[![OpenXR](https://img.shields.io/badge/OpenXR-supported-5c2d91)](docs/openxr-without-steamvr.md)
[![License: BSL-1.0](https://img.shields.io/badge/license-BSL--1.0-green)](#license)

**Play SteamVR and OpenXR games on Linux with your Oculus Rift CV1, the way it works on Windows.** RiftLinux is a
**modified Monado, specialized for full Oculus Rift CV1 support**: an open-source Linux driver and runtime for the
**Oculus Rift CV1** headset, its **Constellation sensors** and its **Oculus Touch controllers**: full 6DoF head and hand tracking, room scale, SteamVR games (Linux games and Windows VR games through Proton),
native OpenXR apps, sound in the Rift's headphones, haptics, finger poses, the Oculus button menu, and a picture that
goes beyond the Oculus runtime: Auto-HDR, HDR games and video, motion smoothing (ASW-style) and the CV1's true
resolution. **Install one package, plug in the Rift, start SteamVR. No Windows, no Oculus app, no OpenHMD, no Room
Setup.**

**One headset, one driver.** RiftLinux is made for the Oculus Rift CV1 and nothing else: the CV1 headset, its
sensors, its Touch controllers and the Oculus Remote. Every other headset driver of Monado is left out, and the SteamVR driver activates only when a CV1 (USB 2833:0031) is plugged in,
so it never gets in the way of another headset. For other headsets use upstream Monado or SteamVR's own drivers.

As far as we know (survey of October 2026), RiftLinux is the most complete Rift CV1 driver for Linux, and the only
one with all of: Touch controllers tracked optically from more than one sensor, sensors calibrated from movement and
placed again by themselves when moved, room scale with the headband's rear LEDs tracked, correct CV1 lens distortion
in SteamVR, motion smoothing, Auto-HDR, and tracking measured against the Oculus runtime's own results.

**New here?** Start with the [user guide](docs/README.md) (also as a website:
<https://aiix.github.io/RiftLinux/>): [Getting started](docs/getting-started.md), [FAQ](docs/faq.md),
[Troubleshooting](docs/troubleshooting.md).

## Project status (21 October 2026)

Used daily on the maintainer's machine, which runs two systems: Ubuntu 25.04 (GNOME 48, X11 and Wayland) and CachyOS
(KDE Plasma 6.8, Wayland), with an NVIDIA GPU (drivers 580 and 615), SteamVR 2.17 and two or three sensors. Every
feature marked done below was judged in the headset; tracking was also scored against the Oculus runtime. The one
package installs and works from a fresh system with nothing else done (checked on a new CachyOS install).
Packages for every release are on the [Releases page](https://github.com/AIIX/RiftLinux/releases); you can also build
them yourself (see [Build](#build)).

✅ done · 🚧 in progress · 📋 planned · ❓ not tested yet · ❌ not possible (outside this driver)

| Area | State | Where it stands |
| --- | --- | --- |
| Head tracking, 6DoF | ✅ Done | Smooth, no bobbing; position held through short occlusions; the view turns with the headset's own gyro, so a wrongly placed sensor cannot turn the world around you |
| Touch controllers | ✅ Done | Tracked as the Oculus runtime tracks them: on Oculus' own camera frames, within 1-2 points of Oculus' tracked share per movement, day and night |
| Two sensors | ✅ Done | Placed from your first movements and saved; a moved or knocked sensor is found and placed again automatically, at start or mid-game; sensors tilted alike are levelled by the headset's gravity, and measured again by themselves when tilted too far |
| Room scale | ✅ Done | Tested and works with three sensors: a sensor behind you is placed by itself, and the headset and both controllers stay tracked all the way around |
| Sensors behind you | ✅ Done | The headband's rear LEDs are measured on your headset by themselves and tracked, as the Oculus runtime does: with your back to the sensors, the headset stays tracked |
| SteamVR | ✅ Done | SteamVR Home, No Man's Sky VR, VTOL VR, PLAY'A VR, Bigscreen; Windows VR games through Proton; OpenXR games through SteamVR |
| Native OpenXR (without SteamVR) | ✅ Done | RiftLinux's own runtime: app menu entry, right-click *Run with the Rift*, `riftlinux-run`; the Rift tab in the headset, with *Close the application* ([guide](docs/openxr-without-steamvr.md)) |
| Picture: Auto-HDR, HDR games, true resolution | ✅ Done | In direct mode (the default); every part has its own switch |
| HDR video with its metadata | ✅ Driver side done | A player hands its HDR10 / HDR10+ / Dolby Vision values over; no player does so yet |
| Motion smoothing (ASW-style) | ✅ Done | As Oculus' ASW 1.0. ASW 2.0 also corrects sideways head movement with the game's depth image, for games that send one; not here yet (SteamVR Home sends none; other games to be measured) |
| One-package install, nothing to set up | ✅ Done | `.deb`, Arch package, `.rpm` and a tarball; registers itself for every user; a welcome in the headset |
| Downloads | ✅ Done | Packages for every release on the Releases page; AUR and COPR planned |
| X11 and Wayland | ✅ Done | GNOME 48 on X11 and Wayland, KDE Plasma 6.8 on Wayland |
| NVIDIA | ✅ Done | Drivers 580 and 615; on Wayland with a VRR monitor the picture can tear (NVIDIA's bug): one setting cures it ([Wayland](#wayland)) |
| AMD and Intel graphics | ❓ Not tested | Expected to work (Vulkan) |
| Pairing controllers without Windows | ✅ Done (Touch) | `riftlinux-setup pair`, with SteamVR closed: the same steps as the Oculus software (compared on USB); tested with both Touch controllers; the Remote not yet tested |
| Desktop in VR | ❌ SteamVR / 📋 | SteamVR's Desktop tab is empty on Linux; WayVR works as an overlay (SDR), in SteamVR and in RiftLinux's own runtime; KWin's VR mode later |
| RiftLinux desktop app | 📋 Planned | After the driver |

## Highlights

- **Install once, play.** One package for your distribution; it registers the driver with SteamVR for every user,
  and the driver gives SteamVR its room, sets the OpenXR runtime and starts in direct mode. A welcome panel in the
  headset says how the Oculus button works.
- **Full 6DoF tracking, Oculus quality.** Head and Touch controllers tracked from the Constellation sensors, fused with
  each device's IMU. Scored on the Oculus runtime's own camera frames: within 1-2 points of Oculus' tracked share per
  movement, day and night.
- **Plays SteamVR games.** Linux games, Windows VR games through Proton, OpenXR games through SteamVR.
- **OpenXR without SteamVR.** RiftLinux's own runtime runs OpenXR applications on the Rift directly: right-click a
  program, *Run with the Rift*. The same tracking, picture and Rift tab, with a button to close the application.
- **A sensor moved or knocked?** Found, placed again and saved automatically, mid-game, with a notice in the
  headset. The very first measuring is explained in the headset while it runs.
- **Motion smoothing (ASW-style).** When a game cannot hold 90 frames a second it is held at 45 or 30 and the missing
  frames are made from its motion: NVIDIA optical flow on RTX 30 and newer, a compute fallback on other GPUs.
- **Auto-HDR, HDR games and HDR video.** Inverse tone mapping for the CV1's OLED over the whole picture (deeper darks,
  brighter highlights, richer colour); games that render in HDR keep their highlights instead of clipping them;
  HDR10, HDR10+ and Dolby Vision mapped by their metadata. Not available on Windows.
- **The CV1's true resolution, a sharper picture.** Games render at the CV1's 1:1 size (as Oculus' pixel density
  1.0); the game's image is resampled once to the panel instead of twice.
- **The Oculus button menu.** A "Rift" tab in SteamVR's dashboard: headphone volume, microphone, floor and height,
  picture, motion smoothing; a second page with the headset's status (worn, controllers' batteries, sensors), the
  display settings and performance numbers. Hold the button to recentre.
- **Comfort and care.** Panel off when the headset is not worn (OLED burn-in), on at once when worn; sound switched
  to the Rift's headphones and its microphone made the default input, both put back after; real-time priority for
  tracking and display.
- **Open and measured.** Sessions recorded and replayed through the tracker, scored against the Oculus runtime;
  plans and every test result published.

## Documentation

| Page | What it covers |
| --- | --- |
| [Getting started](docs/getting-started.md) | What you need, connecting the Rift, installing, the first start, the floor |
| [The Rift tab](docs/rift-tab.md) | The Oculus button menu: every setting, the status page, performance numbers, notices |
| [Sensors](docs/sensors.md) | Placing the Constellation sensors, measuring them, a sensor moved or knocked |
| [Touch controllers and pairing](docs/controllers.md) | Pairing without Windows, buttons, batteries, finger poses, haptics |
| [The picture](docs/picture.md) | Direct mode, Auto-HDR, HDR games and video, motion smoothing, render size |
| [OpenXR without SteamVR](docs/openxr-without-steamvr.md) | RiftLinux's own runtime, `riftlinux-run`, WayVR |
| [Troubleshooting](docs/troubleshooting.md) | Symptoms, causes and fixes; `riftlinux-setup check`; logs |
| [FAQ](docs/faq.md) | Short answers about the Rift CV1 on Linux |

## Supported hardware and systems

| Part | Supported | Notes |
| --- | --- | --- |
| Headset | **Oculus Rift CV1** | Only the CV1. Not: Rift S (upstream Monado has a driver), DK1/DK2, Quest (use ALVR or WiVRn) |
| Sensors | Oculus Constellation sensors: two, or three for room scale | Tested with two and with three |
| Controllers | Oculus Touch (left, right), Oculus Remote | Paired with the headset (as they come, or with `riftlinux-setup pair`) |
| Computer | x86-64 (64-bit Intel or AMD), Linux | The packages are built for x86-64 only; on ARM computers they refuse to install |
| GPU | Vulkan with direct display | Tested: NVIDIA, drivers 580 and 615. AMD and Intel should work but are not tested yet; reports welcome. Motion smoothing's hardware flow needs NVIDIA RTX 30 or newer; other GPUs use the compute fallback |
| Linux | Any distribution that runs Steam | Packages for Debian / Ubuntu (`.deb`), Arch / CachyOS (`pacman`), Fedora (`.rpm`) and a tarball for the rest; tested on Ubuntu 25.04 and CachyOS, installed and removed cleanly on Arch, Fedora and Ubuntu in containers. Steam from the distribution, Valve, Flatpak or Snap |
| Desktop | X11 and Wayland | Tested: GNOME 48 on X11 and Wayland, KDE Plasma 6.8 on Wayland. On Wayland the headset is leased from the desktop (GNOME, KDE Plasma, wlroots-based) |
| VR software | SteamVR (Linux) 2.17 and newer; OpenXR | Native OpenXR apps through Monado's service without SteamVR |

## Rift CV1 on Linux: the options

| Feature | **RiftLinux** | OpenHMD (thaytan's `rift-room-config`) | Monado upstream (`main`) | ALVR / WiVRn |
| --- | --- | --- | --- | --- |
| Drives a Rift CV1 | ✅ | ✅ | ✅ not in any release yet (v25.1.0 has DK2 only) | ❌ stream to standalone headsets (Quest, Pico) |
| What else you need | **Nothing: one package** | SteamVR-OpenHMD or Monado on top | Nothing; but it switches its Rift driver off when OpenHMD is built in | A standalone headset and Wi-Fi |
| Head tracking, position | ✅ every sensor fused, smooth | ⚠️ experimental, poor in practice | ⚠️ opt-in, "very rough", one sensor | - |
| Touch controllers, position | ✅ within 1-2 points of Oculus per movement | ⚠️ "jitter heavily with more than one sensor" | ❌ rotation only, not tracked optically | - |
| More than one sensor | ✅ calibrated from movement, saved per sensor; room scale with three | ⚠️ placed once at start, never corrected | ❌ every sensor assumed at the same spot | - |
| A sensor moved or knocked | ✅ found and placed again automatically, no restart | ❌ | ❌ | - |
| Daylight, reflections | ✅ static bright spots left out | ⚠️ | ⚠️ | - |
| Pairing controllers | ✅ `riftlinux-setup pair` (Touch tested, the Remote not yet) | ❌ | ❌ | - |
| Lens distortion | ✅ fixed in both | ⚠️ fixed polynomial approximation | ⚠️ mapped to the wrong field of view; top and bottom swapped in SteamVR | - |
| Display brightness, persistence | ✅ as the Oculus runtime sets them | ⚠️ | ❌ never set: panel at brightness 0 | - |
| SteamVR | ✅ packaged for Steam's runtime; games tested | ⚠️ SteamVR-OpenHMD, last changed 2022 | ⚠️ driver in-process; Touch grip pose and haptics unfinished | ✅ its purpose |
| Native OpenXR apps | ✅ | ✅ through Monado | ✅ | ✅ (WiVRn) |
| Haptics | ✅ tuned to SteamVR's pulses | ⚠️ untested | ✅ | ✅ |
| Finger poses (capacitive touch) | ✅ hand skeleton for SteamVR | ❌ | ❌ inputs only | ✅ |
| Headset on/off sensor | ✅ and the panel switched off when not worn | ❌ | ✅ | ✅ |
| Sound to the Rift's headphones | ✅ automatic, put back on exit | ❌ manual | ❌ manual | ✅ |
| Oculus button menu | ✅ "Rift" tab: volume, floor, height, picture, status | ❌ | ❌ | ⚠️ own menu |
| Notices in the headset | ✅ welcome, sensors measured and moved, controllers not found, real-time priority | ❌ | ❌ | - |
| Picture: Auto-HDR, HDR games and video, dithering | ✅ in direct mode | ❌ | ❌ | ❌ |
| Missed frames: motion smoothing (ASW-style) | ✅ game held at 45 or 30, in-between frames; SteamVR and native OpenXR | ❌ | ❌ | ❌ |
| Render size | ✅ the CV1's own 1:1 size (as Oculus' pixel density 1.0) | panel-based | panel x 140 % | - |
| Setup | ✅ one package, nothing to set up; `riftlinux-setup` checks and repairs | manual | manual | - |
| Wayland | ✅ GNOME and KDE Plasma tested; headset given back and taken again as it is taken off and put on | ⚠️ via SteamVR | ✅ | ✅ |
| Tracking checked against the Oculus runtime | ✅ our tracker run on Oculus' captured frames | ❌ | ❌ | - |
| Maintained | ✅ | ❌ author moved to Monado (2024) | ✅ | ✅ |

✅ works · ⚠️ partly · 🚧 in progress · ❌ no · - not applicable. Sources: the projects' code and merge requests,
ArchWiki, vronlinux.org, read in October 2026.

## Windows (Oculus runtime) and Linux (RiftLinux)

| Feature | Windows, Oculus runtime | Linux, RiftLinux | Compared |
| --- | --- | --- | --- |
| Install | Oculus installer, guided | One package; registers itself, sets SteamVR's room and the OpenXR runtime; a welcome in the headset | ✅ same |
| First sensor setup | Sensor setup in the Oculus app | Measured from your first movements, saved; the headset says so while it runs | ✅ same |
| Head tracking | ✅ | ✅ smooth, no bobbing; position held through short occlusions | ✅ same |
| Touch tracking | ✅ | ✅ within 1-2 points of Oculus per movement, scored on Oculus' own camera frames | ✅ same |
| Sensor moved mid-session | ✅ automatic | ✅ automatic, with a notice | ✅ same |
| Daylight in the room | ✅ | ✅ | ✅ same |
| Recentre | Hold the Oculus button | Hold the right Oculus button | ✅ same |
| Oculus button menu | Oculus Dash: volume and more | "Rift" tab: volume, mute, microphone, floor, height, picture, status, performance | ✅ same |
| Floor and height | Sensor setup | "Set floor" with a controller on the floor; height slider | ✅ same |
| Boundary | Guardian (Oculus software) | In SteamVR: SteamVR's own boundary (Chaperone), as with every SteamVR headset; the driver gives SteamVR a standing room (walls hidden), a room set up in SteamVR is used and drawn. Native OpenXR apps without SteamVR: no boundary | ✅ SteamVR's Chaperone · none without SteamVR |
| Missed frames | ASW: motion between the game's last two frames (ASW 1.0); with the game's depth image also sideways head movement corrected (ASW 2.0's positional timewarp, only for games that send depth) | Motion smoothing as ASW 1.0: motion between the game's last two frames (NVIDIA optical flow or a compute fallback), game held at 45 or 30; also for native OpenXR apps. No positional timewarp from depth yet | ⚠️ as ASW 1.0 |
| Render size | 1332x1586 a view at pixel density 1.0 | The same 1:1 size from the CV1's lens; SteamVR's supersampling and a render scale on top | ✅ same |
| Sound, haptics | ✅ | ✅ | ✅ same |
| Finger poses | ✅ estimated from the Touch's touch sensors | ✅ the same kind of hand skeleton as the Oculus driver gives SteamVR (estimated from the touch sensors); VTOL VR's hands work | ✅ same kind (not yet compared side by side) |
| Panel off when not worn | ✅ | ✅ | ✅ same |
| Display brightness, persistence | ✅ | ✅ the same values (read from USB captures) | ✅ same |
| Compositor priority | High | Real-time for the Rift's threads through RealtimeKit (no system change) | ✅ same |
| Picture | SDR | Auto-HDR, HDR games and video, dithering | ➕ more on Linux |
| Desktop in VR | Dash desktop (Oculus software) | SteamVR's Desktop tab is empty on Linux (Valve's); WayVR as an overlay (SDR) | ⚠️ through WayVR |
| Flat (non-VR) Steam games while SteamVR runs | ✅ | ❌ the Steam client crashes (Valve's bug, [SteamVR for Linux #963](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/963)) | ❌ Valve |
| Store | Oculus Store and Home (Oculus software) | SteamVR and OpenXR apps; Windows VR games through Proton | - |
| Touch and Remote pairing | Oculus app | `riftlinux-setup pair` (Touch tested, the Remote not yet) | ✅ same (Touch) |
| Firmware updates | Oculus app | Not needed (no new CV1 firmware in years); not planned | - (not needed) |
| Sensors | Two, or more for room scale (rear headband LEDs tracked) | Two, or three for room scale (rear headband LEDs tracked) | ✅ same |

✅ same as Windows · ⚠️ nearly or with a workaround · ➕ more than Windows · 🚧 in progress · 📋 planned · ❌ missing
(outside this driver) · - Oculus' own software (not part of a driver) or not needed.

**Only on Linux (in direct mode, the default):**

- **Auto-HDR**: inverse tone mapping for the CV1's SDR OLED over the whole picture, in a perceptual colour space
  (OKLab): deeper dark tones, brighter highlights, richer colour with skin spared; switch and strength in the Rift
  tab. Windows' Auto HDR does not reach a VR headset.
- **HDR games**: a SteamVR or OpenXR game that renders into float images (linear light, highlights past white) is
  handed over as it is and mapped onto the panel by its measured brightness: highlights stay apart and keep their
  colour instead of clipping to white; a game that stays within white looks exactly as in an 8-bit image.
- **HDR video**: HDR10, HDR10+ and Dolby Vision content mapped onto the panel by its own metadata (static, per scene,
  or the brightness measured per frame when none comes), per layer, hue-preserving. A player running in SteamVR hands
  its metadata to the driver over the driver's socket (`~/.cache/riftlinux/driver.sock`, one JSON line per message;
  `scripts/riftlinux/riftlinux-hdr-send.py` is a working example); a native OpenXR application chains
  `XrCompositionLayerHdrMNDX` on its layer (extension `XR_MNDX_composition_layer_hdr`, provisional, header
  `src/external/openxr_includes/openxr/XR_MNDX_composition_layer_hdr.h`; `riftlinux-openxr-hdr-test` is a working
  example). No player does either yet. Each HDR part has its own switch ([Settings](#settings)).
- **HDR Bright**: HDR games and videos placed brighter, like a TV's bright picture mode (a Rift tab switch).
- **16-bit float compositing** and **dithering** to the panel's 8-bit link: no banding in dark gradients.
- **One resampling** from the game's image to the panel (the lens pass reads it directly), and the layer renderer's
  images at the CV1's own resolution when overlays are shown: a sharper picture than resampling twice.
- A **black level** setting for the CV1's crushed near-blacks (OLED black kept).
- Open, replayable tracking: sessions recorded and replayed, scored against the Oculus runtime.

## Features and fixes in detail

293 commits on top of Monado, 277 of them RiftLinux's own (about 25,000 lines of code, 6,700 more in tools), each
measured on the hardware or in replays of recorded sessions. Taken in along the way: two unmerged upstream drafts, 16
commits (!3015: rig calibration and sensor fusion; !2941: Touch controllers hooked up to the tracker). As taken they
were barely usable on the CV1: the Touch controllers jittered, jumped, flipped and disappeared, the room calibration
was not kept, nothing handled a sensor moved during a session, and !3015's own list of known issues was open. The work below is what
made them work.

**Tracking (head and Touch)**
- Two sensors fused: room calibration from the headset's movement, saved per sensor serial, checked on load.
- Moved sensors: a watch compares the sensors' views all the time, finds the one that moved, places it again and
  saves it, with no restart and controllers kept.
- Touch tracked optically from both sensors, its IMU fused with the camera solves.
- Pose ambiguities of the Touch's LED ring (mirrored and flipped poses) resolved with the controller's own sense of
  gravity; few-LED and edge-on views solved; solves bound to a recent prior, with a held prior after short occlusions.
- Calibration errors fixed: Touch IMU offsets were applied in the wrong order (left controller tilted 5-7°); each
  controller's gyro bias measured at rest and kept per serial; idle radio reports no longer read as motion.
- A sign error in the prior check (all devices) that failed about half of all comparisons.
- Searches bounded in time (a session could hang), static sun spots left out of blind searches (daylight), LED pulse
  and radio settings as the Oculus runtime sends them (from USB captures).
- Smoother output: corrections blended in, dead reckoning through short gaps, exposures of both sensors merged.
- Measured against Oculus: this tracker was run on the Oculus runtime's own camera frames, captured on Windows, and
  scored against the poses Oculus gave for them.

**Lens and display**
- Distortion texture coordinates now index the field of view the views are rendered with (were wrong for the CV1).
- SteamVR: projection top and bottom were swapped; per-eye render target size.
- The CV1's display brightness and persistence set as the Oculus runtime does (Monado sent none: brightness 0).
- Panel off when the headset is not worn (OLED burn-in), on at once when worn.
- SteamVR direct mode (default): SteamVR's frames go through Monado's compositor, with the picture work above; off
  (a settings value) gives SteamVR's own compositor. The lens pass reads the game's image itself (one resampling,
  26-37 % sharper), with the same Auto-HDR as the layer renderer.
- Render size: games are asked for the CV1's 1:1 size at the lens centre (1332x1586 a view), in SteamVR and native
  OpenXR; supersampling is SteamVR's or `display.render_scale`.
- Wayland: the headset leased from the desktop (GNOME, KDE Plasma); given back when its display switches off (not
  worn) and taken again when the desktop offers it; NVIDIA's tearing with a VRR monitor explained (and
  detected with drivers before 615), its cure the user's choice.

**Motion smoothing (ASW-style, direct mode)**
- In-between frames when a game misses refreshes, from motion between its last two frames turned to the newest head
  pose: NVIDIA's optical flow engine (RTX 30 and newer) or block matching with sub-pixel answers on other GPUs, a
  smooth per-pixel field, the best-matching motion chosen per pixel at moving edges.
- Frame lock: the game held at every 2nd or 3rd refresh on a steady grid (SteamVR's per-application throttling,
  Monado's own pacing for native apps), with hysteresis; measured on the maintainer's No Man's Sky recordings and
  with a pace test of known motion.

**SteamVR**
- A driver package built in Valve's Steam Runtime: runs inside Steam's container, nothing else to install.
- Touch input profile, grip and aim poses as on Windows, haptics, finger skeleton, headset presence, controller
  names that Unity games require, controllers' batteries.
- Sound switched to the Rift's headphones and back; the Rift tab; notices; floor and height; clean shutdown.
- Real-time scheduling for the Rift's threads inside SteamVR through RealtimeKit.
- A start that can never be ended by SteamVR's watchdog: Monado's compositor gets a deadline; if it is not up in
  time, SteamVR's own compositor shows the headset for that session.

**Install and first run**
- Packages for Debian / Ubuntu, Arch / CachyOS, Fedora and a tarball, all from one Steam-runtime build; the driver
  registers itself for every user (at install, at login, when the headset is plugged in), also for Flatpak and Snap
  Steam; install hooks that work with or without systemd.
- Out of the box: a standing room for SteamVR without Room Setup, SteamVR set as the OpenXR runtime when none is,
  direct mode by default; `riftlinux-setup` (in the package) checks and repairs registration, device access,
  SteamVR's safe-mode block, compositor priority, room and settings.
- In the headset: a welcome panel once; while the sensors are first measured, a panel that says so (and that the
  controllers appear when it is done), the head held at a standing height meanwhile; a hint for a controller that
  never shows up; a notice when real-time priority is refused or RealtimeKit is missing.

## Install (SteamVR)

**You need:** a Rift CV1 with its Touch controllers, two sensors (or three for room scale), Linux on x86-64, Steam
and SteamVR, a Vulkan GPU driver.
Nothing else: no OpenHMD or other VR software.

Controllers must be paired with the headset. Ones bought with it, or used with it on Windows, already are. For a new
or replacement controller: close SteamVR, run `riftlinux-setup pair`, and hold the Oculus button and Y (left) or B
(right) until the controller's light blinks; one controller at a time, the tool says when each is paired.

1. **Download** the package for your distribution from the
   [Releases page](https://github.com/AIIX/RiftLinux/releases), and `SHA256SUMS` from the same release:

   | Distribution | File |
   | --- | --- |
   | Debian 12 and newer, Ubuntu 22.04 and newer, Linux Mint, Pop!_OS | `riftlinux_<version>_amd64.deb` |
   | Arch, CachyOS, EndeavourOS, Manjaro | `riftlinux-<version>-1-x86_64.pkg.tar.zst` |
   | Fedora 43 and newer | `riftlinux-<version>-1.x86_64.rpm` |
   | Any other x86-64 distribution | `riftlinux-<version>-x86_64.tar.gz` |

   Check the download, in the folder it went to: `sha256sum --ignore-missing -c SHA256SUMS` (it must say `OK`).
   Each package is installed in clean Debian, Ubuntu, Fedora and Arch systems for every release, with the versions
   above.
2. **Install it:**

   ```bash
   sudo apt install ./riftlinux_<version>_amd64.deb                # Debian, Ubuntu, Linux Mint, Pop!_OS
   sudo pacman -U riftlinux-<version>-1-x86_64.pkg.tar.zst         # Arch, CachyOS, EndeavourOS, Manjaro
   sudo dnf install ./riftlinux-<version>-1.x86_64.rpm             # Fedora
   tar xf riftlinux-<version>-x86_64.tar.gz && sudo ./riftlinux-<version>-x86_64/install.sh   # any distribution
   ```

   Your package manager installs everything the package needs with it. The tarball's `install.sh` cannot do that: it
   names any library that is missing, for you to install with your distribution's tools.

   That is all. The package gives your user access to the Rift's devices (udev, `uaccess`), and registers the driver
   with SteamVR for every user: at once for whoever is logged in, again whenever the headset is plugged in and at
   each login. SteamVR does not need to have run before. The driver itself gives SteamVR its room (no Room Setup),
   points OpenXR games at SteamVR when no other runtime is set, and starts in direct mode.
   Packaging it for a distribution: [doc/riftlinux/PACKAGING.md](doc/riftlinux/PACKAGING.md).
3. **Plug in the Rift** (HDMI and USB, sensors on USB) and **start SteamVR from Steam** (Library, or
   `steam steam://rungameid/250820`). Start Steam itself from your desktop: from a remote (SSH) login the Rift's threads
   get no real-time priority. On its first start SteamVR may ask for your password once (its compositor's priority).
4. **First start:** put the headset on. A welcome panel shows how the Oculus button works. Then face the sensors and
   slowly turn your head left and right, then up and down, until the measuring panel goes away: the sensors are
   measured from these views (the controllers appear when it is done; the view stays level meanwhile) and saved
   (`~/.config/monado/constellation/rig.json`). Then open the Rift tab and press **Set floor** with a controller resting
   on the floor.
5. **Something wrong?** `riftlinux-setup check` says what and why; `riftlinux-setup repair` fixes what SteamVR updates or
   crashes undo; `riftlinux-setup direct-mode on|off`, `riftlinux-setup uninstall`. **Moved the sensors?** A moved
   sensor is found and placed again automatically. If tracking still seems off after moving them, measure them
   manually: *Measure* on the Rift tab's *Display & status* page (or `riftlinux-setup recalibrate`), then face the
   sensors and slowly turn your head left and right, then up and down, until the measuring panel goes away (the
   controllers are hidden meanwhile, the view stays level). Every sensor is measured again; the room, SteamVR's walls
   and your floor stay where they were, by the sensors that kept their place. If every sensor moved, the view is placed
   on you again once you stand still, and the floor carried over from your standing eye height, learned as you play.
   The button is greyed while measuring is under way.
6. **OpenXR applications without SteamVR** (optional, with SteamVR closed): RiftLinux's own runtime, the same
   tracking, picture and Rift panel. Right-click a program, an AppImage or a launcher: **Run with the Rift** (KDE's
   Dolphin) or **Open With, RiftLinux Runtime** (GNOME's Files); the runtime starts for it and stops after. Or start
   **RiftLinux Runtime** from the app menu (right-click it, **Stop RiftLinux Runtime**, to end it) and run applications
   with `XR_RUNTIME_JSON=/usr/lib/riftlinux/native/openxr_riftlinux.json`. From a terminal: `riftlinux-run <program>`,
   or `riftlinux-runtime` (Enter stops it). SteamVR games are not affected.

**Upgrade:** install the newer package the same way; every user's copy of the driver follows at their next login or
when the headset is plugged in. **Remove:** `sudo apt remove riftlinux`, `sudo pacman -R riftlinux`,
`sudo dnf remove riftlinux`, or `sudo ./uninstall.sh` in the unpacked tarball's folder. The driver is unregistered
from SteamVR for everyone logged in; your settings and the sensors' calibration stay in `~/.config` (delete
`~/.config/riftlinux` and `~/.config/monado` to remove them too).

**In the headset**
- **Right Oculus button:** opens the Rift tab; pressed again, closes the dashboard.
- **Right Oculus button held 1 s:** recentre.
- **Rift tab** (in SteamVR's dashboard): every setting with a line on what it does, in cards.
  - *Settings* page. *Sound*: headphone volume and mute, the microphone's level and mute. *View*: *Set floor*, *Height* (-50 to +50 cm). *Oculus
    button*: what it does. *Picture* (direct mode): *Auto-HDR*, its *Strength* and *Colour*, *HDR Bright* (HDR games
    and videos brighter, like a TV's bright mode; it needs HDR content and is not Auto-HDR: the tab shows whether any
    is on screen now). *Motion* (direct mode): *Motion smoothing*. *Reset* puts a slider back to its default.
  - *Display & status* page. *Status*: the headset worn or not, each controller's battery (also shown by SteamVR at
    the controllers), the sensors placed (*Measure the sensors manually*: every sensor measured again, the floor kept;
    greyed while measuring is under way), what shows the picture. *Display*: *Direct mode* and *Render scale* (both
    from SteamVR's next start, which the tab says), *Fast path*. *Performance* (off until switched on): the game's
    frames a second now and over the last 10 s, the frames motion smoothing makes, and in SteamVR its GPU time a
    frame and frames shown late.
  - In RiftLinux's own runtime the tab has an *Application* card: *Close the application* asks the running
    application to quit, and ends it if it does not (SteamVR's dashboard has its own Quit).
  - Switches, buttons and pages act when the trigger is let go on the one it was pressed on; a slider moves only when
    its knob is grabbed, and the thumbstick does not change anything.

![The Rift tab, Settings page](docs/images/rift-tab.png)

![The Rift tab, Display & status page](docs/images/rift-tab-status.png)

### Wayland

On a Wayland desktop (GNOME, KDE Plasma, Sway, Hyprland) the driver takes the headset from the desktop through its
display lease (`wp_drm_lease_device_v1`), never as a desktop window. When you take the headset off its display
switches off (as with Oculus on Windows) and the desktop gets it back; when you put it on again the driver takes it
again and the picture returns. Locking the desktop while in VR changes nothing in the
headset.

**NVIDIA with a monitor that offers adaptive sync (VRR, FreeSync, G-Sync Compatible):** NVIDIA's driver can show a
leased display without vsync, so in the headset the lower part of the picture tears and lags behind when you turn your
head (NVIDIA's bug, [open-gpu-kernel-modules #992](https://github.com/NVIDIA/open-gpu-kernel-modules/issues/992), open; X11 is not affected). Here it happened with drivers 570 to 580
on GNOME and not with 615 on KDE Plasma; others report it with 615 too. The cure is to hide VRR from the driver, which
means your monitors no longer offer VRR, on X11 too, so it is your choice and the package never does it by itself.
With drivers before 615, `riftlinux-setup check` and a notice in the headset tell you. To hide VRR:

```bash
riftlinux-setup nvidia-vrr conceal        # asks for your password, then reboot
```

or by hand:

```bash
echo "options nvidia-modeset conceal_vrr_caps=1" | sudo tee /etc/modprobe.d/riftlinux-nvidia-vrr.conf
sudo update-initramfs -u                  # Fedora: sudo dracut -f   Arch: sudo mkinitcpio -P
sudo reboot
```

To undo: `riftlinux-setup nvidia-vrr restore`, or delete that file, run the same initramfs command and reboot.

## FAQ

**Does the Oculus Rift CV1 work on Linux?**
Yes, with RiftLinux: headset, Constellation sensors and Touch controllers with full 6DoF tracking, in SteamVR and in
native OpenXR apps, without Windows or the Oculus app.

**Can I play SteamVR games with an Oculus Rift CV1 on Linux?**
Yes. Install the driver package, start SteamVR from Steam, and play: Linux VR games, Windows VR games through Proton,
and OpenXR games. Tested with No Man's Sky VR, VTOL VR, PLAY'A VR and Bigscreen.

**Do I need Windows, the Oculus app or a Meta account?**
No. RiftLinux talks to the headset, sensors and controllers itself, and pairs controllers with
`riftlinux-setup pair`.

**Do I need OpenHMD, SteamVR-OpenHMD, Envision or Monado installed?**
No. The SteamVR driver package carries everything (tracking, distortion, compositor). OpenHMD is not used at all.

**How good is the tracking compared to the Oculus runtime?**
RiftLinux's tracker was run on the Oculus runtime's own captured camera frames and scored against Oculus' poses: per
movement within 1-2 points of Oculus' tracked share, day and night.

**What happens if I move a sensor?**
It is noticed and placed again automatically, without a restart, and saved for next time.

**Does it have ASW (Asynchronous Spacewarp) like Windows?**
Yes, motion smoothing: in-between frames when a game misses refreshes, with the game held at 45 or 30 frames a
second. Switch it in the Rift tab.

**What is Auto-HDR on a VR headset?**
The CV1's panel is SDR; Auto-HDR maps any game's picture onto it with deeper dark tones, brighter highlights and
richer colour (in OKLab, skin tones spared), HDR games by the brightness they render, and real HDR video by its
metadata. Strength in the Rift tab.

**Does it work on Wayland, KDE Plasma, GNOME, Fedora, Arch, SteamOS?**
Any distribution that runs Steam can run the SteamVR driver (it runs in Steam's own runtime), and there are packages
for Debian / Ubuntu, Arch / CachyOS and Fedora. Tested in the headset: Ubuntu with GNOME on X11 and Wayland, CachyOS
with KDE Plasma on Wayland (see [Wayland](#wayland) for older NVIDIA drivers with a VRR monitor). A Steam Deck has
not been tried.

**Does it work with AMD or Intel graphics?**
It uses Vulkan and is expected to; it has been tested on NVIDIA so far. Motion smoothing uses NVIDIA's optical flow on
RTX 30 and newer and a compute fallback (as good on recorded games) everywhere else.

**Is the Rift S, DK2 or Quest supported?**
No. RiftLinux is for the Rift CV1 only. Rift S: upstream Monado. Quest: ALVR or WiVRn.

**Which driver should I use for the Rift CV1 on Linux?**
RiftLinux. As far as we know (a survey of October 2026), it is the only Rift CV1 driver for Linux with Touch
controllers tracked optically from more than one sensor, sensors measured from movement and placed again by themselves
when moved, pairing without Windows, correct lens distortion in SteamVR, and tracking checked against the Oculus
runtime's own results.

**Does upstream Monado support the Rift CV1?**
Partly: its CV1 driver is in no release yet (v25.1.0 has the DK2 only), its position tracking is opt-in and rough with
one sensor, and it tracks the Touch controllers by rotation only. RiftLinux is built on Monado and contains everything
it needs; you do not install Monado beside it.

**How is it different from OpenHMD, upstream Monado or ALVR?**
See [the comparison](#rift-cv1-on-linux-the-options): OpenHMD's CV1 tracking is experimental and its controllers
jitter with more than one sensor; upstream Monado tracks the Touch controllers by rotation only; ALVR and WiVRn stream
to standalone headsets and do not drive a CV1.

More questions and answers: [docs/faq.md](docs/faq.md).

## Build

```bash
git clone https://github.com/AIIX/RiftLinux.git
cd RiftLinux
```

### SteamVR driver package (what most people want)

```bash
scripts/riftlinux/steamrt-build.sh ~/riftlinux-build
cp -r ~/riftlinux-build/package/rift-cv1-steamvr ~/rift-cv1-steamvr
```

- **Needs on the build machine:** Docker (or a compatible engine) and internet access, nothing else. The script pulls
  Valve's Steam Runtime 3 "sniper" SDK image (`registry.gitlab.steamos.cloud/steamrt/sniper/sdk`), fetches the
  sources of Eigen 3.4.0, hidapi 0.14.0, Ceres Solver 2.2.0, NASM 2.16.03 and libjpeg-turbo 3.2.0, and builds them,
  the driver and the native runtime inside the image with GCC 14. The first build takes longest; later ones only
  rebuild what changed.
- **Result:** `package/rift-cv1-steamvr/` (~35 MB, most of it the HDR test chart): `driver.vrdrivermanifest`,
  `bin/linux64/driver_monado.so`, the Rift tab helper `riftlinux-notices`, a controller check tool, and
  `resources/`; and `package/rift-cv1-native/` (~16 MB): `monado-service`, `libopenxr_monado.so` and
  `openxr_riftlinux.json`, the native runtime. Eigen, Ceres, hidapi and libjpeg-turbo are linked in. The driver runs
  inside Steam's container, which provides the rest; the native runtime needs only libraries every current
  distribution has under the same name (glibc 2.30 or newer, libudev, libusb, D-Bus, Vulkan, libdrm, X11, xcb,
  Wayland, SDL2, GLX).
- **Packages from it:** `scripts/riftlinux/make-packages.sh ~/riftlinux-build <output dir>` makes the `.deb`, Arch
  package, `.rpm` and tarball.

### Native runtime (OpenXR without SteamVR)

`monado-service` and the OpenXR runtime library, for OpenXR applications that run without SteamVR. The packages
include them (`riftlinux-runtime`); build them yourself to work on them.

- **Needs:** CMake 3.13+, Ninja, a C11/C++17 compiler, Python 3, `glslangValidator`, Vulkan headers and loader,
  Eigen3, Ceres Solver 2.1+ with EigenSparse (the constellation tracker needs it), libusb-1.0, libudev, libjpeg
  (sensors on USB 2), OpenGL and D-Bus, and the X11 (Xlib, xcb, Xrandr), Wayland (wayland-protocols 1.22 or newer)
  and libdrm development files: the headset is driven through the X server or the Wayland desktop's lease, so both
  are needed. PulseAudio's headers for the Rift tab's volume (SteamVR helper), SDL2 for `monado-gui`. **Not needed:** OpenHMD, OpenCV, libuvc, hidapi (the Rift driver uses Monado's
  own HID and USB video code).

  Debian 12 and newer, Ubuntu 24.04 and newer (Ubuntu 22.04's Ceres 2.0 is too old for the tracker: build Ceres
  2.1 or newer from source first, as `steamrt-build.sh` does, or use the packages, which run on 22.04):

  ```bash
  sudo apt install git cmake ninja-build g++ pkg-config python3 glslang-tools libvulkan-dev libeigen3-dev \
      libceres-dev libusb-1.0-0-dev libudev-dev libjpeg-dev libgl-dev libglx-dev libopengl-dev libdbus-1-dev \
      libx11-dev libx11-xcb-dev libxcb-randr0-dev libxrandr-dev libwayland-dev wayland-protocols libdrm-dev libsdl2-dev \
      libpulse-dev
  ```

  Arch, CachyOS, EndeavourOS, Manjaro:

  ```bash
  sudo pacman -S --needed git base-devel cmake ninja python glslang vulkan-headers vulkan-icd-loader eigen \
      ceres-solver libusb systemd-libs libjpeg-turbo libglvnd dbus libx11 libxcb libxrandr wayland wayland-protocols \
      libdrm sdl2 libpulse
  ```

  Fedora:

  ```bash
  sudo dnf install git cmake ninja-build gcc-c++ pkgconf python3 glslang vulkan-headers vulkan-loader-devel \
      eigen3-devel ceres-solver-devel libusb1-devel systemd-devel libjpeg-turbo-devel mesa-libGL-devel dbus-devel \
      libX11-devel libxcb-devel libXrandr-devel wayland-devel wayland-protocols-devel libdrm-devel SDL2-devel \
      pulseaudio-libs-devel
  ```

- **Build.** With OpenHMD installed, Monado would build its OpenHMD driver and switch its own Rift driver off; the
  option keeps it on:

  ```bash
  cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DXRT_BUILD_DRIVER_OHMD=OFF
  ninja -C build
  ```

  CMake's summary must show `BIGCERES`, `WAYLAND_DIRECT`, `XCB`, `XLIB` and `JPEG` on, and the drivers `RIFT` and
  `RIFT_SENSOR`. These steps are checked in clean Debian, Ubuntu, Arch and Fedora containers.
- **Run:** `build/src/xrt/targets/service/monado-service`, then an OpenXR app with
  `XR_RUNTIME_JSON=build/openxr_monado-dev.json`. A session recorded with `riftlinux-setup record on` (or
  `CONSTELLATION_TRACKER_DATA_RECORDER_OUTPUT=<file>` for the service) replays with `monado-constellation-replay`.

## Settings

`~/.config/riftlinux/settings.json`, read when SteamVR starts (the Rift tab writes its own values). The RiftLinux
app will edit these. Marked ⚡: also changed while SteamVR runs, within a second of the file being saved.

| Key | Default | What it does |
| --- | --- | --- |
| `notices.enabled`, `.sensors`, `.calibration`, `.tracking`, `.display`, `.system` | `true` | Notices in the headset, all or per kind (`.tracking` includes the controller hint, `.system` the real-time notices) |
| `notices.welcome_shown` | set after the welcome | `false` shows the welcome panel again at the next put-on |
| `tracking.floor_m` | set by *Set floor* | Where the floor is; kept through measuring the sensors again |
| `tracking.eye_height_m` | learned | Your standing eye height, learned while you play; keeps the floor right when the sensors are measured again |
| `tracking.height_cm` | `0` | The height slider |
| `diagnostics.record` | `false` | Records SteamVR's sessions for a bug report (the sensors' views, tracking and output, into `~/.cache/riftlinux`; large files) and puts the tracker's details in SteamVR's log; `riftlinux-setup record on\|off`, from SteamVR's next start |
| `display.black_clamp` | SteamVR's | SteamVR's black clamp (SteamVR's own compositor) |
| `display.direct_mode` | `true` | 🧪 SteamVR's frames through Monado's compositor; `false` gives SteamVR's own compositor (Rift tab, Display & status; or `riftlinux-setup direct-mode on\|off`; from SteamVR's next start) |
| `display.auto_hdr`, `display.auto_hdr_strength`, `display.auto_hdr_colour_strength` | `false`, `50`, `50` | 🧪⚡ Auto-HDR (SDR content), its contrast and highlights in %, its colour (vibrance) in % (50: the defaults below, 100: twice); set from the Rift tab |
| `display.auto_hdr_highlights`, `_shadows`, `_colour`, `_knee` | `0.6`, `0.10`, `0.2`, `0.55` | 🧪 Auto-HDR's strengths at 50 % |
| `display.black_level` | `2.3` | 🧪 Panel levels shown as black lifted to the first visible one, black kept black (2.3 chosen on the maintainer's CV1 with the test pattern; 0 off) |
| `display.dither_bits` | `8` | 🧪 Dither to the link's bit depth (0 off) |
| `display.direct_mode_fast_path` | `true` | 🧪⚡ A single game image goes straight into the lens pass (one resampling, sharper); `false` = through the layer renderer; switched live from the Rift tab |
| `display.render_scale` | `100` | 🧪 Render size asked of games, in % of the CV1's 1:1 size (pixels), before SteamVR's own supersampling; with direct mode on or off; Rift tab slider 50-200 % (from SteamVR's next start) |
| `display.motion_smoothing` | `false` | 🧪⚡ Motion smoothing (in-between frames when a game is late; holds the game at 45 or 30); set from the Rift tab |
| `display.motion_pick` | `true` | 🧪 Motion smoothing picks the best motion per pixel at moving edges (`false`: per 4x4 cell) |
| `display.vibrance` | `0` | 🧪 Colour strength on its own, without Auto-HDR |
| `display.hdr_float_layers` | `true` | 🧪⚡ HDR games: a game's float images shown as HDR (highlights mapped, not clipped); `false`: as SDR |
| `display.hdr_players` | `true` | 🧪⚡ HDR video: the HDR descriptions video players send over the driver's socket used; `false`: their images shown as if none came |
| `display.hdr_metadata` | `true` | 🧪⚡ HDR content's brightness metadata (mastering display, MaxCLL, per-scene peak) used for its mapping; `false`: as if none came, the peak measured |
| `display.hdr_measured_peak` | `true` | 🧪⚡ HDR content's peak measured while it is shown, when no metadata gives one; `false`: 1000 nits assumed, and a float image is taken as HDR whatever it holds |
| `display.hdr_white` | `0.88` | 🧪⚡ HDR content's reference white as panel lightness (about sRGB 215); the room above it is for highlights |
| `display.hdr_bright`, `display.hdr_white_bright` | `false`, `0.94` | 🧪⚡ HDR Bright: HDR content's white placed at this lightness instead of 0.88 (about sRGB 235 instead of 215), everything below it brighter, highlights with less room; set from the Rift tab |
| `display.hdr_test` | off | 🧪 An HDR10 test chart in front of you: `"static"`, `"none"` or `"scene"` metadata |

🧪 = in direct mode. More switches are Monado environment variables, documented where they are read
(`XRT_COMPOSITOR_*` in `src/xrt/auxiliary/render/`, `RIFT_*` in `src/xrt/drivers/rift/`, `CONSTELLATION_TRACKER_*` in
`src/xrt/tracking/constellation/`).

## RiftLinux app (coming)

<!-- Placeholder: download link and screenshots go here when the app is released. -->

A desktop app (Qt, for any desktop, shipped as a Flatpak) is planned after the driver: sensor and controller status,
calibration, pairing, the settings above, notices, and picture settings with a before/after view. It uses
`riftlinux-setup` underneath; nothing in the driver depends on it.

## Known issues

- **Steam client crash** when a flat (non-VR) game opens a window while SteamVR runs: a Valve bug in the 32-bit
  `vrclient.so`, not this driver ([SteamVR for Linux #963](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/963): the same crash, reported for the Desktop tab). Keep VR
  games in VR.
- **SteamVR's compositor crashes as SteamVR quits** in direct mode (a `vrcompositor ... segfault` line in the
  kernel log, a crash file in `/tmp/dumps`). It happens after the session has ended and changes nothing: no data
  lost, SteamVR starts normally next time. It is SteamVR's own exit code: a driver that does nothing but what
  SteamVR's direct mode interface requires (no picture, no references held) gets the same crash, and so would any
  direct-mode driver on Linux ([SteamVR for Linux #990](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/990): the same crash in SteamVR's cleanup, with Valve's own
  streaming driver). Turning direct mode off (`riftlinux-setup direct-mode off`) avoids it, at the cost of Auto-HDR
  and motion smoothing.
- **SteamVR's Desktop tab** is empty on Linux ([SteamVR for Linux #693](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/693)). A desktop in VR: WayVR, as an overlay (SDR).
- **WayVR in RiftLinux's own runtime:** its panels look softer than games, and not every controller button does what
  WayVR's bindings expect; both are being looked into.
- **SteamVR started while the desktop is locked** waits in Steam until you unlock (Steam's own behaviour). Once, on
  KDE Plasma with the screens in standby, the driver's start then stalled: since then such a start falls back to
  SteamVR's own compositor for that session (no Auto-HDR or motion smoothing) instead of being ended by SteamVR.
- **App issues under Proton (not the driver):** PLAY'A VR's default PlayaMedia engine crashes (its AVPro Video
  engine works; the game's Proton report: [Proton #7953](https://github.com/ValveSoftware/Proton/issues/7953)); Bigscreen cannot show another person's streamed
  screen (its decoder needs CUDA-to-D3D11 interop, which Wine only stubs: [Wine bug 51523](https://bugs.winehq.org/show_bug.cgi?id=51523); the game's Proton
  report: [Proton #3676](https://github.com/ValveSoftware/Proton/issues/3676)); DLSS/DLAA in No Man's Sky VR judders under Proton, FSR 2 works (the game's
  Proton report: [Proton #438](https://github.com/ValveSoftware/Proton/issues/438)).
- **Pairing the Oculus Remote** with `riftlinux-setup pair` is not tested yet (Touch controllers are). Firmware
  updates are not needed (no new CV1 firmware in years) and are not planned.
- **NVIDIA on Wayland with a VRR monitor:** the lower part of the headset's picture can tear unless VRR is hidden
  from the driver (NVIDIA's bug, [open-gpu-kernel-modules #992](https://github.com/NVIDIA/open-gpu-kernel-modules/issues/992), open); see [Wayland](#wayland).
- **GNOME with HDR on:** when the headset connects or disconnects (SteamVR starting or stopping, the headset taken
  off or put on) GNOME can leave the desktop's colours washed out; switching HDR off and on in Settings, Displays
  restores them (a GNOME bug, not the driver, likely [mutter #5083](https://gitlab.gnome.org/GNOME/mutter/-/work_items/5083), fixed for GNOME 51; KDE Plasma
  keeps them).

## Branches

| Branch | What |
| --- | --- |
| `main` | RiftLinux (what this README describes), built on Monado's commit `ec188bb13`; upstream fixes come in from Monado's own repository |

## Developing

- Sessions can be recorded (`riftlinux-setup record on`) and replayed through the tracker
  (`monado-constellation-replay`).
- Code style: Monado's (`.clang-format`, `scripts/format-project.sh`).
- Changes that belong upstream are offered to Monado.

## Credits

**RiftLinux is a modified Monado, specialized for full Oculus Rift CV1 support**, by Aditya Mehra (2026). The work
that makes the CV1 work is this project's own:

- the Touch tracking rework and the head tracking fixes;
- two-sensor calibration kept per sensor, and the moved-sensor watch;
- the ground truth: USB captures of the Oculus runtime on Windows, with our tracker scored on Oculus' own camera frames;
- the CV1's display and radio set up as the Oculus runtime sets them;
- the SteamVR driver with direct mode;
- the picture: Auto-HDR, HDR games and video, motion smoothing, the CV1's true resolution;
- Wayland;
- pairing controllers on Linux, checked against USB captures of the Oculus software pairing;
- measuring the sensors again with the floor kept;
- the packages, and the Rift tab and notices in the headset.

Code from others in this tree (each file keeps its authors' notices):

- [Monado][] (Collabora and contributors, BSL-1.0, <https://gitlab.freedesktop.org/monado/monado>): the OpenXR runtime,
  compositor and service RiftLinux is built on.
- Monado's Rift CV1 driver, sensor driver and constellation tracker as found upstream (Beyley Cardellio and Collabora;
  tracker code from Jan Schmidt's OpenHMD work, sensor code by Philipp Zabel), reworked here.
- Two unmerged upstream drafts, taken and completed here: !3015 (Beyley Cardellio: rig calibration, sensor fusion) and
  !2941 (MarkerWicker: Touch controllers in the tracker).
- OKLab and its gamut-cusp code (Björn Ottosson, MIT) in the Auto-HDR shader.
- The controller pairing protocol from ouvrt (Philipp Zabel, LGPL-2.1-or-later OR BSL-1.0), in `riftlinux-setup`.
- stb_image (Sean Barrett, MIT or Unlicense), reading the notice pictures in RiftLinux's own runtime.
- Valve's OpenVR driver headers (BSD-3-Clause) and Steam Runtime SDK, used to build the SteamVR driver.

## License

Code: BSL-1.0, as Monado (see `LICENSE` and the SPDX headers; some external code under its own licenses in
`LICENSES/`). This README: CC-BY-4.0, adapted from Monado's.

## Contributing, security, conduct

- How to report a problem or send a change: [CONTRIBUTING.md](CONTRIBUTING.md). Every commit needs a sign-off
  (`git commit -s`).
- Security problems: privately, as described in [SECURITY.md](SECURITY.md), not in a public issue.
- Everyone in the project's spaces follows the [Code of Conduct](CODE_OF_CONDUCT.md).
- How decisions are made: [GOVERNANCE.md](GOVERNANCE.md).

## Trademarks

Oculus, Rift, Touch and Constellation are trademarks of Meta Platforms, Inc. Steam and SteamVR are trademarks of
Valve Corporation. RiftLinux is an independent project, not affiliated with, endorsed or sponsored by Meta or Valve;
the names are used only to say which hardware and software it works with.

[Monado]: https://monado.freedesktop.org/
