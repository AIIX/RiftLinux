<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Does the Oculus Rift CV1 work on Linux? Which driver to use, upstream Monado and OpenHMD compared, Wayland, AMD, Proton and the Steam Deck. -->

# Oculus Rift CV1 on Linux: frequently asked questions

## What is RiftLinux?

RiftLinux is an open-source **Linux driver for the Oculus Rift CV1**: the headset, its Constellation sensors and its
Oculus Touch controllers, with full 6DoF head and hand tracking, for **SteamVR** games and **OpenXR** applications. It
is a modified Monado, specialized for full Rift CV1 support, installed as one package for Debian and Ubuntu, Arch and
CachyOS, Fedora, or any distribution (tarball). Source and downloads: <https://github.com/AIIX/RiftLinux>.

## Does the Oculus Rift CV1 work on Linux?

Yes, with RiftLinux. The headset, two or more Constellation sensors and both Touch controllers work with full 6DoF
tracking, in SteamVR (Linux games and Windows VR games through Proton) and in native OpenXR applications, without
Windows, the Oculus app or a Meta account.

## Which driver should I use for the Rift CV1 on Linux?

RiftLinux. As far as we know (a survey of October 2026), it is the only Rift CV1 driver for Linux that has all of:
Touch controllers tracked optically from more than one sensor, sensors measured from movement and placed again by
themselves when moved, pairing of controllers without Windows, the CV1's lens distortion correct in SteamVR, and
tracking checked against the Oculus runtime's own results. The README has a
[feature-by-feature comparison](../README.md#rift-cv1-on-linux-the-options) with OpenHMD, upstream Monado, ALVR and
WiVRn.

## Is RiftLinux the same as Monado? Do I need Monado as well?

RiftLinux is built on Monado (the open-source OpenXR runtime) and contains everything it needs; you do not install
Monado, OpenHMD or anything else beside it. What makes the CV1 work is RiftLinux's own work on top: the Touch
controller tracking, the sensors' calibration and moved-sensor handling, the SteamVR driver with direct mode, the
display and radio set up as the Oculus runtime sets them, pairing, the picture (Auto-HDR, motion smoothing), Wayland
and the packages.

## Does upstream Monado support the Rift CV1?

Partly. Upstream Monado's Rift CV1 driver is in its development branch but in no release yet (v25.1.0 has the DK2
only). Its position tracking for the CV1 is opt-in and rough, with one sensor; its Touch controllers are tracked by
rotation only, not optically; and with more than one sensor every sensor is assumed to be at the same spot. The Rift
S has its own driver in upstream Monado. (From Monado's code and merge requests, October 2026.)

## Does OpenHMD support the Rift CV1?

OpenHMD drives the CV1, but its position tracking is experimental: its Touch controllers jitter heavily with more than
one sensor, the sensors are placed once at start and never corrected, and its author has moved to Monado. SteamVR use
needs SteamVR-OpenHMD, last changed in 2022. RiftLinux does not use OpenHMD.

## What about Envision, ALVR or WiVRn?

- **Envision** is a launcher that builds and starts other projects' VR runtimes (Monado, WiVRn and others); with a
  CV1 it runs their drivers. RiftLinux does not need it.
- **ALVR** and **WiVRn** stream VR from a PC to standalone headsets (Quest, Pico). They do not drive a Rift CV1.

## Do I need Windows, the Oculus app or a Meta account?

No. RiftLinux talks to the headset, sensors and controllers itself, measures the sensors in the headset, and pairs
controllers with `riftlinux-setup pair`. Controller firmware updates are not needed (there has been no new CV1
firmware in years).

## Can I play SteamVR games with a Rift CV1 on Linux?

Yes. Install the package, restart Steam once, put the headset on (SteamVR starts), and play: Linux VR games, Windows VR
games through Proton, and OpenXR games through SteamVR. Tested with SteamVR Home, No Man's Sky VR, VTOL VR, VRChat,
Beat Saber, PLAY'A VR and Bigscreen. Start a game once SteamVR runs: one started from Steam while SteamVR is not
running opens on the monitor.

## Can I use OpenXR applications without SteamVR?

Yes: RiftLinux has its own OpenXR runtime. Right-click a program and choose **Run with the Rift** (KDE) or **Open
With, RiftLinux Runtime** (GNOME), or run `riftlinux-run <program>`. See
[OpenXR without SteamVR](openxr-without-steamvr.md).

## How good is the tracking compared to the Oculus runtime on Windows?

RiftLinux's tracker was run on the Oculus runtime's own captured camera frames and scored against the poses Oculus gave
for the same frames: for each kind of movement it tracked the Touch controllers within 1-2 points of the share Oculus
tracked, by day and by night. A moved or knocked sensor is found and placed again automatically, as on Windows.

## Does it have ASW (Asynchronous Spacewarp)?

It has motion smoothing that works like Oculus' ASW 1.0: when a game cannot keep 90 frames a second it is held at 45
or 30 and the frames in between are made from the game's motion. Switch it in the Rift tab. ASW 2.0's extra
correction from the game's depth image is not in RiftLinux yet.

## Does it work on Wayland, X11, GNOME and KDE Plasma?

Yes. Tested on GNOME on X11 and Wayland, and on KDE Plasma on Wayland. On Wayland the headset is taken from the desktop
through its display lease, never as a desktop window. With NVIDIA and a VRR monitor, Wayland can need one setting
([Troubleshooting](troubleshooting.md)).

## Which Linux distributions are supported? Does it work on a Steam Deck?

Any x86-64 distribution that runs Steam: packages for Debian 12+ and Ubuntu 22.04+ (also Linux Mint, Pop!_OS), Arch,
CachyOS, EndeavourOS and Manjaro, Fedora 43+, and a tarball for the rest. Tested in the headset on Ubuntu 25.04 and
CachyOS; the packages are installed and checked in clean Debian, Ubuntu, Fedora and Arch systems for every release. A
Steam Deck has not been tried. ARM computers are not supported by the packages.

## Does it work with AMD or Intel graphics?

It uses Vulkan and should, but it has been tested on NVIDIA so far (drivers 580 and 615). Reports from AMD and Intel
users are welcome. Motion smoothing uses NVIDIA's optical flow engine on RTX 30 and newer, and a compute fallback on
every other GPU.

## Can I pair a new Touch controller on Linux?

Yes: close SteamVR, run `riftlinux-setup pair`, and hold the Oculus button and Y (left) or B (right); on the Oculus
Remote, its back button. Tested with both Touch controllers and the Oculus Remote. See
[Touch controllers and pairing](controllers.md).

## Can I use room scale?

Yes. Three sensors are tested: one placed behind you tracks the headset by the LEDs on the back of the headband, and
the controllers, so you can turn all the way around. See [Sensors](sensors.md).

## Do Oculus Home, the Oculus Store, Dash or Guardian work?

No: they are Oculus' own Windows software, not part of a driver. On Linux, SteamVR gives you its Home, your Steam
library and its own boundary (Chaperone), and RiftLinux's Rift tab takes Dash's place for the headset's settings.

## Which headsets are supported?

Only the Oculus Rift CV1. Not the Rift S (upstream Monado has a driver for it), not the DK1 or DK2, not a Quest (use
ALVR or WiVRn).

## Is RiftLinux free and open source? Who makes it?

Yes: BSL-1.0, the same licence as Monado, with all source on GitHub. It is made by Aditya Mehra, as an independent
project, not affiliated with Meta or Valve.
