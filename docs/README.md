<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: User guide for RiftLinux, the Linux driver for the Oculus Rift CV1: install, first start, sensors, Touch controllers, SteamVR, OpenXR and troubleshooting. -->

# RiftLinux user guide: the Oculus Rift CV1 on Linux

RiftLinux is the open-source Linux driver for the **Oculus Rift CV1**: the headset, its Constellation sensors and its
Oculus Touch controllers, with full 6DoF tracking, in **SteamVR** and in **OpenXR** applications. It is a modified
Monado, specialized for full Rift CV1 support. You install one package; there is no Windows, Oculus app, Meta account,
OpenHMD or Room Setup involved.

This guide explains how to set it up and use it. The [README](../README.md) on the project's front page has the
overview, the comparison with other ways of running a CV1 on Linux, the build instructions and the full settings table.

## Pages

| Page | What it covers |
| --- | --- |
| [Getting started](getting-started.md) | What you need, connecting the Rift, installing the package, the first start, the floor |
| [The Rift tab](rift-tab.md) | The menu on the right Oculus button: every setting, the status page, the performance numbers |
| [Sensors](sensors.md) | Placing the Constellation sensors, measuring them, what happens when one is moved |
| [Touch controllers and pairing](controllers.md) | Pairing without Windows, batteries, buttons, finger poses, haptics |
| [The picture](picture.md) | Direct mode, Auto-HDR, HDR games and video, motion smoothing (ASW-style), render size |
| [OpenXR without SteamVR](openxr-without-steamvr.md) | RiftLinux's own runtime: app menu, right-click "Run with the Rift", `riftlinux-run`, WayVR |
| [Troubleshooting](troubleshooting.md) | A symptom, its cause and the fix; `riftlinux-setup check`; where the logs are |
| [FAQ](faq.md) | Short answers: does the CV1 work on Linux, which driver to use, Wayland, AMD, Proton, Steam Deck |

## In one minute

1. Download the package for your distribution from the [Releases page](https://github.com/AIIX/RiftLinux/releases)
   and install it (`sudo apt install ./riftlinux_<version>_amd64.deb`, `sudo pacman -U ...`, `sudo dnf install ...`).
2. Plug in the Rift (HDMI and USB) and the sensors (USB), and start SteamVR from Steam.
3. Put the headset on. Face the sensors and slowly turn your head left and right, then up and down, until the
   measuring notice goes away. Then press the right Oculus button, open the Rift tab and press **Set floor** with a
   controller lying on the floor.

Details for each step: [Getting started](getting-started.md).
