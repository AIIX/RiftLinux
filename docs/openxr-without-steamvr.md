<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Run OpenXR apps on the Oculus Rift CV1 without SteamVR: RiftLinux's own runtime, right-click Run with the Rift, riftlinux-run, WayVR. -->

# OpenXR apps on the Rift CV1 without SteamVR

Besides the SteamVR driver, RiftLinux has **its own OpenXR runtime**: OpenXR applications run on the Rift CV1 directly,
with no SteamVR involved. It has the same tracking, the same picture (Auto-HDR, motion smoothing) and the same
[Rift tab](rift-tab.md) on the right Oculus button, plus a button to close the running application. It is part of
every package; nothing extra to install.

SteamVR and RiftLinux's runtime cannot hold the headset at the same time: close SteamVR first. SteamVR games are not
affected by any of this.

## Run an application with the Rift

**From the file manager:** right-click a program, an AppImage, a script or a `.desktop` launcher:

- **KDE Plasma (Dolphin):** **Run with the Rift**.
- **GNOME (Files):** **Open With**, **RiftLinux Runtime**.

The runtime starts for the application, and stops again when the application ends.

**From a terminal:**

```bash
riftlinux-run <program> [its arguments]
```

`riftlinux-run` does the same: it starts the runtime if it is not running, runs the program on it, and stops the
runtime afterwards if it started it. If the runtime cannot start, it says why (for example, that SteamVR is running).
It also works from an SSH login: it uses the desktop session of the computer it runs on, also when your SSH client
forwards X.

## Keep the runtime running

- **App menu:** start **RiftLinux Runtime**. A notification says it is running. To stop it, right-click it in the app
  menu: **Stop RiftLinux Runtime**.
- **Terminal:** `riftlinux-runtime` runs it in that terminal; press Enter to stop it. `riftlinux-runtime --status`
  says whether it runs, `riftlinux-runtime --stop` stops it.

While it runs, start OpenXR applications with:

```bash
XR_RUNTIME_JSON=/usr/lib/riftlinux/native/openxr_riftlinux.json <program>
```

The runtime writes its log to `~/.cache/riftlinux/runtime.log`.

## In the headset

- **Right Oculus button:** the Rift tab, as in SteamVR, drawn above every application.
- **Close the application** (Rift tab, *Settings* page, *Application*): asks the application to quit the OpenXR way,
  and ends it if it does not. The way out of an application that has no quit of its own.
- **Measure the sensors manually**, notices, floor and height, sound and microphone: as in SteamVR.
- When you put the headset on, the view is recentred on you. When you take it off, the display switches off; on
  Wayland the desktop gets the display back, and the runtime takes it again when you put the headset on.
- There is no boundary (SteamVR's Chaperone exists only in SteamVR).

## Example: WayVR, your desktop in VR

[WayVR](https://github.com/wayvr-org/wayvr) shows your desktop's windows in VR. With the Rift:

```bash
riftlinux-run ~/Downloads/WayVR-x86_64.AppImage --openxr --show
```

`--openxr` makes WayVR use OpenXR (RiftLinux's runtime) instead of SteamVR. Put the headset on before you start it,
so its panels open in front of you; the space around them is WayVR's own black background. To leave, use **Close the
application** in the Rift tab.

Being looked into: in RiftLinux's runtime WayVR's panels look softer than games, and not every controller button does
what WayVR's bindings expect.

## For developers

- Runtime manifest: `/usr/lib/riftlinux/native/openxr_riftlinux.json`; service:
  `/usr/lib/riftlinux/native/monado-service`.
- Interaction profile for the controllers: Oculus Touch (`/interaction_profiles/oculus/touch_controller`).
- HDR layers: the `XR_MNDX_composition_layer_hdr` extension (provisional), described in the README.
- Building the runtime from source: README, [Build](../README.md#build).
