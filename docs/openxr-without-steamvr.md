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

The runtime starts for the app and stops again when the app ends (or, if apps started after it are still running,
once the last of them ends).

**From a terminal:**

```bash
riftlinux-run <program> [its arguments]
```

`riftlinux-run` does the same: it starts the runtime if it is not running, runs the program on it, and stops the
runtime afterwards if it started it, once no other application uses it. If the runtime can't start, it tells you why (for example, that SteamVR is running).
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
- When you put the headset on, the view is recentred on you. When you take it off, the picture goes black, and after
  a while black the display switches off; on Wayland the desktop then gets the display back, and the runtime takes it
  again when you put the headset on.
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

## WebXR in Chromium

Chromium 157 and later run WebXR through OpenXR on Linux, so WebXR pages can use the Rift through RiftLinux's
runtime. Tested with Chromium from Flathub on NVIDIA. Keep the runtime running (see above), then:

1. Let Chromium's Flatpak reach the runtime (once):

   ```bash
   flatpak override --user org.chromium.Chromium --filesystem=host-os:ro --filesystem=xdg-run/monado_comp_ipc \
       --env=XR_RUNTIME_JSON=/run/host/usr/lib/riftlinux/native/openxr_riftlinux_flatpak.json
   ```

   `host-os:ro` lets the sandbox read the system's `/usr`, where the runtime's library is; `xdg-run/monado_comp_ipc`
   is the runtime's socket. To undo it: `flatpak override --user org.chromium.Chromium --nofilesystem=host-os
   --nofilesystem=xdg-run/monado_comp_ipc --unset-env=XR_RUNTIME_JSON`.

2. Start Chromium with OpenXR switched on (the settings under `chrome://flags` alone do not switch it on):

   ```bash
   flatpak run org.chromium.Chromium --enable-features=OpenXR,WebXRIncubations --force-webxr-runtime=openxr
   ```

With NVIDIA's driver, entering VR then fails ("An unknown error occurred"). Chromium's sandbox for its XR process
blocks a call NVIDIA's driver makes when it starts, and kills that process. That's for Chromium to fix. Adding
`--disable-seccomp-filter-sandbox` gets past it, but it turns off one of Chromium's protections for the whole browser,
so only use it for WebXR, in a separate profile (`--user-data-dir=<a folder of its own>`), not for everyday browsing.

A Chromium that is not a Flatpak works the same way without step 1, with
`XR_RUNTIME_JSON=/usr/lib/riftlinux/native/openxr_riftlinux_fullpath.json` (not tested here). Both files name the
runtime's library by its full path: Chromium's XR process does not accept the `./` path of `openxr_riftlinux.json`.

Chromium hands its pictures to the runtime upside down, described in a way that's only right for headsets whose view
reaches as far up as down. RiftLinux's runtime corrects that for Chromium, so the picture doesn't squeeze or stretch
as you look up and down.

## For developers

- Runtime manifest: `/usr/lib/riftlinux/native/openxr_riftlinux.json` (beside it `openxr_riftlinux_fullpath.json` and
  `openxr_riftlinux_flatpak.json`, naming the library by its full path, see WebXR above); service:
  `/usr/lib/riftlinux/native/monado-service`.
- Interaction profile for the controllers: Oculus Touch (`/interaction_profiles/oculus/touch_controller`).
- HDR layers: the `XR_MNDX_composition_layer_hdr` extension (provisional), described in the README.
- Several applications at once: the one started last is shown; when it ends, the one shown before it comes back.
  An application that offers switching (a home screen) can list the runtime's applications and bring one to the
  front with `libmonado` (`/usr/lib/riftlinux/native/libmonado.so`, header `monado.h` beside it:
  `mnd_root_update_client_list`, `mnd_root_get_client_name`, `mnd_root_set_client_primary`).
- Screenshots of what the headset shows: `mnd_root_take_screenshot(root, png_path, 0)` in `libmonado` writes one PNG
  of the whole view as seen in the headset (both eyes' views joined, every layer, frosted glass and the HDR mapping
  included, the lens correction left out). `png_path` is absolute and its folder must exist. The call blocks until
  the file is written, so call it from a thread of its own, not the one that renders frames. While the headset shows
  black (not worn), the picture is black.
- There is no controller calibration by hand: a Touch controller whose motion sensor is turned or biased against its
  LEDs is learned and corrected while it is used. `mnd_root_calibrate_controller`,
  `mnd_root_get_controller_calibration` and `mnd_root_forget_controller_calibration` stay in `libmonado` for programs
  built against them and return `MND_ERROR_OPERATION_FAILED`; `mnd_root_set_controller_calibration_quiet` does nothing.
- A lens test (a pattern to sharpen the lenses by, as the Oculus software's "Adjust Your Lenses"):
  `mnd_root_set_lens_pattern(root, true)` in `libmonado` has the runtime draw a green cross of one-pixel lines around
  each eye's lens centre, in panel pixels after the lens correction, so the lines land on panel pixels; `false`
  removes it, and it goes when the application that turned it on disconnects. Leave the middle of the view (81 by 81
  panel pixels around each lens centre) dark for it.
- Frosted glass (what is behind a quad or cylinder layer blurred): the `XR_MNDX_composition_layer_backdrop_blur`
  extension (provisional), described in the README; `riftlinux-setup backdrop-blur off` switches it off.
- Building the runtime from source: README, [Build](../README.md#build).
