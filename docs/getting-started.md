<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Set up an Oculus Rift CV1 on Linux: what you need, connecting the headset and sensors, installing RiftLinux, and the first start in SteamVR. -->

# Getting started: set up the Oculus Rift CV1 on Linux

This page takes you from a Rift CV1 in its box to playing SteamVR games on Linux with RiftLinux: what you need, how
to connect it, how to install the driver, and what happens the first time you put the headset on.

## What you need

- **An Oculus Rift CV1** with its **Oculus Touch controllers** and **two Constellation sensors**, or three for room
  scale. Only the CV1 is supported: not the Rift S, the DK1 or DK2, or a Quest.
- **A 64-bit Intel or AMD computer** (x86-64) running Linux. The packages refuse to install on ARM.
- **A graphics card with a Vulkan driver and an HDMI port.** Tested on NVIDIA (drivers 580 and 615). AMD and Intel
  graphics should work (Vulkan) but have not been tested yet; reports are welcome.
- **Steam and SteamVR** (SteamVR 2.17 or newer), from your distribution, Valve, Flatpak or Snap. SteamVR is not needed
  for [OpenXR applications without SteamVR](openxr-without-steamvr.md).
- **Any desktop:** X11 or Wayland. Tested on GNOME (X11 and Wayland) and KDE Plasma (Wayland).

Nothing else: no Windows, no Oculus app, no Meta account, no OpenHMD, Envision or other VR software.

## Connect the Rift

1. **Headset:** its HDMI plug into your graphics card, its USB plug into a USB 3 port of the computer.
2. **Sensors:** each into a USB 3 port (three on the same USB 3 controller work).
3. **Controllers:** each needs an AA battery. Controllers that came with the headset, or were used with it on Windows,
   are already paired with it. A new or replacement controller needs pairing once:
   [Touch controllers and pairing](controllers.md).

Place the sensors where both can see the space you play in; [Sensors](sensors.md) has more on this.

## Install the driver

1. **Download** the package for your distribution and `SHA256SUMS` from the
   [Releases page](https://github.com/AIIX/RiftLinux/releases):

   | Distribution | File |
   | --- | --- |
   | Debian 12 and newer, Ubuntu 22.04 and newer, Linux Mint, Pop!_OS | `riftlinux_<version>_amd64.deb` |
   | Arch, CachyOS, EndeavourOS, Manjaro | `riftlinux-<version>-1-x86_64.pkg.tar.zst` |
   | Fedora 43 and newer | `riftlinux-<version>-1.x86_64.rpm` |
   | Any other x86-64 distribution | `riftlinux-<version>-x86_64.tar.gz` |

2. **Check the download**, in the folder it went to: `sha256sum --ignore-missing -c SHA256SUMS` must say `OK`.
3. **Install it:**

   ```bash
   sudo apt install ./riftlinux_<version>_amd64.deb                # Debian, Ubuntu, Linux Mint, Pop!_OS
   sudo pacman -U riftlinux-<version>-1-x86_64.pkg.tar.zst         # Arch, CachyOS, EndeavourOS, Manjaro
   sudo dnf install ./riftlinux-<version>-1.x86_64.rpm             # Fedora
   tar xf riftlinux-<version>-x86_64.tar.gz && sudo ./riftlinux-<version>-x86_64/install.sh   # any distribution
   ```

That is all there is to set up. The package gives your user access to the Rift's USB devices and registers the
driver with SteamVR for every user: at once for whoever is logged in, and again at each login and whenever the
headset is plugged in. SteamVR does not need to have run before. The driver gives SteamVR its room (no Room Setup),
points OpenXR games at SteamVR when no other OpenXR runtime is set, and starts in direct mode (SteamVR's frames go
through RiftLinux's own compositor; see [The picture](picture.md)).

## The first start

1. **Restart Steam once after installing** (Steam, Exit, and start it again from your desktop): Steam loads the part
   of RiftLinux that starts SteamVR as it starts. Start Steam on the desktop itself, not from a remote (SSH) login:
   from there the Rift's tracking and display threads do not get real-time priority.
2. **Put the headset on: SteamVR starts by itself** (or start it from your Library, or with
   `steam steam://rungameid/250820`). On its first start SteamVR may ask for your password once (for its compositor's
   priority). A welcome panel shows how the Oculus button works. It is shown once.
3. **Measure the sensors.** A blue notice, *Measuring where the sensors are*, appears. **Face the sensors and slowly
   turn your head left and right, then up and down, and keep going until the notice goes away.** The sensors are
   placed from these views of the headset. Meanwhile the controllers are hidden and your view is kept level. When it
   is done, the controllers appear and the sensors' places are saved for next time
   (`~/.config/monado/constellation/rig.json`). The next starts use the saved places.
4. **Set the floor.** Press the **right Oculus button**: SteamVR's dashboard opens on the **Rift** tab. Lay a
   controller on the floor, point at **Set floor** with the other one and press the trigger. The **Height** slider
   below it raises or lowers your view if you want.
5. **Recentre** whenever the view is not in front of you: **hold the right Oculus button for a second.**

Now start a game from SteamVR's library or from Steam. Linux VR games, Windows VR games (through Proton) and OpenXR
games all work the same way. Start games once SteamVR runs: a VR game started from Steam while SteamVR is not running
opens on the monitor (put the headset on first).

## Upgrade and remove

- **Upgrade:** install the newer package the same way, then restart Steam once. Every user's copy of the driver
  follows at their next login or when the headset is plugged in.
- **Remove:** `sudo apt remove riftlinux`, `sudo pacman -R riftlinux`, `sudo dnf remove riftlinux`, or
  `sudo ./uninstall.sh` in the unpacked tarball's folder. The driver is unregistered from SteamVR. Your settings and
  the sensors' calibration stay in `~/.config/riftlinux` and `~/.config/monado`; delete those folders to remove them
  too.

## Next

- [The Rift tab](rift-tab.md): volume, microphone, floor, Auto-HDR, motion smoothing, status.
- [Troubleshooting](troubleshooting.md), if something does not work: start with `riftlinux-setup check`.
