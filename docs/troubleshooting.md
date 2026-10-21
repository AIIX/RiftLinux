<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Fix Oculus Rift CV1 problems on Linux: SteamVR safe mode, a dark headset, Wayland tearing on NVIDIA, sensors, controllers, sound, logs. -->

# Troubleshooting the Oculus Rift CV1 on Linux

Start here: **`riftlinux-setup check`** looks at everything RiftLinux needs (the driver registered with SteamVR,
device access, SteamVR's safe-mode block, compositor priority, the room, direct mode, Wayland and NVIDIA settings) and
says what is wrong and why. It changes nothing. **`riftlinux-setup repair`** fixes what SteamVR updates or crashes
undo.

## SteamVR and the driver

**SteamVR does not show the headset, or says the "monado" add-on is blocked.**
After a crash SteamVR blocks the driver that was loaded (its safe mode). Close SteamVR, then run
`riftlinux-setup repair`, or in SteamVR: *Settings*, *Startup / Shutdown*, *Manage Add-ons*, switch **monado** on.
(The driver's name in SteamVR is *monado*.) Logging in again or plugging the headset in also lifts the block.

**A notice says "Real-time priority refused".**
Steam was started from a remote (SSH) login, where Linux does not give real-time priority. Start Steam from your
desktop. **"RealtimeKit is not installed"**: install your distribution's `rtkit` package.

**SteamVR does not start while the screen is locked.**
Steam waits for you to unlock the desktop before it starts SteamVR (Steam's own behaviour); unlock and it goes on.

**The Steam client crashes when a flat (non-VR) game opens while SteamVR runs.**
A Valve bug in SteamVR for Linux, not the driver ([SteamVR for Linux #963](https://github.com/ValveSoftware/SteamVR-for-Linux/issues/963): the same crash, reported for the Desktop tab). Keep VR games in VR.

**SteamVR's compositor crashes as SteamVR quits.**
After the session has ended, and harmless: SteamVR's own exit code. Nothing is lost.

## The picture

**The headset is dark and its light is orange.**
Its display is off because it is not on your head (to protect the OLED). Put it on: the light turns white and the
picture comes back at once.

**On Wayland with NVIDIA, the lower part of the picture tears and lags when you turn your head.**
NVIDIA's driver can show the headset without vsync when a monitor offers adaptive sync (VRR, FreeSync, G-Sync
Compatible): NVIDIA's bug, [open-gpu-kernel-modules #992](https://github.com/NVIDIA/open-gpu-kernel-modules/issues/992). Hide VRR from it: `riftlinux-setup nvidia-vrr conceal` and
reboot (your monitors then lose VRR; `riftlinux-setup nvidia-vrr restore` undoes it). The README's [Wayland](../README.md#wayland)
section has the manual steps.

**GNOME with HDR on: the desktop's colours look washed out after using the headset.**
GNOME can lose its HDR state when the headset connects or disconnects (likely [mutter #5083](https://gitlab.gnome.org/GNOME/mutter/-/work_items/5083), fixed
for GNOME 51). Switch HDR off and on in *Settings*, *Displays*. KDE Plasma keeps it.

**No Auto-HDR, HDR or motion smoothing.**
They need direct mode: `riftlinux-setup direct-mode status`, and `on` to switch it on (from SteamVR's next start).

## Tracking

**Measuring the sensors takes long, or the notice stays.**
Measuring needs views of the headset from different directions: face the sensors and slowly turn your head left and
right, then up and down, with the headset in view of both sensors. If the notice says *Measuring the sensors didn't
work*, it tries again by itself. See [Sensors](sensors.md).

**Tracking jumps or is off after moving the sensors.**
A moved sensor is placed again by itself. If tracking still seems off, measure again: Rift tab, *Display & status*,
**Measure the sensors manually** (or `riftlinux-setup recalibrate`).

**The view is not in front of me.** Hold the right Oculus button for a second.
**The floor is too high or too low.** Lay a controller on the floor and press **Set floor** in the Rift tab.

**A controller does not appear.**
Press a button on it to wake it, and check its battery (the Rift tab's *Display & status* page shows each one). While
the sensors are being measured, controllers are hidden until it is done. A new controller has to be paired once:
[Touch controllers and pairing](controllers.md).

**Sensors behind me do not track the headset.** The headband's rear LEDs are measured on your headset the first time
the front of it is tracked while you turn; turn slowly a full circle once, facing the front sensors part of the time.
If you changed the strap a lot, the measurement is checked again by itself.

## Sound

**No sound in the Rift's headphones, or the microphone is not used.**
RiftLinux makes the Rift's headphones the default output and its microphone the default input while the headset is in
use, and puts your own back afterwards. Check **Mute** and the levels in the Rift tab (*Settings*, *Sound*), and that
your desktop's sound settings show the Rift's audio device.

## RiftLinux's own runtime (OpenXR without SteamVR)

**"The RiftLinux runtime did not start: ..."** The message gives the reason. The most common: *SteamVR is running and
holds the headset*: close SteamVR first.

**An application is stuck or has no way to quit.** Rift tab, *Settings*, **Close the application**.

**The runtime keeps running after the application.** `riftlinux-runtime --stop`, or right-click **RiftLinux
Runtime** in the app menu, **Stop RiftLinux Runtime**.

## Logs and reporting a problem

- RiftLinux's own runtime: `~/.cache/riftlinux/runtime.log`.
- The driver in SteamVR: SteamVR's logs folder inside your Steam folder (`~/.local/share/Steam/logs` or
  `~/.steam/debian-installation/logs`): `vrserver.txt`, and `vrstartup-linux.txt` for tracking messages.
- Registration with SteamVR: `~/.cache/riftlinux/register.log`.

To report a problem, open an issue as described in [CONTRIBUTING.md](../CONTRIBUTING.md) with what you did, what you
saw, the output of `riftlinux-setup check`, and the log of that session.

For a tracking problem, record a session: run `riftlinux-setup record on`, start SteamVR and do what shows the
problem, then attach the recording from `~/.cache/riftlinux` and SteamVR's startup log
(`~/.steam/steam/logs/vrstartup-linux.txt`). Turn it off afterwards with `riftlinux-setup record off`.
