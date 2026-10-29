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

**`riftlinux-setup fix`** puts in place the configuration RiftLinux is tested with, all of it at once (close SteamVR
first): SteamVR as the OpenXR and the OpenVR runtime, this driver as SteamVR's only headset driver (another CV1 setup's
SteamVR-OpenHMD, ALVR or a copy in SteamVR's own drivers folder is switched off, their files stay), SteamVR's settings
games need, direct mode on, the devices' rules and USB power, and what other setups leave behind: `XR_RUNTIME_JSON`,
`VR_OVERRIDE` or `VR_CONFIG_PATH` in your login files or a game's launch options, Proton forced on SteamVR itself,
another VR service (Monado, WiVRn, ouvrt) holding the Rift, a headset left in pairing mode. It lists every change and
asks once; every file it changes is kept next to it as a `.bak-riftlinux-<time>` copy. Changes to Steam's own files
need Steam closed too, and steps that need root ask for your password.

**`riftlinux-setup fix last-known`** is for RiftLinux's own settings file when it cannot be read after an edit by
hand: the broken file is kept aside and the last good settings are put back ([Settings](settings.md)).

## SteamVR and the driver

**A setting does not apply, or a command says the settings file "is not valid JSON".**
`~/.config/riftlinux/settings.json` was edited by hand and cannot be read. RiftLinux uses the last settings it could
read until then and writes nothing into the file; `riftlinux-setup check` shows the line and column. Fix it by hand,
or run `riftlinux-setup fix last-known` to put the last good settings back (the broken file is kept).

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

**A game does not start from inside VR, or waits forever.**
Steam asks some questions before a game's first start (an agreement, a launch option, a third-party account) on the
monitor, not in the headset; SteamVR's Desktop tab is empty on Linux, so take the headset off and answer them there.
Some games also ask on their own window first. A game whose anti-cheat its developer has not enabled for Linux
starts and is then removed from online play: its own log says so, and no driver can change that.

**SteamVR shows "SteamVR has encountered an unexpected problem", or the headset stays dark after a game quit.**
SteamVR's compositor stopped (seen with NVIDIA as a game quit or as SteamVR left its standby) and SteamVR does not start
it again by itself. RiftLinux notices it, closes SteamVR and, with Steam running, has Steam start it again (once in ten
minutes). If that does not happen, close SteamVR and start it again; if it does not close, log out and in.

**SteamVR Home does not open, and SteamVR's menu stays open with "Resume".**
Run `riftlinux-setup check`. If it reports leaked GPU handles in the desktop session, log out and back in (or
restart): the desktop is holding thousands of them, every program started from it gets a copy, and SteamVR Home then
cannot open its files. Seen with GNOME on NVIDIA after recording the screen (OBS).

**With direct mode off, SteamVR stops when the headset is put down.**
Seen on KDE Plasma (Wayland): SteamVR's own compositor loses the display when the headset's display switches off.
Keep direct mode on: `riftlinux-setup direct-mode on`.

**SteamVR's standby ends when the headset is moved or picked up (for example on a bed someone moves on).**
That is the `pick-up` standby setting (SteamVR's own rule, as on Windows). `riftlinux-setup standby worn` has only
putting the headset on end it ([Settings](settings.md#steamvrs-standby)).

**The picture comes on while the headset lies on a bed or is held in the hands.**
RiftLinux counts a headset that moves and turns as a head does for 20 seconds as worn, for face shields and pads the
wear sensor cannot see through. Held and turned in the hands for that long it counts too. To need longer,
`riftlinux-setup wear-sensor motion low` (30 seconds); with a normal face pad, `riftlinux-setup wear-sensor motion
off` counts the wear sensor alone ([Settings](settings.md#the-wear-sensor-face-shields-pads-and-glasses)).

**SteamVR starts when you pick up the headset, and you did not want it to.**
Steam starts SteamVR when the Rift is put on or picked up; picked up only to put it away, SteamVR quits again by
itself when the headset is not put on within two minutes. To have only putting it on start SteamVR:
`riftlinux-setup autostart trigger put-on`. If you use RiftLinux's own runtime without SteamVR, or
start SteamVR yourself: `riftlinux-setup autostart off` ([Settings](settings.md#steamvr-starting-when-you-put-the-headset-on)).

**A game started from Steam runs on the monitor instead of in the headset.**
SteamVR was not running when the game started. Put the headset on first (SteamVR then starts by itself), or start
SteamVR from Steam, then the game. `riftlinux-setup autostart status` says whether SteamVR starts when you put the
headset on.
If SteamVR was running, another runtime takes the game: `riftlinux-setup check` names it, and
`riftlinux-setup fix` puts SteamVR back.

**Beat Saber (or another OpenXR game under Proton) says OpenXR is not installed or not set up.**
The game was started while SteamVR was not running: Proton then turns OpenXR off for that game (its own safeguard,
since SteamVR's OpenXR start would hang), with any headset. Quit the game, put the headset on (SteamVR then starts by
itself), and start the game again. If SteamVR was running, `riftlinux-setup check` names what else takes OpenXR games
away, and `riftlinux-setup fix` puts SteamVR back.

**A game added to Steam yourself ("Add a Non-Steam Game", run with Proton) does not start from SteamVR's library.**
Steam gives SteamVR the game's Windows program to start, which Linux cannot run; SteamVR gives up and starts Home
again. RiftLinux then asks Steam to start the game as Steam's own Play button does (with Proton), so it starts a
moment later. With 0.1.2 and earlier, start it from Steam's library (or *View Details*, *Play* in SteamVR) instead.

## The picture

**The headset is dark.**
It is not on your head: its picture is black (the light stays white), and after a while black its display switches
off (the light orange) to protect the OLED. Put it on: the picture comes back at once.

**The picture goes dark, or SteamVR pauses, while you wear the headset.**
The wear sensor between the lenses does not see your face: a face shield, a thicker face pad or a covered sensor
window. RiftLinux then counts the headset as worn once it has moved and turned as a head does for 20 seconds, so the
picture comes back after that; `riftlinux-setup wear-sensor motion high` makes it 8 seconds. Better: see what the
sensor reads with `riftlinux-setup wear-sensor test`, then set its levels to fit, or turn it off:
[Settings](settings.md#the-wear-sensor-face-shields-pads-and-glasses).

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

**The desktop's sound stays on the Rift's headphones after SteamVR crashed.** The next SteamVR session puts your own
output and microphone back as it ends; or choose them in your desktop's sound settings.

## RiftLinux's own runtime (OpenXR without SteamVR)

**"The RiftLinux runtime did not start: ..."** The message gives the reason. The most common: *SteamVR is running and
holds the headset*: close SteamVR first.

**An application is stuck or has no way to quit.** Rift tab, *Settings*, **Close the application**.

**The runtime keeps running after the application.** `riftlinux-runtime --stop`, or right-click **RiftLinux
Runtime** in the app menu, **Stop RiftLinux Runtime**.

## Logs and reporting a problem

- RiftLinux's own runtime: `~/.cache/riftlinux/runtime.log`.
- The driver in SteamVR: SteamVR's logs folder inside your Steam folder (`~/.local/share/Steam/logs` or
  `~/.steam/debian-installation/logs`): `vrserver.txt`, and `vrstartup-linux.txt` for the driver's own messages
  (tracking, the display, a settings file it cannot read; lines starting `monado:` or `RiftLinux:`).
- Registration with SteamVR: `~/.cache/riftlinux/register.log`.

To report a problem, open an issue as described in [CONTRIBUTING.md](../CONTRIBUTING.md) with what you did, what you
saw, the output of `riftlinux-setup check`, and the log of that session.

For a tracking problem, record a session: run `riftlinux-setup record on`, start SteamVR and do what shows the
problem, then attach the recording from `~/.cache/riftlinux` and SteamVR's startup log
(`~/.steam/steam/logs/vrstartup-linux.txt`). Turn it off afterwards with `riftlinux-setup record off`.
