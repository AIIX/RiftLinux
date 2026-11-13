<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: RiftLinux settings for the Oculus Rift CV1 on Linux: SteamVR starting when the headset is put on, Steam's menu when a game starts, the wear sensor (face shields, pads), when the display switches off, direct mode, recordings. -->

# Settings for the Oculus Rift CV1 on Linux

Most of RiftLinux needs no settings. The ones you may want to change are listed here, with the `riftlinux-setup`
command that changes each one. The commands write `~/.config/riftlinux/settings.json` for you, keep a copy of the
previous file next to it, and say when the change applies. The Rift tab in the headset writes its own settings to the
same file ([The Rift tab](rift-tab.md)); the README has [every key in the file](../README.md#settings).

If you edit the file by hand and it cannot be read any more (a missing comma or brace), nothing is lost: RiftLinux
leaves the file as it is, uses the last settings it could read (kept as `settings.json.last-known`) and says so in its
log; the commands and the Rift tab do not write into it. `riftlinux-setup check` shows where the file is broken, and
for keys it can read, which value is wrong and what RiftLinux uses instead. To put the last good settings back:

```
riftlinux-setup fix last-known
```

The broken file is kept as `settings.json.broken-<time>`. Without a last good copy the settings start from the
defaults, and `riftlinux-setup fix` then checks the rest.

## The commands

| Command | What it changes | Applies |
| --- | --- | --- |
| `riftlinux-setup autostart on\|off\|status` | Steam starts SteamVR when the Rift is put on or picked up (on by default) | At once |
| `riftlinux-setup autostart trigger put-on\|pick-up\|default` | What starts it: putting the Rift on only, or also picking it up (handled and turned, not a knock); default: picking up too, unless movement is left out (`wear-sensor motion off`) | At once |
| `riftlinux-setup autostart quit-unused on\|off` | A SteamVR started by picking the Rift up quits again when it is not put on (on by default) | Next start |
| `riftlinux-setup controller-tracking current\|0.1.4\|status` | The controllers' tracking as it was up to RiftLinux 0.1.4, if the current one doesn't suit your setup | Next start |
| `riftlinux-setup controller-coast <ms>\|status` | How long a controller the sensors no longer see keeps moving on its motion sensor before it stays where it is, 150 to 2000 ms (1000 by default, 300 before 0.1.5) | Next start |
| `riftlinux-setup controller-sleep 3\|5\|7\|off\|status` | Minutes after the headset is taken off before the Touch controllers may go to sleep (5 by default; the settings file takes 1 to 60); off keeps them awake | Next start |
| `riftlinux-setup controller-alignment status\|on\|off\|forget` | A controller whose motion sensor disagrees with its LEDs, learned and corrected by itself ([A controller that freezes or jumps](#a-controller-that-freezes-or-jumps-in-view)); `off` tracks it as before | Next start |
| `riftlinux-setup standby worn\|pick-up\|status` | When SteamVR's standby ends: the headset worn (default), or also moved, by SteamVR's own rule | Next start |
| `riftlinux-setup backdrop-blur on\|off\|status` | RiftLinux's own runtime: frosted glass (what is behind a layer blurred) for the layers an application asks it for; on by default, off: those layers without it. Nothing else uses it | Next runtime start |
| `riftlinux-setup lsfg-vk off\|on\|status` | lsfg-vk (Lossless Scaling's frame generation for Linux), when installed, kept out of SteamVR's server, its helper and RiftLinux's own runtime (off, the default), where it crashed SteamVR as RiftLinux's display started; on: allowed there as before. Games keep it either way | Next start |
| `riftlinux-setup ipd <mm>\|auto\|status` | The distance between your eyes set by hand (50 to 80 mm), for a headset whose IPD slider is not read; `auto` (the default) follows the slider | Next start (the Rift tab's IPD row: at once) |
| `riftlinux-setup game-menu close\|keep\|status` | Steam's menu, which Steam opens as a game starts: closed once the game shows (default), or kept open | Next start |
| `riftlinux-setup wear-sensor test` | Nothing: shows what the headset's wear sensor reads while you put it on and take it off | - |
| `riftlinux-setup wear-sensor on\|off\|status` | Whether the wear sensor is used; off counts the headset as worn whenever it runs | Next start |
| `riftlinux-setup wear-sensor levels <worn above> <not worn below>\|default` | The wear sensor's two levels (30 and 15 by default) | Next start |
| `riftlinux-setup wear-sensor motion high\|medium\|low\|<seconds>\|off\|on\|default` | Moving and turning as on a head counts as worn after 8 s (high), 20 s (medium, the default), 30 s (low) or the seconds given, for a face shield or pad the sensor cannot see through; off: the wear sensor alone | Next start |
| `riftlinux-setup display-off <seconds>\|never\|default\|status` | How long after the headset is taken off its display goes black (5 s by default, at least 1 s) | Next start |
| `riftlinux-setup not-worn black [<minutes>\|never]\|off\|default\|status` | A headset taken off: its display kept on showing black, switched off after a while black (10 minutes by default, at least 1), or switched off at once | Next start |
| `riftlinux-setup direct-mode on\|off\|status` | SteamVR's frames through RiftLinux's compositor (Auto-HDR, motion smoothing) or SteamVR's own | Next start |
| `riftlinux-setup fix last-known` | A settings file that cannot be read: kept aside, and the last good one put back | Next start |
| `riftlinux-setup record on\|off\|status` | Recording SteamVR sessions for a bug report ([Troubleshooting](troubleshooting.md#logs-and-reporting-a-problem)) | Next start |

"Next start" is the next start of SteamVR; the `wear-sensor`, `display-off`, `not-worn`, `controller-sleep` and
`controller-alignment` settings also apply at the next start of RiftLinux's own runtime when you use it without SteamVR.

Other commands, which change no setting: `riftlinux-setup check` (what is wrong and why; `--json` for a program to
read), `repair` (what SteamVR updates or crashes undo), `fix` (the configuration RiftLinux is tested with, in one go;
[Troubleshooting](troubleshooting.md)), `pair` (pairing controllers: [Touch controllers and pairing](controllers.md)),
`recalibrate` (the sensors measured again: [Sensors](sensors.md)), `nvidia-vrr status|conceal|restore` (the README's
Wayland section), `register` (what the package runs at each login and plug-in), `install` and `uninstall` (for a
driver folder of your own, without the package). `--yes` answers the one question `fix`, `repair`, `install` and
`uninstall` ask.

## SteamVR starting when you put the headset on

With Steam running, SteamVR starts by itself when you put the Rift on or pick it up, as the Oculus app does on
Windows. A game you then start from Steam goes straight into the headset. Steam starts SteamVR the same way as when
you click it in your library. Nothing happens while SteamVR or RiftLinux's own runtime is already using the headset,
and quitting SteamVR with the headset on does not start it again: take the headset off or put it down first.
Picking the headset up counts when it is handled and turned as on its way to a face; a knock on the table or someone
moving on the bed it lies on does not start SteamVR. Picking it up only to put it away starts SteamVR too; it quits
again by itself when the headset is not put on within two minutes and no game starts (the same for a SteamVR that Steam
started again after its compositor stopped).

With movement left out of wearing (`riftlinux-setup wear-sensor motion off`, the wear sensor alone), picking the
headset up does not start SteamVR either: only putting it on does.

To have only putting the headset on start SteamVR (its wear sensor, at the levels set below), not picking it up:

```
riftlinux-setup autostart trigger put-on
```

With the wear sensor turned off, putting the headset on cannot be seen, so picking it up still starts SteamVR.
`riftlinux-setup autostart trigger pick-up` makes picking it up start SteamVR whatever the wear-sensor setting;
`riftlinux-setup autostart trigger default` goes back to the default (picking up counts unless movement is left out). To keep a SteamVR that picking the headset up
started running even when the headset is not put on:

```
riftlinux-setup autostart quit-unused off
```

Steam watches for this from its start until it quits.

Turn it off if you use RiftLinux's own runtime without SteamVR, or prefer to start SteamVR yourself:

```
riftlinux-setup autostart off
```

## SteamVR's standby

SteamVR goes into standby while the headset is not worn and comes out of it when you put the headset on. Moving the
headset where it lies does not wake it. SteamVR's own rule also wakes it when the headset moves, as with the Oculus
software on Windows, where it comes out as soon as the headset is picked up. To have that:

```
riftlinux-setup standby pick-up
```

The picture still waits until the headset is worn: it stays black in your hands (held and turned for 20 seconds it
counts as worn, see [the wear sensor](#the-wear-sensor-face-shields-pads-and-glasses)). A headset lying on a bed
someone moves on wakes SteamVR too, which is why it is not the default. `riftlinux-setup standby worn` goes back to the
default.

## Steam's menu when a game starts

Steam opens its menu in the headset as a game starts. RiftLinux closes it once the game shows, so the game is not
behind it; the menu button opens it as usual, and a menu you open with it while the game runs stays open. To keep
Steam's menu open when a game starts:

```
riftlinux-setup game-menu keep
```

`riftlinux-setup game-menu close` goes back to the default. SteamVR's own menu as it starts stays open, in front of
you once the headset is on.

## When the controllers go to sleep

The Touch controllers stay awake while you wear the headset and for five minutes after you take it off; then they go
to sleep, which saves their batteries, and a button press wakes them. To change the minutes, or keep them awake for
as long as SteamVR runs (their batteries drain meanwhile):

```
riftlinux-setup controller-sleep 3
riftlinux-setup controller-sleep off
```

`riftlinux-setup controller-sleep 5` goes back to the default.

## A controller out of the sensors' view

When your hand goes behind your back, the sensors lose that controller. It keeps moving on its own motion sensor for up
to a second, like with the Oculus software, as long as it stays near where the sensors last saw it. After that it stays
where it is, still turning with your hand, until the sensors find it again. Then it's shown where it is, like with the
Oculus software, instead of sliding over from where it was held. To change how long it keeps moving:

```
riftlinux-setup controller-coast 1000   # the default
riftlinux-setup controller-coast 300    # as before 0.1.5
```

## A controller that freezes or jumps in view

The tracker checks every position it finds for a controller against the controller's own motion sensor, and throws
away anything that disagrees on which way is down. If that sensor sits slightly turned inside the controller, or its
accelerometer reads a little off, the two disagree all the time, and the controller freezes or jumps even in full view
of the sensors.

RiftLinux learns this while you play, from how the sensors see the controller, and corrects it. There's nothing for you
to do, just like with the Oculus software. A controller a few degrees off (3 or more, which some healthy ones read) only
has that check allow for its own offset, and the rest of its tracking stays as it was; this is learned anew each
session. One that clearly disagrees (6 degrees or more, as a faulty one does) is corrected throughout, and what is
learned for it is kept for that controller and checked again at every start.

```
riftlinux-setup controller-alignment status   # what is in use, for which controller
riftlinux-setup controller-alignment off      # every controller tracked as before (settings.json controllers.imu_align)
riftlinux-setup controller-alignment forget   # with SteamVR and the runtime closed: drop what was kept
```

## The wear sensor: face shields, pads and glasses

The CV1 has a sensor between the lenses that sees whether the headset is on a face. RiftLinux switches the display off
a few seconds after the headset is taken off (an OLED panel showing a still picture for long can burn in) and tells
SteamVR, which pauses then. A face shield, a thicker face pad or a covered or dirty sensor window can make the sensor
miss your face: the picture goes dark or SteamVR pauses while you wear the headset. RiftLinux also counts the
headset as worn once it has moved and turned as a head does for 20 seconds, and for as long as it keeps moving, so
with such a headset the picture comes 20 seconds after you pick it up to put it on. A headset held in the hands moves
like a head too, for the seconds it takes to put it on, and is not counted before; one lying on a bed someone moves
on shakes but does not turn, and is never counted. For the picture sooner (8 seconds) or later (30 seconds), or to
count the wear sensor alone:

```
riftlinux-setup wear-sensor motion high
riftlinux-setup wear-sensor motion low
riftlinux-setup wear-sensor motion off
```

`riftlinux-setup wear-sensor motion 30` sets the seconds; `riftlinux-setup wear-sensor motion default` goes back to
20 seconds (medium).

First see what the sensor reads, with SteamVR closed:

```
riftlinux-setup wear-sensor test
```

Put the headset on during the test, then take it off. Worn, the sensor should read clearly above the first level (30)
and off your head below the second (15). If it reads lower on your face, for example 20 to 25, set levels between
what it reads worn and what it reads off your head:

```
riftlinux-setup wear-sensor levels 18 8
```

If it reads the same on and off your face, turn it off:

```
riftlinux-setup wear-sensor off
```

The headset then counts as worn whenever SteamVR runs: the display never switches off by itself and SteamVR never
pauses. The panel stays lit while SteamVR runs, even on the desk, so quit SteamVR when you put the headset down for
longer. `riftlinux-setup wear-sensor on` turns it back on.

## When the headset is taken off

5 seconds after the headset is taken off its display goes black, and the moment it is on your head again the picture
is back. The display stays switched on, showing pure black: an OLED panel gives no light then, and nothing is lit that
could burn in. After 10 minutes black it switches off. Switching the display off takes the headset off its HDMI
connection, which the desktop sees as a monitor unplugged (and plugged in again when the headset is put on); GNOME 48
on NVIDIA keeps some graphics memory every time, so this is kept for longer breaks. In the first 30 seconds after
SteamVR or the runtime starts, the display stays on whatever the headset reads, so that the compositor can take it.

```
riftlinux-setup display-off 30          # black 30 seconds after it is taken off
riftlinux-setup not-worn black 20       # switched off after 20 minutes black
riftlinux-setup not-worn off            # switched off at once, as RiftLinux 0.1.2 did
```

`display-off never` keeps the picture on as long as SteamVR runs (the same burn-in risk as above); `not-worn black
never` keeps it black instead of switching it off; `default` goes back to the defaults.

## IPD: the distance between your eyes

The picture follows the headset's IPD slider as you move it, in SteamVR and in RiftLinux's own runtime, and the headset
shows the distance in its lower right corner as it changes. Set the slider where the picture looks sharpest.

On some headsets the slider's sensor doesn't report: the lenses move, but the distance stays at 57.5 mm (the Oculus
software on Windows shows the same number then). If that's yours, measure your IPD (with a ruler in front of a mirror,
or a phone app), move the lenses to match, and set the distance by hand in the Rift tab's **IPD** row (drag the
slider, and **Auto** follows the headset's slider again), or with:

```
riftlinux-setup ipd 63.5      # set by hand, in millimetres
riftlinux-setup ipd auto      # follow the headset's slider again (the default)
riftlinux-setup ipd test      # what the slider reads while you move it (SteamVR closed)
```

## The settings file

The commands above write these keys; you can also edit the file by hand while SteamVR is closed. A key that is not in
the file has its default.

```json
{
  "steamvr": {
    "start_when_put_on": true,
    "start_on_pick_up": true,
    "quit_if_not_put_on": true,
    "close_menu_for_games": true,
    "standby": "worn"
  },
  "controllers": {
    "sleep_after_min": 5,
    "coast_ms": 1000
  },
  "headset": {
    "wear_sensor": true,
    "worn_above": 30,
    "unworn_below": 15,
    "worn_by_motion": true,
    "worn_by_motion_after_s": 20,
    "display_off_after_s": 5,
    "not_worn": "black",
    "black_then_off_min": 10
  },
  "display": {
    "direct_mode": true
  }
}
```

| Key | Default | What it does |
| --- | --- | --- |
| `steamvr.start_when_put_on` | `true` | Steam starts SteamVR when the Rift is put on or picked up |
| `steamvr.start_on_pick_up` | as `headset.worn_by_motion` (`true`) | `false`: only putting the Rift on starts it (with the wear sensor on); not set: picking up counts unless movement is left out of wearing |
| `steamvr.quit_if_not_put_on` | `true` | A SteamVR that Steam started by itself (the Rift picked up, or SteamVR started again after its compositor stopped) quits again when the headset is not worn within two minutes and no game starts |
| `steamvr.close_menu_for_games` | `true` | Steam's menu, opened as a game starts, closes once the game shows; `false` keeps it |
| `steamvr.standby` | `"worn"` | SteamVR's standby ends when the headset is worn; `"pick-up"`: also when it moves, by SteamVR's own rule |
| `controllers.sleep_after_min` | `5` | Minutes (1 to 60) after the headset is taken off before the controllers may sleep; negative: kept awake while SteamVR runs |
| `controllers.tracking` | `"current"` | `"0.1.4"`: the controllers' tracking as it was up to RiftLinux 0.1.4 |
| `controllers.coast_ms` | `1000` | How long a controller the sensors no longer see keeps moving on its motion sensor before it stays where it is, 150 to 2000 ms (300 before 0.1.5) |
| `controllers.imu_align` | `true` | A controller whose motion sensor disagrees with its LEDs is learned and corrected by itself (a few degrees off: only the check of which way is down allows for it, learned anew each session); `false`: tracked as before ([A controller that freezes or jumps](#a-controller-that-freezes-or-jumps-in-view)) |
| `headset.wear_sensor` | `true` | `false`: the headset counts as worn whenever it runs |
| `headset.worn_above`, `headset.unworn_below` | `30`, `15` | The wear sensor's levels: worn above the first, not worn below the second, unchanged in between |
| `headset.worn_by_motion` | `true` | Moving as on a head counts as worn, whatever the wear sensor reads; `false`: the wear sensor alone |
| `headset.worn_by_motion_after_s` | `20` | Seconds of moving and turning as on a head before it counts as worn (1 to 60; `riftlinux-setup wear-sensor motion high` 8, `medium` 20, `low` 30) |
| `headset.display_off_after_s` | `5` | Seconds not worn before the display goes black (or off); negative: never |
| `headset.not_worn` | `"black"` | `"black"`: the display stays on showing black while the headset is not worn; `"off"`: it switches off |
| `headset.black_then_off_min` | `10` | With `"black"`: minutes black before the display switches off; negative: never |
| `headset.ipd_mm` | not set (the slider) | The distance between your eyes set by hand, 50 to 80 mm, for a headset whose IPD slider is not read; `0` or not set follows the slider (`riftlinux-setup ipd`) |
| `display.direct_mode` | `true` | SteamVR's frames through RiftLinux's compositor; `false`: SteamVR's own (`riftlinux-setup direct-mode`) |
| `display.backdrop_blur` | `true` | RiftLinux's own runtime: frosted glass for the layers an application asks it for; `false`: those layers without it (`riftlinux-setup backdrop-blur`) |
| `display.lsfg_vk` | `false` | lsfg-vk (Lossless Scaling's frame generation) kept out of SteamVR's server and RiftLinux's own runtime; `true`: allowed there as before (`riftlinux-setup lsfg-vk`) |

The picture, notice, tracking and recording keys are in the README's [settings table](../README.md#settings).
