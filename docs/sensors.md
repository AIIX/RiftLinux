<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Oculus Rift CV1 sensors on Linux with RiftLinux: placing and measuring them, room scale with a sensor behind you, moved or knocked sensors. -->

# Oculus Rift CV1 sensors on Linux: placing and measuring them

The Rift CV1 is tracked from outside: its **Constellation sensors** are cameras that see the infrared LEDs on the
headset and on the Touch controllers. RiftLinux finds each device in the sensors' images, combines what every sensor
sees with the device's own motion sensors (IMU), and so gives SteamVR and OpenXR applications full 6DoF positions.
For that it needs to know where each sensor stands: it **measures** them, saves their places, and notices when one
is moved.

## Where to put the sensors

- Both sensors in front of where you play, a distance apart, each turned so it sees that space.
- The headset in view of **both** sensors where you sit or stand: that is where tracking is best, and what measuring
  needs.
- On something that does not move or wobble. A knocked sensor is noticed and placed again (below), but tracking is
  steadier when they stay put.
- Each sensor on a USB 3 port; three on the same USB 3 controller work.

Bright light is fine: lamps and sun spots that never move are left out by the tracker.

## Measuring the sensors

The first time you put the headset on, RiftLinux measures where the sensors are, and a blue notice says so. It needs
views of the headset from different directions:

> **Face the sensors and slowly turn your head left and right, then up and down, and keep going until the notice
> goes away.**

Meanwhile the controllers are hidden (they need the sensors' places) and your view is kept level. When the notice
says *Sensors measured*, the controllers appear and the places are saved in `~/.config/monado/constellation/rig.json`.
Every later start uses them.

## A sensor moved or knocked

You do not need to do anything. RiftLinux compares what the sensors see all the time; when one has moved, an amber
notice says *Sensor N moved*, tracking goes on with the other sensors, and the moved one is placed again from the
headset's position and saved. A green notice, *Sensor N placed again*, says when it is done. This works in the middle
of a game, without a restart.

## Measuring again by hand

If tracking still seems off after you moved sensors (for example, after moving both), measure every sensor again:

- In the headset: right Oculus button, **Rift** tab, **Display & status**, **Measure the sensors manually**. The
  button is greyed while measuring is under way.
- From a terminal: `riftlinux-setup recalibrate`. With SteamVR running it starts at once; otherwise at SteamVR's next
  start.

Then do as for the first measuring: face the sensors and slowly turn your head left and right, then up and down,
until the notice goes away. Your room, SteamVR's walls and your floor stay where they were, kept by the sensors that
did not move. If every sensor moved, the view is placed on you again once you stand still, and the floor is carried
over from your usual eye height, which RiftLinux learns while you play.

## Room scale: a sensor behind you

- A sensor you add is placed by itself: start SteamVR (or the runtime), put the headset on and turn your head left and
  right until the measuring notice goes away. The sensors already placed keep tracking meanwhile.
- Sensors behind you see the LEDs on the back of the headband. The strap is adjustable, so where those LEDs sit is
  measured on your headset by itself, from the moments the front of the headset is tracked, then saved, as the Oculus
  runtime does. With your back to the sensors, the headset stays tracked.

## Questions

**How accurate is the tracking?** RiftLinux's tracker was run on the Oculus runtime's own camera frames, captured on
Windows, and scored against the poses the Oculus runtime gave for the same frames: for each kind of movement it tracked
the controllers within 1-2 points of the share Oculus tracked, by day and by night.

**Do I need the Oculus app's sensor setup?** No. Measuring happens in the headset, on Linux.

**Where is the calibration stored, and how do I start over?** In `~/.config/monado/constellation/rig.json`. To start
over, use *Measure the sensors manually* or `riftlinux-setup recalibrate`; the previous file is kept next to it as
`rig.json.before-recalibration`.
