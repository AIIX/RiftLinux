<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: Oculus Touch controllers on Linux: pairing without Windows, buttons and finger poses in SteamVR, batteries and haptics, with RiftLinux. -->

# Oculus Touch controllers on Linux: pairing, buttons and batteries

RiftLinux tracks both **Oculus Touch** controllers optically from every sensor, fused with each controller's own
motion sensors, and hands SteamVR and OpenXR applications the same inputs and poses the Oculus driver gives them on
Windows. Pairing works on Linux too; no Windows or Oculus app is needed.

## Pairing a controller (no Windows needed)

Controllers that came with your headset, or were used with it on Windows, are already paired with it. A new,
replacement or second-hand controller has to be paired once:

1. **Close SteamVR** (the headset restarts into pairing mode, which SteamVR must not hold).
2. Run `riftlinux-setup pair` in a terminal.
3. On the controller, **hold the Oculus button and Y** (left controller) **or B** (right controller) until its light
   blinks. On the Oculus Remote, **hold its back button**.
4. The tool says when the controller is paired. Pair one controller at a time; run the tool again for the next.

The headset goes back to normal by itself afterwards. Pairing follows the same steps as the Oculus software (checked
against USB captures of it on Windows), and has been tested with both Touch controllers and the **Oculus Remote**.

## Waking a controller

Controllers go to sleep five minutes after you take the headset off. Press any button to wake one. The minutes are a
setting: [Settings](settings.md#when-the-controllers-go-to-sleep). If one does not show up after you put the
headset on, a notice says which: press a button on it, or check its battery.

## Batteries

Each Touch controller takes one AA battery. The **Rift tab**'s *Display & status* page shows each controller's battery
level as the controller reports it, and SteamVR shows it at the controllers too.

## Buttons, sticks and touch sensors

In SteamVR the controllers appear as **Oculus Touch** controllers with the same input profile as on Windows, so games
use their usual Touch bindings: A, B, X and Y; both thumbsticks (moved, clicked and touched); triggers and grips;
the thumbrests (touched); and the capacitive sensors on the buttons. All of them reach SteamVR, checked one by one.

The **right Oculus button** opens the [Rift tab](rift-tab.md), held for a second it recentres the view; the **left
menu button** opens SteamVR's dashboard.

OpenXR applications in [RiftLinux's own runtime](openxr-without-steamvr.md) get the Oculus Touch interaction profile.

## Finger poses and haptics

- **Finger poses:** a hand skeleton estimated from the controllers' touch sensors, the same kind the Oculus driver gives
  SteamVR on Windows. Games with SteamVR hands show your fingers resting, pointing or gripping (VTOL VR's hands work).
- **Haptics:** vibration in both controllers, tuned to SteamVR's short pulses.
- **Grip and aim poses** match the Oculus driver's, so held objects and laser pointers sit where they do on Windows.
