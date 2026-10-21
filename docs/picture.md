<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->
<!-- description: The Rift CV1's picture on Linux: direct mode, Auto-HDR for its OLED panel, HDR games and video, ASW-style motion smoothing, true render size. -->

# The picture: direct mode, Auto-HDR, HDR and motion smoothing on the Rift CV1

RiftLinux drives the CV1's display with its own compositor, in SteamVR as well as in its own runtime. That is where
the lens correction happens, and where RiftLinux adds what the Oculus runtime on Windows does not have: Auto-HDR for
the CV1's OLED panel, HDR games and video, and 16-bit compositing with dithering. It also makes up for frames a game
misses, as Oculus' ASW does.

## Direct mode

In SteamVR, **direct mode** (on by default) means SteamVR hands each finished frame to RiftLinux's compositor, which
corrects it for the lenses and shows it on the headset. With direct mode off, SteamVR's own compositor drives the
headset and the picture is plain SDR: no Auto-HDR, HDR or motion smoothing. Switch it in the Rift tab (*Display &
status*, *Direct mode*, from SteamVR's next start) or with `riftlinux-setup direct-mode on|off`.

RiftLinux's own runtime always uses its compositor.

## Auto-HDR

The CV1 has an SDR OLED panel with true blacks. **Auto-HDR** maps everything you see onto it with deeper dark tones,
brighter highlights and richer colour (skin tones are spared), worked out in a perceptual colour space (OKLab). It
applies to every game, video and the dashboard. Rift tab: **Auto-HDR** on or off, **Strength** (contrast and
highlights) and **Colour**, both 50 % by default.

Windows' own Auto HDR does not reach a VR headset; on the Rift this exists only on Linux.

## HDR games and HDR video

- **HDR games:** a game that renders in 16-bit float images (light beyond white) is shown as HDR: its highlights are
  mapped onto the panel by their measured brightness and keep their colour, instead of clipping to white. A game that
  stays within white looks exactly as an 8-bit one.
- **HDR video:** HDR10, HDR10+ and Dolby Vision content is mapped by its own metadata (static, per scene, or the
  brightness measured while it plays). A player has to hand that metadata over: in SteamVR over the driver's socket, in
  native OpenXR with the `XR_MNDX_composition_layer_hdr` extension. The README's
  [Only on Linux](../README.md#windows-oculus-runtime-and-linux-riftlinux) section has the details for player authors.
  No player does this yet.
- **HDR Bright** (Rift tab): HDR games and videos placed brighter, like a TV's bright picture mode. It needs HDR
  content; the tab shows whether any is on screen.

Every HDR part has its own switch in `~/.config/riftlinux/settings.json` (README, [Settings](../README.md#settings)).

## Motion smoothing (ASW-style)

When a game cannot keep 90 frames a second, **motion smoothing** holds it at 45 (or 30) and makes the frames in between
from the motion between its last two frames, turned to your newest head position. This is what Oculus' ASW 1.0 does.
On NVIDIA RTX 30 and newer it uses the GPU's optical flow engine; on other GPUs a compute fallback that did as well on
recorded games. Switch it in the Rift tab (*Settings*, *Motion smoothing*); the *Performance* card shows how many
frames it makes.

ASW 2.0 on Windows also corrects sideways head movement with the game's depth image, for games that send one; that
is not in RiftLinux yet.

## Sharpness and render size

- Games are asked for the CV1's own 1:1 size at the lens centre (1332x1586 a view, Oculus' pixel density 1.0).
  SteamVR's supersampling setting works on top, and the Rift tab's **Render scale** (50-200 %, from SteamVR's next
  start) sets it for RiftLinux.
- The **fast path** (on) reads the game's image straight in the lens correction: one resampling instead of two, a
  sharper picture. It switches off by itself when overlays such as the dashboard are shown.
- The panel's near-black levels are lifted so the first shades above black stay visible, black stays black; the
  picture is dithered to the panel's 8-bit link, so dark gradients do not band.

## The panel itself

- Brightness and persistence (how long each row is lit) are set as the Oculus runtime sets them.
- The display switches off when you take the headset off (OLED burn-in) and on again at once when you put it on. The
  light on the front is white while it is on, orange while it is off.
