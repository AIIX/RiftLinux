<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->

# Contributing to RiftLinux

Thank you for helping. RiftLinux is a driver for one headset, the Oculus Rift CV1, its sensors and its Touch
controllers, so most help comes as bug reports from people with the hardware, and as changes tested on it. This
document says what a useful report contains and what a change needs before it is merged.

## Quick links

- **Something does not work?** Open a bug report: see [Reporting a problem](#reporting-a-problem).
- **An idea?** Open a feature request before writing code: see [Proposing a feature](#proposing-a-feature).
- **A security problem?** Do not open a public issue: see [SECURITY.md](SECURITY.md).
- **A change ready?** See [Sending a change](#sending-a-change).

Everyone in the project's spaces follows the [Code of Conduct](CODE_OF_CONDUCT.md).

## Reporting a problem

A report we can act on has these parts:

1. **What you did**, step by step: which game or app, what you did in it, when the problem appeared.
2. **What you expected**, and **what happened instead**, with any message the headset, SteamVR or the desktop showed.
3. **Your setup**:
   - the RiftLinux version (`cat /usr/lib/riftlinux/steamvr/rift-cv1/VERSION`);
   - the distribution, the desktop and its session (for example Ubuntu 25.04, GNOME, Wayland);
   - the graphics card and driver version (for example NVIDIA RTX 3080, driver 615.78);
   - the SteamVR version, and how many sensors, on which USB ports (USB 2 or 3).
4. **The output of `riftlinux-setup --json check`**: it shows the state of everything the driver depends on.
5. **SteamVR's logs** from the session with the problem: `~/.local/share/Steam/logs/vrserver.txt` (the driver's
   lines start with `monado:`), the driver's own log `~/.steam/steam/logs/vrstartup-linux.txt` and, for picture
   problems, `vrcompositor.txt`. Attach them or paste the part around the problem.
6. **What you already tried** (optional, but it saves a round trip).

The logs contain the serial numbers of your headset and sensors and your user name in file paths. That is fine to
share; remove anything you prefer to keep private.

For tracking problems (jitter, jumps, a lost controller), say where the sensors stand (height, distance, angle) and
whether it is day or night in the room, and attach a recording of a session that shows it: run
`riftlinux-setup record on`, start SteamVR and do what shows the problem, then attach the newest
`~/.cache/riftlinux/riftlinux-*.ctdata` and `~/.steam/steam/logs/vrstartup-linux.txt` (the tracker's details).
Recording changes nothing you see; turn it off afterwards with `riftlinux-setup record off`. Recordings are large:
if the issue does not take the file, say so and the maintainer asks for another way.

## Proposing a feature

Open a feature request first and describe what you want and why. RiftLinux has a few fixed rules (below); a short
discussion before the code saves work on both sides.

## Sending a change

### How

1. Fork the repository, make a branch from `main`, and open a pull request against `main`.
2. One change per pull request. For a large change, open a draft pull request early.
3. Fill in the pull request template; its checklist is checked automatically.

### Sign-off (required)

Every commit carries a `Signed-off-by:` line with your name and e-mail (`git commit -s`). With it you certify the
[Developer Certificate of Origin 1.1](LICENSES/LicenseRef-DCO-1.1.txt): that you wrote the change or have the right
to submit it under the project's licence. Only a person can certify it; if a change has several authors, each of
them signs off, and one sign-off matches the commit's author. A check on every pull request lists commits without
it. To add it to commits you already made:

```bash
git rebase --exec 'git commit --amend --no-edit -s' origin/main
git push --force-with-lease
```

### Commit messages

- One summary line, the area first, as in the existing history: `d/rift:` (the Rift driver), `t/constellation:`
  (tracking), `st/ovrd:` (the SteamVR driver), `a/render:` (the compositor's shaders and passes), `c/main:` (the
  compositor), `scripts/riftlinux:` (tools and packaging), `README:`.
- Then a body that explains why: what was wrong, how you found it, how you checked the fix.
- Plain words; a reader of the history should understand the change without anything else.

### Code

- Style: Monado's `.clang-format` for C and C++ (`scripts/format-project.sh`); the surrounding code's style elsewhere.
  No formatting changes to code you did not otherwise change.
- Comments say why, in plain words, and stand on their own.
- Every new or changed file has SPDX headers (`SPDX-FileCopyrightText: <year> <your name> <e-mail>` and
  `SPDX-License-Identifier: BSL-1.0` for code). `reuse lint` must pass.
- The build has no new warnings.

### Testing

Say in the pull request how you tested, on which hardware.

- **Every change:** the SteamVR driver builds (`scripts/riftlinux/steamrt-build.sh <out dir>`) and SteamVR starts
  with it; SteamVR's log shows no new errors.
- **Tracking:** replays of recorded sessions (`monado-constellation-replay`) show the same accepted solves as before
  where the change is not meant to alter them, and the change's effect where it is. Tracking changes are not merged
  without replay results.
- **Picture:** the panel's output before and after (`display.capture_output` in the settings file), on the same scene,
  the view's edges included.
- **Install and first run:** a fresh user (no settings, no SteamVR configuration) still gets a picture and tracking
  from the package alone.

## What we do not take

- **Other headsets.** RiftLinux supports the Rift CV1 only; for other headsets use upstream Monado or their own
  drivers.
- **A required setup step.** Installing the package must be enough: no command the user has to run, no Room Setup.
- **Telemetry**, phone-home or anything that sends data without the user asking for it.
- **Tracking changes without replay evidence**, or changes that make tracking worse in any recorded session.
- **Mass reformatting**, or unrelated changes in one pull request.

## Licence

RiftLinux is under the Boost Software License 1.0 (BSL-1.0), as Monado, which it is built on. Contributions are
accepted under the same licence; the sign-off certifies that you may contribute them.

## Upstream Monado

RiftLinux follows its own rules, in this file. A fix to Monado's general code (not specific to the CV1) can also be
offered to Monado at <https://gitlab.freedesktop.org/monado/monado>, under Monado's own rules.
