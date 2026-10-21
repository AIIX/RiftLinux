<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: BSL-1.0

Keep every section. If an item does not apply, tick it and write "(does not apply: reason)".
-->

## What it does

<!-- One or two sentences. -->

## Why

<!-- The problem it solves, and how you found it. -->

## Issue

Closes #

## How it was tested

<!--
Required. Say on which hardware and system (GPU, distribution, desktop, X11 or Wayland), and paste the evidence:
the end of the build (scripts/riftlinux/steamrt-build.sh), the relevant lines of SteamVR's log, and for tracking
changes the replay comparison; for picture changes the before/after captures.
-->

```text
```

## Checklist

- [ ] The SteamVR driver builds without new warnings, and SteamVR starts with it.
- [ ] Tracking changes: replays of recorded sessions attached, and no recorded session gets worse.
- [ ] Picture changes: before/after captures of the same scene, the view's edges included.
- [ ] A fresh user still gets a picture and tracking from the package alone (no new setup step).
- [ ] Every commit is signed off (`git commit -s`).
- [ ] New and changed files have SPDX headers; `reuse lint` passes.
- [ ] Commit messages: one summary line with the area first, then a body that explains why.
- [ ] It does one thing, for the Rift CV1, and is not in CONTRIBUTING.md's "What we do not take".
