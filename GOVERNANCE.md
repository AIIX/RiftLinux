<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->

# Governance

## Who decides

RiftLinux is led by its maintainer, Aditya Mehra ([@AIIX](https://github.com/AIIX)), who decides what is merged,
what the project's rules are, and when a release is made. Decisions are explained in the pull request or issue they
concern.

## What guides the decisions

- **One headset.** RiftLinux supports the Oculus Rift CV1, its sensors, its Touch controllers and the Oculus Remote,
  and nothing else.
- **It works out of the box.** One package, no setup step, a picture and tracking from the first start, on any
  distribution that runs Steam.
- **Measured, not guessed.** Tracking changes are checked on recorded sessions; picture changes on captures of the
  panel's output; both on the hardware before they ship.
- **No regressions.** A change that makes any of the above worse is not merged, whatever else it improves.

## Releases

The maintainer cuts releases from `main`. Each release has a tag `vX.Y.Z` and the packages for every supported
distribution family attached on the Releases page.

## Monado

RiftLinux is built on Monado (from Monado's commit `ec188bb13`) and brings in upstream fixes from Monado's own
repository. It is an independent project with its own rules; it is not part of Monado and not run by Monado's
maintainers.

## Becoming a maintainer

Someone who has contributed steadily and well may be asked to help maintain the project. This file will then say who
maintains what.
