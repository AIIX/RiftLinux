<!--
SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
SPDX-License-Identifier: CC-BY-4.0
-->

# Security policy

How to report a security problem in RiftLinux, and what happens after. Please report privately, as below, and not in
a public issue: a public report of an unfixed problem puts every user at risk.

## In scope

- The SteamVR driver and RiftLinux's own runtime and compositor (`monado-service`, the OpenXR runtime library).
- `riftlinux-setup` and the packages, with their install and removal hooks.
- The udev rules: they give the user at the machine access to the Rift's, the sensors' and the controllers' USB
  devices.
- The driver's local socket (`~/.cache/riftlinux/driver.sock`) and its settings file
  (`~/.config/riftlinux/settings.json`): both are read by the driver while SteamVR runs.

## Not in scope

- Steam, SteamVR, Proton and SteamVR's compositor: report those to Valve.
- Graphics drivers: report those to their vendors.
- The headset's, sensors' and controllers' firmware.
- Problems in Monado's code that are not specific to RiftLinux: report them to Monado
  (<https://gitlab.freedesktop.org/monado/monado>) as well.
- An attacker who can already run programs as the logged-in user: such a person can already read and change
  everything the driver uses.

## How to report

- **Preferred:** GitHub's private vulnerability reporting: the repository's **Security** tab, **Report a
  vulnerability**. Only the maintainer sees it.
- **Or e-mail** <aix.m@outlook.com> with the subject starting **`[RiftLinux Security]`**.

Please include what you found and where (file, function or feature), how to reproduce it, what an attacker could do
with it, the RiftLinux version (`cat /usr/lib/riftlinux/steamvr/rift-cv1/VERSION`) or commit, and whether and how you
would like to be credited.

## After you report

- Reports are reviewed as time allows; there are no fixed response times.
- The maintainer may ask for more information.
- Whether a report is treated as a vulnerability, and when it is fixed, is the maintainer's decision.
- Fixes ship in normal releases. Whether a fix gets a release note or an advisory is the maintainer's decision; there
  is no CVE process by default.

Please keep the details private until a fix is released, and tell the maintainer first if you plan to publish
anything about it. If the problem is mentioned publicly, you can ask to be credited. There is no bug bounty.

## Rules for testing

Test only on your own computer, headset and data. No testing against anyone else's systems or devices, no
denial-of-service testing against anything but your own machine, no social engineering. Stop once the problem is
shown, and do not keep or share data you reached. This policy is not a legal agreement and authorises nothing that
is otherwise unlawful.
