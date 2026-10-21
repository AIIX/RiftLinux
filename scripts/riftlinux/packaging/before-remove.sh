#!/bin/sh
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
#
# Run as root before the package's files are removed (deb prerm "remove", rpm %preun when $1 is 0, uninstall.sh; not
# on upgrades): the driver unregistered from SteamVR for everyone logged in, so SteamVR stops loading their copy of it.
# Each user's copy, settings and calibration stay (nothing in a home is deleted); users not logged in can run
# `riftlinux-setup uninstall` before, or remove the entry from SteamVR's settings (Startup / Manage add-ons).
if [ -x /usr/bin/riftlinux-setup ] && [ -f /usr/lib/riftlinux/users.sh ]; then
	. /usr/lib/riftlinux/users.sh
	for user in $(riftlinux_logged_in_users); do
		riftlinux_as_user "$user" /usr/bin/riftlinux-setup --yes uninstall >/dev/null 2>&1 || true
	done
fi
exit 0
