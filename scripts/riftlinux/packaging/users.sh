# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
#
# Shared by after-install.sh and before-remove.sh (sourced, POSIX sh): who is logged in, and running a command as one
# of them, with or without systemd.

# The users logged in now, root left out: from logind (systemd-logind or elogind), else from utmp (who).
riftlinux_logged_in_users() {
	{
		if command -v loginctl >/dev/null 2>&1 && loginctl list-users --no-legend >/dev/null 2>&1; then
			loginctl list-users --no-legend 2>/dev/null | awk '{print $2}'
		else
			who 2>/dev/null | awk '{print $1}'
		fi
	} | sort -u | grep -v '^root$' || true
}

# Runs "$@" as user $1 with that user's HOME (runuser where it exists, else su).
riftlinux_as_user() {
	user=$1
	shift
	home=$(getent passwd "$user" | cut -d: -f6)
	[ -n "$home" ] || return 1
	if command -v runuser >/dev/null 2>&1; then
		runuser -u "$user" -- env HOME="$home" "$@"
	else
		su -s /bin/sh "$user" -c 'HOME="$0" exec "$@"' "$home" "$@"
	fi
}
