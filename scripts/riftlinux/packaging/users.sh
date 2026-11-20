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

# Runs "$@" as user $1 in a clean environment of that user's own (HOME, USER, PATH, the session's runtime folder),
# nothing of the installer's (runuser where it exists, else su).
riftlinux_as_user() {
	user=$1
	shift
	home=$(getent passwd "$user" | cut -d: -f6)
	uid=$(id -u "$user" 2>/dev/null)
	[ -n "$home" ] && [ -n "$uid" ] || return 1
	runtime=/run/user/$uid
	[ -d "$runtime" ] || runtime=
	if command -v runuser >/dev/null 2>&1; then
		runuser -u "$user" -- env -i HOME="$home" USER="$user" LOGNAME="$user" PATH=/usr/local/bin:/usr/bin:/bin \
			LANG=C.UTF-8 ${runtime:+XDG_RUNTIME_DIR="$runtime"} "$@"
	else
		su -s /bin/sh "$user" -c 'h=$1 u=$2 r=$3; shift 3; exec env -i HOME="$h" USER="$u" LOGNAME="$u" \
			PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 ${r:+XDG_RUNTIME_DIR="$r"} "$@"' sh "$home" "$user" "$runtime" "$@"
	fi
}
