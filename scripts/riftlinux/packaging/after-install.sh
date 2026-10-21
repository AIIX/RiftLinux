#!/bin/sh
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
#
# Run as root after the package's files are in place (deb postinst, rpm %post, install.sh): device access now, and
# the driver registered at once for everyone logged in (later logins: the autostart entry and the udev rule).
udevadm control --reload-rules 2>/dev/null || true
udevadm trigger --subsystem-match=usb --attr-match=idVendor=2833 2>/dev/null || true
udevadm trigger --subsystem-match=hidraw 2>/dev/null || true
# The app menu's RiftLinux Runtime, its "Open With" and icon (packages' own hooks do this too; the tarball needs it).
update-desktop-database -q /usr/share/applications 2>/dev/null || true
gtk-update-icon-cache -q -t /usr/share/icons/hicolor 2>/dev/null || true
. /usr/lib/riftlinux/users.sh
for user in $(riftlinux_logged_in_users); do
	# Through the user's service manager where there is one (systemd), else as the user directly (OpenRC, runit,
	# s6 with elogind; a container).
	if [ -d /run/systemd/system ] && systemctl --user --machine="$user@.host" daemon-reload 2>/dev/null &&
		systemctl --user --machine="$user@.host" start riftlinux-register.service 2>/dev/null; then
		continue
	fi
	riftlinux_as_user "$user" /usr/bin/riftlinux-setup register --quiet 2>/dev/null || true
done
# NVIDIA with a monitor offering adaptive sync: on Wayland the headset tears. Only said, never changed here: the fix
# takes VRR away from the monitors, so it is the user's choice (riftlinux-setup nvidia-vrr conceal).
if [ -e /proc/driver/nvidia/version ] && /usr/bin/riftlinux-setup nvidia-vrr status 2>/dev/null | grep -q "offer(s) adaptive sync"; then
	cat <<'MSG'

RiftLinux: your NVIDIA GPU has a monitor offering adaptive sync (VRR). On a Wayland desktop NVIDIA then
shows the Rift's frames without vsync: the lower part of the picture tears and lags when you turn your
head (X11 is not affected). To fix it (your monitors no longer offer VRR):
    riftlinux-setup nvidia-vrr conceal      then reboot
Undo with `riftlinux-setup nvidia-vrr restore`. Details: /usr/share/doc/riftlinux/README.md

MSG
fi
exit 0
