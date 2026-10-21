#!/bin/sh
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
#
# Makes the release packages from a driver package built by steamrt-build.sh (one package install and the CV1
# works): a .deb (Debian, Ubuntu, Mint, Pop!_OS) and a .tar.gz with install.sh for every other distribution.
# Usage: make-packages.sh <steamrt-build out dir> <dest dir>
# RIFTLINUX_URL: the project's public page (the RiftLinux repository), put in the packages; left out while unset.
# RIFTLINUX_VERSION: the release's version (X.Y.Z) instead of one from git (needed when SRC is not a git checkout).
# Layout installed (same for both):
#   /usr/lib/riftlinux/steamvr/rift-cv1/        the SteamVR driver (each user gets a copy SteamVR's container can see)
#   /usr/lib/riftlinux/native/                  RiftLinux's own OpenXR runtime (monado-service, libopenxr_monado.so,
#                                               openxr_riftlinux.json), for OpenXR applications without SteamVR
#   /usr/bin/riftlinux-setup                    check / repair / pair; `register` runs by itself
#   /usr/bin/riftlinux-runtime                  starts (and --stop stops) the own runtime
#   /usr/bin/riftlinux-run                      runs an OpenXR application on it, starting and stopping it
#   /usr/share/applications/riftlinux-runtime.desktop   the app menu's "RiftLinux Runtime" (and "Open With")
#   /usr/share/kio/servicemenus/riftlinux-run.desktop   Dolphin's right-click "Run with the Rift"
#   /usr/share/icons/hicolor/256x256/apps/riftlinux.png
#   /usr/lib/udev/rules.d/70-rift-cv1.rules     device access (uaccess) and registration when the headset appears
#   /usr/lib/systemd/user/riftlinux-register.service
#   /etc/xdg/autostart/riftlinux-register.desktop
#   /usr/lib/riftlinux/after-install.sh         run as root after installing
#   /usr/lib/riftlinux/before-remove.sh         run as root before removing (not on upgrades)
# For distribution packagers: doc/riftlinux/PACKAGING.md.
set -e
OUT=$(realpath "${1:?steamrt-build out dir}")
DEST=$(realpath -m "${2:?dest dir}")
SRC=$(cd "$(dirname "$0")/../.." && pwd)
PKG="$OUT/package/rift-cv1-steamvr"
[ -f "$PKG/bin/linux64/driver_monado.so" ] || { echo "no driver package in $PKG (run steamrt-build.sh)"; exit 1; }
NAT="$OUT/package/rift-cv1-native"
[ -x "$NAT/monado-service" ] || { echo "no native runtime in $NAT (run steamrt-build.sh)"; exit 1; }
# A release sets RIFTLINUX_VERSION (X.Y.Z); a build from git gets X.Y.Z+gitDATE.COMMIT.
VER="${RIFTLINUX_VERSION:-0.1.0+git$(git -C "$SRC" log -1 --format=%cd --date=format:%Y%m%d).$(git -C "$SRC" rev-parse --short=9 HEAD)}"
URL="${RIFTLINUX_URL:-}"
STAGE=$(mktemp -d)
trap 'rm -rf "$STAGE"' EXIT
R="$STAGE/root"
install -d "$R/usr/lib/riftlinux/steamvr" "$R/usr/lib/riftlinux/native" "$R/usr/bin" "$R/usr/lib/udev/rules.d" \
	"$R/usr/lib/systemd/user" "$R/etc/xdg/autostart" "$R/usr/share/doc/riftlinux" "$R/usr/share/applications" \
	"$R/usr/share/kio/servicemenus" "$R/usr/share/icons/hicolor/256x256/apps"
cp -r "$PKG" "$R/usr/lib/riftlinux/steamvr/rift-cv1"
echo "$VER" > "$R/usr/lib/riftlinux/steamvr/rift-cv1/VERSION"
# The setup tool and rules live in /usr/bin and /usr/lib/udev; the copies steamrt-build.sh puts in the driver
# folder are left out here.
rm -rf "$R/usr/lib/riftlinux/steamvr/rift-cv1/riftlinux-setup" "$R/usr/lib/riftlinux/steamvr/rift-cv1/udev"
install -m 0755 "$SRC/scripts/riftlinux/riftlinux-setup" "$R/usr/bin/riftlinux-setup"
install -m 0755 "$NAT/monado-service" "$NAT/libopenxr_monado.so" "$R/usr/lib/riftlinux/native/"
install -m 0644 "$NAT/openxr_riftlinux.json" "$R/usr/lib/riftlinux/native/"
install -m 0755 "$SRC/scripts/riftlinux/packaging/riftlinux-runtime" "$R/usr/bin/riftlinux-runtime"
install -m 0755 "$SRC/scripts/riftlinux/packaging/riftlinux-run" "$R/usr/bin/riftlinux-run"
install -m 0644 "$SRC/scripts/riftlinux/packaging/riftlinux-runtime.desktop" "$R/usr/share/applications/"
install -m 0644 "$SRC/scripts/riftlinux/packaging/riftlinux-run.kde-servicemenu.desktop" \
	"$R/usr/share/kio/servicemenus/riftlinux-run.desktop"
install -m 0644 "$PKG/resources/notices/quick-tab.png" "$R/usr/share/icons/hicolor/256x256/apps/riftlinux.png"
install -m 0644 "$NAT/libjpeg-turbo-LICENSE.md" "$R/usr/share/doc/riftlinux/libjpeg-turbo-LICENSE.md"
install -m 0644 "$SRC/scripts/riftlinux/udev/70-rift-cv1.rules" "$R/usr/lib/udev/rules.d/"
install -m 0644 "$SRC/scripts/riftlinux/packaging/riftlinux-register.service" "$R/usr/lib/systemd/user/"
install -m 0644 "$SRC/scripts/riftlinux/packaging/riftlinux-register.desktop" "$R/etc/xdg/autostart/"
install -m 0755 "$SRC/scripts/riftlinux/packaging/after-install.sh" "$R/usr/lib/riftlinux/after-install.sh"
install -m 0755 "$SRC/scripts/riftlinux/packaging/before-remove.sh" "$R/usr/lib/riftlinux/before-remove.sh"
install -m 0644 "$SRC/scripts/riftlinux/packaging/users.sh" "$R/usr/lib/riftlinux/users.sh"
install -m 0644 "$SRC/doc/riftlinux/PACKAGING.md" "$R/usr/share/doc/riftlinux/PACKAGING.md"
install -m 0644 "$SRC/README.md" "$R/usr/share/doc/riftlinux/README.md"
cat > "$R/usr/share/doc/riftlinux/copyright" <<COPY
RiftLinux: a modified Monado, specialized for full Oculus Rift CV1 support (SteamVR and OpenXR).${URL:+ $URL}
Copyright 2026 Aditya Mehra <aix.m@outlook.com>; Monado: Collabora, Ltd. and the Monado contributors; see the SPDX
headers in the source. License: BSL-1.0 (Boost Software License 1.0); bundled Eigen (MPL-2.0), Ceres Solver
(BSD-3-Clause), hidapi (BSD-3-Clause) and libjpeg-turbo (IJG License, BSD-3-Clause, zlib; this software is based in
part on the work of the Independent JPEG Group; see libjpeg-turbo-LICENSE.md) linked in.
COPY
# System files: not writable by group or others.
chmod -R u+rwX,go+rX,go-w "$R"
mkdir -p "$DEST"

# .deb
D="$STAGE/deb"
cp -a "$R" "$D"
mkdir -p "$D/DEBIAN"
SIZE=$(du -sk "$D" | cut -f1)
cat > "$D/DEBIAN/control" <<CTRL
Package: riftlinux
Version: $VER
Architecture: amd64
Maintainer: Aditya Mehra <aix.m@outlook.com>
Installed-Size: $SIZE
Depends: python3, libc6 (>= 2.30), libudev1, libusb-1.0-0, libdbus-1-3, libvulkan1, libdrm2, libx11-6, libx11-xcb1,
 libxcb1, libxcb-randr0, libwayland-client0, libsdl2-2.0-0, libglx0
Recommends: rtkit, libcap2-bin, libnotify-bin
Section: misc
Priority: optional
Description: Oculus Rift CV1 driver for Linux (SteamVR and OpenXR)
 RiftLinux drives the Oculus Rift CV1 headset, its Constellation sensors and its
 Touch controllers on Linux: full 6DoF tracking, SteamVR and Proton games,
 native OpenXR, motion smoothing, Auto-HDR. Install it, plug in the Rift, start
 SteamVR from Steam: the driver registers itself for every user, gives SteamVR
 its room and points OpenXR games at SteamVR. No Windows or Oculus app needed.
CTRL
[ -z "$URL" ] || sed -i "/^Description:/i Homepage: $URL" "$D/DEBIAN/control"
printf '#!/bin/sh\nset -e\n[ "$1" = configure ] && /usr/lib/riftlinux/after-install.sh\nexit 0\n' > "$D/DEBIAN/postinst"
printf '#!/bin/sh\nset -e\n[ "$1" = remove ] && /usr/lib/riftlinux/before-remove.sh\nexit 0\n' > "$D/DEBIAN/prerm"
chmod 0755 "$D/DEBIAN/postinst" "$D/DEBIAN/prerm"
echo "/etc/xdg/autostart/riftlinux-register.desktop" > "$D/DEBIAN/conffiles"
fakeroot dpkg-deb --build "$D" "$DEST/riftlinux_${VER}_amd64.deb" >/dev/null 2>&1 || dpkg-deb --root-owner-group --build "$D" "$DEST/riftlinux_${VER}_amd64.deb" >/dev/null
echo "deb: $DEST/riftlinux_${VER}_amd64.deb"

# .tar.gz with install.sh / uninstall.sh
T="$STAGE/riftlinux-$VER-x86_64"
mkdir -p "$T"
cp -a "$R" "$T/root"
(cd "$R" && find . -type f -o -type l | sed 's|^\./|/|' | sort) > "$T/FILES"
cat > "$T/install.sh" <<'INST'
#!/bin/sh
# Installs RiftLinux on any distribution: run as root (sudo ./install.sh). Then plug in the Rift and start SteamVR.
set -e
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0"; exit 1; }
case "$(uname -m)" in
x86_64) ;;
*) echo "RiftLinux is built for x86-64 (64-bit Intel and AMD) computers; this one is $(uname -m). Nothing installed."
	exit 1 ;;
esac
cd "$(dirname "$0")"
cp -a root/. /
/usr/lib/riftlinux/after-install.sh
cp FILES /usr/lib/riftlinux/FILES
echo "RiftLinux installed. Plug in the Rift and start SteamVR from Steam."
# The SteamVR driver gets its libraries from Steam's runtime; the own runtime (riftlinux-runtime) from the system.
MISSING=$(ldd /usr/lib/riftlinux/native/monado-service /usr/lib/riftlinux/native/libopenxr_monado.so 2>/dev/null |
	awk '/not found/ {print $1}' | sort -u | tr '\n' ' ')
[ -z "$MISSING" ] || echo "For riftlinux-runtime (OpenXR without SteamVR), install your distribution's packages for: $MISSING"
INST
cat > "$T/uninstall.sh" <<'UNIN'
#!/bin/sh
# Removes what install.sh installed; the driver is unregistered from SteamVR for everyone logged in (each user's copy
# and settings stay).
set -e
[ "$(id -u)" = 0 ] || { echo "run as root: sudo $0"; exit 1; }
/usr/lib/riftlinux/before-remove.sh || true
while read -r f; do rm -f "$f"; done < /usr/lib/riftlinux/FILES
rm -rf /usr/lib/riftlinux
udevadm control --reload-rules 2>/dev/null || true
echo "RiftLinux removed."
UNIN
chmod 0755 "$T/install.sh" "$T/uninstall.sh"
tar -C "$STAGE" -czf "$DEST/riftlinux-$VER-x86_64.tar.gz" "riftlinux-$VER-x86_64"
echo "tar: $DEST/riftlinux-$VER-x86_64.tar.gz"

# Arch (CachyOS, EndeavourOS, Manjaro) and Fedora packages of the same files, each built by its own distribution's
# tools in a container (Docker; RIFTLINUX_NO_CONTAINERS=1 skips them). The PKGBUILD and the spec are left next to the
# packages: an AUR package or a COPR build needs only the tarball's address in them.
if [ -z "${RIFTLINUX_NO_CONTAINERS:-}" ] && command -v docker >/dev/null 2>&1; then
	# Build images with the packaging tools, made once and kept.
	docker image inspect riftlinux-pkg-arch >/dev/null 2>&1 ||
		printf 'FROM archlinux:latest\nRUN pacman -Syu --noconfirm --needed base-devel && useradd -m builder\n' |
		docker build -q -t riftlinux-pkg-arch - >/dev/null
	docker image inspect riftlinux-pkg-fedora >/dev/null 2>&1 ||
		printf 'FROM fedora:latest\nRUN dnf -y install rpm-build && dnf clean all\n' | docker build -q -t riftlinux-pkg-fedora - >/dev/null

	# Arch: pkgver takes no "+" or "-".
	AVER=$(echo "$VER" | tr '+-' '..')
	A="$STAGE/arch"
	mkdir -p "$A"
	cp "$DEST/riftlinux-$VER-x86_64.tar.gz" "$A/"
	cp "$SRC/scripts/riftlinux/packaging/arch/riftlinux.install" "$A/"
	SUM=$(sha256sum "$A/riftlinux-$VER-x86_64.tar.gz" | cut -d' ' -f1)
	cat > "$A/PKGBUILD" <<PKGB
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
# Maintainer: Aditya Mehra <aix.m@outlook.com>
# The prebuilt driver (built in Valve's Steam runtime SDK, loaded by SteamVR in its own container) and the setup tool.
pkgname=riftlinux
pkgver=$AVER
pkgrel=1
pkgdesc="Oculus Rift CV1 driver for Linux (SteamVR and OpenXR): tracking, Touch, Auto-HDR, motion smoothing"
arch=('x86_64')
url="$URL"
license=('BSL-1.0')
depends=('python' 'rtkit' 'glibc' 'systemd-libs' 'libusb' 'dbus' 'vulkan-icd-loader' 'libdrm' 'libx11' 'libxcb' 'wayland'
	'sdl2' 'libglvnd')
optdepends=('steam: SteamVR runs the driver' 'libnotify: notifications from the RiftLinux Runtime app entry')
install=riftlinux.install
options=('!strip' '!debug')
source=("riftlinux-$VER-x86_64.tar.gz")
sha256sums=('$SUM')

package() {
	cp -a "\$srcdir/riftlinux-$VER-x86_64/root/." "\$pkgdir/"
}
PKGB
	docker run --rm -v "$A:/build" riftlinux-pkg-arch sh -c \
		'chown -R builder /build && cd /build && su builder -c "makepkg --nodeps --noconfirm -f" >/build/makepkg.log 2>&1;
		 echo "makepkg exit $?"; chown -R '"$(id -u):$(id -g)"' /build' | tail -1
	for f in "$A"/riftlinux-*.pkg.tar.zst; do [ -f "$f" ] && cp "$f" "$DEST/" && echo "arch: $DEST/$(basename "$f")"; done
	cp "$A/PKGBUILD" "$DEST/PKGBUILD" && cp "$A/riftlinux.install" "$DEST/riftlinux.install"

	# Fedora (and openSUSE, with its own names): Version takes no "+" or "-"; the git part goes into Release.
	F="$STAGE/rpm"
	mkdir -p "$F/SOURCES" "$F/SPECS"
	cp -a "$R" "$F/SOURCES/root"
	RVER=$(echo "$VER" | sed 's/+.*//')
	case "$VER" in
	*+*) RREL="0.$(echo "$VER" | sed 's/^[^+]*+//' | tr '+-' '..')" ;;
	*) RREL=1 ;;
	esac
	cat > "$F/SPECS/riftlinux.spec" <<SPEC
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
# The prebuilt driver (built in Valve's Steam runtime SDK, loaded by SteamVR in its own container) and the setup tool.
%global debug_package %{nil}
%global __os_install_post %{nil}
Name:           riftlinux
Version:        $RVER
Release:        $RREL
Summary:        Oculus Rift CV1 driver for Linux (SteamVR and OpenXR)
License:        BSL-1.0
BuildArch:      x86_64
# The SteamVR driver's libraries are the Steam runtime's, not the system's, so no automatic requirements; the own
# runtime's (riftlinux-runtime) are listed by library name, valid on Fedora and openSUSE alike.
AutoReqProv:    no
Requires:       python3
Requires:       libudev.so.1()(64bit) libusb-1.0.so.0()(64bit) libdbus-1.so.3()(64bit) libvulkan.so.1()(64bit)
Requires:       libdrm.so.2()(64bit) libX11.so.6()(64bit) libX11-xcb.so.1()(64bit) libxcb.so.1()(64bit)
Requires:       libxcb-randr.so.0()(64bit) libwayland-client.so.0()(64bit) libSDL2-2.0.so.0()(64bit) libGLX.so.0()(64bit)
Recommends:     rtkit
Recommends:     libnotify

%description
RiftLinux drives the Oculus Rift CV1 headset, its Constellation sensors and its
Touch controllers on Linux: full 6DoF tracking, SteamVR and Proton games,
native OpenXR, motion smoothing, Auto-HDR. Install it, plug in the Rift, start
SteamVR from Steam: the driver registers itself for every user, gives SteamVR
its room and points OpenXR games at SteamVR. No Windows or Oculus app needed.

%install
cp -a %{_sourcedir}/root/. %{buildroot}/

%post
/usr/lib/riftlinux/after-install.sh

%preun
if [ \$1 -eq 0 ]; then /usr/lib/riftlinux/before-remove.sh; fi

%files
/usr/bin/riftlinux-setup
/usr/bin/riftlinux-runtime
/usr/bin/riftlinux-run
/usr/share/applications/riftlinux-runtime.desktop
/usr/share/kio/servicemenus/riftlinux-run.desktop
/usr/share/icons/hicolor/256x256/apps/riftlinux.png
/usr/lib/riftlinux
/usr/lib/udev/rules.d/70-rift-cv1.rules
/usr/lib/systemd/user/riftlinux-register.service
%config(noreplace) /etc/xdg/autostart/riftlinux-register.desktop
/usr/share/doc/riftlinux
SPEC
	[ -z "$URL" ] || sed -i "/^License:/a URL:            $URL" "$F/SPECS/riftlinux.spec"
	docker run --rm -v "$F:/rpm" riftlinux-pkg-fedora sh -c \
		'rpmbuild -bb --define "_topdir /rpm" /rpm/SPECS/riftlinux.spec >/rpm/rpmbuild.log 2>&1; echo "rpmbuild exit $?";
		 chown -R '"$(id -u):$(id -g)"' /rpm' | tail -1
	for f in "$F"/RPMS/x86_64/riftlinux-*.rpm; do [ -f "$f" ] && cp "$f" "$DEST/" && echo "rpm: $DEST/$(basename "$f")"; done
	cp "$F/SPECS/riftlinux.spec" "$DEST/riftlinux.spec"
fi
