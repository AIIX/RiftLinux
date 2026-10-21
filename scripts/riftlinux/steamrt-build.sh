#!/bin/sh
# SPDX-FileCopyrightText: 2026 Aditya Mehra <aix.m@outlook.com>
# SPDX-License-Identifier: BSL-1.0
#
# Builds the SteamVR driver (driver_monado.so) and RiftLinux's own OpenXR runtime (monado-service and
# libopenxr_monado.so) in Valve's Steam Runtime 3 "sniper" SDK, with the libraries the runtime does not have (Ceres,
# hidapi, Eigen) built from source and linked in statically; the Rift path needs no OpenCV. SteamVR started from
# Steam runs in that runtime's container; a driver built on the host needs host libraries there.
# libjpeg (sensors on USB 2) is built from source and linked in too: distributions name it differently
# (libjpeg.so.62 on Debian and Fedora, libjpeg.so.8 on Ubuntu and Arch), and the native runtime runs on the system,
# outside Steam's container. No systemd library either (its only use is socket activation, not used): the runtime
# also starts on distributions without systemd. What every distribution provides under the same name (libusb,
# libudev, D-Bus, Vulkan, X11, xcb, Wayland, libdrm, SDL2, zlib) stays dynamic.
#
# Usage: steamrt-build.sh <out dir>        (run on the build machine, with Docker; the fork is this script's ../..)
# Result: <out dir>/package/rift-cv1-steamvr/ (driver.vrdrivermanifest, bin/linux64/driver_monado.so, resources/)
#         <out dir>/package/rift-cv1-native/  (monado-service, libopenxr_monado.so, openxr_riftlinux.json)
set -e
OUT=$(realpath -m "${1:?out dir}")
SRC=$(cd "$(dirname "$0")/../.." && pwd)
IMAGE=registry.gitlab.steamos.cloud/steamrt/sniper/sdk:latest
mkdir -p "$OUT/dl" "$OUT/deps" "$OUT/build"

# Sources, fetched on the host so the container needs no network.
fetch() { [ -s "$OUT/dl/$2" ] || curl -sSL -o "$OUT/dl/$2" "$1"; }
fetch https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz eigen-3.4.0.tar.gz
fetch https://github.com/libusb/hidapi/archive/refs/tags/hidapi-0.14.0.tar.gz hidapi-0.14.0.tar.gz
fetch https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz ceres-2.2.0.tar.gz
fetch https://www.nasm.us/pub/nasm/releasebuilds/2.16.03/nasm-2.16.03.tar.xz nasm-2.16.03.tar.xz
fetch https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/3.2.0/libjpeg-turbo-3.2.0.tar.gz libjpeg-turbo-3.2.0.tar.gz
# Wayland protocols 1.38 (the SDK has 1.20; Monado's Wayland direct mode needs drm-lease-v1, 1.22 or newer).
fetch https://gitlab.freedesktop.org/wayland/wayland-protocols/-/releases/1.38/downloads/wayland-protocols-1.38.tar.xz wayland-protocols-1.38.tar.xz

docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$SRC:/src:ro" -v "$OUT:/out" "$IMAGE" sh -ec '
J=$(nproc)
export CC=gcc-14 CXX=g++-14
D=/out/deps
COMMON="-G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$D -DCMAKE_PREFIX_PATH=$D
	-DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF"
cd /tmp

if [ ! -e $D/share/eigen3/cmake/Eigen3Config.cmake ]; then
	tar xf /out/dl/eigen-3.4.0.tar.gz
	cmake -S eigen-3.4.0 -B eigen-b $COMMON -DEIGEN_BUILD_DOC=OFF -DEIGEN_BUILD_PKGCONFIG=ON
	cmake --install eigen-b
fi

if [ ! -e $D/lib/libhidapi-libusb.a ]; then
	tar xf /out/dl/hidapi-0.14.0.tar.gz
	cmake -S hidapi-hidapi-0.14.0 -B hidapi-b $COMMON -DHIDAPI_WITH_LIBUSB=ON -DHIDAPI_WITH_HIDRAW=ON
	cmake --build hidapi-b -j$J && cmake --install hidapi-b
fi

# Ceres with its own minimal logging, no gflags, no SuiteSparse, no LAPACK: Eigen sparse only. Warnings and errors
# only: miniglog otherwise prints every solver iteration, thousands of lines a second in the terminal SteamVR ran in.
# The marker file makes an older, chatty build rebuild.
if [ ! -e $D/lib/libceres.a ] || [ ! -e $D/.ceres-quiet ]; then
	tar xf /out/dl/ceres-2.2.0.tar.gz
	cmake -S ceres-solver-2.2.0 -B ceres-b $COMMON -DMINIGLOG=ON -DMINIGLOG_MAX_LOG_LEVEL=-1 -DGFLAGS=OFF -DSUITESPARSE=OFF -DLAPACK=OFF \
		-DEIGENSPARSE=ON -DUSE_CUDA=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DPROVIDE_UNINSTALL_TARGET=OFF
	cmake --build ceres-b -j$J && cmake --install ceres-b && touch $D/.ceres-quiet
fi

# libjpeg-turbo, static, with its SIMD code (NASM, which the SDK lacks, built first). The libjpeg 6b interface
# (the one of libjpeg.so.62), as Monado expects.
if [ ! -x $D/bin/nasm ]; then
	tar xf /out/dl/nasm-2.16.03.tar.xz
	(cd nasm-2.16.03 && ./configure -q --prefix=$D && make -j$J nasm >/dev/null && install -D nasm $D/bin/nasm)
fi
if [ ! -e $D/lib/libjpeg.a ]; then
	tar xf /out/dl/libjpeg-turbo-3.2.0.tar.gz
	cmake -S libjpeg-turbo-3.2.0 -B ljt-b $COMMON -DENABLE_SHARED=OFF -DENABLE_STATIC=ON -DWITH_TURBOJPEG=OFF \
		-DREQUIRE_SIMD=ON -DCMAKE_ASM_NASM_COMPILER=$D/bin/nasm -DCMAKE_INSTALL_LIBDIR=lib
	cmake --build ljt-b -j$J && cmake --install ljt-b
fi

# One headset, one driver (RiftLinux is specialised for the Rift CV1): only the Rift, its sensors and the UVC
# camera driver they need; every other Monado driver off (none probes for devices inside SteamVR).
if [ ! -e $D/share/pkgconfig/wayland-protocols.pc ]; then
	tar xf /out/dl/wayland-protocols-1.38.tar.xz -C /tmp
	mkdir -p $D/share/wayland-protocols $D/share/pkgconfig
	cp -r /tmp/wayland-protocols-1.38/stable /tmp/wayland-protocols-1.38/staging /tmp/wayland-protocols-1.38/unstable \
		$D/share/wayland-protocols/
	printf "pkgdatadir=$D/share/wayland-protocols\nName: Wayland Protocols\nDescription: Wayland protocol files\nVersion: 1.38\n" \
		> $D/share/pkgconfig/wayland-protocols.pc
fi
# -U "JPEG_*": an older build directory found the shared libjpeg of the SDK; now the static one in $D.
# -U "*SYSTEMD*": the options that depend on the systemd library fall back to off with it.
cmake -S /src -B /out/build -G Ninja -U "*WAYLAND_PROTOCOLS*" -U "JPEG_*" -U "*SYSTEMD*" -DXRT_HAVE_WAYLAND_DIRECT=ON \
	-DXRT_HAVE_SYSTEMD=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_PREFIX_PATH=$D \
	-DXRT_HAVE_SYSTEM_CJSON=OFF -DXRT_FEATURE_STEAMVR_PLUGIN=ON \
	-DXRT_BUILD_DRIVER_RIFT=ON -DXRT_BUILD_DRIVER_RIFT_SENSOR=ON -DXRT_BUILD_DRIVER_UVC=ON -DXRT_BUILD_DRIVER_SIMULATED=OFF \
	-DXRT_BUILD_DRIVER_ANDROID=OFF -DXRT_BUILD_DRIVER_ARDUINO=OFF -DXRT_BUILD_DRIVER_BLUBUR_S1=OFF \
	-DXRT_BUILD_DRIVER_CONTACTGLOVE=OFF -DXRT_BUILD_DRIVER_DAYDREAM=OFF -DXRT_BUILD_DRIVER_DEPTHAI=OFF \
	-DXRT_BUILD_DRIVER_EUROC=OFF -DXRT_BUILD_DRIVER_HANDTRACKING=OFF -DXRT_BUILD_DRIVER_HDK=OFF \
	-DXRT_BUILD_DRIVER_HYDRA=OFF -DXRT_BUILD_DRIVER_ILLIXR=OFF -DXRT_BUILD_DRIVER_NS=OFF \
	-DXRT_BUILD_DRIVER_OHMD=OFF -DXRT_BUILD_DRIVER_OPENGLOVES=OFF -DXRT_BUILD_DRIVER_PSMV=OFF \
	-DXRT_BUILD_DRIVER_PSSENSE=OFF -DXRT_BUILD_DRIVER_PSVR=OFF -DXRT_BUILD_DRIVER_PSVR2=OFF \
	-DXRT_BUILD_DRIVER_QWERTY=OFF -DXRT_BUILD_DRIVER_REALSENSE=OFF -DXRT_BUILD_DRIVER_REMOTE=OFF \
	-DXRT_BUILD_DRIVER_RIFT_S=OFF -DXRT_BUILD_DRIVER_ROKID=OFF -DXRT_BUILD_DRIVER_SIMULAVR=OFF \
	-DXRT_BUILD_DRIVER_SOLARXR=OFF -DXRT_BUILD_DRIVER_STEAMVR_LIGHTHOUSE=OFF \
	-DXRT_BUILD_DRIVER_SURVIVE=OFF -DXRT_BUILD_DRIVER_TWRAP=OFF -DXRT_BUILD_DRIVER_ULV2=OFF \
	-DXRT_BUILD_DRIVER_ULV5=OFF -DXRT_BUILD_DRIVER_VF=OFF -DXRT_BUILD_DRIVER_VIVE=OFF \
	-DXRT_BUILD_DRIVER_WMR=OFF -DXRT_BUILD_DRIVER_XREAL_AIR=OFF
cmake --build /out/build -j$J --target driver_monado monado-service openxr_monado
'

PKG="$OUT/package/rift-cv1-steamvr"
rm -rf "$PKG"
mkdir -p "$PKG"
cp -r "$OUT/build/steamvr-monado/." "$PKG/"
# The setup tool and udev rules travel with the driver.
cp "$SRC/scripts/riftlinux/riftlinux-setup" "$PKG/" && mkdir -p "$PKG/udev" && cp "$SRC/scripts/riftlinux/udev/70-rift-cv1.rules" "$PKG/udev/"
strip --strip-unneeded "$PKG/bin/linux64/driver_monado.so"
echo "package: $PKG"
du -sh "$PKG"
readelf -d "$PKG/bin/linux64/driver_monado.so" | grep NEEDED

# The native runtime: the service, the OpenXR library apps load, its manifest, and libjpeg-turbo's licence (linked in).
NAT="$OUT/package/rift-cv1-native"
rm -rf "$NAT"
mkdir -p "$NAT"
cp "$OUT/build/src/xrt/targets/service/monado-service" "$OUT/build/src/xrt/targets/openxr/libopenxr_monado.so" "$NAT/"
strip "$NAT/monado-service"
strip --strip-unneeded "$NAT/libopenxr_monado.so"
printf '{\n  "file_format_version": "1.0.0",\n  "runtime": {"name": "RiftLinux", "library_path": "./libopenxr_monado.so"}\n}\n' \
	> "$NAT/openxr_riftlinux.json"
cp "$OUT/deps/share/doc/libjpeg-turbo/LICENSE.md" "$NAT/libjpeg-turbo-LICENSE.md"
echo "native runtime: $NAT"
du -sh "$NAT"
readelf -d "$NAT/monado-service" "$NAT/libopenxr_monado.so" | grep NEEDED | sort -u
