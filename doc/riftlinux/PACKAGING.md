# Packaging RiftLinux (Oculus Rift CV1 driver) for a distribution

For distribution and repository maintainers (AUR, Fedora/COPR, openSUSE, Debian/Ubuntu PPAs, NixOS, Gentoo). The goal
of the project is that installing this one package is all a user does: restart Steam once, plug in the Rift, put it on
(SteamVR starts), and the headset, sensors and controllers work, as the Oculus software does on Windows. A package that follows this page
gives that; one that leaves out a piece (most often the per-user registration) leaves users with a dark headset.

The reference packages are made by `scripts/riftlinux/make-packages.sh` (a `.deb`, an Arch package, an `.rpm` and a
`.tar.gz` with `install.sh`). Copy their layout and hooks.

## 1. Build: in Valve's Steam Runtime, not with the distribution's toolchain

SteamVR runs its drivers inside Steam's container (pressure-vessel, Steam Runtime 3 "sniper"). That container has its
own `/usr`: it does not see the distribution's libraries, and its glibc and libstdc++ are older than most
distributions'. A `driver_monado.so` built with the distribution's compiler and libraries either needs libraries the
container does not have or newer symbol versions than it offers, and SteamVR fails to load it (or runs it only outside
the container, where games then show a grey screen).

So build the SteamVR driver in the sniper SDK image, as `scripts/riftlinux/steamrt-build.sh` does (Docker or Podman,
about 10 minutes the first time):

```bash
scripts/riftlinux/steamrt-build.sh /path/to/out      # -> /path/to/out/package/rift-cv1-steamvr and rift-cv1-native
scripts/riftlinux/make-packages.sh /path/to/out /path/to/dest
```

It builds only the Rift CV1 drivers (`XRT_BUILD_DRIVER_RIFT`, `_RIFT_SENSOR`, UVC; all others off), links Eigen, Ceres,
hidapi and libjpeg-turbo statically and needs nothing at run time beyond what the Steam Runtime ships. Where a build
service cannot run containers, package the release archive (a `-bin` package) instead of rebuilding.

The same build makes RiftLinux's own OpenXR runtime (`monado-service` and `libopenxr_monado.so`, started with
`riftlinux-runtime`, for OpenXR applications without SteamVR). It runs on the system, not in Steam's container, so it
needs the system's libraries. They are the same on every current distribution, with glibc 2.30 or newer:

| Library | Debian, Ubuntu | Arch | Fedora, openSUSE (rpm) |
| --- | --- | --- | --- |
| libudev.so.1 | libudev1 | systemd-libs | libudev.so.1()(64bit) |
| libusb-1.0.so.0 | libusb-1.0-0 | libusb | libusb-1.0.so.0()(64bit) |
| libdbus-1.so.3 | libdbus-1-3 | dbus | libdbus-1.so.3()(64bit) |
| libvulkan.so.1 | libvulkan1 | vulkan-icd-loader | libvulkan.so.1()(64bit) |
| libdrm.so.2 | libdrm2 | libdrm | libdrm.so.2()(64bit) |
| libX11.so.6, libX11-xcb.so.1 | libx11-6, libx11-xcb1 | libx11 | libX11.so.6()(64bit), libX11-xcb.so.1()(64bit) |
| libxcb.so.1, libxcb-randr.so.0 | libxcb1, libxcb-randr0 | libxcb | libxcb.so.1()(64bit), libxcb-randr.so.0()(64bit) |
| libwayland-client.so.0 | libwayland-client0 | wayland | libwayland-client.so.0()(64bit) |
| libSDL2-2.0.so.0 | libsdl2-2.0-0 | sdl2 | libSDL2-2.0.so.0()(64bit) |
| libGLX.so.0 | libglx0 | libglvnd | libGLX.so.0()(64bit) |

libjpeg is linked in (distributions name it differently: `libjpeg.so.62` on Debian and Fedora, `libjpeg.so.8` on
Ubuntu and Arch) and so is no dependency; neither is libsystemd (the service makes its own socket), so it also runs
without systemd. `riftlinux-setup check` lists any of these that are missing. Building the runtime with the
distribution's own toolchain instead is possible (the README's Build section), but the SteamVR driver must come from
the Steam Runtime build.

The packages are for x86-64 only (they are built in the x86-64 Steam Runtime SDK image); on other architectures they refuse to install, and `riftlinux-setup register` registers nothing.

## 2. What to install

| Path | What |
| --- | --- |
| `/usr/lib/riftlinux/steamvr/rift-cv1/` | The SteamVR driver folder as built (manifest, `bin/linux64/`, `bin/linux32/`, `resources/`) plus a `VERSION` file. Without `riftlinux-setup` and `udev/`, which go below. `bin/linux32/driver_monado.so` is the part Steam's 32-bit client loads as it starts (SteamVR's watchdog drivers): it starts SteamVR when the headset is put on; Steam loads a new one only when Steam restarts |
| `/usr/lib/riftlinux/native/` | The own runtime: `monado-service`, `libopenxr_monado.so`, `openxr_riftlinux.json` (the OpenXR manifest, for `XR_RUNTIME_JSON`) |
| `/usr/bin/riftlinux-setup` | Python 3 tool: `register` (run by itself, see 3), `check`, `repair`, `direct-mode`, `pair`, `uninstall` |
| `/usr/bin/riftlinux-runtime` | Starts the own runtime (refuses while SteamVR runs); `--stop`, `--status` |
| `/usr/bin/riftlinux-run` | Runs an OpenXR application on the own runtime, starting it first and stopping it after if it was not running |
| `/usr/share/applications/riftlinux-runtime.desktop` | App menu entry "RiftLinux Runtime" (action "Stop"); "Open With" handler for programs, AppImages, scripts and launchers |
| `/usr/share/kio/servicemenus/riftlinux-run.desktop` | Dolphin's right-click "Run with the Rift" |
| `/usr/share/icons/hicolor/256x256/apps/riftlinux.png` | The entries' icon |
| `/usr/lib/udev/rules.d/70-rift-cv1.rules` | Device access for the user at the machine (`uaccess`), USB power saving off for the Rift's devices (also after the kernel's camera driver takes a sensor), and the trigger that registers the driver when the headset appears |
| `/usr/lib/systemd/user/riftlinux-register.service` | Oneshot user unit: `riftlinux-setup register --quiet` |
| `/etc/xdg/autostart/riftlinux-register.desktop` | The same at every desktop login (any desktop, also without systemd user units) |
| `/usr/lib/riftlinux/after-install.sh` | Run as root after installing and after upgrades |
| `/usr/lib/riftlinux/before-remove.sh` | Run as root before removing (not on upgrades) |
| `/usr/share/doc/riftlinux/` | README, this file, copyright |

`/usr/lib64/riftlinux/...`, `/usr/local/lib/riftlinux/...` and `/opt/riftlinux/...` are also found
(`riftlinux-setup` looks in all four, or where `RIFTLINUX_SYSTEM_DIR` says). Do not edit the driver manifest; do not
put the driver into Steam's or SteamVR's own folders (SteamVR has no system-wide driver directory, and Steam updates
replace its folders).

## 3. Why each user needs a copy, and who makes it

The container shares the user's home with the host but not `/usr`, so SteamVR cannot load the driver from
`/usr/lib/riftlinux`. `riftlinux-setup register`, run as the user, keeps a copy in the user's home and registers it in
SteamVR's driver list (`openvrpaths.vrpath`, created if SteamVR has never run):

| User's Steam | Copy | Driver list |
| --- | --- | --- |
| Native (distribution or Valve package) | `~/.local/share/riftlinux/steamvr/rift-cv1` | `~/.config/openvr/openvrpaths.vrpath` |
| Flatpak (`com.valvesoftware.Steam`) | `~/.var/app/com.valvesoftware.Steam/data/riftlinux/steamvr/rift-cv1` | `~/.var/app/com.valvesoftware.Steam/config/openvr/openvrpaths.vrpath` |
| Snap (`steam`) | `~/snap/steam/common/.local/share/riftlinux/steamvr/rift-cv1` | `~/snap/steam/common/.config/openvr/openvrpaths.vrpath` |

The copy is replaced as a whole when the installed driver changes (a stamp file, `.riftlinux-managed`, records which
version it is), so a package upgrade reaches every user at their next login or headset plug-in. It also unregisters
other drivers named `monado` (only one can load) and lifts SteamVR's safe-mode block after a crash. It never asks
anything and never fails the caller.

It runs, without anyone typing it:
- right after the package is installed, for everyone logged in (`after-install.sh`, through each user's systemd
  instance, else `runuser`);
- when the headset is plugged in (udev `SYSTEMD_USER_WANTS` on 2833:0031 starts the user unit);
- at each desktop login (XDG autostart).

## 4. Hooks

| | deb | rpm | Arch (`.install`) | tarball |
| --- | --- | --- | --- | --- |
| After install and upgrade | `postinst configure` | `%post` | `post_install`, `post_upgrade` | `install.sh` |
| Before removal (not upgrade) | `prerm remove` | `%preun` when `$1` is `0` | `pre_remove` | `uninstall.sh` |

Each runs the matching script in `/usr/lib/riftlinux/`. `after-install.sh` reloads udev rules, triggers the Rift's
devices (so access works without re-plugging) and registers the driver for logged-in users. `before-remove.sh`
unregisters it for logged-in users; their copies, settings (`~/.config/riftlinux`) and sensor calibration
(`~/.config/monado`) stay, as nothing in a home is deleted.

`scripts/riftlinux/make-packages.sh` makes all four from one build: the `.deb` and the tarball directly, the Arch
package with `makepkg` in an Arch container and the `.rpm` with `rpmbuild` in a Fedora container (Docker;
`RIFTLINUX_NO_CONTAINERS=1` skips those two). The generated `PKGBUILD` (+ `riftlinux.install`) and
`riftlinux.spec` are left next to the packages: for the AUR or a COPR build only the release archive's address
changes in them. The PKGBUILD packages the archive's `root/` as it is (`options=('!strip' '!debug')`: the driver is
built in the Steam runtime and must stay as built); the spec sets `AutoReqProv: no` (the driver's libraries are the
Steam runtime's, not the system's) and no debug package.

The hooks find the logged-in users through logind (systemd-logind or elogind) and fall back to `who`; they start
the registration through the user's systemd where there is one, else run `riftlinux-setup register` as the user
(`runuser`, else `su`): `/usr/lib/riftlinux/users.sh`. Later logins are covered by the XDG autostart entry, which
desktops honour with or without systemd.

## 5. Dependencies

| | Why |
| --- | --- |
| `python3` (required) | `riftlinux-setup`; standard library only |
| `usbutils` (required) | The `lsusb` command: Steam uses it to see that the headset is connected (SteamVR's headset detection on Linux) |
| udev with systemd-logind (`uaccess`) | Device access for the user at the machine, no group needed |
| `rtkit` (recommended; a dependency of the Arch package, as Arch does not install it with a desktop) | Real-time priority for the tracking and compositor threads inside SteamVR (RealtimeKit over D-Bus; the container reaches the system bus). Without it the headset says so when it is put on |
| `libcap` tools (recommended) | `setcap` for SteamVR's compositor priority, offered by `riftlinux-setup repair` |
| Steam and SteamVR | Installed by the user from Steam; not a package dependency (any Steam: native, Flatpak, Snap) |

NVIDIA with a monitor offering adaptive sync: on Wayland the headset's picture tears unless VRR is hidden from the
driver (`options nvidia-modeset conceal_vrr_caps=1`). That takes VRR away from the user's monitors, so packages must
not set it by themselves; `after-install.sh` only prints a note when it applies, and the user runs
`riftlinux-setup nvidia-vrr conceal` (or the README's commands).

Distributions without systemd-logind (no `uaccess`) need a group-based rule instead, for example
`MODE="0660", GROUP="plugdev"` on the same matches, and the user in that group.

## 6. What the driver does by itself at SteamVR's start

Nothing of this needs the package's help, listed so packagers do not add it again:
- gives SteamVR a standing room (2 x 2 m, walls hidden) when the user has not set one up, so no Room Setup prompt;
- points OpenXR games at SteamVR when no OpenXR runtime is set (`~/.config/openxr/1/active_runtime.json`);
- runs in direct mode (its own compositor: Auto-HDR, motion smoothing, the CV1's true resolution) unless the user
  turns it off (`riftlinux-setup direct-mode off`); SteamVR's compositor takes the headset instead then;
- on X11 takes the headset through RandR (NVIDIA or Mesa), on Wayland through the desktop's DRM lease; never shows a
  desktop window;
- shows black while the headset is not worn and switches its display off after a while, and keeps SteamVR's standby
  to the wear sensor;
- keeps the user's settings file readable: one that cannot be read is never written, the last good one
  (`settings.json.last-known`) is used instead.

## 7. Checking a package

On a fresh user account with Steam and SteamVR installed but never started:

1. Install the package; `riftlinux-setup check` as the user: driver registered (for each Steam the user has), device
   access ok.
2. Restart Steam, plug in the Rift and put it on (SteamVR starts; or start it from Steam): the SteamVR log (`~/.local/share/Steam/logs/vrserver.txt`, or the
   Flatpak/Snap equivalent) has `Loaded server driver monado` from the user's copy; picture and tracking in the
   headset, no Room Setup.
3. Upgrade the package: after the next login or plug-in the user's copy has the new `VERSION`.
4. Remove the package: the driver is no longer in the user's `openvrpaths.vrpath`.

Without hardware: install each package in a clean container of its distribution with that distribution's own
tool, check the files, run `riftlinux-setup register` for a fresh user (an empty Steam folder: SteamVR's driver list
created, the user's copy made) and `riftlinux-setup --json check` on that distribution's Python, and remove the
package again. The releases are checked that way.

Problems: report them to the project with the output of `riftlinux-setup --json check`.
