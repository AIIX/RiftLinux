# Copyright 2024, Gavin John <gavinnjohn@gmail.com>
# Copyright 2025-2026, coolGi <remy.alipoursafari@collabora.com>
# SPDX-License-Identifier: CC0-1.0 OR MIT OR BSL-1.0
#
# RiftLinux: `nix build` gives the runtime (monado-service, the OpenXR runtime, the Rift panel's pieces, the udev rules
# for the CV1); the NixOS module `nixosModules.default` (`services.riftlinux.enable = true;`) makes it the system's
# OpenXR runtime with NixOS's Monado service and gives the headset's devices to the logged-in user.

{
  inputs = {
    # Whenever an upstream change is merged, update this and
    # remove the packages from the ...ToUpstream lists below
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-26.05";
    flake-parts.url = "github:hercules-ci/flake-parts";
    # Allow the use of certain git dependencies
    nixpkgs-xr.url = "github:nix-community/nixpkgs-xr";
    nixpkgs-xr.inputs.nixpkgs.follows = "nixpkgs";
  };

  outputs =
    inputs@{ nixpkgs, flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ];
      perSystem =
        {
          system,
          pkgs,
          lib,
          ...
        }:
        let
          devTools = with pkgs; [
            # Tools that are required in order to develop with Monado
            # but are not required to build Monado itself
            ninja

            # Needed for running things in ./scripts
            clang-tools
            cmake-format
            codespell
            # Recommended for debugging
            gdb
            lldb
            vulkan-tools

            # Needed for Android
            gradle
            gradle-completion
            # Needed for running `survive-websocketd`
            websocketd
          ];

          nativeBuildInputsToUpstream = with pkgs; [
            # If there are any nativeBuildInputs that are not in nixpkgs, add them here
            # nativeBuildInputs are packages that are needed to develop and/or build the project (i.e. tooling)
          ];

          buildInputsToUpstream = with pkgs; [
            # If there are any buildInputs that are not in nixpkgs, add them here
            # buildInputs are any packages that are needed at runtime (i.e. dependencies)

            # RiftLinux: the Rift CV1's constellation tracker (Ceres), its sensors on USB 2 (JPEG) and the Rift panel's
            # volume (PulseAudio)
            ceres-solver
            libjpeg_turbo
            libpulseaudio

            # Survive has not received a new tagged release in years, so use the
            # version provided by nixpkgs-xr instead
            (inputs.nixpkgs-xr.packages.${system}.libsurvive)
          ];

          package = pkgs.monado.overrideAttrs (oldAttrs: {
            pname = "riftlinux";
            version = "git-${inputs.self.shortRev or "dirty"}";
            src = ./.;

            nativeBuildInputs = oldAttrs.nativeBuildInputs ++ nativeBuildInputsToUpstream;
            buildInputs = oldAttrs.buildInputs ++ buildInputsToUpstream;
            # RiftLinux's own Rift driver (OpenHMD would replace it)
            cmakeFlags = lib.remove "-DXRT_HAVE_STEAM:BOOL=TRUE" oldAttrs.cmakeFlags ++ [ "-DXRT_BUILD_DRIVER_OHMD=OFF" ];

            patches = [ ];

            # The CV1's devices for the logged-in user (the rule starting RiftLinux's SteamVR registration service is
            # left out: there is none here)
            postInstall = (oldAttrs.postInstall or "") + ''
              mkdir -p $out/lib/udev/rules.d
              grep -v riftlinux-register.service ${./scripts/riftlinux/udev/70-rift-cv1.rules} \
                > $out/lib/udev/rules.d/70-rift-cv1.rules
            '';

            meta = oldAttrs.meta // {
              description = "RiftLinux: a modified Monado, specialized for full Oculus Rift CV1 support";
              homepage = "https://github.com/AIIX/RiftLinux";
            };
          });
        in
        {
          packages.default = package;
          devShells.default = package.overrideAttrs (oldAttrs: {
            nativeBuildInputs = oldAttrs.nativeBuildInputs ++ devTools;
          });

          formatter = pkgs.nixfmt-rfc-style;
        };

      flake.nixosModules.default =
        {
          config,
          lib,
          pkgs,
          ...
        }:
        let
          cfg = config.services.riftlinux;
        in
        {
          options.services.riftlinux = {
            enable = lib.mkEnableOption "RiftLinux, the Oculus Rift CV1 runtime";
            package = lib.mkOption {
              type = lib.types.package;
              default = inputs.self.packages.${pkgs.stdenv.hostPlatform.system}.default;
              description = "The RiftLinux package.";
            };
          };
          config = lib.mkIf cfg.enable {
            services.monado = {
              enable = true;
              package = cfg.package;
              defaultRuntime = true;
            };
            services.udev.packages = [ cfg.package ];
            environment.systemPackages = [ cfg.package ];
          };
        };
    };
}
