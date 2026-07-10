{
  description = "Patroclus - VE.CAN Energy Meter Reader for Victron GX";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
  };

  outputs = inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];

      perSystem = { pkgs, system, ... }: {
        devShells.default = (pkgs.buildFHSEnv {
          name = "patroclus-dev";
          targetPkgs = pkgs: with pkgs; [
            platformio-core
            python3
            picocom
            mosquitto
          ];
          profile = ''
            echo ""
            echo "══════════════════════════════════════════════════════════════"
            echo "  Patroclus - VE.CAN Energy Meter Reader"
            echo "══════════════════════════════════════════════════════════════"
            echo ""
            echo "  Build & Upload (one app per board; -e selects the app)"
            echo "    pio run -e split_phase            Build patroclus01 (split-phase input)"
            echo "    pio run -e inverter               Build patroclus02 (inverter in/out)"
            echo "    pio run -e split_phase -t upload  Build and upload USB"
            echo "    pio run -e split_phase_ota -t upload   Upload over HTTP OTA"
            echo "    pio run -e inverter_ota -t upload      (patroclus02 OTA)"
            echo ""
            echo "  Control Connection"
            echo "    pio device monitor          Local USB"
            echo "    mosquitto_sub/pub           Remote MQTT (see README)"
            echo ""
            echo "  Device Commands (via serial)"
            echo "    STATUS    Show WiFi, MQTT, CAN status and readings"
            echo "    CANDUMP   Toggle raw CAN frame hex output"
            echo "    HELP      List available commands"
            echo ""
            echo "  Troubleshooting"
            echo "    Hold BOOT + press RESET to enter bootloader mode"
            echo ""
            echo "══════════════════════════════════════════════════════════════"
            echo ""
          '';
          runScript = "bash";
        }).env;
      };
    };
}
