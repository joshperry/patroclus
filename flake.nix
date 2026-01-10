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
            echo "  Build & Upload"
            echo "    pio run                           Build firmware (listen-only)"
            echo "    pio run -t upload                 Build and upload"
            echo "    CAN_ACTIVE_MODE=1 pio run         Build with CAN ACK (bench)"
            echo ""
            echo "  Serial Monitor"
            echo "    pio device monitor          PlatformIO monitor (115200 baud)"
            echo "    picocom -b 115200 /dev/ttyACM0   Alternative monitor"
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
