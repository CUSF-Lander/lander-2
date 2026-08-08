#!/usr/bin/env bash
# Build the CUSF Lander firmware inside Docker.
#
# Works on Linux, macOS (incl. Apple Silicon) and WSL2 on Windows.
# No ESP-IDF toolchain needed on the host — only Docker.
#
# Usage:
#   ./scripts/build.sh              # build both firmware projects
#   ./scripts/build.sh lander       # build lander only
#   ./scripts/build.sh ground-station  # build ground-station only
#
# Output:
#   lander/build/merged.bin          (ESP32, offset 0x0)
#   ground-station/build/merged.bin  (ESP32-S3, offset 0x0)

set -euo pipefail
cd "$(dirname "$0")/.."

command -v docker >/dev/null 2>&1 || {
    echo "ERROR: docker not found. Install Docker Desktop (Win/Mac) or Docker Engine (Linux)." >&2
    exit 1
}

if [ ! -d lander/components/esp32_BNO08x ]; then
    echo "ERROR: BNO08x submodule missing. Run: git submodule update --init --recursive" >&2
    exit 1
fi

TARGET="${1:-all}"

build_one() {
    echo "============================================================="
    echo " Building: $1"
    echo "============================================================="
    docker compose run --rm "$1"
}

case "$TARGET" in
    all)
        build_one lander
        build_one ground-station
        ;;
    lander|ground-station)
        build_one "$TARGET"
        ;;
    *)
        echo "Unknown target '$TARGET'. Use: all | lander | ground-station" >&2
        exit 1
        ;;
esac

echo
echo "Done. Flash from the host with esptool:"
echo "  Lander (ESP32):       esptool.py --chip esp32  -p <PORT> write_flash 0x0 lander/build/merged.bin"
echo "  Ground station (S3):  esptool.py --chip esp32s3 -p <PORT> write_flash 0x0 ground-station/build/merged.bin"
