# Lander

Flight firmware for a thrust-vector-controlled lander, targeting the **ESP32** on **ESP-IDF v6.0.2**. Two projects live in this repo:

- `lander/` — flight controller (IMU, Kalman filter, DShot motors, ESP-NOW telemetry)
- `ground-station/` — ESP-NOW receiver + `gui.py` desktop GUI

## Getting started

Clone with submodules (the BNO08x IMU driver is a submodule, and the build fails without it):

```bash
git clone --recurse-submodules <repo-url>
# in an existing clone:
git submodule update --init --recursive
```

## Install Docker

Docker is the only install needed — the build toolchain (ESP-IDF v6.0.2) runs inside the image, so no ESP-IDF or esptool install is required on the host. There is no universal installer: pick your OS below.

| OS | Install |
|---|---|
| Linux | `curl -fsSL https://get.docker.com \| sh` (works on most distros), then `sudo usermod -aG docker $USER` and re-login |
| macOS | `brew install --cask docker` (or download Docker Desktop from docker.com) |
| Windows | `winget install Docker.DockerDesktop` (or download Docker Desktop; needs WSL2) |

Verify with `docker --version && docker compose version`.

## Build

Both projects build inside the official `espressif/idf:v6.0.2` image (`docker-compose.yml`). No ESP-IDF toolchain needed on the host.

```bash
docker compose run --rm lander            # lander firmware
# or with a native ESP-IDF v6.0.2 install:
idf.py -C lander build
```

Outputs: `lander/build/merged.bin` and `ground-station/build/merged.bin`.

The chip target defaults to ESP32 (set by `IDF_TARGET` in `docker-compose.yml` / each project's `sdkconfig`). To build for an ESP32-S3 ground station, run once:

```bash
docker compose run --rm ground-station idf.py set-target esp32s3
```

## Flash

`<PORT>` is `COM3` on Windows, `/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on macOS.

**Linux (esptool runs inside the container — no host install):**

```bash
docker run --rm --user root \
  -v "$(pwd):/workspace" -w /workspace/lander \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  espressif/idf:v6.0.2 \
  esptool.py --chip esp32 -p /dev/ttyUSB0 --baud 460800 write_flash 0x0 build/merged.bin
```

**WSL2:** attach the USB serial with usbipd-win (`usbipd bind`) first, then use the Linux command above.

**macOS / Windows:** Docker Desktop cannot forward serial ports into containers, so flash from the host:

```bash
pip install esptool
# Lander (ESP32):
esptool.py --chip esp32 -p <PORT> --baud 460800 write_flash 0x0 lander/build/merged.bin
# Ground station (ESP32-S3):
esptool.py --chip esp32s3 -p <PORT> --baud 460800 write_flash 0x0 ground-station/build/merged.bin
```

## Ground station GUI

```bash
cd ground-station
pip install -r requirements.txt
python gui.py
```

Select the serial port, click Connect, then ARM and drive the motors with the power slider.

## VS Code

Open `workspace.code-workspace` (not a single folder). Machine-specific settings in `.vscode/settings.json` are git-ignored; configure the ESP-IDF extension locally.
