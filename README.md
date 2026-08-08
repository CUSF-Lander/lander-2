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

## Build

With Docker (recommended — no ESP-IDF install needed):

```bash
./scripts/build.sh            # Windows: .\scripts\build.ps1
```

Or manually with `docker compose run --rm lander` / `docker compose run --rm ground-station`.
Outputs: `lander/build/merged.bin` and `ground-station/build/merged.bin`.

With a native ESP-IDF v6.0.2 install:

```bash
idf.py -C lander build
idf.py -C ground-station build
```

## Flash

```bash
pip install esptool
# Lander (ESP32):
esptool.py --chip esp32 -p <PORT> --baud 460800 write_flash 0x0 lander/build/merged.bin
# Ground station (ESP32-S3):
esptool.py --chip esp32s3 -p <PORT> --baud 460800 write_flash 0x0 ground-station/build/merged.bin
```

`<PORT>` is `COM3` on Windows, `/dev/ttyUSB0` on Linux, `/dev/cu.usbserial-*` on macOS.

## Ground station GUI

```bash
cd ground-station
pip install -r requirements.txt
python gui.py
```

Select the serial port, click Connect, then ARM and drive the motors with the power slider.

## VS Code

Open `workspace.code-workspace` (not a single folder). Machine-specific settings in `.vscode/settings.json` are git-ignored; configure the ESP-IDF extension locally.
