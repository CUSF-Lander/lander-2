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

Docker is the only install needed to build — the toolchain (ESP-IDF v6.0.2) runs inside the image, which ships native `amd64` and `arm64` builds (Apple Silicon runs it without emulation).

| OS | Install | Start the engine |
|---|---|---|
| Linux | `curl -fsSL https://get.docker.com \| sh`, then `sudo usermod -aG docker $USER` and log out/in | `sudo systemctl enable --now docker` |
| macOS | `brew install --cask docker` (or Docker Desktop from docker.com) | Open **Docker.app** and wait for "Engine running" in the menu bar |
| Windows | `winget install Docker.DockerDesktop` (needs WSL2: `wsl --install`) | Open **Docker Desktop** and wait for "Engine running" |

On macOS and Windows, Docker runs inside a VM that Docker Desktop starts, so **Docker Desktop must be open** before any `docker` command works (see [Troubleshooting](#troubleshooting)).

Verify with:

```bash
docker info              # must print a "Server:" section, not a connection error
docker compose version
```

**Windows:** run the commands below from PowerShell, or (faster builds) clone the repo inside WSL (`\\wsl$\Ubuntu\home\...`) and run them from a WSL shell. Builds on a `C:\` checkout work but file I/O through the VM is slow.

## Build

Both projects build inside the official `espressif/idf:v6.0.2` image (`docker-compose.yml`). Run from the repo root — the same commands work on Linux, macOS and Windows:

```bash
docker compose run --rm lander            # lander firmware
docker compose run --rm ground-station    # ground-station firmware
# or with a native ESP-IDF v6.0.2 install:
idf.py -C lander build
```

Outputs: `lander/build/merged.bin` and `ground-station/build/merged.bin`.

The chip target defaults to ESP32 (set by `IDF_TARGET` in `docker-compose.yml` / each project's `sdkconfig`). To build for an ESP32-S3 ground station, run once:

```bash
docker compose run --rm ground-station idf.py set-target esp32s3
```

## Flash

Docker Desktop (macOS/Windows) cannot pass USB serial ports into containers, so on those OSes you build in Docker and flash from the host. Only Linux can flash from inside the container.

| OS | `<PORT>` looks like | Find it with |
|---|---|---|
| Linux | `/dev/ttyUSB0` or `/dev/ttyACM0` | `ls /dev/ttyUSB* /dev/ttyACM*` |
| macOS | `/dev/cu.usbserial-*`, `/dev/cu.SLAB_USBtoUART`, `/dev/cu.wchusbserial*` | `ls /dev/cu.*` |
| Windows | `COM3` | Device Manager → Ports (COM & LPT) |

The board needs a USB-UART driver on macOS/Windows if the port doesn't appear: CP210x (Silicon Labs) or CH340 (WCH), depending on the board.

**Linux — esptool inside the container, nothing to install:**

```bash
docker run --rm --user root \
  -v "$(pwd):/workspace" -w /workspace/lander \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  espressif/idf:v6.0.2 \
  python -m esptool --chip esp32 -p /dev/ttyUSB0 -b 460800 write-flash 0x0 build/merged.bin
```

**macOS / Windows — esptool on the host** (Python 3 needed, no ESP-IDF):

```bash
pip install esptool      # once; use pip3/python3 on macOS if `python` isn't found
# Lander (ESP32):
python -m esptool --chip esp32 -p <PORT> -b 460800 write-flash 0x0 lander/build/merged.bin
# Ground station (ESP32-S3; use --chip esp32 if you didn't set-target):
python -m esptool --chip esp32s3 -p <PORT> -b 460800 write-flash 0x0 ground-station/build/merged.bin
```

This host method also works on Linux.

**WSL2:** either flash from Windows with the host command above, or attach the USB serial to WSL with [usbipd-win](https://github.com/dorssel/usbipd-win) (`usbipd bind` / `usbipd attach --wsl`) and use the Linux command.

If flashing fails with "Failed to connect", hold the board's **BOOT** button while esptool says `Connecting...`, or drop to `-b 115200`.

## Troubleshooting

**`Cannot connect to the Docker daemon` / `failed to connect to the docker API` (macOS, Windows)** — the Docker engine isn't running. Open Docker Desktop and wait for "Engine running", then check `docker info`. If it still fails:

```bash
docker context ls                 # the active context has a *
docker context use desktop-linux  # Docker Desktop's context (or: default)
echo $DOCKER_HOST                 # should be empty; if not: unset DOCKER_HOST
```

Using Colima instead of Docker Desktop: `colima start` and `docker context use colima`.

**`permission denied ... /var/run/docker.sock` (Linux)** — your user isn't in the `docker` group yet: `sudo usermod -aG docker $USER`, then log out and back in.

**`CMakeCache.txt directory ... is different` or other odd CMake errors** — the `build/` folder was created by a different environment (native ESP-IDF, another OS, or another clone path). Delete it and rebuild: `rm -rf lander/build` (PowerShell: `Remove-Item -Recurse -Force lander\build`).

**`build/` files owned by root (Linux)** — the container runs as root. Fix with `sudo chown -R $USER: lander/build ground-station/build`.

**`esp32_BNO08x` / component not found** — the submodule is missing: `git submodule update --init --recursive`.

## Ground station GUI

```bash
cd ground-station
pip install -r requirements.txt
python gui.py
```

Select the serial port, click Connect, then ARM and drive the motors with the power slider.

## VS Code

Open `workspace.code-workspace` (not a single folder). Machine-specific settings in `.vscode/settings.json` are git-ignored; configure the ESP-IDF extension locally.
