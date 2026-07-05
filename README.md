# Lander

Firmware for the lander and its ground station, targeting the **ESP32** on
**ESP-IDF v6.0.1**. This repo is a VS Code multi-root workspace with two
independent ESP-IDF projects:

- [`lander/`](lander/) — flight controller (IMU, Kalman filter, DShot motors, ESP-NOW telemetry)
- [`ground-station/`](ground-station/) — ESP-NOW receiver + [`gui.py`](ground-station/gui.py) desktop GUI

## First-time setup

1. **Clone with submodules** (the `esp32_BNO08x` IMU driver is a git submodule —
   the build fails with "does not contain a CMakeLists.txt" if it's missing):

   ```bash
   git clone --recurse-submodules <repo-url>
   # or, in an existing clone:
   git submodule update --init --recursive
   ```

2. **Install ESP-IDF v6.0.1.** Use the
   [ESP-IDF Installation Manager (EIM)](https://dl.espressif.com/dl/esp-idf/)
   or the VS Code ESP-IDF extension's "Configure" flow. The default install
   location is `~/.espressif` (`%USERPROFILE%\.espressif` on Windows).

3. **Open the workspace**, not a single folder: open
   [`workspace.code-workspace`](workspace.code-workspace) in VS Code and install
   the recommended extensions when prompted (ESP-IDF, clangd, Python).

4. **Point the ESP-IDF extension at your local install.** This is per-machine
   config and is intentionally **not** committed — set it locally via
   Command Palette → "ESP-IDF: Configure ESP-IDF Extension". (See
   [Per-machine config](#per-machine-config) below.)

## Building

### Linux

**Via task (fastest):**
```bash
Ctrl+Shift+B → "Build Select Project" → Lander or Ground Station
```

**Via CLI:**
```bash
source ~/.espressif/tools/activate_idf_v6.0.1.sh
idf.py -C lander build                    # or: -C ground-station
```

### macOS

**Via task (fastest):**
```bash
Cmd+Shift+B → "Build Select Project" → Lander or Ground Station
```
(The task automatically sources `~/.espressif/tools/activate_idf_v6.0.1.sh`.)

**Via CLI:**
```bash
source ~/.espressif/tools/activate_idf_v6.0.1.sh
idf.py -C lander build
```

### Windows

**Via ESP-IDF extension (recommended):**
1. Open the workspace
2. Command Palette (Ctrl+Shift+P) → "ESP-IDF: Pick a Workspace Folder"
   → select `lander` or `ground-station`
3. Click the **Build** button in the bottom status bar, or press Ctrl+E then B

**Via CLI (PowerShell):**
```powershell
# Activate ESP-IDF (installed via EIM or Windows installer)
& "${env:USERPROFILE}\.espressif\tools\idf-cmd-tools\cmdline_tools\bin\idf.cmd" -C lander build
```

### All Platforms (Extension)

The ESP-IDF extension works on all platforms. Use the bottom-bar build/flash/monitor buttons:
1. Command Palette → "ESP-IDF: Pick a Workspace Folder" (select `lander` or `ground-station`)
2. Click **Build** (or **Flash**, **Monitor**)

## Per-machine config

Machine-specific VS Code settings are **not** tracked — `.vscode/settings.json`
is git-ignored (see [.gitignore](.gitignore)). The ESP-IDF extension writes
values like `idf.currentSetup`, `idf.port`, and absolute toolchain paths there;
committing them breaks other contributors. Configure them locally through the
extension and they'll stay on your machine.

Shared, portable config **is** tracked: the workspace layout and build task in
[`workspace.code-workspace`](workspace.code-workspace) and the recommended
extensions in [`.vscode/extensions.json`](.vscode/extensions.json). The
Microsoft **CMake Tools** extension is intentionally disabled for this
workspace (it configures ESP-IDF projects with the host compiler and corrupts
`build/`); build only through ESP-IDF.
