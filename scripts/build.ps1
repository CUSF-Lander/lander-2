# Build the CUSF Lander firmware inside Docker.
#
# Works on Windows (Docker Desktop) and any other OS via PowerShell.
# No ESP-IDF toolchain needed on the host — only Docker.
#
# Usage:
#   .\scripts\build.ps1                # build both firmware projects
#   .\scripts\build.ps1 lander         # build lander only
#   .\scripts\build.ps1 ground-station # build ground-station only
#
# Output:
#   lander\build\merged.bin          (ESP32, offset 0x0)
#   ground-station\build\merged.bin  (ESP32-S3, offset 0x0)

param(
    [string]$Target = "all"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) {
    Write-Error "docker not found. Install Docker Desktop for Windows first."
    exit 1
}

if (-not (Test-Path "lander\components\esp32_BNO08x")) {
    Write-Error "BNO08x submodule missing. Run: git submodule update --init --recursive"
    exit 1
}

function Build-One([string]$name) {
    Write-Host "============================================================="
    Write-Host " Building: $name"
    Write-Host "============================================================="
    docker compose run --rm $name
}

switch ($Target) {
    "all" {
        Build-One "lander"
        Build-One "ground-station"
    }
    "lander" { Build-One "lander" }
    "ground-station" { Build-One "ground-station" }
    default {
        Write-Error "Unknown target '$Target'. Use: all | lander | ground-station"
        exit 1
    }
}

Write-Host ""
Write-Host "Done. Flash from the host with esptool:"
Write-Host "  Lander (ESP32):       esptool.py --chip esp32  -p <PORT> write_flash 0x0 lander\build\merged.bin"
Write-Host "  Ground station (S3):  esptool.py --chip esp32s3 -p <PORT> write_flash 0x0 ground-station\build\merged.bin"
