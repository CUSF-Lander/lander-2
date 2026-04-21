# ESP32 Lander Ground Station

An ESP-IDF project that turns an ESP32 into a ground station receiver for a model rocket/lander. It listens for telemetry data from the lander via the ESP-NOW protocol and pipes it out so we can monitor things like orientation, acceleration, and position in real-time. Made by Krystian Filipek.

## Features

- **ESP-NOW Reception**: Uses ESP-NOW to listen for high-speed, structural telemetry data without needing a WiFi router.
- **Python Telemetry Dashboard**: Includes a custom GUI (gui.py) to visualize the lander's 3D orientation and real-time positional drift.
- **Hardware & Software E-STOP**: Ability to remotely command the lander to shut down operations or recalibrate its IMU on the fly.
- **Data Logging**: Saves live flight data straight into auto-generated CSV files for post-flight analysis.

## Technical Implementation

The project relies on ESP-IDF for the low-level embedded receiver and Python for the 3D dashboard. 

| Component | Detail |
|---|---|
| **Microcontroller** | ESP32 |
| **Framework** | ESP-IDF (v5.5.1) |
| **Language** | C++ & Python |
| **Protocol** | ESP-NOW (2.4 GHz) |
| **UI Framework** | Tkinter & Matplotlib |

## Getting Started

### Prerequisites

- An ESP32 development board.
- ESP-IDF v5.5.1 installed and in your PATH.
- Python 3.x with a few dependencies (matplotlib, 
umpy, 	tkbootstrap, pyserial).

### Build and Run

1.  **Flash the Ground Station**:
    `sh
    idf.py build
    idf.py flash
    `

2.  **Launch the Dashboard**:
    Make sure your ESP32 is plugged in, then spin up the Python GUI to visualize the data:
    `sh
    python gui.py
    `

## Why I made this

I built this ground station to:
- Practice handling raw sensor telemetry (Euler angles, spatial position, pressure) over a custom wireless protocol.
- Integrate embedded C code with a Python desktop application over Serial.
- Needed a dashboard for the CUSF Lander project. 

<div align="center">

Developed by [kfilipekk](https://github.com/kfilipekk)

</div>
