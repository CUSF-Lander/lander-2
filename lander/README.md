
# Cambridge University SpaceFlight - Propulsive Lander - Flight Computer Software

Flight computer software developed on top of ESP-IDF for a liquid fuelled, propulsively landed rocket hopper that flies to 100m and back, to compete in The Lander Challenge.

This version of the software is to be used by a small scale single rotor (with 2 counter-rotating propellers) drone to demonstrate Thrust Vector Control and relevant state estimation and  flight control algorithms. 

The current plan is to implement a Linear Quadratic Regulator for Full State Feedback. 

### Current Software, Control Systems, Mechanical, and Electronics Sub-teams:
![IMG_8239](https://github.com/user-attachments/assets/0ef6a085-4057-411b-95f5-9252d37085a4)

# Setup Guide
1. Clone the repository by `git clone https://github.com/CUSF-Lander/lander-2.git --recurse-submodules`

## Motor wiring

The current counter-rotating propeller stack uses this DShot mapping:

| Propeller | ESP32 pin | ESC instance |
| --- | ---: | --- |
| Bottom | GPIO 4 | ESC 1 |
| Top | GPIO 25 | ESC 2 |

ESP-IDF 6 allocates RMT channels dynamically, so the application fixes the
GPIO mapping but does not request channel numbers 0 and 1 directly.

With the previous sequential initialization, the bottom propeller initialized
first and the top propeller initialized approximately five seconds later. The
current implementation arms both ESCs simultaneously by sending zero-throttle
DShot packets to both channels during the arming period.

The motor leads are now wired for the required directions. Temporary DShot
reversal remains available in the firmware for future bench configurations, but
`MOTOR_REVERSE_BOTH_ON_STARTUP` is disabled by default.

For safety, both the firmware and ground-station throttle default to zero. The
firmware rejects an ARM command while a nonzero throttle is stored, and an
ESTOP resets the requested throttle to zero.

## Servo wiring

The two TVC servos are connected to the PCA9685 Servo FeatherWing:

| Servo | PCA9685 channel |
| --- | ---: |
| TVC servo 1 | 0 |
| TVC servo 2 | 7 |

The FeatherWing is hardwired to the Feather's I2C pins: SDA is GPIO 22 and SCL
is GPIO 20. The temporary serial servo test is retained but disabled for the
normal application build.

## Problems encountered and fixes

- [2026-07-30 — Debugging DShot: RMT clock, shared ground and ESC arming](learning_diary/2026-07-30-debugging-dshot-rmt-clock-and-esc-wiring.md)
