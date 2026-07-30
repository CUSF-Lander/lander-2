
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

| Propeller | ESP32 pin | RMT channel | ESC instance |
| --- | ---: | ---: | --- |
| Bottom (Motor is reversed!) | GPIO 4 | 0 | ESC 1 |
| Top (Motor is reversed!) | GPIO 25 | 1 | ESC 2 |

With the previous sequential initialization, the bottom propeller initialized
first and the top propeller initialized approximately five seconds later. The
current implementation arms both ESCs simultaneously by sending zero-throttle
DShot packets to both channels during the arming period.

Note: The motors are reversed by applying tempoary DShot reversal commands after both ESCs are armed sucessfully 
## Problems encountered and fixes

- [2026-07-30 — Debugging DShot: RMT clock, shared ground and ESC arming](learning_diary/2026-07-30-debugging-dshot-rmt-clock-and-esc-wiring.md)
