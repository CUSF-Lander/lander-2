
# Cambridge University SpaceFlight - Propulsive Lander - Flight Computer Software

Flight computer software developed on top of ESP-IDF for a liquid fuelled, propulsively landed rocket hopper that flies to 100m and back, to compete in The Lander Challenge.

This version of the software is to be used by a small scale single rotor (with 2 counter-rotating propellers) drone to demonstrate Thrust Vector Control and relevant state estimation and  flight control algorithms. 

The current plan is to implement a Linear Quadratic Regulator for Full State Feedback. 

### Current Software, Control Systems, Mechanical, and Electronics Sub-teams:
![IMG_8239](https://github.com/user-attachments/assets/0ef6a085-4057-411b-95f5-9252d37085a4)

# Setup Guide
1. Clone the repository by `git clone https://github.com/CUSF-Lander/lander-2.git --recurse-submodules`

## Current flight-test configuration

The committed defaults are for the first short-tether attitude-control test,
not autonomous hover:

| Compile-time option | Default | Behaviour |
| --- | ---: | --- |
| `SERVO_TERMINAL_TEST_MODE` | `0` | Isolated terminal-controlled servo test disabled |
| `MOTOR_WIRING_TEST_MODE` | `0` | Combined 5% motor and servo-sweep test disabled |
| `GIMBAL_ATTITUDE_MAPPING_TEST_MODE` | `0` | Direct 1:1 IMU-to-gimbal direction test disabled |
| `MANUAL_THROTTLE_ATTITUDE_ONLY_MODE` | `1` | Ground-station motor throttle with closed-loop TVC attitude control enabled |
| `MOTOR_REVERSE_BOTH_ON_STARTUP` | `0` | No temporary DShot motor reversal; the motor leads provide the required directions |

In this configuration, the ground station commands equal collective throttle
to both motors while the 50 Hz hover controller commands the two gimbal servos.
Altitude control and horizontal-position control are deliberately excluded.

## Hardware mapping and calibration

### Motors

| Propeller | ESP32 pin | ESC instance |
| --- | ---: | --- |
| Bottom | GPIO 4 | ESC 1 |
| Top | GPIO 25 | ESC 2 |

ESP-IDF 6 allocates RMT transmit channels dynamically. The application fixes
the motor GPIOs but does not request hardware RMT channel numbers directly.
Both ESCs are armed simultaneously with five seconds of zero-throttle DShot
frames; this replaces the old sequential initialization in which the bottom
propeller became ready about five seconds before the top propeller.

The motor leads are physically wired for the required counter-rotation.
Temporary BLHeli DShot reversal commands remain available behind
`MOTOR_REVERSE_BOTH_ON_STARTUP` for future bench configurations.

### TVC servos

| Controller gimbal | PCA9685 channel | Midpoint | Direction | Limited range |
| --- | ---: | ---: | ---: | ---: |
| Gimbal 1 | 0 | 110 degrees | Inverted | 97-123 degrees |
| Gimbal 2 | 7 | 80 degrees | Inverted | 67-93 degrees |

The PCA9685 Servo FeatherWing uses the Feather's fixed I2C connection: SDA is
GPIO 22 and SCL is GPIO 20. Controller outputs remain zero-centred radians.
The actuator calibration converts them to physical servo angles as follows:

```text
channel 0 = 110 degrees - gimbal 1 deflection
channel 7 =  80 degrees - gimbal 2 deflection
```

Both gimbal commands are initially limited to +/-13 degrees. The midpoints,
direction multipliers, and limits are defined together in `globalvars.cpp` so
they can be recalibrated without changing the controller's sign convention.

A dedicated actuator task writes both servos at 50 Hz. The PCA9685 channels are
fully off during startup and ESTOP, rather than continuing to emit a midpoint
pulse. A successful ARM enables them. Invalid/non-finite angles or a failed
servo write latch ESTOP, set motor throttle to zero, and disable both channels.

## Expected startup, ARM, and ESTOP behaviour

Normal startup performs the following sequence:

1. Start in ESTOP with motor power set to zero.
2. Enable the STEMMA QT/I2C power rail and initialize I2C and the PCA9685.
3. Explicitly disable both TVC servo channels so stale PCA9685 outputs cannot
   move the gimbal during sensor initialization.
4. Initialize the BMP390 and average 20 stationary samples over approximately
   one second to define launch altitude as zero.
5. Initialize ESP-NOW, GPS UART, and the BNO08x IMU.
6. Start state-estimation, position-control, hover-control, servo-actuation,
   DShot motor, telemetry, and barometer tasks.
7. Arm both ESCs together with five seconds of zero-throttle DShot frames. The
   vehicle remains software-ESTOP'd until a valid ground-station ARM command.

ARM is accepted only when the stored ground-station throttle is zero. A valid
ARM command then:

- moves the barometric zero to the latest valid reading;
- captures the current IMU roll, pitch, and yaw as the attitude reference;
- clears shared position, velocity, position-controller, and hover-controller
  outputs;
- requests a coordinated reset of the Kalman state and controller integrators;
  and
- clears ESTOP, enabling the servo outputs and manual motor throttle.

`ZERO_IMU` performs the same reference capture and coordinated reset, but it is
accepted only while ESTOP is active. This prevents the control reference from
changing during flight.

ESTOP immediately stores zero motor power. The motor task continuously sends
zero DShot while ESTOP is active, and the servo task removes PWM from both TVC
channels. Loss of ground-station traffic for 500 ms also latches ESTOP. ARM is
rejected if a nonzero throttle value is stored.

## Default tethered attitude-control mode

With `MANUAL_THROTTLE_ATTITUDE_ONLY_MODE=1` and the two test modes disabled:

- ground-station power is converted to the same DShot throttle for both motors;
- the 50 Hz inner LQR uses IMU attitude and angular-rate errors relative to the
  attitude captured at ARM;
- the LQR force/moment output passes through the controls team's existing
  force-to-gimbal inverse mapping;
- the resulting gimbal deflections are clamped, converted through the physical
  midpoint/direction calibration, and sent by the independent actuator task;
- altitude, vertical velocity, and altitude-integral errors are forced to zero;
- calculated controller motor speeds are forced to zero and are not passed to
  DShot or the Kalman process model; and
- horizontal-position gains remain zero, and `U_pos` is explicitly excluded
  from the attitude reference. This also prevents a failed position estimate
  from contaminating attitude control through IEEE-754 `0 * NaN` propagation.

This mode is manual-throttle attitude stabilization. It does not automatically
hold altitude or position, and it does not convert the controller's requested
motor angular velocity into DShot commands.

## Hardware test modes

Only one isolated test mode should be enabled at a time.

### Direct IMU-to-gimbal direction test

Set `GIMBAL_ATTITUDE_MAPPING_TEST_MODE=1` and keep
`MOTOR_WIRING_TEST_MODE=0`. This mode retains normal sensors, ARM, ESTOP, and
communications, but bypasses the LQR magnitude and inverse mapping:

- ARM captures the neutral attitude.
- Measured roll displacement maps 1:1 to gimbal 1.
- Measured pitch displacement maps 1:1 to gimbal 2.
- Yaw displacement is logged but does not map to a gimbal axis.
- Both ESCs are locked to zero DShot; nonzero `SET_POWER` commands are ignored.
- Existing servo midpoints, inversion, and +/-13-degree limits remain active.

Expected examples after ARM are:

| Attitude displacement | Controller command | PCA9685 command |
| --- | --- | --- |
| Roll +8 degrees | Gimbal 1 +8 degrees | Channel 0 = 102 degrees |
| Roll -8 degrees | Gimbal 1 -8 degrees | Channel 0 = 118 degrees |
| Pitch +8 degrees | Gimbal 2 +8 degrees | Channel 7 = 72 degrees |
| Pitch -8 degrees | Gimbal 2 -8 degrees | Channel 7 = 88 degrees |

This mode verifies the complete IMU-axis, gimbal-channel, electrical-direction,
and linkage-direction convention without thrust or fast LQR saturation.

### Combined motor and servo wiring test

Set `MOTOR_WIRING_TEST_MODE=1`. This isolated test bypasses IMU, GPS, ESP-NOW,
BMP390, and all flight-control tasks. It:

1. centres channel 0 at 110 degrees and channel 7 at 80 degrees;
2. waits one second for the gimbal to settle;
3. arms both ESCs together at zero for five seconds;
4. commands both motors to a fixed 5% DShot throttle;
5. sweeps gimbal 1 by +/-10 degrees with a seven-second period; and
6. sweeps gimbal 2 by +/-10 degrees with a ten-second period.

The different periods produce a repeating 70-second set of gimbal-position
combinations. A DShot or servo-output error commands motor zero and disables the
servos. This mode intentionally bypasses the normal heartbeat and software
ESTOP path, so use it only with the vehicle restrained and the propeller area
clear; terminate it by resetting or removing power.

### Serial servo calibration test

Set `SERVO_TERMINAL_TEST_MODE=1` for an isolated PCA9685 test. Motors, sensors,
GPS, ESP-NOW, and flight-control tasks remain disabled. All 16 PCA9685 channels
are turned off at startup, then the terminal accepts an absolute angle from
0-180 degrees for `SERVO_TERMINAL_TEST_CHANNEL`; enter `off` to remove its PWM.

## Logging and diagnostics

- State and IMU summaries are rate-limited to approximately 2 Hz.
- Attitude-control logs show roll/pitch/yaw error, zero-centred gimbal commands,
  final absolute channel 0/channel 7 angles, and manual throttle.
- The direct mapping mode logs attitude displacement and confirms motors are
  locked at 0%.
- The ESP-NOW callback no longer prints a TX line for every 100 Hz telemetry
  packet; periodic aggregate success statistics remain available.

## Known limitations before free flight

- The current force-to-gimbal calculation uses `acos` for gimbal 2, which does
  not encode a signed angle. The controls team still needs to validate and
  correct this inverse mapping around the hover operating point.
- DShot percentage has not been calibrated against measured motor angular
  velocity or thrust. The provisional simulation convention is 100% = 25
  rad/s (approximately 239 RPM), but the firmware does not currently use this
  as a physical DShot conversion and the real relationship is not assumed to
  be linear.
- Setting `MANUAL_THROTTLE_ATTITUDE_ONLY_MODE=0` does not by itself enable
  autonomous thrust or altitude control. An explicit omega-to-DShot/thrust
  mapping and motor actuation integration are still required.
- The legacy ESP-IDF I2C API still builds under ESP-IDF 6 but is scheduled for
  removal in ESP-IDF 7 and should be migrated separately.

## Hardware validation status

On 2026-08-08, the ESP-IDF 6 firmware was built, flashed, and exercised on the
drone hardware. The following were confirmed:

- I2C/PCA9685, BMP390, BNO08x, ESP-NOW, and DShot startup;
- simultaneous ESC initialization and ground-station motor control;
- ARM, ESTOP, link-loss failsafe, and zero-throttle arming checks;
- servo channels 0 and 7, physical midpoints 110/80 degrees, and mechanical
  travel around both midpoints;
- both servo outputs require inversion relative to controller-positive gimbal
  commands; and
- the zero-thrust 1:1 attitude mapping moves both gimbals in the intended
  corrective physical direction.

The current source also builds successfully with ESP-IDF 6.0.2. Still to test
is the powered short-tether response of the complete LQR and force-to-gimbal
inverse mapping. Do not treat the successful static direction test as proof of
closed-loop stability.

## Problems encountered and fixes

- [2026-07-30 — Debugging DShot: RMT clock, shared ground and ESC arming](learning_diary/2026-07-30-debugging-dshot-rmt-clock-and-esc-wiring.md)
