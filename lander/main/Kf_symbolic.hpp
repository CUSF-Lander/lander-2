#pragma once

#include "esp_dsp.h"

// Updates the linearised Kalman matrices around the current operating point.
// Must be called once per filter iteration before the prediction step.
//
// Produces:
//   A (18×18) — state transition (kinematic integrators + nonlinear Jacobians)
//   B (18×4)  — control input Jacobian
//   H (9×18)  — IMU-only observation matrix
//                Z_imu = [roll, pitch, yaw, ax_body, ay_body, az_body, wx, wy, wz]
//
// Barometer and GPS rows are appended in the caller to form the full H.
//
// Parameters:
//   p, q, u  — roll, pitch, yaw [rad]           (state X[3..5])
//   ax,ay,az — body-frame linear accelerations  (state X[12..14])
//   wt1, wt2 — motor speeds [rad/s]             (control U[2], U[3])
//   wx,wy,wz — angular velocities [rad/s]       (state X[9..11])
//   Kt       — motor thrust coefficient  [N/(rad/s)²]
//   Km       — motor torque coefficient  [N·m/(rad/s)²]
//   a1, a2   — gimbal angles [rad]              (control U[0], U[1])
//   a        — gimbal moment arm [m]
//   m        — vehicle mass [kg]
//   g        — gravitational acceleration [m/s²]
//   Jx,Jy,Jz — principal moments of inertia [kg·m²]
void update_kalman_matrices(dspm::Mat& A, dspm::Mat& B, dspm::Mat& H,
                            float p, float q, float u,
                            float ax, float ay, float az,
                            float wt1, float wt2, float wx, float wy, float wz,
                            float Kt, float Km, float a1, float a2, float a,
                            float m, float g, float Jx, float Jy, float Jz);
