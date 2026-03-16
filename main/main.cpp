#include <stdio.h>
#include <inttypes.h>
#include <cmath>
#include <string.h>
#include "Kf_symbolic.hpp"
//#include <esp_system.h> // Include for esp_cpu_get_load
#include <esp_heap_caps.h>
#include "globalvars.hpp"
#include "imu_init.hpp"
#include "i2c/servo.hpp"
#include "i2c/i2c_setup.hpp"
#include "motor_init.hpp"
#include "i2c/bmp390.hpp"
#include "espnow_init.hpp"

static const constexpr char* TAG = "Main";

// Clamp v to [lo, hi]
static inline float limit_f(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}


char pmem[512] = {0}; // Buffer to store the CPU usage data


// RTOS Task to log the current stats of the data from the IMU every 500ms
void measure_datarate(void *pvParameters)
{   
    while (1) {
        size_t free_heap_size = esp_get_free_heap_size(); // check the amount of ram left

        //check CPU Usage
        /*uint32_t cpu_usage_core_0 = 0;
        uint32_t cpu_usage_core_1 = 0;
        esp_cpu_get_load(ESP_CPU_MAIN, &cpu_usage_core_0); // ESP_CPU_MAIN is Core 0
        esp_cpu_get_load(ESP_CPU_APP, &cpu_usage_core_1);  // ESP_CPU_APP is Core 1*/

        /*if (timestamps.size() != 0) {
            latest_timestamp = timestamps.back();
        } else {
            latest_timestamp = 0;
        }*/

        //ESP_LOGI(TAG, "ABOUT TO PRINT");
        /*vTaskGetRunTimeStats(pmem);
        ESP_LOGI(TAG, "CPU Usage: %s", pmem);*/
        
        //original logging message with vectors
        //ESP_LOGI(TAG, "Seconds since boot (s): %lld, Vector Sizes: Euler: %d, AngVel: %d, Grav: %d, AngAccel: %d, LinAcc: %d, Free Heap Size %u", (latest_timestamp/1000000), euler_data.size(), velocity_data.size(), gravity_data.size(), ang_accel_data.size(),lin_accel_data.size(), free_heap_size);

        // snapshot globals
        portENTER_CRITICAL(&global_spinlock);
        uint64_t t_now = latest_timestamp;
        int32_t ec = euler_counter;
        auto pos = latest_position;
        auto euler = latest_euler_data;
        auto ang_vel = latest_ang_velocity_data;
        auto vel = latest_velocity;
        auto grav = latest_gravity_data;
        auto ang_accel = latest_ang_accel_data;
        auto lin_accel = latest_lin_accel_data;
        portEXIT_CRITICAL(&global_spinlock);

        //logging message with latest data
        if (!estop_triggered) {
            ESP_LOGI(TAG, "Seconds since boot (s): %llu, euler datapoints %" PRId32 ", Latest Pos: (x: %.2f y: %.2f z: %.2f), Latest Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg], Latest Angular Velocity: (x: %.2f y: %.2f z: %.2f)[rad/s], Latest Linear Velocity: (x: %.2f y: %.2f z: %.2f) [m/s], Latest Gravity: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Angular Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Latest Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2], Free Heap Size %" PRIu32, \
                (t_now/1000000), ec, pos.x, pos.y, pos.z, \
                euler.x, euler.y, euler.z, \
                ang_vel.x, ang_vel.y, ang_vel.z, \
                vel.x, vel.y, vel.z, \
                grav.x, grav.y, grav.z, \
                ang_accel.x, ang_accel.y, ang_accel.z, \
                lin_accel.x, lin_accel.y, lin_accel.z, free_heap_size);
        }

        //todo: error not handled when vector size is 0
        //ESP_LOGI(TAG, "Last Euler Angle: (x (roll): %.2f y (pitch): %.2f z (yaw): %.2f)[deg]", euler_data.back().x, euler_data.back().y, euler_data.back().z);
        //ESP_LOGI(TAG, "Linear Accel: (x: %.2f y: %.2f z: %.2f)[m/s^2]", lin_accel_data.back().x, lin_accel_data.back().y, lin_accel_data.back().z);
        vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 500ms
    }
}

// Kalman filter state estimation task: 100Hz (10ms period)
//
// State vector X (18x1), row-major index:
//   [0=x,  1=y,  2=z,  3=roll, 4=pitch, 5=yaw,       (position / attitude, m / rad)
//    6=vx, 7=vy, 8=vz, 9=wx,  10=wy,   11=wz,         (velocity / ang-rate, m/s / rad/s)
//    12=ax,13=ay,14=az,15=αx,  16=αy,  17=αz]         (accel / ang-accel, m/s² / rad/s²)
//
// Control input U (4x1): [a1 (gimbal1), a2 (gimbal2), wt1 (motor1), wt2 (motor2)]
//
// Measurement vector Z:
//   without GPS (10x1): [roll,pitch,yaw, ax_body,ay_body,az_body, wx,wy,wz, baro_alt]
//   with    GPS (13x1): same + [gps_x, gps_y, gps_z]
//
// H structure:
//   H_imu (9×18)  — dynamic, linearised around current state (from update_kalman_matrices)
//   H_nogps(10×18) — H_imu rows 0-8 + fixed baro row (→ z, col 2)
//   H_gps (13×18) — H_nogps rows 0-9 + fixed GPS rows (→ x,y,z, cols 0,1,2)
//
// Kalman equations:
//   Xpre = A*X + B*U
//   X    = Xpre + Kf*(Z - H*Xpre)
void state_estimation(void *pvParameters)
{
    constexpr uint32_t dt_ms = 10;
    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);

    // -------------------------------------------------------------------------
    // Vehicle physical constants — PLACEHOLDER: fill in before use
    // -------------------------------------------------------------------------
    constexpr float KT   = 0.0f;   // thrust coefficient        [N/(rad/s)²]
    constexpr float KM   = 0.0f;   // motor torque coefficient  [N·m/(rad/s)²]
    constexpr float ARM  = 0.0f;   // gimbal moment arm         [m]
    constexpr float MASS = 1.0f;   // vehicle mass              [kg]
    constexpr float GRAV = 9.81f;  // gravitational acceleration [m/s²]
    constexpr float JX   = 1.0f;   // moment of inertia x       [kg·m²]
    constexpr float JY   = 1.0f;   // moment of inertia y       [kg·m²]
    constexpr float JZ   = 1.0f;   // moment of inertia z       [kg·m²]

    // -------------------------------------------------------------------------
    // State vector — zero-initialised, persists across iterations
    // -------------------------------------------------------------------------
    static float X[18] = {0};

    // -------------------------------------------------------------------------
    // Dynamic matrices (rebuilt each iteration via update_kalman_matrices)
    // dspm::Mat uses heap storage; static ensures single allocation at task start.
    // -------------------------------------------------------------------------
    static dspm::Mat A(18, 18);
    static dspm::Mat B(18,  4);
    static dspm::Mat H_imu(9, 18);

    // -------------------------------------------------------------------------
    // H_nogps (10×18): H_imu rows 0-8 (copied each loop) + fixed baro row 9
    // H_gps   (13×18): H_nogps rows 0-9 (copied each loop) + fixed GPS rows 10-12
    // Fixed rows are set once here; memcpy inside the loop only touches rows 0-8/0-9.
    // -------------------------------------------------------------------------
    static float H_nogps_full[10 * 18] = {0};
    static float H_gps_full[13 * 18]   = {0};

    // Baro row (row 9 in both): altitude → z (state col 2)
    H_nogps_full[9 * 18 + 2] = 1.0f;
    H_gps_full  [9 * 18 + 2] = 1.0f;
    // GPS rows (rows 10-12 in H_gps_full): direct x, y, z position measurements
    H_gps_full[10 * 18 + 0] = 1.0f;  // GPS x → state col 0
    H_gps_full[11 * 18 + 1] = 1.0f;  // GPS y → state col 1
    H_gps_full[12 * 18 + 2] = 1.0f;  // GPS z → state col 2

    // -------------------------------------------------------------------------
    // Kf_nogps (18×10) and Kf_gps (18×13) — fixed Kalman gains, row-major
    // PLACEHOLDER: fill in tuned values before use.
    // -------------------------------------------------------------------------
    static const float Kf_nogps[18 * 10] = {0};
    static const float Kf_gps[18 * 13]   = {0};

    // GPS freshness tracking
    static float last_gps[3] = {0.0f, 0.0f, 0.0f};

    while (1)
    {
        // -----------------------------------------------------------------
        // 1. Read sensor data under spinlock
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        auto meas_euler   = latest_euler_data;
        auto meas_ang_vel = latest_ang_velocity_data;
        auto meas_lin_acc = latest_lin_accel_data;
        double baro_alt   = altitude;
        double gps_x_d    = latest_gps_position.x;
        double gps_y_d    = latest_gps_position.y;
        double gps_z_d    = latest_gps_position.z;
        portEXIT_CRITICAL(&global_spinlock);

        float gps_x = static_cast<float>(gps_x_d);
        float gps_y = static_cast<float>(gps_y_d);
        float gps_z = static_cast<float>(gps_z_d);

        // -----------------------------------------------------------------
        // 2. GPS freshness check
        // -----------------------------------------------------------------
        bool gps_present = (last_gps[0] != gps_x || last_gps[1] != gps_y || last_gps[2] != gps_z);
        if (gps_present) {
            last_gps[0] = gps_x;
            last_gps[1] = gps_y;
            last_gps[2] = gps_z;
        }

        // -----------------------------------------------------------------
        // 3. Build control input U = [a1, a2, wt1, wt2]
        //    PLACEHOLDER: populate with actual actuator readings when available.
        // -----------------------------------------------------------------
        float U[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        // U[0] = gimbal1_angle; U[1] = gimbal2_angle;
        // U[2] = motor1_speed;  U[3] = motor2_speed;

        // -----------------------------------------------------------------
        // 4. Rebuild linearised A, B, H_imu around current state and inputs
        // -----------------------------------------------------------------
        update_kalman_matrices(A, B, H_imu,
            X[3],  X[4],  X[5],   // roll, pitch, yaw
            X[12], X[13], X[14],  // body-frame ax, ay, az
            U[2],  U[3],          // wt1, wt2 (motor speeds)
            X[9],  X[10], X[11], // wx, wy, wz
            KT, KM, U[0], U[1], ARM, MASS, GRAV, JX, JY, JZ);

        // Copy dynamic H_imu rows into the combined matrices (rows 0–8 only;
        // baro and GPS rows were set once before the loop and are not touched).
        memcpy(H_nogps_full, H_imu.data, 9 * 18 * sizeof(float));
        memcpy(H_gps_full,   H_imu.data, 9 * 18 * sizeof(float));

        // -----------------------------------------------------------------
        // 5. Build measurement vector Z and select H / Kf
        // -----------------------------------------------------------------
        float Z[13] = {0};
        Z[0] = meas_euler.x * deg2rad;               // roll  [rad]
        Z[1] = meas_euler.y * deg2rad;               // pitch [rad]
        Z[2] = meas_euler.z * deg2rad;               // yaw   [rad]
        Z[3] = static_cast<float>(meas_lin_acc.x);   // ax body [m/s²]
        Z[4] = static_cast<float>(meas_lin_acc.y);   // ay body
        Z[5] = static_cast<float>(meas_lin_acc.z);   // az body
        Z[6] = static_cast<float>(meas_ang_vel.x);   // wx [rad/s]
        Z[7] = static_cast<float>(meas_ang_vel.y);   // wy
        Z[8] = static_cast<float>(meas_ang_vel.z);   // wz
        Z[9] = static_cast<float>(baro_alt);          // baro altitude [m]

        const float *H_ptr;
        const float *Kf_ptr;
        int meas_dim;

        if (gps_present) {
            Z[10]    = gps_x;
            Z[11]    = gps_y;
            Z[12]    = gps_z;
            H_ptr    = H_gps_full;
            Kf_ptr   = Kf_gps;
            meas_dim = 13;
        } else {
            H_ptr    = H_nogps_full;
            Kf_ptr   = Kf_nogps;
            meas_dim = 10;
        }

        // -----------------------------------------------------------------
        // 6. Prediction: Xpre = A*X + B*U
        // -----------------------------------------------------------------
        float Xpre[18]  = {0};
        float tmp18[18] = {0};
        float bu18[18]  = {0};

        dspm_mult_f32(A.data, X,    tmp18, 18, 18, 1);  // A(18×18) * X(18×1)
        dspm_mult_f32(B.data, U,    bu18,  18,  4, 1);  // B(18×4)  * U(4×1)
        for (int i = 0; i < 18; i++) Xpre[i] = tmp18[i] + bu18[i];

        // -----------------------------------------------------------------
        // 7. Innovation: innov = Z - H*Xpre
        // -----------------------------------------------------------------
        float HXpre[13] = {0};
        float innov[13] = {0};

        dspm_mult_f32(H_ptr, Xpre, HXpre, meas_dim, 18, 1);  // H(m×18) * Xpre(18×1)
        for (int i = 0; i < meas_dim; i++) innov[i] = Z[i] - HXpre[i];

        // -----------------------------------------------------------------
        // 8. Update: X = Xpre + Kf*innov
        // -----------------------------------------------------------------
        float Kf_innov[18] = {0};

        dspm_mult_f32(Kf_ptr, innov, Kf_innov, 18, meas_dim, 1);  // Kf(18×m) * innov(m×1)
        for (int i = 0; i < 18; i++) X[i] = Xpre[i] + Kf_innov[i];

        // -----------------------------------------------------------------
        // 9. Write estimated state to globals
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        latest_position.x = static_cast<double>(X[0]);
        latest_position.y = static_cast<double>(X[1]);
        latest_position.z = static_cast<double>(X[2]);
        latest_velocity.x = static_cast<double>(X[6]);
        latest_velocity.y = static_cast<double>(X[7]);
        latest_velocity.z = static_cast<double>(X[8]);
        portEXIT_CRITICAL(&global_spinlock);

        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
}

// Position controller (LQR outer loop) — runs at POS_CTRL_PERIOD_MS
//
// Reads x, y, vx, vy, yaw from the Kalman filter globals.
// Computes:  output = K_pos * [x_err, y_err, vx_err, vy_err, int_x, int_y]'
// Writes:    U_pos.roll / U_pos.pitch — desired angles [rad] for the hover controller.
//
// State error is rotated into the body frame using the current yaw before the
// LQR gain is applied (hover assumption: roll ≈ 0, pitch ≈ 0).
void position_controller(void *pvParameters)
{
    // Task timing — CONTROL_LOOP_INTERVAL must equal PERIOD_MS / 1000
    constexpr uint32_t PERIOD_MS            = 20;       // 50 Hz
    constexpr float    CONTROL_LOOP_INTERVAL = 0.02f;   // [s]  matches PERIOD_MS

    // Integral windup limit — PLACEHOLDER: tune for the vehicle
    constexpr float pos_int_limit = 0.5f;

    // Output saturation: ±10 degrees in roll / pitch
    constexpr float OUT_LIMIT = 10.0f * static_cast<float>(M_PI / 180.0);

    // K_pos (2×6) — LQR gain matrix, row-major
    // output = K_pos * [x_err, y_err, vx_err, vy_err, int_x_err, int_y_err]'
    // PLACEHOLDER: fill in tuned values before use.
    static const float K_pos[2 * 6] = {0};

    // SP_pos (6×1) — position setpoint; all zeros = hover at origin
    static const float SP_pos[6] = {0, 0, 0, 0, 0, 0};

    // Integral accumulators (persist across iterations)
    static float error_integral_x = 0.0f;
    static float error_integral_y = 0.0f;

    while (1)
    {
        // -----------------------------------------------------------------
        // 1. Read Kalman filter outputs under spinlock
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        float x   = static_cast<float>(latest_position.x);
        float y   = static_cast<float>(latest_position.y);
        float vx  = static_cast<float>(latest_velocity.x);
        float vy  = static_cast<float>(latest_velocity.y);
        float yaw = latest_euler_data.z * static_cast<float>(M_PI / 180.0); // deg → rad
        portEXIT_CRITICAL(&global_spinlock);

        // -----------------------------------------------------------------
        // 2. Build state and compute error = SP_pos - X_pos
        // -----------------------------------------------------------------
        float X_pos[6] = {x, y, vx, vy, 0.0f, 0.0f};
        float error[6];
        for (int i = 0; i < 6; i++) error[i] = SP_pos[i] - X_pos[i];

        // -----------------------------------------------------------------
        // 3. Rotate position and velocity errors to body frame
        //    (yaw rotation only — valid under hover assumption roll≈0, pitch≈0)
        //    Intermediate values saved so each rotation uses original components.
        // -----------------------------------------------------------------
        float cyaw = cosf(yaw), syaw = sinf(yaw);

        float ex  = error[0]*cyaw + error[1]*syaw;
        float ey  = error[1]*cyaw - error[0]*syaw;
        float evx = error[2]*cyaw + error[3]*syaw;
        float evy = error[3]*cyaw - error[2]*syaw;
        error[0] = ex;
        error[1] = ey;
        error[2] = evx;
        error[3] = evy;

        // -----------------------------------------------------------------
        // 4. Euler-integrate x/y errors; clamp to prevent windup
        // -----------------------------------------------------------------
        error_integral_x += error[0] * CONTROL_LOOP_INTERVAL;
        error_integral_y += error[1] * CONTROL_LOOP_INTERVAL;

        error_integral_x = limit_f(error_integral_x, -pos_int_limit, pos_int_limit);
        error_integral_y = limit_f(error_integral_y, -pos_int_limit, pos_int_limit);

        error[4] = error_integral_x;
        error[5] = error_integral_y;

        // -----------------------------------------------------------------
        // 5. LQR: output (2×1) = K_pos (2×6) * error (6×1)
        // -----------------------------------------------------------------
        float output[2] = {0.0f, 0.0f};
        dspm_mult_f32(K_pos, error, output, 2, 6, 1);

        // Clamp output to ±10° in roll and pitch
        output[0] = limit_f(output[0], -OUT_LIMIT, OUT_LIMIT);
        output[1] = limit_f(output[1], -OUT_LIMIT, OUT_LIMIT);

        // -----------------------------------------------------------------
        // 6. Publish U_pos for the hover controller
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        U_pos.roll  = output[0];
        U_pos.pitch = output[1];
        portEXIT_CRITICAL(&global_spinlock);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}

// Hover controller (LQR inner loop) — runs at 50 Hz
//
// Reads roll, pitch, yaw [rad], gx, gy, gz [rad/s] from IMU globals;
//        z [m], vz [m/s] from KF globals; U_pos from position controller.
//
// State error (9×1): [roll, pitch, yaw, gx, gy, gz, z, vz, int_z]
//   SP_hover[0..1] are augmented by U_pos.roll/pitch each iteration.
//
// LQR:        lqr_out (4×1) = K_hov (4×9) * error (9×1)
//             lqr_out = [F1, F2, F3, Mz]
//
// Actuation mapping (thrust geometry):
//   F_norm  = ||[F1, F2, F3]||
//   alpha_1 = asin(F2 / F_norm)
//   alpha_2 = acos(F3 / sqrt(F1² + F3²))
//   omega_1 = sqrt(F_norm / Kt)
//   lambda  = 1 - Mz / (c * F3)
//   omega_2 = sqrt(|lambda * F_norm| / Kt)
//
// Writes result to U_hov global.
void hover_controller(void *pvParameters)
{
    constexpr uint32_t PERIOD_MS             = 20;      // 50 Hz
    constexpr float    CONTROL_LOOP_INTERVAL = 0.02f;   // [s]

    constexpr float deg2rad = static_cast<float>(M_PI / 180.0);

    // Actuation mapping constants
    constexpr float KT_ACT = 0.021f;  // thrust coefficient [N/(rad/s)²]
    constexpr float C_ARM  = 0.05f;   // gimbal moment arm [m]

    // Anti-windup throttle threshold — PLACEHOLDER: fill in before use
    constexpr float MAX_THROTTLE   = 0.0f;  // maximum throttle value
    constexpr float DSHOT_THROTTLE = 0.0f;  // current throttle PLACEHOLDER

    // K_hov (4×9) — LQR gain matrix, row-major
    // lqr_out = K_hov * [roll_err, pitch_err, yaw_err, gx_err, gy_err, gz_err, z_err, vz_err, int_z_err]'
    // PLACEHOLDER: fill in tuned values before use.
    static const float K_hov[4 * 9] = {0};

    // SP_hover (9×1) — hover setpoint; all zeros (level attitude, hold position)
    // SP_hover[0] and [1] are augmented by U_pos each iteration.
    static const float SP_hover[9] = {0};

    // Altitude integral accumulator
    static float error_integral_z = 0.0f;

    while (1)
    {
        // -----------------------------------------------------------------
        // 1. Read sensor data under spinlock
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        float roll  = latest_euler_data.x * deg2rad;
        float pitch = latest_euler_data.y * deg2rad;
        float yaw   = latest_euler_data.z * deg2rad;
        float gx    = static_cast<float>(latest_ang_velocity_data.x);
        float gy    = static_cast<float>(latest_ang_velocity_data.y);
        float gz    = static_cast<float>(latest_ang_velocity_data.z);
        float z     = static_cast<float>(latest_position.z);
        float vz    = static_cast<float>(latest_velocity.z);
        float upos_roll  = U_pos.roll;
        float upos_pitch = U_pos.pitch;
        portEXIT_CRITICAL(&global_spinlock);

        // -----------------------------------------------------------------
        // 2. Build reference: SP_hover augmented by position controller output
        // -----------------------------------------------------------------
        float ref[9];
        for (int i = 0; i < 9; i++) ref[i] = SP_hover[i];
        ref[0] += upos_roll;   // desired roll  [rad]
        ref[1] += upos_pitch;  // desired pitch [rad]

        // -----------------------------------------------------------------
        // 3. Build state and compute error = ref - X
        // -----------------------------------------------------------------
        float X_hov[9] = {roll, pitch, yaw, gx, gy, gz, z, vz, 0.0f};
        float error[9];
        for (int i = 0; i < 9; i++) error[i] = ref[i] - X_hov[i];

        // Wrap yaw error to (-π, π]
        if (error[2] > static_cast<float>(M_PI))
            error[2] -= 2.0f * static_cast<float>(M_PI);

        // -----------------------------------------------------------------
        // 4. Altitude integral with anti-windup
        // -----------------------------------------------------------------
        float error_z = error[6];
        if (DSHOT_THROTTLE < MAX_THROTTLE || error_z < 0.0f) {
            error_integral_z += error_z * CONTROL_LOOP_INTERVAL;
        }
        error[8] = error_integral_z;

        // -----------------------------------------------------------------
        // 5. LQR: lqr_out (4×1) = K_hov (4×9) * error (9×1)
        //    lqr_out = [F1, F2, F3, Mz]
        // -----------------------------------------------------------------
        float lqr_out[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        dspm_mult_f32(K_hov, error, lqr_out, 4, 9, 1);

        float F1 = lqr_out[0];
        float F2 = lqr_out[1];
        float F3 = lqr_out[2];
        float Mz = lqr_out[3];

        // -----------------------------------------------------------------
        // 6. Actuation mapping: [F1, F2, F3, Mz] → [alpha1, alpha2, omega1, omega2, lambda]
        // -----------------------------------------------------------------
        float F_norm  = sqrtf(F1*F1 + F2*F2 + F3*F3);
        float alpha_1 = asinf(F2 / F_norm);
        float alpha_2 = acosf(F3 / sqrtf(F1*F1 + F3*F3));
        float omega_1 = sqrtf(F_norm / KT_ACT);
        float lambda  = 1.0f - (Mz / (C_ARM * F3));
        float F_2norm = lambda * F_norm;
        float omega_2 = sqrtf(fabsf(F_2norm) / KT_ACT);

        // -----------------------------------------------------------------
        // 7. Publish actuation output
        // -----------------------------------------------------------------
        portENTER_CRITICAL(&global_spinlock);
        U_hov.alpha1 = alpha_1;
        U_hov.alpha2 = alpha_2;
        U_hov.omega1 = omega_1;
        U_hov.omega2 = omega_2;
        U_hov.lambda = lambda;
        portEXIT_CRITICAL(&global_spinlock);

        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}

void esp_now_task(void *pvParameters)
{
    while(1) {
        if (!estop_triggered) {
            esp_now_send_data();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); //send every 10ms
    }
}

void bmp390_task(void *pvParameters)
{
    while (1)
    {
        double t, p, a;
        esp_err_t result = bmp390_get_data(&t, &p, &a);

        if (result == ESP_OK) {
            portENTER_CRITICAL(&global_spinlock);
            temperature = t;
            pressure = p;
            altitude = a;
            portEXIT_CRITICAL(&global_spinlock);

            if (!estop_triggered) {
                ESP_LOGI(TAG, "Temperature: %.2f °C, Pressure: %.2f Pa, Altitude: %.2f m", t, p, a);
            }
        } else {
            if (!estop_triggered) {
                ESP_LOGE(TAG, "Failed to read data from BMP390 sensor");
            }
        } 
        vTaskDelay(pdMS_TO_TICKS(500)); //Run every 0.5 seconds?
    }
}

void testServo() {
    while(1){
        
        pca9685_set_servo_angle(1, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 45);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 90);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 135);
        vTaskDelay(pdMS_TO_TICKS(1000));
        pca9685_set_servo_angle(1, 180);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // 110-120 is the start point, 540-550 is the end point
    }
}

extern "C" void app_main(void)
{
    //Initialise I2C with error checking
    esp_err_t i2c_result = i2c_master_init();
    if (i2c_result == ESP_OK) {
        ESP_LOGI(TAG, "I2C initialized successfully");
    } else {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(i2c_result));
        return;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); //Let I2C stabilise
    
    pca9685_init();
    bmp390_init(); 
    init_espnow_sender();

    ESP_LOGI(TAG, "Waiting for IMU power-up...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    imu_init();

    // Create the vector logging task with higher stack
    BaseType_t measure_datarate_result = xTaskCreatePinnedToCore(measure_datarate, "measure datarate", 8192, NULL, 1, NULL, APP_CPU_NUM);
    if (measure_datarate_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create vector logging task!");
    } else {
        ESP_LOGI(TAG, "Vector logging task started.");
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Give task time to start
    
    // Launch state estimation task
    BaseType_t state_estimation_task = xTaskCreatePinnedToCore(state_estimation, "state estimation", 4096, NULL, 2, NULL, APP_CPU_NUM);
    if(state_estimation_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create state estimation task!");
    } else {
        ESP_LOGI(TAG, "State estimation task started.");
    }

    // Launch position controller task (LQR outer loop, 50 Hz)
    BaseType_t pos_ctrl_task = xTaskCreatePinnedToCore(position_controller, "pos_ctrl", 4096, NULL, 2, NULL, APP_CPU_NUM);
    if(pos_ctrl_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create position controller task!");
    } else {
        ESP_LOGI(TAG, "Position controller task started.");
    }

    // Launch hover controller task (LQR inner loop, 50 Hz)
    BaseType_t hov_ctrl_task = xTaskCreatePinnedToCore(hover_controller, "hov_ctrl", 4096, NULL, 2, NULL, APP_CPU_NUM);
    if(hov_ctrl_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create hover controller task!");
    } else {
        ESP_LOGI(TAG, "Hover controller task started.");
    }

    // while (1)
    // {
    //     // delay time is irrelevant, we just don't want to trip WDT
    //     vTaskDelay(100UL / portTICK_PERIOD_MS); //originally 10000UL
    // }

    // Initialize the motor
    /*void all_motor_init(void){
        // Initialize the 2 motors with the specified GPIO pins and RMT channels
        initializeMotor(GPIO_NUM_4, RMT_CHANNEL_0);
        initializeMotor(GPIO_NUM_5, RMT_CHANNEL_1);
    }*/

    //code for RTOS Task
    //TODO IMU: Tempoarily disabled to test the IMU Only
    
    BaseType_t motor_task = xTaskCreatePinnedToCore(init_2_motors, "initialize 2 motors", 4096, NULL, 3, NULL, APP_CPU_NUM);
    if(motor_task != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor task!");
    } else {
        ESP_LOGI(TAG, "motor task started.");
    }
    

    //reverting to function based code for testing 
    //init_2_motors();

    //move esp_now_task to CPU 0 to avoid watchdog issues on CPU 1
    BaseType_t esp_now_task_handle = xTaskCreatePinnedToCore(esp_now_task, "esp_now_task", 4096, NULL, 4, NULL, PRO_CPU_NUM);
    if(esp_now_task_handle != pdPASS) {
        ESP_LOGE(TAG, "Failed to create esp_now task!");
    } else {
        ESP_LOGI(TAG, "esp_now task started on CPU 0.");
    } 

    //launch BMP390 sensor task
    BaseType_t bmp390_task_handle = xTaskCreatePinnedToCore(bmp390_task, "bmp390_task", 4096, NULL, 1, NULL, APP_CPU_NUM);
    if(bmp390_task_handle != pdPASS) {
        ESP_LOGE(TAG, "Failed to create BMP390 task!");
    } else {
        ESP_LOGI(TAG, "BMP390 task started.");
    }

    //todo:code will never reach here as we are testing the motor in an infinite loop

    while (1)
    {
        // delay time is irrelevant, we just don't want to trip WDT
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}