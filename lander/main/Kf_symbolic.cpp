#include "Kf_symbolic.hpp"
#include <math.h>
#include <string.h>

void update_kalman_matrices(dspm::Mat& A, dspm::Mat& B, dspm::Mat& H,
                            float p, float q, float u,
                            float ax, float ay, float az,
                            float wt1, float wt2, float wx, float wy, float wz,
                            float Kt, float Km, float a1, float a2, float a,
                            float m, float g, float Jx, float Jy, float Jz) {

    // Reset all matrices to zero
    memset(A.data, 0, sizeof(float) * 18 * 18);
    memset(B.data, 0, sizeof(float) * 18 *  4);
    memset(H.data, 0, sizeof(float) *  9 * 18);

    // Pre-compute trig and repeated terms
    float cp = cosf(p); float sp = sinf(p);
    float cq = cosf(q); float sq = sinf(q);
    float cu = cosf(u); float su = sinf(u);

    float ca1 = cosf(a1); float sa1 = sinf(a1);
    float ca2 = cosf(a2); float sa2 = sinf(a2);

    float wt1_sq = wt1 * wt1;
    float wt2_sq = wt2 * wt2;
    float wt_sq  = wt1_sq + wt2_sq;

    float dt       = 0.01f;
    float dt2_half = 0.00005f;  // 0.5 * dt^2

    // -----------------------------------------------------------------
    // A (18×18) — state transition matrix
    // -----------------------------------------------------------------
   
    // Thrust-to-mass scale (MATLAB's t37 = 0.021 * wt_sq)
    float T = Kt * wt_sq / m;

    // Rotation-matrix column entries
    float r31 = cp*cu + sp*sq*su;   // t31
    float r32 = sp*su + cp*cu*sq;   // t32
    float r35 = cp*su - cu*sp*sq;   // t35
    float r36 = cu*sp - cp*sq*su;   // t36

    // Combined gravity + thrust terms
    float f42 = g*sq       + T*ca1*sa2;   // t42
    float f43 = g*cq*sp    + T*sa1;       // t43
    float f44 = g*cp*cq    - T*ca1*ca2;   // t44

    // ---- Rows 0-5: position / attitude integrators -------------------
    A(0, 0) = 1.0f;  A(0,  6) = dt;  A(0, 12) = dt2_half;
    A(1, 1) = 1.0f;  A(1,  7) = dt;  A(1, 13) = dt2_half;
    A(2, 2) = 1.0f;  A(2,  8) = dt;  A(2, 14) = dt2_half;
    A(3, 3) = 1.0f;  A(3,  9) = dt;  A(3, 15) = dt2_half;
    A(4, 4) = 1.0f;  A(4, 10) = dt;  A(4, 16) = dt2_half;
    A(5, 5) = 1.0f;  A(5, 11) = dt;  A(5, 17) = dt2_half;

    // ---- Rows 6-11: velocity / rate integrators ----------------------
    A(6,  6) = 1.0f;  A(6,  12) = dt;
    A(7,  7) = 1.0f;  A(7,  13) = dt;
    A(8,  8) = 1.0f;  A(8,  14) = dt;
    A(9,  9) = 1.0f;  A(9,  15) = dt;
    A(10,10) = 1.0f;  A(10, 16) = dt;
    A(11,11) = 1.0f;  A(11, 17) = dt;

    // ---- Rows 12-14: linear accel Jacobian w.r.t. euler angles ------
    A(12, 3) = T*(ca1*ca2*r35 - sa1*r32);
    A(12, 4) = g*cq*cq*cu - g*sp*sq*r35 - cu*sq*f42
             + g*cp*sq*r32 - cp*cq*cu*f44 - cq*cu*sp*f43;
    A(12, 5) = r31*f43 - r36*f44 - cq*su*f42;

    A(13, 3) = T*(sa1*r36 - ca1*ca2*r31);
    A(13, 4) = g*cq*cq*su + g*sp*sq*r31 - sq*su*f42
             - g*cp*sq*r36 - cp*cq*su*f44 - cq*sp*su*f43;
    A(13, 5) = r35*f43 - r32*f44 + cq*cu*f42;

    A(14, 3) = -T*cq*(cp*sa1 + sp*ca1*ca2);
    A(14, 4) = -cq*f42 + cp*sq*f44 + sp*sq*f43;
    // A(14, 5) is identically zero

    // ---- Rows 15-17: angular accel Jacobian (gyroscopic, cols 9-11) --
    A(15, 10) = (Jy - Jz) * wz / Jx;
    A(15, 11) = (Jy - Jz) * wy / Jx;

    A(16,  9) = (Jz - Jx) * wz / Jy;
    A(16, 11) = (Jz - Jx) * wx / Jy;

    A(17,  9) = (Jx - Jy) * wy / Jz;
    A(17, 10) = (Jx - Jy) * wx / Jz;

    
    // -----------------------------------------------------------------
    // B (18×4) — control input matrix
    // U = [a1 (gimbal1), a2 (gimbal2), wt1 (motor1), wt2 (motor2)]
    // -----------------------------------------------------------------

    // Linear acceleration rows (12–14)
    B(12, 0) = (Kt*ca1*wt_sq*(cp*su - cu*sp*sq))/m - (Kt*ca2*sa1*wt_sq*(sp*su + cp*cu*sq))/m - (Kt*sa1*sa2*cq*cu*wt_sq)/m;
    B(12, 1) = (Kt*ca1*ca2*cq*cu*wt_sq)/m - (Kt*ca1*sa2*wt_sq*(sp*su + cp*cu*sq))/m;
    B(12, 2) = (2.0f*Kt*wt1*sa1*(cp*su - cu*sp*sq))/m + (2.0f*Kt*wt1*ca1*ca2*(sp*su + cp*cu*sq))/m + (2.0f*Kt*wt1*ca1*sa2*cq*cu)/m;
    B(12, 3) = (2.0f*Kt*wt2*sa1*(cp*su - cu*sp*sq))/m + (2.0f*Kt*wt2*ca1*ca2*(sp*su + cp*cu*sq))/m + (2.0f*Kt*wt2*ca1*sa2*cq*cu)/m;

    B(13, 0) = (Kt*ca2*sa1*wt_sq*(cu*sp - cp*sq*su))/m - (Kt*ca1*wt_sq*(cp*cu + sp*sq*su))/m - (Kt*sa1*sa2*cq*su*wt_sq)/m;
    B(13, 1) = (Kt*ca1*sa2*wt_sq*(cu*sp - cp*sq*su))/m + (Kt*ca1*ca2*cq*su*wt_sq)/m;
    B(13, 2) = (2.0f*Kt*wt1*ca1*sa2*cq*su)/m - (2.0f*Kt*wt1*ca1*ca2*(cu*sp - cp*sq*su))/m - (2.0f*Kt*wt1*sa1*(cp*cu + sp*sq*su))/m;
    B(13, 3) = (2.0f*Kt*wt2*ca1*sa2*cq*su)/m - (2.0f*Kt*wt2*ca1*ca2*(cu*sp - cp*sq*su))/m - (2.0f*Kt*wt2*sa1*(cp*cu + sp*sq*su))/m;

    B(14, 0) = (Kt*sa1*sa2*sq*wt_sq)/m - (Kt*ca1*cq*sp*wt_sq)/m - (Kt*ca2*cp*sa1*cq*wt_sq)/m;
    B(14, 1) = -(Kt*ca1*ca2*sq*wt_sq)/m - (Kt*ca1*cp*sa2*cq*wt_sq)/m;
    B(14, 2) = (2.0f*Kt*wt1*ca1*ca2*cp*cq)/m - (2.0f*Kt*wt1*sa1*cq*sp)/m - (2.0f*Kt*wt1*ca1*sa2*sq)/m;
    B(14, 3) = (2.0f*Kt*wt2*ca1*ca2*cp*cq)/m - (2.0f*Kt*wt2*sa1*cq*sp)/m - (2.0f*Kt*wt2*ca1*sa2*sq)/m;

    // Angular acceleration rows (15–17)
    B(15, 0) = -(Km*wt1_sq*sa1*sa2 - Km*wt2_sq*sa1*sa2 + Kt*a*wt1_sq*ca1 + Kt*a*wt2_sq*ca1) / Jx;
    B(15, 1) =  (Km*ca1*ca2*wt1_sq - Km*ca1*ca2*wt2_sq) / Jx;
    B(15, 2) = -(2.0f*Kt*a*wt1*sa1 - 2.0f*Km*wt1*ca1*sa2) / Jx;
    B(15, 3) = -(2.0f*Kt*a*wt2*sa1 + 2.0f*Km*wt2*ca1*sa2) / Jx;

    B(16, 0) =  (Km*wt2_sq*ca1 - Km*wt1_sq*ca1 + Kt*a*wt1_sq*sa1*sa2 + Kt*a*wt2_sq*sa1*sa2) / Jy;
    B(16, 1) = -(Kt*a*ca1*ca2*wt1_sq + Kt*a*ca1*ca2*wt2_sq) / Jy;
    B(16, 2) = -(2.0f*Km*wt1*sa1 + 2.0f*Kt*a*wt1*ca1*sa2) / Jy;
    B(16, 3) =  (2.0f*Km*wt2*sa1 - 2.0f*Kt*a*wt2*ca1*sa2) / Jy;

    B(17, 0) = -(Km*ca2*sa1*wt1_sq - Km*ca2*sa1*wt2_sq) / Jz;
    B(17, 1) = -(Km*ca1*sa2*wt1_sq - Km*ca1*sa2*wt2_sq) / Jz;
    B(17, 2) =  (2.0f*Km*wt1*ca1*ca2) / Jz;
    B(17, 3) = -(2.0f*Km*wt2*ca1*ca2) / Jz;

    // -----------------------------------------------------------------
    // H (9×18) — IMU-only observation matrix
    // Z_imu = [roll, pitch, yaw, ax_world, ay_world, az_world, wx, wy, wz]
    // Barometer and GPS rows are appended by the caller.
    // -----------------------------------------------------------------

    // Direct euler angle measurements (rows 0–2 → state cols 3,4,5)
    H(0, 3) = 1.0f;
    H(1, 4) = 1.0f;
    H(2, 5) = 1.0f;

    // Accel Jacobian (rows 3-5): body-frame accel states rotated to world
    // frame (H(·,12/13/14) are rotation-matrix rows), matching Z[3..5].
    H(3, 4)  = -az*cq - ax*cu*sq - ay*sq*su;
    H(3, 5)  =  ay*cq*cu - ax*cq*su;
    H(3, 12) =  cq*cu;
    H(3, 13) =  cq*su;
    H(3, 14) = -sq;

    H(4, 3)  =  ax*(sp*su + cp*cu*sq) - ay*(cu*sp - cp*sq*su) + az*cp*cq;
    H(4, 4)  =  ax*cq*cu*sp - az*sp*sq + ay*cq*sp*su;
    H(4, 5)  = -ax*(cp*cu + sp*sq*su) - ay*(cp*su - cu*sp*sq);
    H(4, 12) =  cu*sp*sq - cp*su;
    H(4, 13) =  cp*cu + sp*sq*su;
    H(4, 14) =  cq*sp;

    H(5, 3)  =  ax*(cp*su - cu*sp*sq) - ay*(cp*cu + sp*sq*su) - az*cq*sp;
    H(5, 4)  =  ax*cp*cq*cu - az*cp*sq + ay*cp*cq*su;
    H(5, 5)  =  ax*(cu*sp - cp*sq*su) + ay*(sp*su + cp*cu*sq);
    H(5, 12) =  sp*su + cp*cu*sq;
    H(5, 13) =  cp*sq*su - cu*sp;
    H(5, 14) =  cp*cq;

    // Direct angular velocity measurements (rows 6–8 → state cols 9,10,11)
    H(6,  9) = 1.0f;
    H(7, 10) = 1.0f;
    H(8, 11) = 1.0f;
}
