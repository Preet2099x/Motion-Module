#pragma once

// Distance from the IMU alone — accelerometer double-integrated for distance,
// gyro used to decide whether the robot is genuinely moving or the signal is
// just noise/bias drift.
//
// This is deliberately INDEPENDENT of the encoder pipeline in distance.cpp.
// Nothing in Drive or Navigation reads it; it runs alongside so the two
// sources can be compared. Inertial distance drifts quadratically, so treat
// `travelled` as a short-window figure and a slip detector, not as ground
// truth over a long run.

struct ImuDistanceData
{
    float travelled;      // absolute path length, monotonic (m)
    float displacement;   // signed along-forward-axis displacement (m)
    float velocity;       // integrated forward velocity (m/s)
    float accel;          // bias-corrected forward acceleration (m/s^2)

    bool  moving;         // gyro+accel gate: real motion, not drift
    bool  calibrated;     // boot bias estimate is valid
    bool  turnFrozen;     // integrator held off because the body is rotating

    float accelBias;      // learned zero offset on the forward axis (m/s^2)
    float gyroMag;        // |gyro| this sample (deg/s)
    float liaMag;         // |linear accel| this sample (m/s^2)
    int   calibProgress;  // stationary samples banked toward the boot estimate

    unsigned long zuptCount; // times velocity was force-zeroed by the gate
};

extern ImuDistanceData imuDistance;

void ImuDistance_Init();
void ImuDistance_Reset();          // zeroes distance, keeps the learned bias
void ImuDistance_Recalibrate();    // throws away the bias and relearns it
void ImuDistance_Update(float dt); // dt in seconds

bool ImuDistance_IsMoving();

// One-line dump of every gate that can hold the integrator at zero.
void ImuDistance_PrintStatus();
