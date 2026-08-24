#include "imu_distance.h"

#include <Arduino.h>
#include <math.h>
#include <var.h>

ImuDistanceData imuDistance;

//======================================
// Mounting
//======================================
// Which BNO axis points along the robot's driving direction, and its sign.
// These are the only two things to change if the board is remounted.
// The BNO runs in NDOF, so imuLia* is already gravity-compensated — no
// tilt correction is needed here.

#define IMU_FORWARD_AXIS imuLiaX
#define IMU_FORWARD_SIGN (1.0f)

//======================================
// Tuning
//======================================

// ── Stationary detection ──
// "Is it really moving, or is this just drift?" Both sensors must be quiet.
// Accel alone is not enough: during a slow in-place turn the linear accel is
// tiny, and accel-only detection would wrongly call that stationary. The gyro
// is what catches it.
static const float ACCEL_QUIET_THRESHOLD = 0.12f; // m/s^2 — |LIA| below this reads as still
static const float GYRO_QUIET_THRESHOLD  = 1.5f;  // deg/s — |gyro| below this reads as still
static const int   ZUPT_SAMPLES          = 5;     // consecutive quiet samples before zeroing (50 ms @ 100 Hz)

// ── Bias learning ──
// Only ever updated while confidently stationary, so real motion can never
// be absorbed into the bias estimate.
static const float BIAS_ALPHA    = 0.02f; // EMA weight per stationary sample
static const int   CALIB_SAMPLES = 100;   // stationary samples needed at boot (1 s @ 100 Hz)

// ── Integration guards ──
// Without these, sensor noise integrates into velocity and velocity noise
// integrates into distance — the classic runaway.
static const float ACCEL_DEADBAND      = 0.05f;  // m/s^2 — below this integrates as 0
static const float VELOCITY_LEAK       = 0.995f; // per-sample decay, bleeds residual drift
static const float MAX_PLAUSIBLE_SPEED = 3.0f;   // m/s — hard clamp, robot cannot exceed this
static const float MAX_DT              = 0.10f;  // s — reject stalled loop ticks

// ── Rotation gate ──
// During an in-place turn the forward axis picks up tangential and centripetal
// terms that are not translation. Above this rate, freeze the integrator
// rather than feed it garbage.
// Tradeoff: Navigation's arc codes (111/121/115/125/117/127) rotate while
// genuinely translating. Set this ABOVE the arc yaw rate and BELOW the
// in-place-turn rate, or arcs will be dropped from the distance total.
static const float TURN_RATE_FREEZE = 25.0f; // deg/s
static const bool  FREEZE_ON_TURN   = true;

//======================================

static float prevAccel  = 0.0f;
static int   quietCount = 0;
static int   calibCount = 0;

void ImuDistance_Init()
{
    imuDistance.accelBias  = 0.0f;
    imuDistance.calibrated = false;
    imuDistance.calibProgress = 0;
    calibCount = 0;

    ImuDistance_Reset();
}

void ImuDistance_Reset()
{
    // Keeps accelBias/calibrated — relearning costs a second of standing
    // still, and a reset is usually asked for mid-run.
    imuDistance.travelled    = 0.0f;
    imuDistance.displacement = 0.0f;
    imuDistance.velocity     = 0.0f;
    imuDistance.accel        = 0.0f;

    imuDistance.moving     = false;
    imuDistance.turnFrozen = false;
    imuDistance.gyroMag    = 0.0f;
    imuDistance.zuptCount  = 0;

    prevAccel  = 0.0f;
    quietCount = 0;
}

void ImuDistance_Recalibrate()
{
    imuDistance.accelBias  = 0.0f;
    imuDistance.calibrated = false;
    imuDistance.calibProgress = 0;
    calibCount = 0;

    ImuDistance_Reset();
    Serial.println("IMUDIST: Recalibrating — hold the robot still");
}

bool ImuDistance_IsMoving()
{
    return imuDistance.moving;
}

void ImuDistance_PrintStatus()
{
    // Every gate that can pin travelled at 0.000, in the order they apply.
    Serial.print("IMUDBG: cal=");
    Serial.print(imuDistance.calibrated ? 1 : 0);

    Serial.print(" n=");
    Serial.print(imuDistance.calibProgress);
    Serial.print("/");
    Serial.print(CALIB_SAMPLES);

    Serial.print(" lia=");
    Serial.print(imuDistance.liaMag, 3);
    Serial.print("(thr ");
    Serial.print(ACCEL_QUIET_THRESHOLD, 2);
    Serial.print(")");

    Serial.print(" gyr=");
    Serial.print(imuDistance.gyroMag, 3);
    Serial.print("(thr ");
    Serial.print(GYRO_QUIET_THRESHOLD, 2);
    Serial.print(")");

    Serial.print(" mov=");
    Serial.print(imuDistance.moving ? 1 : 0);

    Serial.print(" frz=");
    Serial.print(imuDistance.turnFrozen ? 1 : 0);

    Serial.print(" bias=");
    Serial.print(imuDistance.accelBias, 3);

    Serial.print(" a=");
    Serial.print(imuDistance.accel, 3);

    Serial.print(" v=");
    Serial.print(imuDistance.velocity, 3);

    // Raw axes -- whichever one swings hardest when you push the robot
    // straight forward is the one IMU_FORWARD_AXIS should point at.
    Serial.print(" | LIA x=");
    Serial.print(imuLiaX, 2);
    Serial.print(" y=");
    Serial.print(imuLiaY, 2);
    Serial.print(" z=");
    Serial.print(imuLiaZ, 2);

    // LIA is only trustworthy once acc reaches 3.
    // TODO: bnoCalSys/Gyr/Acc/Mag are not implemented anywhere — nothing reads
    // the BNO055 CALIB_STAT register (0x35) or exports it via var.h, so this
    // block did not compile and broke the whole build. Re-enable once bno.cpp
    // actually publishes those four values.
    Serial.println();
}

void ImuDistance_Update(float dt)
{
    float ax = IMU_FORWARD_SIGN * IMU_FORWARD_AXIS;

    float liaMag = sqrtf(imuLiaX * imuLiaX +
                         imuLiaY * imuLiaY +
                         imuLiaZ * imuLiaZ);

    float gyroMag = sqrtf(imuGx * imuGx +
                          imuGy * imuGy +
                          imuGz * imuGz);

    imuDistance.gyroMag = gyroMag;
    imuDistance.liaMag  = liaMag;

    // ── Motion gate ─────────────────────────────────────
    // Stationary only when BOTH sensors agree nothing is happening.
    bool quiet = (liaMag < ACCEL_QUIET_THRESHOLD) &&
                 (gyroMag < GYRO_QUIET_THRESHOLD);

    if (quiet)
    {
        if (quietCount < ZUPT_SAMPLES)
            quietCount++;
    }
    else
    {
        quietCount = 0;
    }

    bool stationary = (quietCount >= ZUPT_SAMPLES);
    imuDistance.moving = !stationary;

    // ── Bias learning (stationary only) ─────────────────
    if (stationary)
    {
        if (!imuDistance.calibrated)
        {
            calibCount++;
            imuDistance.calibProgress = calibCount;
            // Running mean over the boot window — converges faster than an EMA.
            imuDistance.accelBias += (ax - imuDistance.accelBias) / (float)calibCount;

            if (calibCount >= CALIB_SAMPLES)
            {
                imuDistance.calibrated = true;
                Serial.print("IMUDIST: Calibrated, bias=");
                Serial.print(imuDistance.accelBias, 4);
                Serial.println(" m/s^2");
            }
        }
        else
        {
            imuDistance.accelBias += (ax - imuDistance.accelBias) * BIAS_ALPHA;
        }
    }

    imuDistance.accel = ax - imuDistance.accelBias;

    float a = imuDistance.accel;
    if (fabsf(a) < ACCEL_DEADBAND)
        a = 0.0f;

    // Stalled or oversized tick — resync rather than integrate a bad dt.
    if (dt <= 0.0f || dt > MAX_DT)
    {
        prevAccel = a;
        return;
    }

    // ── Zero-velocity update ────────────────────────────
    // The whole point of the gate: when the robot is genuinely still, kill
    // the accumulated velocity so it cannot leak into distance.
    if (stationary)
    {
        if (imuDistance.velocity != 0.0f)
            imuDistance.zuptCount++;

        imuDistance.velocity   = 0.0f;
        imuDistance.turnFrozen = false;
        prevAccel = a;
        return;
    }

    // No trustworthy bias yet — gate everything until the boot window closes.
    if (!imuDistance.calibrated)
    {
        prevAccel = a;
        return;
    }

    // ── Rotation gate ───────────────────────────────────
    if (FREEZE_ON_TURN && gyroMag > TURN_RATE_FREEZE)
    {
        imuDistance.velocity   = 0.0f;
        imuDistance.turnFrozen = true;
        prevAccel = a;
        return;
    }
    imuDistance.turnFrozen = false;

    // ── Integrate: accel -> velocity -> distance ────────
    // Trapezoidal on both stages.
    float newVelocity = imuDistance.velocity + 0.5f * (prevAccel + a) * dt;

    newVelocity *= VELOCITY_LEAK;

    if (newVelocity > MAX_PLAUSIBLE_SPEED)
        newVelocity = MAX_PLAUSIBLE_SPEED;
    else if (newVelocity < -MAX_PLAUSIBLE_SPEED)
        newVelocity = -MAX_PLAUSIBLE_SPEED;

    float ds = 0.5f * (imuDistance.velocity + newVelocity) * dt;

    imuDistance.displacement += ds;
    imuDistance.travelled    += fabsf(ds);
    imuDistance.velocity      = newVelocity;

    prevAccel = a;
}
