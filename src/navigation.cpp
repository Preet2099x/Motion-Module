#include <Arduino.h>
#include <math.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"

// Forward declaration from var.h — motion() drives the motors directly
extern void motion(int _data);

// ─── State ─────────────────────────────────────────────

enum NavigationState
{
    NAV_IDLE,
    NAV_PURSUIT,      // Arcing/driving toward home
    NAV_PARKING,      // Close to home — slow approach
    NAV_CREEP,        // Very close — precision align-then-inch
    NAV_FINAL_ALIGN   // At home — turning to heading 0°
};

static NavigationState state = NAV_IDLE;

// ─── Tuning ────────────────────────────────────────────

static const float HOME_TOLERANCE   = 0.005f; // m — exact home coordinates (5 mm)
static const float PARK_RADIUS      = 0.30f;  // m — switch to parking phase
static const float CREEP_RADIUS     = 0.10f;  // m — switch to precision creep

// Pursuit heading-error thresholds (degrees)
static const float STRAIGHT_ZONE    = 5.0f;   // drive straight
static const float ARC_GENTLE_ZONE  = 15.0f;  // gentle arc (111/121)
static const float ARC_MEDIUM_ZONE  = 35.0f;  // medium arc (115/125)
static const float ARC_TIGHT_ZONE   = 90.0f;  // tight arc  (117/127)
                                                // beyond this: in-place turn

// Parking heading-error thresholds (degrees)
static const float PARK_STRAIGHT_ZONE = 5.0f;
static const float PARK_ARC_ZONE      = 20.0f;

// Creep heading-error threshold — must align precisely before inching forward
static const float CREEP_ALIGN_ZONE   = 1.0f;  // degrees (strict alignment)

// ─── Helpers ───────────────────────────────────────────

static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// Calculate the heading the robot must face so that its physical-forward
// direction points toward home (0, 0).
// Physical forward = heading + 180°, so the robot must set heading =
// atan2(y, x)  (the direction FROM home TO robot).
static float calculateHeadingToHome()
{
    float heading = atan2f(odometry.y, odometry.x) * 180.0f / PI;

    if (heading < 0)
        heading += 360.0f;

    return heading; // in odometry space (0–360 deg)
}

// ─── Public API ────────────────────────────────────────

void Navigation_Init()
{
    state = NAV_IDLE;
}

bool Navigation_IsRunning()
{
    return (state != NAV_IDLE);
}

void Navigation_Cancel()
{
    if (state == NAV_PURSUIT || state == NAV_PARKING || state == NAV_CREEP)
    {
        motion(0); // Stop motors since we were driving directly
    }
    state = NAV_IDLE;
}

void Navigation_ReturnHome()
{
    if (state != NAV_IDLE)
        return;

    float dx = -odometry.x;
    float dy = -odometry.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < HOME_TOLERANCE)
    {
        Serial.println("NAVIGATION: Already at home");
        return;
    }

    // Cancel any running drive task — we'll be calling motion() directly
    if (Drive_IsRunning())
        Drive_Cancel();

    if (dist < PARK_RADIUS)
    {
        Serial.println("NAVIGATION: Returning home (parking — already close)");
        state = NAV_PARKING;
    }
    else
    {
        Serial.println("NAVIGATION: Returning home (pursuit mode)");
        state = NAV_PURSUIT;
    }
}

// ─── Update (called every loop tick) ───────────────────

void Navigation_Update()
{
    if (state == NAV_IDLE)
        return;

    // ── PURSUIT: arc toward home at full speed ──────────
    if (state == NAV_PURSUIT)
    {
        float dx = -odometry.x;
        float dy = -odometry.y;
        float dist = sqrtf(dx * dx + dy * dy);

        // Transition to parking when close
        if (dist < PARK_RADIUS)
        {
            Serial.print("NAVIGATION: Entering parking phase (dist=");
            Serial.print(dist, 2);
            Serial.println(" m)");
            state = NAV_PARKING;
            return; // will handle parking on next tick
        }

        // Heading error: target heading – current heading
        float targetHeading = calculateHeadingToHome();
        float headingError  = normalizeAngle(targetHeading - odometry.heading);
        float absError      = fabs(headingError);

        // Select motion code based on heading error magnitude & sign
        if (absError < STRAIGHT_ZONE)
        {
            motion(1); // straight forward (with heading-PID in motion.cpp)
        }
        else if (absError < ARC_GENTLE_ZONE)
        {
            // Gentle arc — low differential, steady forward speed
            motion(headingError > 0 ? 111 : 121);
        }
        else if (absError < ARC_MEDIUM_ZONE)
        {
            // Medium arc
            motion(headingError > 0 ? 115 : 125);
        }
        else if (absError < ARC_TIGHT_ZONE)
        {
            // Tight arc — large differential
            motion(headingError > 0 ? 117 : 127);
        }
        else
        {
            // Way off course — fall back to in-place turn (arcs can't correct >90°)
            motion(headingError > 0 ? 15 : 25);
        }

        return;
    }

    // ── PARKING: slow approach with gentle codes only ───
    if (state == NAV_PARKING)
    {
        float dx = -odometry.x;
        float dy = -odometry.y;
        float dist = sqrtf(dx * dx + dy * dy);

        // Transition to creep for final precision
        if (dist < CREEP_RADIUS)
        {
            motion(0); // stop before creep — kill momentum
            Serial.print("NAVIGATION: Entering creep phase (dist=");
            Serial.print(dist, 3);
            Serial.println(" m)");
            state = NAV_CREEP;
            return;
        }

        // Heading error
        float targetHeading = calculateHeadingToHome();
        float headingError  = normalizeAngle(targetHeading - odometry.heading);
        float absError      = fabs(headingError);

        // Parking uses ONLY gentle codes (≤x3 sub-range → lowest PWM)
        if (absError < PARK_STRAIGHT_ZONE)
        {
            // Nearly on target — use crawl straight forward
            motion(3); // PWM 85 straight
        }
        else if (absError < PARK_ARC_ZONE)
        {
            // Moderate correction — gentle arc
            motion(headingError > 0 ? 111 : 121);
        }
        else
        {
            // Large heading error while parking — in-place turn with enough power to not stall
            if (absError > 15.0f) {
                motion(headingError > 0 ? 15 : 25); // PWM 120
            } else if (absError > 5.0f) {
                motion(headingError > 0 ? 13 : 23); // PWM 80
            } else {
                motion(headingError > 0 ? 11 : 21); // PWM 60
            }
        }

        return;
    }

    // ── CREEP: precision align-then-inch for the last few cm ──
    if (state == NAV_CREEP)
    {
        float dx = -odometry.x;
        float dy = -odometry.y;
        float dist = sqrtf(dx * dx + dy * dy);

        // Reached home — stop and do final heading alignment
        if (dist < HOME_TOLERANCE)
        {
            motion(0); // stop motors

            DriveCommand cmds[1];
            cmds[0].type  = CMD_TURN_TO_ABS;
            cmds[0].value = 0.0f; // align to zero heading

            if (Drive_SubmitTask(cmds, 1))
            {
                Serial.println("NAVIGATION: Home reached, aligning to 0 deg");
                state = NAV_FINAL_ALIGN;
            }
            else
            {
                Serial.println("NAVIGATION: Home reached (align submit failed)");
                state = NAV_IDLE;
            }
            return;
        }

        // Heading error
        float targetHeading = calculateHeadingToHome();
        float headingError  = normalizeAngle(targetHeading - odometry.heading);
        float absError      = fabs(headingError);

        // CREEP STRATEGY:
        // 1. If heading is off, STOP and crawl-turn to align first
        // 2. Only inch forward once heading is within CREEP_ALIGN_ZONE
        // This prevents overshoot from driving at an angle
        if (absError >= CREEP_ALIGN_ZONE)
        {
            // Stop forward motion, align heading with scaled in-place turn to prevent stall
            if (absError > 15.0f) {
                motion(headingError > 0 ? 15 : 25); // PWM 120
            } else if (absError > 5.0f) {
                motion(headingError > 0 ? 13 : 23); // PWM 80
            } else {
                motion(headingError > 0 ? 11 : 21); // PWM 60
            }
        }
        else
        {
            // Heading aligned — inch forward perfectly straight
            motion(3); // PWM 85 straight
        }

        return;
    }

    // ── FINAL ALIGN: wait for drive to finish turn ──────
    if (state == NAV_FINAL_ALIGN)
    {
        if (!Drive_IsRunning())
        {
            Serial.println("NAVIGATION: Return home complete");
            state = NAV_IDLE;
        }
    }
}
