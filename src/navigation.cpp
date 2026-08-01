#include <Arduino.h>
#include <math.h>
#include <var.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"
#include "distance.h"

// Forward declaration from var.h — motion() drives the motors directly
extern void motion(int _data);

// ─── State ─────────────────────────────────────────────

enum NavigationState
{
    NAV_IDLE,
    NAV_PURSUIT,        // Arcing/driving toward home
    NAV_PARKING,        // Close to home — slow approach
    NAV_CREEP,          // Very close — precision align-then-inch
    NAV_FINAL_ALIGN,    // At home — turning to heading 0°
    NAV_OBSTACLE_AVOID  // Front camera locked — turn/advance/turn-back sidestep
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

// Obstacle-avoidance ("go around it") tuning
static const float OBSTACLE_TURN_ANGLE     = 90.0f; // degrees to sidestep, either side
static const float OBSTACLE_ADVANCE_DIST   = 0.40f; // metres driven past the obstacle
static const float OBSTACLE_TURN_TOLERANCE = 1.0f;  // degrees — turn done within this
static const float OBSTACLE_TURN_CRAWL_ZONE = 5.0f;
static const float OBSTACLE_TURN_SLOW_ZONE  = 15.0f;

// ─── Obstacle-avoidance state ──────────────────────────
// The maneuver drives motion() directly (like PURSUIT/PARKING/CREEP) instead
// of handing off to Drive — this keeps it immune to Drive's pause flag, which
// must never freeze Navigation's own direct-drive phases. It keeps retrying
// (alternating sides) for as long as frontCamLocked stays true; there is no
// give-up — a camera lock is always something to route around, never a
// reason to just sit still.
enum ObstaclePhase
{
    OBSTACLE_TURN_AWAY, // turning ~90 deg off the original heading
    OBSTACLE_ADVANCE,   // driving past the obstacle
    OBSTACLE_TURN_BACK  // turning back to the original heading
};

static NavigationState obstacleReturnState = NAV_PURSUIT; // phase to resume once clear
static float obstacleOriginalHeading = 0.0f;              // heading to restore once clear (fixed across retries)
static float obstacleTargetHeading = 0.0f;                // current turn sub-step's target
static float obstacleStartDistance = 0.0f;                // distance.travelled baseline for the advance step
static ObstaclePhase obstaclePhase = OBSTACLE_TURN_AWAY;
static bool  obstacleTryRightNext = true;                 // alternate sides each attempt
static int   obstacleRetryCount = 0;                      // informational only — no cap

// ─── Helpers ───────────────────────────────────────────

static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// Wrap to [0, 360) — the convention odometry.heading / CMD_TURN_TO_ABS use.
static float wrapTo360(float angle)
{
    while (angle >= 360.0f)
        angle -= 360.0f;
    while (angle < 0.0f)
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

// Pick a side (alternating each call) and (re)start the turn-away sub-phase.
// Does NOT touch obstacleOriginalHeading/obstacleReturnState — those stay
// fixed for the whole retry sequence so the reference direction never drifts.
static void pickSideAndStartTurn()
{
    float turnSign = obstacleTryRightNext ? 1.0f : -1.0f;
    obstacleTryRightNext = !obstacleTryRightNext; // alternate side on next attempt

    obstacleTargetHeading = wrapTo360(obstacleOriginalHeading + turnSign * OBSTACLE_TURN_ANGLE);
    obstaclePhase = OBSTACLE_TURN_AWAY;

    Serial.print("NAVIGATION: Sidestepping ");
    Serial.print(turnSign > 0 ? "right (" : "left (");
    Serial.print(OBSTACLE_TURN_ANGLE, 0);
    Serial.println(" deg) to get around obstacle");
}

// First entry into obstacle avoidance from a driving phase (PURSUIT/PARKING/
// CREEP) — captures the heading to come back to once clear.
static void beginObstacleManeuver(NavigationState returnState)
{
    obstacleReturnState = returnState;
    obstacleOriginalHeading = odometry.heading;
    obstacleRetryCount = 0;
    motion(0);
    pickSideAndStartTurn();
    state = NAV_OBSTACLE_AVOID;
}

// Turn in place toward `target`, tiering speed by remaining error like
// Drive's CMD_LEFT/RIGHT. Returns true once within tolerance.
static bool driveTowardHeading(float target)
{
    float remaining = normalizeAngle(target - odometry.heading);
    float absRemaining = fabs(remaining);

    if (absRemaining <= OBSTACLE_TURN_TOLERANCE)
    {
        motion(0);
        return true;
    }

    bool turnRight = remaining > 0;
    if (absRemaining <= OBSTACLE_TURN_CRAWL_ZONE)
        motion(turnRight ? 11 : 21);
    else if (absRemaining <= OBSTACLE_TURN_SLOW_ZONE)
        motion(turnRight ? 13 : 23);
    else
        motion(turnRight ? 15 : 25);

    return false;
}

// Runs one tick of the current obstacle-avoidance sub-phase. Called every
// loop while state == NAV_OBSTACLE_AVOID.
static void updateObstacleAvoid()
{
    switch (obstaclePhase)
    {
    case OBSTACLE_TURN_AWAY:
        if (driveTowardHeading(obstacleTargetHeading))
        {
            obstacleStartDistance = distance.travelled;
            obstaclePhase = OBSTACLE_ADVANCE;
            Serial.println("NAVIGATION: Turn complete — advancing past obstacle");
        }
        break;

    case OBSTACLE_ADVANCE:
        // The sidestep direction can be blocked too — don't just sit here,
        // immediately try the other side instead of waiting it out.
        if (frontCamLocked)
        {
            motion(0);
            obstacleRetryCount++;
            Serial.println("NAVIGATION: Sidestep path also blocked — trying other side");
            pickSideAndStartTurn();
            break;
        }

        if (distance.travelled - obstacleStartDistance >= OBSTACLE_ADVANCE_DIST)
        {
            motion(0);
            obstaclePhase = OBSTACLE_TURN_BACK;
            Serial.println("NAVIGATION: Cleared obstacle — turning back on course");
        }
        else
        {
            motion(1); // straight forward
        }
        break;

    case OBSTACLE_TURN_BACK:
        if (driveTowardHeading(obstacleOriginalHeading))
        {
            if (frontCamLocked)
            {
                obstacleRetryCount++;
                Serial.println("NAVIGATION: Still blocked on original course — retrying sidestep");
                pickSideAndStartTurn();
            }
            else
            {
                Serial.println("NAVIGATION: Path clear — resuming return-home");
                obstacleRetryCount = 0;
                state = obstacleReturnState;
            }
        }
        break;
    }
}

// ─── Public API ────────────────────────────────────────

void Navigation_Init()
{
    state = NAV_IDLE;
    obstacleRetryCount = 0;
}

bool Navigation_IsRunning()
{
    return (state != NAV_IDLE);
}

void Navigation_Cancel()
{
    if (state == NAV_PURSUIT || state == NAV_PARKING || state == NAV_CREEP || state == NAV_OBSTACLE_AVOID)
    {
        motion(0); // Stop motors since we were driving directly
    }
    else if (state == NAV_FINAL_ALIGN)
    {
        Drive_Cancel(); // this phase hands off to Drive — cancel it too
    }
    obstacleRetryCount = 0;
    state = NAV_IDLE;
}

void Navigation_ReturnHome()
{
    if (state != NAV_IDLE)
        return;

    obstacleRetryCount = 0;
    obstacleTryRightNext = true;

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

    // ── OBSTACLE DETECTED: switch into the sidestep maneuver ─────────
    // Only the driving phases react here — FINAL_ALIGN hands off to Drive
    // and is covered by Drive's own camera-pause instead.
    if ((state == NAV_PURSUIT || state == NAV_PARKING || state == NAV_CREEP) && frontCamLocked)
    {
        beginObstacleManeuver(state);
        return;
    }

    // ── OBSTACLE AVOID: run the turn/advance/turn-back sequence directly.
    // Keeps retrying (alternating sides) for as long as frontCamLocked is
    // true — never just stops and waits for the obstacle to move itself.
    if (state == NAV_OBSTACLE_AVOID)
    {
        updateObstacleAvoid();
        return;
    }

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
