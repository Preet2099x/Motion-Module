#include <Arduino.h>
#include <math.h>
#include <var.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"
#include "distance.h"
#include "camlock.h"

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
    NAV_OBSTACLE_AVOID  // Front camera locked — turn toward the side with more room until it clears
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

// Obstacle-avoidance tuning
static const unsigned long AVOID_TURN_CLEAR_HOLD_MS = 100; // front must read clear this long before the turn ends
static const unsigned long AVOID_SETTLE_MS = 120;   // short ramp-down between the turn and driving on
static const unsigned long AVOID_STICKY_MS = 6000;  // keep turning the same way for this long after an object
static const float AVOID_SWITCH_AFTER_DEG = 220.0f; // turned this far with the front still locked: try the other side
static const float AVOID_SIDE_NEAR_CM  = 55.0f;     // a side closer than this is "in the way": don't curve toward it
static const float AVOID_SIDE_HYST_CM  = 15.0f;     // ...until it reads this much further than NEAR
static const unsigned long AVOID_SIDE_CLEAR_HOLD_MS = 100; // ...for this long
static const float AVOID_FRONT_SLOW_CM = 80.0f;     // something this close ahead: crawl instead of full speed
static const float AVOID_ROOM_CAP_CM   = 200.0f;    // room beyond this adds nothing
static const float AVOID_ROOM_DEFAULT_CM = 100.0f;  // assumed when a side has no fresh reading and is not locked
static const float AVOID_TIE_CM        = 20.0f;     // sides this close in room: turn toward the side home is on

// ─── Obstacle-avoidance state ──────────────────────────
// No separate "go round it" route: every tick the driving phases (PURSUIT/
// PARKING/CREEP) pick the motion that heads straight for home, and navMotion()
// only vetoes what the cameras say is unsafe.
//
//   Front locked   -> NAV_OBSTACLE_AVOID: look at the left and right cameras,
//                     spin in place toward the one with more room, stop as soon
//                     as the front reads clear, then carry on.
//   Home now lies on the side we turned away from. Its camera is watched: while
//                     something is close there the robot drives straight on
//                     (it never curves toward it), and the moment it opens up
//                     the robot curves back toward home, gently.
//   Side locked    -> an in-place turn toward it is flipped the other way (held
//                     still if both sides are locked).
enum ObstaclePhase
{
    AVOID_TURN,
    AVOID_SETTLE
};

static NavigationState avoidReturnState = NAV_PURSUIT; // phase to resume once clear
static ObstaclePhase avoidPhase = AVOID_TURN;
static int avoidDir = 1;      // +1 turn right, -1 turn left
static int avoidLastDir = 0;  // way turned last time (0 = none yet)
static unsigned long avoidLastMs = 0;
static unsigned long avoidClearSince = 0; // 0 = front not currently clear
static unsigned long avoidSettleStart = 0;
static float avoidTurned = 0.0f;          // degrees rotated in the current turn
static float avoidLastHeading = 0.0f;
static unsigned long avoidWaitMs = 0;     // last "boxed in" message
static bool avoidRecovering = false;      // just got round something: curve back toward home gently

// "Something close beside us" per side, latched. [0]=left [1]=right
static bool sideNearFlag[2] = {false, false};
static unsigned long sideClearSince[2] = {0, 0};

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

static bool sideLocked(int dir)
{
    return dir > 0 ? rightCamLocked : leftCamLocked;
}

// Refresh the per-side "something close" flags from the side cameras.
static void updateSideNear()
{
    const unsigned long now = millis();

    for (int i = 0; i < 2; i++)
    {
        int dir = (i == 1) ? 1 : -1;
        float d = CamDist_Free(dir > 0 ? CAM_RIGHT : CAM_LEFT);
        bool close = sideLocked(dir);

        if (d >= 0.0f)
            close = close || d < AVOID_SIDE_NEAR_CM + (sideNearFlag[i] ? AVOID_SIDE_HYST_CM : 0.0f);

        if (close)
        {
            sideNearFlag[i] = true;
            sideClearSince[i] = 0;
        }
        else if (sideNearFlag[i])
        {
            if (sideClearSince[i] == 0)
                sideClearSince[i] = now;
            if (now - sideClearSince[i] >= AVOID_SIDE_CLEAR_HOLD_MS)
                sideNearFlag[i] = false;
        }
    }
}

static bool sideNear(int dir)
{
    return sideNearFlag[dir > 0 ? 1 : 0];
}

// Free room (cm) on a side, from its camera.
static float sideRoom(int dir)
{
    float d = CamDist_Free(dir > 0 ? CAM_RIGHT : CAM_LEFT);

    if (d < 0.0f) // no fresh reading: judge by the lock flag alone
        d = sideLocked(dir) ? 0.0f : AVOID_ROOM_DEFAULT_CM;
    return (d > AVOID_ROOM_CAP_CM) ? AVOID_ROOM_CAP_CM : d;
}

// Which way to turn round an object: +1 right, -1 left. The side with more
// room, unless they are about the same - then the side home is on. Within a few
// seconds of the last object the same way is kept (so it does not flip-flop),
// as long as that side has not got something close.
static int chooseAvoidSide()
{
    if (avoidLastDir != 0 && millis() - avoidLastMs < AVOID_STICKY_MS && !sideNear(avoidLastDir))
        return avoidLastDir;

    float right = sideRoom(1);
    float left = sideRoom(-1);

    if (fabs(right - left) < AVOID_TIE_CM)
    {
        float homeError = normalizeAngle(calculateHeadingToHome() - odometry.heading);
        return (homeError >= 0.0f) ? 1 : -1;
    }
    return (right > left) ? 1 : -1;
}

static void startTurn(int dir)
{
    avoidDir = dir;
    avoidLastDir = dir;
    avoidLastMs = millis();
    avoidPhase = AVOID_TURN;
    avoidClearSince = 0;
    avoidTurned = 0.0f;
    avoidLastHeading = odometry.heading;

    float left = CamDist_Free(CAM_LEFT);
    float right = CamDist_Free(CAM_RIGHT);

    Serial.print("NAVIGATION: Object ahead - turning ");
    Serial.print(dir > 0 ? "right" : "left");
    Serial.print(" (room L=");
    if (left >= 0.0f)
        Serial.print(left, 0);
    else
        Serial.print("?");
    Serial.print(" R=");
    if (right >= 0.0f)
        Serial.print(right, 0);
    else
        Serial.print("?");
    Serial.println(" cm)");
}

// Front locked while a driving phase wanted to go forward: stop and start
// turning toward the side with more room. `returnState` is the phase to resume.
static void beginObstacleManeuver(NavigationState returnState)
{
    avoidReturnState = returnState;
    motion_EmergencyStop(); // instant stop; motion(0) only ramps down per call
    state = NAV_OBSTACLE_AVOID;
    startTurn(chooseAvoidSide());
}

// Runs one tick of the turn that follows a front lock. Called every loop while
// state == NAV_OBSTACLE_AVOID.
static void updateObstacleAvoid()
{
    unsigned long now = millis();
    avoidLastMs = now; // still dealing with this object

    switch (avoidPhase)
    {
    case AVOID_TURN:
    {
        avoidTurned += fabs(normalizeAngle(odometry.heading - avoidLastHeading));
        avoidLastHeading = odometry.heading;

        // Never rotate into something beside us: if the side we are turning
        // toward is locked, switch sides; if both are locked, hold still.
        if (sideLocked(avoidDir))
        {
            motion_EmergencyStop();
            if (!sideLocked(-avoidDir))
            {
                Serial.println("NAVIGATION: Object beside the turn - switching side");
                startTurn(-avoidDir);
            }
            else if (now - avoidWaitMs >= 2000)
            {
                avoidWaitMs = now;
                Serial.println("NAVIGATION: Boxed in on both sides - waiting");
            }
            break;
        }

        // Another object turned up on the side we are turning toward while the
        // other side is open: it is less turning to go the other way.
        if (sideNear(avoidDir) && !sideNear(-avoidDir))
        {
            motion_EmergencyStop();
            Serial.println("NAVIGATION: Object on the turn side - going the other way");
            startTurn(-avoidDir);
            break;
        }

        if (frontCamLocked)
            avoidClearSince = 0;
        else if (avoidClearSince == 0)
            avoidClearSince = now;

        // Front has read clear for a moment: stop turning right here. The
        // driving phase takes over and curves back toward home as soon as the
        // camera on the home side allows it.
        if (avoidClearSince != 0 && now - avoidClearSince >= AVOID_TURN_CLEAR_HOLD_MS)
        {
            avoidSettleStart = now;
            avoidPhase = AVOID_SETTLE;
            avoidRecovering = true;
            Serial.println("NAVIGATION: Front clear - heading for home");
            break;
        }

        // Turned a long way and still blocked: this side is no good.
        if (avoidTurned >= AVOID_SWITCH_AFTER_DEG)
        {
            Serial.println("NAVIGATION: Still blocked after a wide turn - trying the other side");
            startTurn(-avoidDir);
            break;
        }

        motion(avoidDir > 0 ? 13 : 23); // slow in-place turn toward the chosen side
        break;
    }

    case AVOID_SETTLE:
        motion(0); // ramps the motors down instead of slamming them to a stop
        if (now - avoidSettleStart >= AVOID_SETTLE_MS)
            state = avoidReturnState; // PURSUIT/PARKING/CREEP heads for home from here
        break;
    }
}

// Runs the motion `cmd` that heads for home, after checking it against the
// cameras. `headingError` is the signed error to home (+ = home is to the
// right). See the state comment above for the rules.
static void navMotion(int cmd, float headingError)
{
    const int want = (headingError > 0.0f) ? 1 : -1; // side home lies on
    const bool straight = (cmd == 1 || cmd == 3);
    const bool arc = (cmd >= 111 && cmd <= 130);
    const bool spin = (cmd >= 11 && cmd <= 30);

    updateSideNear();

    // Anything that drives forward needs the front clear.
    if ((straight || arc) && frontCamLocked)
    {
        beginObstacleManeuver(state);
        return;
    }

    // Back on the line to home: done recovering.
    if (fabs(headingError) < STRAIGHT_ZONE)
        avoidRecovering = false;

    // Home is on a side with something close: do not turn toward it, keep
    // going straight and keep watching that camera. Once it opens up the turn
    // toward home comes back by itself.
    static bool holdNoted = false;
    if (state == NAV_PURSUIT && (arc || spin) && sideNear(want))
    {
        if (!holdNoted)
        {
            holdNoted = true;
            Serial.print("NAVIGATION: Object close on the ");
            Serial.print(want > 0 ? "right" : "left");
            Serial.println(" - holding straight");
        }
        cmd = 1;
    }
    else if (!sideNear(want))
    {
        holdNoted = false;
    }

    // In-place turn toward a locked side would swing the robot into it: turn
    // the other way, or hold still if both sides are locked.
    if (cmd >= 11 && cmd <= 30 && sideLocked(want))
    {
        if (sideLocked(-want))
        {
            motion(0);
            return;
        }
        cmd += (want > 0) ? 10 : -10; // 11-20 (right) <-> 21-30 (left)
    }

    // Just got round something: curve back toward home gently, not sharply.
    if (avoidRecovering && cmd >= 115 && cmd <= 130)
        cmd = (want > 0) ? 111 : 121;

    // Something close ahead (not yet locked): crawl instead of full speed, so
    // the robot can stop short of it.
    if (cmd == 1)
    {
        float front = CamDist_Free(CAM_FRONT);
        if (front >= 0.0f && front < AVOID_FRONT_SLOW_CM)
            cmd = 3;
    }

    motion(cmd);
}

// ─── Public API ────────────────────────────────────────

void Navigation_Init()
{
    state = NAV_IDLE;
    avoidLastDir = 0;
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
    state = NAV_IDLE;
}

void Navigation_ReturnHome()
{
    if (state != NAV_IDLE)
        return;

    avoidLastDir = 0;
    avoidRecovering = false;
    sideNearFlag[0] = sideNearFlag[1] = false;
    sideClearSince[0] = sideClearSince[1] = 0;

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

    // ── OBSTACLE AVOID: the front locked, so turn toward the side with more
    // room until it reads clear, then hand straight back to the home-seeking
    // phase. The driving phases check the cameras themselves (navMotion);
    // FINAL_ALIGN hands off to Drive and is covered by Drive's own camera-pause.
    if (state == NAV_OBSTACLE_AVOID)
    {
        updateSideNear();
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
            navMotion(1, headingError); // straight forward (with heading-PID in motion.cpp)
        }
        else if (absError < ARC_GENTLE_ZONE)
        {
            // Gentle arc — low differential, steady forward speed
            navMotion(headingError > 0 ? 111 : 121, headingError);
        }
        else if (absError < ARC_MEDIUM_ZONE)
        {
            // Medium arc
            navMotion(headingError > 0 ? 115 : 125, headingError);
        }
        else if (absError < ARC_TIGHT_ZONE)
        {
            // Tight arc — large differential
            navMotion(headingError > 0 ? 117 : 127, headingError);
        }
        else
        {
            // Way off course — fall back to in-place turn (arcs can't correct >90°)
            navMotion(headingError > 0 ? 15 : 25, headingError);
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
            navMotion(3, headingError); // PWM 85 straight
        }
        else if (absError < PARK_ARC_ZONE)
        {
            // Moderate correction — gentle arc
            navMotion(headingError > 0 ? 111 : 121, headingError);
        }
        else
        {
            // Large heading error while parking — in-place turn with enough power to not stall
            if (absError > 15.0f) {
                navMotion(headingError > 0 ? 15 : 25, headingError); // PWM 120
            } else if (absError > 5.0f) {
                navMotion(headingError > 0 ? 13 : 23, headingError); // PWM 80
            } else {
                navMotion(headingError > 0 ? 11 : 21, headingError); // PWM 60
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
                navMotion(headingError > 0 ? 15 : 25, headingError); // PWM 120
            } else if (absError > 5.0f) {
                navMotion(headingError > 0 ? 13 : 23, headingError); // PWM 80
            } else {
                navMotion(headingError > 0 ? 11 : 21, headingError); // PWM 60
            }
        }
        else
        {
            // Heading aligned — inch forward perfectly straight
            navMotion(3, headingError); // PWM 85 straight
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
