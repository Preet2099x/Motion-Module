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
    NAV_OBSTACLE_AVOID  // Front camera locked — go round the object on the freer side
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
static const unsigned long AVOID_CLEAR_HOLD_MS = 300;  // front/side must read clear this long before moving on
static const unsigned long AVOID_SETTLE_MS     = 250;  // stand still between the turn and the pass
static const unsigned long AVOID_STICKY_MS     = 6000; // keep going round the same side / same object for this long
static const float AVOID_BACKUP_DIST     = 0.20f; // m reversed before turning, to give the turn room
static const float AVOID_BACKUP_MIN_CM   = 50.0f; // rear camera must show at least this free to reverse
static const unsigned long AVOID_BACKUP_MAX_MS = 2500;
static const float AVOID_EXTRA_TURN_DEG  = 30.0f; // keep turning this far after the front reads clear
static const float AVOID_SWITCH_AFTER_DEG = 220.0f; // turned this far with the front still locked: try the other side
static const float AVOID_PASS_MIN_DIST   = 0.50f; // m driven past the object before rejoining the route home
static const float AVOID_PASS_MAX_DIST   = 2.00f; // m - stop waiting for the side camera to clear
static const float AVOID_SIDE_KEEP_CM    = 60.0f; // while passing, steer away from anything closer than this on a side
static const float AVOID_SIDE_CLEAR_CM   = 100.0f; // the object-side camera counts as clear above this

// Side choice: each side gets a score in "cm of free room" and the higher one wins.
//   score = free room on that side (capped)
//         + a bias toward the side home lies on (less angle to rejoin the route)
//         - a cost per degree already turned AWAY from that side (undoing a turn
//           is wasted angle)
static const float AVOID_FREE_CAP_CM     = 200.0f; // room beyond this adds nothing
static const float AVOID_FREE_DEFAULT_CM = 100.0f; // assumed when a side has no fresh reading and is not locked
static const float AVOID_HOME_BIAS_CM    = 40.0f;  // score bonus for the side home lies on (full at 90 deg off)
static const float AVOID_TURN_COST_CM_PER_DEG = 1.0f;
static const float AVOID_SWITCH_MARGIN_CM = 25.0f; // the other side must beat the current one by this much to switch

// ─── Obstacle-avoidance state ──────────────────────────
// Reactive, re-evaluated every tick. It drives motion() directly (like
// PURSUIT/PARKING/CREEP), so Drive's pause flag can never freeze it.
//   BACKUP  first contact with an object: reverse a little (if the rear camera
//           shows room) so the turn has space.
//   TURN    pick the side with more free space and rotate in place toward it
//           until the front reads clear, plus a margin. Never rotates into a
//           side camera that is locked.
//   SETTLE  brief stop between the turn and the pass.
//   PASS    drive forward past the object, curving away from anything close on
//           either side. Done when the camera on the object's side reads clear
//           and a minimum distance has been driven; the interrupted phase
//           (PURSUIT/PARKING/CREEP) then resumes and steers back toward home.
// If the front locks again at any point (the same object or a new one) it
// goes back to TURN and picks a side again, so it keeps routing round
// whatever is in the way. There is no give-up: it never just sits still
// unless both sides are blocked.
enum ObstaclePhase
{
    AVOID_BACKUP,
    AVOID_TURN,
    AVOID_SETTLE,
    AVOID_PASS
};

static NavigationState avoidReturnState = NAV_PURSUIT; // phase to resume once clear
static ObstaclePhase avoidPhase = AVOID_TURN;
static int avoidDir = 1;      // +1 turn right, -1 turn left
static int avoidLastDir = 0;  // side used last time (0 = none yet)
static unsigned long avoidLastMs = 0;
static unsigned long avoidClearSince = 0; // 0 = not currently clear
static unsigned long avoidSettleStart = 0;
static float avoidTurned = 0.0f;       // degrees rotated in the current TURN (absolute)
static float avoidRot = 0.0f;          // signed degrees since the encounter began: + = turned right
static float avoidLastHeading = 0.0f;
static float avoidPassStartDist = 0.0f;
static float avoidBackStartDist = 0.0f;
static unsigned long avoidBackStartMs = 0;
static float avoidClearMark = 0.0f;        // avoidTurned when the front first read clear
static unsigned long avoidWaitMs = 0;      // last "boxed in" message

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

// Score for going round on `dir` (+1 right, -1 left): free room on that side,
// plus a small bias toward the side home lies on, minus the angle we would
// have to undo to use it. Higher is better.
static float sideScore(int dir)
{
    float free = CamDist_Free(dir > 0 ? CAM_RIGHT : CAM_LEFT);

    if (free < 0.0f) // no fresh reading: judge by the lock flag alone
        free = sideLocked(dir) ? 0.0f : AVOID_FREE_DEFAULT_CM;
    if (free > AVOID_FREE_CAP_CM)
        free = AVOID_FREE_CAP_CM;

    // heading error to home: + means home lies to the right of where we face
    float homeError = normalizeAngle(calculateHeadingToHome() - odometry.heading);
    float homeAlign = constrain(homeError * dir / 90.0f, -1.0f, 1.0f);

    // already turned toward the other side by avoidRot degrees: using this
    // side means turning back through that angle first
    float undo = -dir * avoidRot;
    if (undo < 0.0f)
        undo = 0.0f;

    return free + AVOID_HOME_BIAS_CM * homeAlign - AVOID_TURN_COST_CM_PER_DEG * undo;
}

// +1 = right, -1 = left: the side with the better score. Within a few seconds
// of the last avoidance the side used then is kept unless the other is clearly
// better (so the robot hugs one object instead of flip-flopping).
static int chooseAvoidSide()
{
    float right = sideScore(1);
    float left = sideScore(-1);

    if (avoidLastDir != 0 && millis() - avoidLastMs < AVOID_STICKY_MS && !sideLocked(avoidLastDir))
    {
        float cur = (avoidLastDir > 0) ? right : left;
        float oth = (avoidLastDir > 0) ? left : right;
        return (oth > cur + AVOID_SWITCH_MARGIN_CM) ? -avoidLastDir : avoidLastDir;
    }

    return (right >= left) ? 1 : -1;
}

static void startTurn(int dir, bool newEncounter)
{
    if (newEncounter)
        avoidRot = 0.0f; // flips within one encounter keep counting from the same start
    avoidDir = dir;
    avoidLastDir = dir;
    avoidLastMs = millis();
    avoidPhase = AVOID_TURN;
    avoidClearSince = 0;
    avoidTurned = 0.0f;
    avoidLastHeading = odometry.heading;

    float left = CamDist_Free(CAM_LEFT);
    float right = CamDist_Free(CAM_RIGHT);

    Serial.print("NAVIGATION: Object ahead - going round on the ");
    Serial.print(dir > 0 ? "right" : "left");
    Serial.print(" (free L=");
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

// A new encounter (or a re-route after passing): forget the angle turned so
// far, so the side choice is not skewed by the previous one, then pick a side.
static void startFreshTurn()
{
    avoidRot = 0.0f;
    startTurn(chooseAvoidSide(), true);
}

// First entry into obstacle avoidance from a driving phase (PURSUIT/PARKING/
// CREEP): remember which phase to resume once clear.
static void beginObstacleManeuver(NavigationState returnState)
{
    avoidReturnState = returnState;
    motion_EmergencyStop(); // instant stop; motion(0) only ramps down per call
    state = NAV_OBSTACLE_AVOID;

    // First contact with this object: back off a little so the turn has room,
    // but only if the rear camera says it is free. Not repeated while we are
    // still dealing with the same object.
    bool sameEncounter = (avoidLastDir != 0) && (millis() - avoidLastMs < AVOID_STICKY_MS);
    float back = CamDist_Free(CAM_BACK);
    bool backFree = !backCamLocked && (back < 0.0f || back >= AVOID_BACKUP_MIN_CM);

    if (!sameEncounter && backFree)
    {
        avoidPhase = AVOID_BACKUP;
        avoidBackStartDist = distance.travelled;
        avoidBackStartMs = millis();
        avoidLastMs = millis();
        Serial.println("NAVIGATION: Too close - backing off before turning");
        return;
    }

    startFreshTurn();
}

// The object we are going round sits on the side opposite the way we turned.
static bool objectSideClear()
{
    int objectDir = -avoidDir;
    float d = CamDist_Free(objectDir > 0 ? CAM_RIGHT : CAM_LEFT);

    if (d >= 0.0f)
        return d >= AVOID_SIDE_CLEAR_CM;

    return !sideLocked(objectDir);
}

// Runs one tick of the current obstacle-avoidance sub-phase. Called every
// loop while state == NAV_OBSTACLE_AVOID.
static void updateObstacleAvoid()
{
    unsigned long now = millis();
    avoidLastMs = now; // still dealing with this object

    switch (avoidPhase)
    {
    case AVOID_BACKUP:
    {
        float backed = distance.travelled - avoidBackStartDist;
        if (backCamLocked || backed >= AVOID_BACKUP_DIST || now - avoidBackStartMs >= AVOID_BACKUP_MAX_MS)
        {
            motion_EmergencyStop();
            startFreshTurn();
            break;
        }

        motion(4); // slow straight backward
        break;
    }

    case AVOID_TURN:
    {
        float step = normalizeAngle(odometry.heading - avoidLastHeading);
        avoidTurned += fabs(step);
        avoidRot += step; // signed: + = turned right
        avoidLastHeading = odometry.heading;

        // Never rotate into something beside us: if the side we are turning
        // toward is locked, switch sides; if both are locked, hold still.
        if (sideLocked(avoidDir))
        {
            motion_EmergencyStop();
            if (!sideLocked(-avoidDir))
            {
                Serial.println("NAVIGATION: Object beside the turn - switching side");
                startTurn(-avoidDir, false);
            }
            else if (now - avoidWaitMs >= 2000)
            {
                avoidWaitMs = now;
                Serial.println("NAVIGATION: Boxed in on both sides - waiting");
            }
            break;
        }

        // Keep working out which way makes sense: if more turning this way
        // would run into another object (or needs far more angle) and the other
        // side is clearly better even after undoing the turn made so far, switch.
        if (sideScore(-avoidDir) > sideScore(avoidDir) + AVOID_SWITCH_MARGIN_CM)
        {
            motion_EmergencyStop();
            Serial.println("NAVIGATION: Other side is the shorter way round - switching");
            startTurn(-avoidDir, false);
            break;
        }

        if (frontCamLocked)
        {
            avoidClearSince = 0;
        }
        else if (avoidClearSince == 0)
        {
            avoidClearSince = now;
            avoidClearMark = avoidTurned;
        }

        // Front has been clear for a moment AND we have turned a margin past
        // the point where it cleared: stop turning and get ready to pass.
        if (avoidClearSince != 0 &&
            now - avoidClearSince >= AVOID_CLEAR_HOLD_MS &&
            avoidTurned - avoidClearMark >= AVOID_EXTRA_TURN_DEG)
        {
            motion_EmergencyStop();
            avoidSettleStart = now;
            avoidPhase = AVOID_SETTLE;
            Serial.println("NAVIGATION: Front clear - settling");
            break;
        }

        // Turned a long way and still blocked: this side is no good.
        if (avoidTurned >= AVOID_SWITCH_AFTER_DEG)
        {
            Serial.println("NAVIGATION: Still blocked after a wide turn - trying the other side");
            startTurn(-avoidDir, false);
            break;
        }

        motion(avoidDir > 0 ? 13 : 23); // slow in-place turn toward the chosen side
        break;
    }

    case AVOID_SETTLE:
        motion_EmergencyStop();
        if (now - avoidSettleStart >= AVOID_SETTLE_MS)
        {
            avoidPassStartDist = distance.travelled;
            avoidClearSince = 0;
            avoidPhase = AVOID_PASS;
            Serial.println("NAVIGATION: Passing the object");
        }
        break;

    case AVOID_PASS:
    {
        // Something ahead again (the same object or a new one): pick a side again.
        if (frontCamLocked)
        {
            motion_EmergencyStop();
            Serial.println("NAVIGATION: Blocked again while passing - re-routing");
            startFreshTurn();
            break;
        }

        float driven = distance.travelled - avoidPassStartDist;
        bool clearNow = (driven >= AVOID_PASS_MIN_DIST) && objectSideClear();

        if (!clearNow)
            avoidClearSince = 0;
        else if (avoidClearSince == 0)
            avoidClearSince = now;

        if ((avoidClearSince != 0 && now - avoidClearSince >= AVOID_CLEAR_HOLD_MS) ||
            driven >= AVOID_PASS_MAX_DIST)
        {
            Serial.println("NAVIGATION: Clear of the object - resuming route home");
            state = avoidReturnState; // PURSUIT/PARKING/CREEP steers back toward home
            break;
        }

        // Keep watching both sides while passing: steer away from whatever is
        // close, so the robot's flanks never brush the object (or a second one).
        int objectDir = -avoidDir;
        float dObj = CamDist_Free(objectDir > 0 ? CAM_RIGHT : CAM_LEFT);
        float dTurn = CamDist_Free(avoidDir > 0 ? CAM_RIGHT : CAM_LEFT);
        bool objNear = (dObj >= 0.0f) ? (dObj < AVOID_SIDE_KEEP_CM) : sideLocked(objectDir);
        bool turnNear = (dTurn >= 0.0f) ? (dTurn < AVOID_SIDE_KEEP_CM) : sideLocked(avoidDir);

        if (objNear && !turnNear)
            motion(avoidDir > 0 ? 111 : 121); // gentle curve away from the object
        else if (turnNear && !objNear)
            motion(avoidDir > 0 ? 121 : 111); // gentle curve away from what is beside the turn side
        else
            motion(3); // slow straight forward
        break;
    }
    }
}

// Runs a motion code for the home-seeking phases unless a camera lock covers
// it (front: 1/111-130, back: 2/211-230, left: 21-30/121-130/221-230, right:
// 11-20/111-120/211-220). A blocked code starts going round instead of driving
// into whatever the side camera sees.
static void navMotion(int cmd)
{
    if (CamLock_IsBlocked(cmd))
    {
        beginObstacleManeuver(state);
        return;
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
            navMotion(1); // straight forward (with heading-PID in motion.cpp)
        }
        else if (absError < ARC_GENTLE_ZONE)
        {
            // Gentle arc — low differential, steady forward speed
            navMotion(headingError > 0 ? 111 : 121);
        }
        else if (absError < ARC_MEDIUM_ZONE)
        {
            // Medium arc
            navMotion(headingError > 0 ? 115 : 125);
        }
        else if (absError < ARC_TIGHT_ZONE)
        {
            // Tight arc — large differential
            navMotion(headingError > 0 ? 117 : 127);
        }
        else
        {
            // Way off course — fall back to in-place turn (arcs can't correct >90°)
            navMotion(headingError > 0 ? 15 : 25);
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
            navMotion(3); // PWM 85 straight
        }
        else if (absError < PARK_ARC_ZONE)
        {
            // Moderate correction — gentle arc
            navMotion(headingError > 0 ? 111 : 121);
        }
        else
        {
            // Large heading error while parking — in-place turn with enough power to not stall
            if (absError > 15.0f) {
                navMotion(headingError > 0 ? 15 : 25); // PWM 120
            } else if (absError > 5.0f) {
                navMotion(headingError > 0 ? 13 : 23); // PWM 80
            } else {
                navMotion(headingError > 0 ? 11 : 21); // PWM 60
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
                navMotion(headingError > 0 ? 15 : 25); // PWM 120
            } else if (absError > 5.0f) {
                navMotion(headingError > 0 ? 13 : 23); // PWM 80
            } else {
                navMotion(headingError > 0 ? 11 : 21); // PWM 60
            }
        }
        else
        {
            // Heading aligned — inch forward perfectly straight
            navMotion(3); // PWM 85 straight
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
