#include "follow.h"
#include "drive.h"
#include "navigation.h"

#include <Arduino.h>
#include <var.h>

// ─── How this works ─────────────────────────────────────
//
// Four anchors mounted on the robot, each reports its range (metres) to one
// beacon carried by the operator:
//
//     A  FRONT   — mounted pointing forward
//     B  RIGHT   — mounted on the right side
//     C  LEFT    — mounted on the left side
//     D  BACK    — mounted pointing backward
//
// Two simple states, no bearing math, no fixed turn angles:
//
//   TURN:  A is not yet the closest anchor. Rotate in place toward
//          whichever of B/C currently reads smaller (that's the side the
//          beacon leans toward). Keep rotating until A becomes the
//          closest of all four — i.e. the beacon is roughly ahead.
//
//   DRIVE: A is the closest anchor. Drive forward, and every ~300ms check
//          which of B/C has gotten smaller since the last check — curve
//          gently toward that side (motion 115 = curve right toward B,
//          125 = curve left toward C). If neither moved, drive straight.
//          If A stops being the closest anchor, the beacon has drifted
//          off-ahead — stop curving and go back to TURN.
//
// Turning uses only the slowest follow-me-only tier (31/41, PWM 90) and
// driving uses the slow shared gear (motion 3) rather than full speed —
// intentionally low PWM for now while this logic is being tried out, so a
// wrong turn is a slow bump, not a collision.
//
// Stops when the front anchor (A) reads below the arrival threshold
// (default 2m).

// ─── Tuning ────────────────────────────────────────────

static const float ANCHOR_EMA_ALPHA = 0.3f; // 0..1, higher = less smoothing

// Arrival threshold: if ANY raw anchor distance drops below this while
// driving, stop following — the robot has arrived.
static float arrivalRange = 2.00f; // m

// Hard proximity stop on the RAW per-anchor ranges — if any single anchor
// reads under this, stop immediately.
static float proximityStopRange = 1.50f;
static const float PROXIMITY_RELEASE = 0.30f; // m — extra before resuming

static const unsigned long DATA_TIMEOUT_MS = 3500;
static const unsigned long DEBUG_INTERVAL_MS = 700;

// How often to re-check which side is closing in and re-pick a steer
// direction. Checking every single tick would just react to sample noise.
static const unsigned long STEER_CHECK_MS = 300;
static float steerMargin = 0.02f; // m — tune with followm

// ─── State ─────────────────────────────────────────────

enum FollowState
{
    FOLLOW_OFF,
    FOLLOW_TURN,  // A not yet closest — rotating toward B or C
    FOLLOW_DRIVE, // A is closest — driving forward, curving toward B/C
    FOLLOW_HOLD   // arrived — A < arrivalRange
};

static FollowState state = FOLLOW_OFF;
static bool followDebug = true;
static bool followMonitor = false;

static bool turnInvert = false; // flips which way "right"/"left" spins the motors

// Raw, unfiltered, for the proximity/arrival stops
static float rawA = 0.0f, rawB = 0.0f, rawC = 0.0f, rawD = 0.0f;
static bool proximityLatched = false;
static unsigned long lastSampleMs = 0;

// EMA-smoothed anchor ranges, used for every steering decision below —
// safety stops above deliberately keep using the raw values so they never
// lag behind a real close approach.
static float emaA = 0.0f, emaB = 0.0f, emaC = 0.0f, emaD = 0.0f;
static bool emaInit = false;

// DRIVE steering: reference B/C captured at the last check, and the motion
// code currently in effect (3 = straight, 115 = curve right, 125 = curve
// left; pre-invert).
static unsigned long steerCheckMs = 0;
static float steerRefB = 0.0f, steerRefC = 0.0f;
static int driveCode = 3;

static unsigned long debugTimer = 0;

// ─── Helpers ───────────────────────────────────────────

// "Aimed" test: is the front anchor the closest of the four? This alone
// gates stop-turning/start-driving, and re-aim-if-drifted while driving.
static bool frontIsClosest()
{
    return emaA < emaB && emaA < emaC && emaA < emaD;
}

// Right/left motion codes, honoring followrev. Verified against drive.cpp:
// CMD_RIGHT uses 11/13/15 and the analogous "left wheel faster" curve
// family 111-120 (which includes 115); CMD_LEFT uses 21/23/25 and the
// "right wheel faster" family 121-130 (includes 125). So B (right anchor)
// pairs with the 31.../115 codes, C (left anchor) with 41.../125.
static int rightTurnCode()  { return turnInvert ? 41 : 31; }
static int leftTurnCode()   { return turnInvert ? 31 : 41; }
static int rightCurveCode() { return turnInvert ? 125 : 115; }
static int leftCurveCode()  { return turnInvert ? 115 : 125; }

static bool proximityBlocked()
{
    float nearest = (rawA < rawB) ? rawA : rawB;
    if (rawC > 0.0f && rawC < nearest)
        nearest = rawC;
    if (rawD > 0.0f && rawD < nearest)
        nearest = rawD;

    if (nearest <= 0.0f)
        return proximityLatched;

    if (proximityLatched)
    {
        if (nearest > proximityStopRange + PROXIMITY_RELEASE)
            proximityLatched = false;
    }
    else if (nearest < proximityStopRange)
    {
        proximityLatched = true;
    }

    return proximityLatched;
}

// Check if the front anchor is below the arrival threshold
static bool hasArrived()
{
    return rawA > 0.0f && rawA < arrivalRange;
}

static void printLiveFix()
{
    if (millis() - debugTimer < DEBUG_INTERVAL_MS)
        return;
    debugTimer = millis();

    Serial.print("FOLLOW: A=");
    Serial.print(rawA, 2);
    Serial.print(" B=");
    Serial.print(rawB, 2);
    Serial.print(" C=");
    Serial.print(rawC, 2);
    Serial.print(" D=");
    Serial.print(rawD, 2);
    Serial.print(" | st=");
    Serial.print((int)state);
    Serial.print(" cmd=");
    Serial.println(state == FOLLOW_DRIVE ? driveCode : 0);
}

// ─── Measurement intake ────────────────────────────────

static float parseRangeField(const String &s, int from, int to)
{
    String field = s.substring(from, to);

    int tag = field.indexOf("TAG=");
    if (tag >= 0)
        return field.substring(tag + 4).toFloat();

    return field.toFloat();
}

bool Follow_IngestLine(const String &line)
{
    String u = line;
    u.toUpperCase();

    int ai = u.indexOf("A=");
    if (ai < 0)
        return false;

    int bi = u.indexOf("B=", ai + 2);
    if (bi < 0)
        return false;

    int ci = u.indexOf("C=", bi + 2);
    int di = (ci >= 0) ? u.indexOf("D=", ci + 2) : -1;

    float a = parseRangeField(u, ai + 2, bi);
    float b = parseRangeField(u, bi + 2, (ci >= 0) ? ci : u.length());
    float cc = (ci >= 0) ? parseRangeField(u, ci + 2, (di >= 0) ? di : u.length()) : 0.0f;
    float d = (di >= 0) ? parseRangeField(u, di + 2, u.length()) : 0.0f;

    // Raw values captured before rejection — proximity stop must not be filtered out
    if (a > 0.0f && a < 50.0f)
    {
        rawA = a;
        emaA = emaInit ? emaA + ANCHOR_EMA_ALPHA * (a - emaA) : a;
    }
    if (b > 0.0f && b < 50.0f)
    {
        rawB = b;
        emaB = emaInit ? emaB + ANCHOR_EMA_ALPHA * (b - emaB) : b;
    }
    if (cc > 0.0f && cc < 50.0f)
    {
        rawC = cc;
        emaC = emaInit ? emaC + ANCHOR_EMA_ALPHA * (cc - emaC) : cc;
    }
    if (d > 0.0f && d < 50.0f)
    {
        rawD = d;
        emaD = emaInit ? emaD + ANCHOR_EMA_ALPHA * (d - emaD) : d;
    }
    emaInit = true;

    if (a <= 0.05f || a > 50.0f || b <= 0.05f || b > 50.0f || cc <= 0.05f || cc > 50.0f || d <= 0.05f || d > 50.0f)
        return true; // addressed to us, just not a usable quad

    lastSampleMs = millis();
    return true;
}

// ─── Public API ────────────────────────────────────────

void Follow_Init()
{
    state = FOLLOW_OFF;
    lastSampleMs = 0;
    rawA = rawB = rawC = rawD = 0.0f;
    emaA = emaB = emaC = emaD = 0.0f;
    emaInit = false;
    proximityLatched = false;
}

bool Follow_IsRunning()
{
    return (state != FOLLOW_OFF);
}

void Follow_Start()
{
    if (state != FOLLOW_OFF)
    {
        Serial.println("FOLLOW: already running");
        return;
    }

    // Follow-me drives motion() directly, so nothing else may own the motors.
    Drive_Cancel();
    Navigation_Cancel();

    proximityLatched = false;
    steerCheckMs = 0;
    state = FOLLOW_TURN;

    Serial.println("FOLLOW: --- FOLLOW-ME ARMED --- waiting for beacon data");
    Follow_PrintConfig();
}

void Follow_Cancel()
{
    if (state == FOLLOW_OFF)
        return;

    motion(0);
    state = FOLLOW_OFF;
    proximityLatched = false;
}

void Follow_Stop()
{
    if (state == FOLLOW_OFF)
    {
        Serial.println("FOLLOW: not running");
        return;
    }

    Follow_Cancel();
    Serial.println("FOLLOW: --- STOPPED ---");
}

void Follow_PrintConfig()
{
    Serial.print("FOLLOW: arrival<");
    Serial.print(arrivalRange, 2);
    Serial.print(" | proximity<");
    Serial.print(proximityStopRange, 2);
    Serial.print(" | steer margin ");
    Serial.print(steerMargin, 2);
    Serial.print(" m | turns ");
    Serial.print(turnInvert ? "INVERTED" : "normal");
    Serial.println(" | TURN+CURVE (low PWM)");
}

// ─── Update ────────────────────────────────────────────

void Follow_Update()
{
    if (state == FOLLOW_OFF)
    {
        if (followMonitor && (lastSampleMs != 0) &&
            (millis() - lastSampleMs <= DATA_TIMEOUT_MS))
            printLiveFix();
        return;
    }

    // Pause ('p', or a camera pause) must freeze follow-me too.
    if (Drive_IsPaused())
    {
        motion(0);
        return;
    }

    // Beacon link lost: stop and wait, never coast blind.
    if ((lastSampleMs == 0) || (millis() - lastSampleMs > DATA_TIMEOUT_MS))
    {
        motion(0);
        if (state != FOLLOW_TURN)
        {
            Serial.println("FOLLOW: beacon data lost — stopping");
            state = FOLLOW_TURN;
        }
        return;
    }

    // Arrival check: front anchor < arrivalRange → stop following
    if (hasArrived() && state != FOLLOW_HOLD)
    {
        motion(0);
        Serial.print("FOLLOW: arrived — A=");
        Serial.print(rawA, 2);
        Serial.println(" m");
        state = FOLLOW_HOLD;
        return;
    }

    // Hard proximity stop (safety backstop, tighter than arrival).
    if (proximityBlocked())
    {
        motion(0);
        if (state != FOLLOW_HOLD)
        {
            float nr = (rawA < rawB) ? rawA : rawB;
            if (rawC > 0.0f && rawC < nr)
                nr = rawC;
            if (rawD > 0.0f && rawD < nr)
                nr = rawD;
            Serial.print("FOLLOW: proximity stop — anchor ");
            Serial.print(nr, 2);
            Serial.println(" m");
            state = FOLLOW_HOLD;
        }
        return;
    }

    if (followDebug)
        printLiveFix();

    switch (state)
    {
    // ── TURN: A not yet closest — rotate toward whichever of B/C is
    // smaller (low PWM only, for now) ──
    case FOLLOW_TURN:
    {
        if (frontIsClosest())
        {
            Serial.println("FOLLOW: aimed — driving forward");
            state = FOLLOW_DRIVE;
            steerCheckMs = millis();
            steerRefB = emaB;
            steerRefC = emaC;
            driveCode = 3;
            motion(driveCode);
            break;
        }

        if (emaB < emaC)
            motion(rightTurnCode()); // beacon leans right, toward B
        else
            motion(leftTurnCode());  // beacon leans left, toward C
        break;
    }

    // ── DRIVE: A is closest — drive forward, curving toward whichever
    // of B/C has closed in since the last check ──
    case FOLLOW_DRIVE:
    {
        if (!frontIsClosest())
        {
            Serial.println("FOLLOW: beacon no longer ahead — re-aiming");
            state = FOLLOW_TURN;
            break;
        }

        if (millis() - steerCheckMs >= STEER_CHECK_MS)
        {
            float droppedB = steerRefB - emaB; // + = B got closer
            float droppedC = steerRefC - emaC; // + = C got closer
            steerRefB = emaB;
            steerRefC = emaC;
            steerCheckMs = millis();

            if (droppedB > steerMargin && droppedB >= droppedC)
                driveCode = rightCurveCode();
            else if (droppedC > steerMargin && droppedC > droppedB)
                driveCode = leftCurveCode();
            else
                driveCode = 3;
        }

        motion(driveCode);
        break;
    }

    // ── HOLD: arrived or proximity latched ──
    // If the beacon moves away again (A > arrivalRange), resume.
    case FOLLOW_HOLD:
    {
        motion(0);
        if (!hasArrived() && !proximityBlocked())
        {
            Serial.println("FOLLOW: beacon moved away — resuming");
            state = FOLLOW_TURN;
        }
        break;
    }

    default:
        break;
    }
}

// ─── Commands ──────────────────────────────────────────

bool Follow_HandleCommand(const String &line)
{
    if (line.equalsIgnoreCase("follow") || line.equalsIgnoreCase("followme"))
    {
        Follow_Start();
        return true;
    }

    if (line.equalsIgnoreCase("followstop"))
    {
        Follow_Stop();
        return true;
    }

    if (line.equalsIgnoreCase("followmon"))
    {
        followMonitor = !followMonitor;
        Serial.print("FOLLOW: monitor mode ");
        Serial.print(followMonitor ? "ON" : "OFF");
        Serial.println(" (readout only — motors stay idle)");
        return true;
    }

    if (line.equalsIgnoreCase("followcfg"))
    {
        Follow_PrintConfig();
        return true;
    }

    if (line.equalsIgnoreCase("followdbg"))
    {
        followDebug = !followDebug;
        Serial.print("FOLLOW: debug print ");
        Serial.println(followDebug ? "ON" : "OFF");
        return true;
    }

    if (line.equalsIgnoreCase("followrev"))
    {
        turnInvert = !turnInvert;
        Serial.print("FOLLOW: turn direction ");
        Serial.println(turnInvert ? "INVERTED" : "normal");
        return true;
    }

    String upper = line;
    upper.toUpperCase();

    if (upper.startsWith("FOLLOWM "))
    {
        float v = line.substring(8).toFloat();
        if (v >= 0.01f && v <= 0.20f)
        {
            steerMargin = v;
            Serial.print("FOLLOW: steer margin set to ");
            Serial.print(steerMargin, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: steer margin must be 0.01-0.20 m");
        }
        return true;
    }

    if (upper.startsWith("FOLLOWP "))
    {
        float v = line.substring(8).toFloat();
        if (v >= 0.20f && v <= 5.0f)
        {
            proximityStopRange = v;
            proximityLatched = false;
            Serial.print("FOLLOW: anchor stop set to ");
            Serial.print(proximityStopRange, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: anchor stop must be 0.20-5.00 m");
        }
        return true;
    }

    return false;
}
