#include "follow.h"
#include "drive.h"
#include "navigation.h"
#include "distance.h"
#include "velocity.h"

#include <Arduino.h>
#include <math.h>
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
// Direction logic (no trigonometry):
//
//     Front/back axis:  compare A vs D
//       A < D  →  beacon is AHEAD      (front component)
//       D < A  →  beacon is BEHIND     (back component)
//
//     Left/right axis:  compare B vs C
//       B < C  →  beacon is to the RIGHT
//       C < B  →  beacon is to the LEFT
//
//     Combine both axes → 8 cardinal directions.
//
// The robot turns a fixed angle per direction until A is the closest
// anchor (beacon is in front), then drives forward toward it.
// Stops when any anchor reads below the arrival threshold (default 2m).

// ─── Tuning ────────────────────────────────────────────

// Noise margin: a difference smaller than this is treated as "can't tell"
// rather than acted on. Prevents dithering when two anchors read almost
// the same distance.
static float compareMargin = 0.15f; // m

// Turn angles — small steps, so the robot converges gradually and never
// overshoots. After each step it re-checks and turns more if needed.
static const float TURN_FRONT_SIDE = 20.0f;  // deg — FRONT-LEFT / FRONT-RIGHT
static const float TURN_SIDE       = 30.0f;  // deg — LEFT / RIGHT
static const float TURN_BACK_SIDE  = 45.0f;  // deg — BACK-LEFT / BACK-RIGHT
static const float TURN_BACK       = 45.0f;  // deg — BACK (multiple steps to 180°)

// Max distance to drive before stopping to re-aim (metres)
static const float DRIVE_REALIGN_DIST = 0.5f;

// If min(all anchors) increases by more than this, we've passed the beacon
static const float DRIVE_WRONG_WAY_MARGIN = 0.08f; // m

// Arrival threshold: if ANY raw anchor distance drops below this while
// driving, stop following — the robot has arrived.
static float arrivalRange = 2.00f; // m

// Hard proximity stop on the RAW per-anchor ranges — if any single anchor
// reads under this, stop immediately.
static float proximityStopRange = 1.50f;
static const float PROXIMITY_RELEASE = 0.30f; // m — extra before resuming

static const int MEASURE_SAMPLES = 2;
static const unsigned long BRAKE_MS = 200; // motion(0) coast before trusting a sample
static const unsigned long MEASURE_TIMEOUT_MS = 9000;

static const unsigned long DATA_TIMEOUT_MS = 3500;
static const unsigned long DEBUG_INTERVAL_MS = 700;

// If the robot turns this many times in a row without ever reaching FRONT,
// something is wrong — flag it instead of spinning silently forever.
static const int STUCK_TURN_WARNING = 8;

// ─── State ─────────────────────────────────────────────

enum FollowState
{
    FOLLOW_OFF,
    FOLLOW_MEASURE, // stopped, averaging a clean fix
    FOLLOW_TURN,    // rotating a fixed amount for the current bucket
    FOLLOW_DRIVE,   // aimed at beacon, driving forward
    FOLLOW_HOLD     // arrived — any anchor < arrivalRange
};

enum Direction
{
    DIR_FRONT,
    DIR_FRONT_RIGHT,
    DIR_FRONT_LEFT,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_BACK_RIGHT,
    DIR_BACK_LEFT,
    DIR_BACK
};

static FollowState state = FOLLOW_OFF;
static bool followDebug = true;
static bool followMonitor = false;

static bool turnInvert = false; // flips which way "right"/"left" spins the motors

// Accumulator for the stationary average
static float accA = 0.0f, accB = 0.0f, accC = 0.0f, accD = 0.0f;
static int accN = 0;
static unsigned long measureStart = 0;
static bool measureBraked = false;

// Latest averaged fix
static float avgA = 0.0f, avgB = 0.0f, avgC = 0.0f, avgD = 0.0f;
static float avgRange = 0.0f;
static Direction lastDir = DIR_FRONT;

// Raw, unfiltered, for the proximity stop
static float rawA = 0.0f, rawB = 0.0f, rawC = 0.0f, rawD = 0.0f;
static bool proximityLatched = false;
static unsigned long lastSampleMs = 0;

// Turn bookkeeping
static float turnStartHeading = 0.0f;
static float turnStepSize = 0.0f;
static int turnSign = 1; // +1 = right, -1 = left
static int consecutiveTurns = 0;

static unsigned long debugTimer = 0;

// Drive tracking — monitor whether we're actually getting closer
static float driveStartDist = 0.0f;  // distance.travelled at drive start
static float driveBaseMin   = 0.0f;  // min(A,B,C,D) baseline (should decrease)

// Helper: smallest positive raw anchor
static float rawMin()
{
    float m = rawA;
    if (rawB > 0.0f && rawB < m) m = rawB;
    if (rawC > 0.0f && rawC < m) m = rawC;
    if (rawD > 0.0f && rawD < m) m = rawD;
    return m;
}

// Wrong-way detection — if a turn made things worse, flip direction
static float preTurnMin = 0.0f; // min range before the turn started

// ─── Helpers ───────────────────────────────────────────

static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// Tiered turn speed: slower as we approach the target angle.
// Uses follow-me-only motion codes 31-40 (right) and 41-50 (left)
// so these PWMs can be tuned without touching drive/navigation.
static int turnCode(float remaining, int s)
{
    if (turnInvert)
        s = -s;

    if (remaining <= 20.0f)
        return (s > 0) ? 31 : 41; // PWM 90
    if (remaining <= 45.0f)
        return (s > 0) ? 34 : 44; // PWM 130
    return (s > 0) ? 37 : 47;     // PWM 190
}

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

static const char *dirName(Direction d)
{
    switch (d)
    {
    case DIR_FRONT:       return "FRONT";
    case DIR_FRONT_RIGHT: return "FRONT-RIGHT";
    case DIR_FRONT_LEFT:  return "FRONT-LEFT";
    case DIR_RIGHT:       return "RIGHT";
    case DIR_LEFT:        return "LEFT";
    case DIR_BACK_RIGHT:  return "BACK-RIGHT";
    case DIR_BACK_LEFT:   return "BACK-LEFT";
    case DIR_BACK:        return "BACK";
    default:              return "?";
    }
}

// ─── Direction classifier ──────────────────────────────
//
// Two comparisons, one per axis:
//   frontness = D - A   →  positive means A is closer → beacon is AHEAD
//   rightness = C - B   →  positive means B is closer → beacon is RIGHT
//
// If neither axis clears the margin, fall back to whichever single
// anchor reads the smallest distance.

static Direction classify()
{
    float frontness = avgD - avgA; // + = beacon ahead (A closer)
    float rightness = avgC - avgB; // + = beacon right (B closer)

    bool isFront = frontness > compareMargin;
    bool isBack  = frontness < -compareMargin;
    bool isRight = rightness > compareMargin;
    bool isLeft  = rightness < -compareMargin;

    // Both axes have a clear signal
    if (isFront && isRight) return DIR_FRONT_RIGHT;
    if (isFront && isLeft)  return DIR_FRONT_LEFT;
    if (isFront)            return DIR_FRONT;

    if (isBack && isRight)  return DIR_BACK_RIGHT;
    if (isBack && isLeft)   return DIR_BACK_LEFT;
    if (isBack)             return DIR_BACK;

    if (isRight)            return DIR_RIGHT;
    if (isLeft)             return DIR_LEFT;

    // Neither axis cleared the margin — fallback: which anchor is closest?
    float minDist = avgA;
    Direction fallback = DIR_FRONT;

    if (avgB < minDist) { minDist = avgB; fallback = DIR_RIGHT; }
    if (avgC < minDist) { minDist = avgC; fallback = DIR_LEFT;  }
    if (avgD < minDist) { minDist = avgD; fallback = DIR_BACK;  }

    return fallback;
}

// ─── Turn / measure logic ──────────────────────────────

static void beginMeasure()
{
    motion(0);
    measureStart = millis();
    measureBraked = false;
    accA = accB = accC = accD = 0.0f;
    accN = 0;
    state = FOLLOW_MEASURE;
}

static void beginTurn(Direction d)
{
    switch (d)
    {
    case DIR_FRONT_RIGHT:
        turnSign = 1;
        turnStepSize = TURN_FRONT_SIDE;
        break;
    case DIR_FRONT_LEFT:
        turnSign = -1;
        turnStepSize = TURN_FRONT_SIDE;
        break;
    case DIR_RIGHT:
        turnSign = 1;
        turnStepSize = TURN_SIDE;
        break;
    case DIR_LEFT:
        turnSign = -1;
        turnStepSize = TURN_SIDE;
        break;
    case DIR_BACK_RIGHT:
        turnSign = 1;
        turnStepSize = TURN_BACK_SIDE;
        break;
    case DIR_BACK_LEFT:
        turnSign = -1;
        turnStepSize = TURN_BACK_SIDE;
        break;
    case DIR_BACK:
    default:
        turnSign = 1; // dead behind — default to right
        turnStepSize = TURN_BACK;
        break;
    }

    turnStartHeading = currentHeading;
    state = FOLLOW_TURN;
    consecutiveTurns++;

    if (consecutiveTurns == STUCK_TURN_WARNING)
    {
        Serial.println("FOLLOW: many turns without reaching FRONT — check followrev / followm");
        consecutiveTurns = 0;
    }

    Serial.print("FOLLOW: ");
    Serial.print(dirName(d));
    Serial.print(" -> turn ");
    Serial.print(turnSign > 0 ? "RIGHT " : "LEFT ");
    Serial.print(turnStepSize, 0);
    Serial.println(" deg");
}

// Check if any raw anchor is below the arrival threshold
static bool hasArrived()
{
    if (rawA > 0.0f && rawA < arrivalRange) return true;
    if (rawB > 0.0f && rawB < arrivalRange) return true;
    if (rawC > 0.0f && rawC < arrivalRange) return true;
    if (rawD > 0.0f && rawD < arrivalRange) return true;
    return false;
}

// Quick direction check from live raw data (no averaging needed)
// Used while driving to detect if beacon drifted off-front
static Direction classifyRaw()
{
    float frontness = rawD - rawA;
    float rightness = rawC - rawB;

    bool isFront = frontness > compareMargin;
    bool isBack  = frontness < -compareMargin;
    bool isRight = rightness > compareMargin;
    bool isLeft  = rightness < -compareMargin;

    if (isFront && isRight) return DIR_FRONT_RIGHT;
    if (isFront && isLeft)  return DIR_FRONT_LEFT;
    if (isFront)            return DIR_FRONT;

    if (isBack && isRight)  return DIR_BACK_RIGHT;
    if (isBack && isLeft)   return DIR_BACK_LEFT;
    if (isBack)             return DIR_BACK;

    if (isRight)            return DIR_RIGHT;
    if (isLeft)             return DIR_LEFT;

    // Fallback: smallest raw anchor
    float minDist = rawA;
    Direction fallback = DIR_FRONT;
    if (rawB > 0.0f && rawB < minDist) { minDist = rawB; fallback = DIR_RIGHT; }
    if (rawC > 0.0f && rawC < minDist) { minDist = rawC; fallback = DIR_LEFT;  }
    if (rawD > 0.0f && rawD < minDist) { minDist = rawD; fallback = DIR_BACK;  }
    return fallback;
}

// Called once a stationary averaged fix is ready.
static void decide()
{
    Direction d = classify();
    lastDir = d;

    // If the previous turn made things WORSE (min range increased),
    // flip direction — the other way is more likely correct.
    float curMin = rawMin();
    if (consecutiveTurns > 0 && preTurnMin > 0.0f && curMin > preTurnMin + 0.05f)
    {
        Serial.println("FOLLOW: last turn made range worse — flipping direction");
        if (d == DIR_RIGHT)       d = DIR_LEFT;
        else if (d == DIR_LEFT)   d = DIR_RIGHT;
        else if (d == DIR_BACK_RIGHT) d = DIR_BACK_LEFT;
        else if (d == DIR_BACK_LEFT)  d = DIR_BACK_RIGHT;
        else if (d == DIR_FRONT_RIGHT) d = DIR_FRONT_LEFT;
        else if (d == DIR_FRONT_LEFT)  d = DIR_FRONT_RIGHT;
        lastDir = d;
    }

    if (d == DIR_FRONT)
    {
        consecutiveTurns = 0;
        Serial.print("FOLLOW: aimed at beacon (R=");
        Serial.print(avgRange, 2);
        Serial.println(" m) — driving forward");
        state = FOLLOW_DRIVE;
        driveStartDist = distance.travelled;
        driveBaseMin = rawMin();
        motion(1); // drive forward
    }
    else
    {
        preTurnMin = rawMin(); // save before turning so we can detect wrong-way
        beginTurn(d);
    }
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
    Serial.print(" | dir=");
    Serial.print(dirName(lastDir));
    Serial.print(" st=");
    Serial.println((int)state);
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
        rawA = a;
    if (b > 0.0f && b < 50.0f)
        rawB = b;
    if (cc > 0.0f && cc < 50.0f)
        rawC = cc;
    if (d > 0.0f && d < 50.0f)
        rawD = d;

    if (a <= 0.05f || a > 50.0f || b <= 0.05f || b > 50.0f || cc <= 0.05f || cc > 50.0f || d <= 0.05f || d > 50.0f)
        return true; // addressed to us, just not a usable quad

    lastSampleMs = millis();

    // Only accumulate while genuinely stopped — rotating changes heading
    // between samples, which would blend different orientations.
    if ((state == FOLLOW_MEASURE && measureBraked) || state == FOLLOW_HOLD)
    {
        accA += a;
        accB += b;
        accC += cc;
        accD += d;
        accN++;
    }

    return true;
}

// ─── Public API ────────────────────────────────────────

void Follow_Init()
{
    state = FOLLOW_OFF;
    lastSampleMs = 0;
    rawA = rawB = rawC = rawD = 0.0f;
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
    consecutiveTurns = 0;
    beginMeasure();

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
    Serial.print(" | margin ");
    Serial.print(compareMargin, 2);
    Serial.print(" m | turns ");
    Serial.print(turnInvert ? "INVERTED" : "normal");
    Serial.println(" | TURN+DRIVE");
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
        if (state != FOLLOW_MEASURE)
        {
            Serial.println("FOLLOW: beacon data lost — stopping");
            beginMeasure();
        }
        return;
    }

    // Arrival check: any anchor < arrivalRange → stop following
    if (hasArrived() && state != FOLLOW_HOLD)
    {
        motion(0);
        float nr = rawA;
        if (rawB > 0.0f && rawB < nr) nr = rawB;
        if (rawC > 0.0f && rawC < nr) nr = rawC;
        if (rawD > 0.0f && rawD < nr) nr = rawD;
        Serial.print("FOLLOW: arrived — nearest anchor ");
        Serial.print(nr, 2);
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
            accA = accB = accC = accD = 0.0f;
            accN = 0;
        }
        return;
    }

    if (followDebug)
        printLiveFix();

    switch (state)
    {
    // ── MEASURE: stopped, averaging ──
    case FOLLOW_MEASURE:
    {
        motion(0);

        // Let the PWM ramp-down finish before counting anything
        if (!measureBraked)
        {
            if (millis() - measureStart < BRAKE_MS)
                break;
            measureBraked = true;
            measureStart = millis();
            accA = accB = accC = accD = 0.0f;
            accN = 0;
            break;
        }

        if (accN < MEASURE_SAMPLES)
        {
            if (millis() - measureStart > MEASURE_TIMEOUT_MS && accN >= 2)
            {
                Serial.print("FOLLOW: measure timed out, using ");
                Serial.print(accN);
                Serial.println(" samples");
            }
            else
            {
                break;
            }
        }

        avgA = accA / accN;
        avgB = accB / accN;
        avgC = accC / accN;
        avgD = accD / accN;
        avgRange = (avgA + avgB + avgC + avgD) / 4.0f;

        decide();
        break;
    }

    // ── TURN: rotate while continuously checking live beacon data ──
    // Instead of blindly completing the full planned angle, we re-check
    // classifyRaw() every tick. If it reads FRONT, jump straight to
    // driving. If it changes to a different bucket, stop and re-aim.
    case FOLLOW_TURN:
    {
        // Live re-check from raw data (no averaging needed)
        Direction liveDir = classifyRaw();
        lastDir = liveDir;

        if (liveDir == DIR_FRONT)
        {
            // We've turned enough — beacon is now in front, start driving
            consecutiveTurns = 0;
            Serial.println("FOLLOW: aimed mid-turn — driving forward");
            state = FOLLOW_DRIVE;
            motion(1);
            break;
        }

        float turned = fabs(normalizeAngle(currentHeading - turnStartHeading));
        float remaining = turnStepSize - turned;

        if (remaining <= 0.0f)
        {
            // Completed full planned angle but not yet FRONT — re-measure
            motion(0);
            beginMeasure();
        }
        else
        {
            motion(turnCode(remaining, turnSign));
        }
        break;
    }

    // ── DRIVE: aimed at beacon, driving forward ──
    case FOLLOW_DRIVE:
    {
        // Re-check direction from live raw data every tick
        Direction liveDir = classifyRaw();
        lastDir = liveDir;

        // Distance driven since last re-aim
        float driven = distance.travelled - driveStartDist;
        float curMin = rawMin();

        // 1) Passed closest point: min(all anchors) started increasing
        if (curMin > driveBaseMin + DRIVE_WRONG_WAY_MARGIN)
        {
            motion(0);
            Serial.print("FOLLOW: passed closest point (min ");
            Serial.print(driveBaseMin, 2);
            Serial.print(" -> ");
            Serial.print(curMin, 2);
            Serial.println(") — re-aiming");
            beginMeasure();
            break;
        }

        // 2) Driven far enough → mandatory re-aim
        if (driven >= DRIVE_REALIGN_DIST)
        {
            motion(0);
            Serial.print("FOLLOW: driven ");
            Serial.print(driven, 2);
            Serial.println(" m — re-aiming");
            beginMeasure();
            break;
        }

        // 3) Beacon drifted off-front → stop and re-aim
        if (liveDir != DIR_FRONT && liveDir != DIR_FRONT_LEFT && liveDir != DIR_FRONT_RIGHT)
        {
            motion(0);
            Serial.print("FOLLOW: beacon drifted ");
            Serial.print(dirName(liveDir));
            Serial.println(" — re-aiming");
            beginMeasure();
            break;
        }

        // Update baseline if getting closer
        if (curMin < driveBaseMin)
            driveBaseMin = curMin;

        motion(1); // keep driving straight
        break;
    }

    // ── HOLD: arrived or proximity latched ──
    // If the beacon moves away again (all anchors > arrivalRange), resume.
    case FOLLOW_HOLD:
    {
        motion(0);
        if (!hasArrived() && !proximityBlocked())
        {
            Serial.println("FOLLOW: beacon moved away — resuming");
            beginMeasure();
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
        if (v >= 0.03f && v <= 0.30f)
        {
            compareMargin = v;
            Serial.print("FOLLOW: compare margin set to ");
            Serial.print(compareMargin, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: margin must be 0.03-0.30 m");
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
