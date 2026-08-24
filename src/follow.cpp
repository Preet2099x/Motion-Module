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
// Three anchors range to the beacon the operator carries:
//
//     A  FORWARD  — mounted pointing forward
//     B  RIGHT    — mounted on the right
//     C  LEFT     — mounted on the left
//
//     B <-> C   = 30 cm
//     A <-> B   = A <-> C  = 34-35 cm
//
// No trigonometry anywhere in this file. Direction comes from plain
// comparisons between the three raw ranges — whichever anchor reads closer is
// the direction the beacon is more toward. That is the whole method.
//
//     rightness = C - B     positive -> B is closer -> beacon is to the RIGHT
//     frontness = avg(B,C) - A   positive -> A is closer -> beacon is AHEAD
//
// The frontness sign was checked against this exact geometry (A 31 cm ahead of
// the B-C line, B/C 15 cm either side): projecting a far beacon at bearing t
// onto each anchor gives A closer whenever cos(t) > 0, i.e. whenever the
// beacon is genuinely ahead.  So "front" is A SMALLER than B and C, not larger
// — if that reads backwards on the real hardware, `followfb` flips the sign in
// one command rather than needing a re-derivation.
//
// Combining the two comparisons with a noise margin gives 8 buckets (compass
// points) plus "aligned".  Each bucket maps to one action:
//
//     aligned / FRONT        -> drive forward
//     FRONT-LEFT/RIGHT       -> small nudge turn, then look again
//     LEFT / RIGHT           -> bigger turn, then look again
//     BACK-LEFT/RIGHT/BACK   -> biggest turn, then look again
//
// Every turn is followed by a completely fresh measurement and a fresh
// decision — there is no "was the turn good" judgement to get wrong, which is
// what caused the infinite-turn bug in the earlier trig-based version. The
// tradeoff is that this needs the margin to be bigger than the sensor noise,
// or two consecutive readings can disagree on the bucket and the robot
// dithers; `followm` tunes that margin live.
//
// "Range" for the stop/resume/drive-leg logic is just the average of the three
// readings — not a real geometric range, an approximation good enough for a
// threshold with hysteresis. The proximity stop is separate and reads the raw
// per-anchor minimum directly, with no averaging at all.

// ─── Tuning ────────────────────────────────────────────

// Noise margin for the front/back and left/right comparisons. A difference
// smaller than this is treated as "can't tell" rather than acted on — the
// single biggest defence against dithering on sensor noise.
// THIS HAS A HARD PHYSICAL CEILING. The signals cannot exceed the anchor
// spacings that produce them:
//
//     max |frontness| = A's forward offset  = 0.311 m
//     max |rightness| = B-to-C full spacing = 0.300 m
//
// So the margin is not a free tuning knob — it is a fraction of a very small
// range. If it is set too high, there are angles where NEITHER signal can
// ever clear it, and the beacon becomes invisible. That happens above
// 0.216 m, from (m/0.311)^2 + (m/0.300)^2 >= 1.
//
// It was previously 0.25, i.e. past that limit, which opened a blind cone
// around +-135 deg — beacon behind and off to a side, both signals maxing out
// at ~0.21. A real log showed exactly this: a stationary beacon at 130-180 deg
// read as "can't tell", the fallback drove forward, and the robot walked away
// from it for the entire run, range climbing 2.94 -> 4.24 m.
//
// (An earlier tuning pass "measured" 0.25 as better than 0.15. That test grid
// sampled bearings at 10/45/80/120/170 deg and simply never landed in the
// blind cone. Re-run on a dense 15 deg grid, 0.15 converges in 3.9 turns and
// 0.25 in 8.6.)
static float compareMargin = 0.15f; // m

// Turn sizes per bucket. Larger for buckets further from "already aimed".
// Slightly bigger than the wider margin alone accounts for: undershooting
// costs a whole extra look-and-turn cycle, overshooting costs the same, so
// biasing a little long is cheaper on average than biasing short.
static const float TURN_NUDGE = 25.0f; // deg — FRONT-LEFT / FRONT-RIGHT
static const float TURN_SIDE = 65.0f;  // deg — LEFT / RIGHT
static const float TURN_BACK = 110.0f; // deg — BACK-LEFT / BACK-RIGHT / BACK

// A direction decided by a TIEBREAK (one axis confident, the other just a
// sub-margin lean) is real evidence but not enough to commit the full turn
// to. So: turn this little instead, then measure fresh as normal. If the
// lean was right, the next look either agrees again (another small probe,
// now closer) or clears the margin outright (now confident -> full turn).
// If the lean was wrong, the next look sees the OPPOSITE sign and classify()
// just returns a different bucket — there is no separate "was that turn
// good" judgement to write or get wrong; the existing fresh-measurement-
// every-time design does the verify-or-abandon step for free.
static const float PROBE_STEP = 12.0f; // deg

// Hard proximity stop on the RAW per-anchor ranges — no averaging, no
// geometry. If any single anchor reads under this, stop immediately.
static float proximityStopRange = 0.70f;
static const float PROXIMITY_RELEASE = 0.30f; // m — extra before resuming

static const int MEASURE_SAMPLES = 4;
static const unsigned long BRAKE_MS = 400; // motion(0) coast before trusting a sample
static const unsigned long MEASURE_TIMEOUT_MS = 9000;

static const unsigned long DATA_TIMEOUT_MS = 3500;
static const unsigned long DEBUG_INTERVAL_MS = 700;

// If the robot turns this many times in a row without ever reaching a
// FRONT-ish bucket, something is wrong (inverted sense, bad margin, side
// facing wrong way) — flag it instead of spinning silently forever.
static const int STUCK_TURN_WARNING = 8;

// ─── State ─────────────────────────────────────────────

enum FollowState
{
    FOLLOW_OFF,
    FOLLOW_MEASURE, // stopped, averaging a clean fix
    FOLLOW_TURN,    // rotating a fixed amount for the current bucket
    FOLLOW_HOLD     // latched by the proximity stop; resumes when it clears
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
static bool fbInvert = false;   // flips which way frontness is read

// Alternate classifier: exact trilateration + atan2 + 45deg sectors, instead
// of the threshold-comparison method above. Simulated against the measured
// hardware noise this converges in ~3.2 turns on average vs ~1.7 for the
// threshold method — sector boundaries have no deadband, so noise near any
// boundary flips the answer, same failure mode as the margin-less comparison
// table this project already moved past once. Kept in as a live-toggleable
// A/B test (`followtri`) because simulation is a model, not the real robot.
static bool useTrilateration = false;

// This build does not drive. AT ALL. `follow` aims at the beacon and holds,
// and there is no state, flag or command that can make it translate — the
// drive state, the leg machinery and beginLeg() are gone from the file
// rather than merely disabled, so there is no path to reintroduce motion by
// getting a toggle wrong. Only motion(0) and the in-place turn codes are
// ever issued.

// A's forward offset from the B-C line, and B/C's half-spacing from centre —
// used ONLY by the trilateration path. sqrt(34.5^2-15^2) and 30/2 cm.
static const float TRI_HA = 0.3107f;
static const float TRI_HALF = 0.15f;

// Accumulator for the stationary average
static float accA = 0.0f, accB = 0.0f, accC = 0.0f;
static int accN = 0;
static unsigned long measureStart = 0;
static bool measureBraked = false;

// Latest averaged fix
static float avgA = 0.0f, avgB = 0.0f, avgC = 0.0f;
static float avgRange = 0.0f;
static Direction lastDir = DIR_FRONT;
static bool lastWasTiebreak = false; // classify() set this from a weak lean, not a confident reading

// Raw, unfiltered, for the proximity stop
static float rawA = 0.0f, rawB = 0.0f, rawC = 0.0f;
static bool proximityLatched = false;
static unsigned long lastSampleMs = 0;

// Leg / turn bookkeeping
static float turnStartHeading = 0.0f;
static float turnStepSize = 0.0f;
static int turnSign = 1; // +1 = right, -1 = left
static int consecutiveTurns = 0;

static unsigned long debugTimer = 0;
static bool camStopReported = false;

// ─── Helpers ───────────────────────────────────────────

static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// motion.cpp: forward is both dir pins LOW; 31-40 drives the left wheel
// forward and reverses the right (spins RIGHT), 41-50 the opposite — the
// same pin pattern as 11-30, just a follow-me-only code range so these PWMs
// can be tuned without touching drive.cpp/navigation.cpp's turn speed.
// turnInvert swaps them if the robot disagrees in the field.
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

// The whole "geometry": two comparisons, a margin, and a lookup.
// Exact trilateration: solve the beacon's (x,y) in the robot frame from the
// three ranges (x=forward, y=right), then bucket by a plain 45deg sector on
// atan2(y,x). Unlike the far-field approximation this holds at any range —
// verified numerically to zero error against known positions — but it is
// still built from the same three noisy inputs, so the noise problem is
// unchanged: only the packaging is different.
static float lastTriAngle = 0.0f;

static Direction classifyTrilateration()
{
    float dA = avgA, dB = avgB, dC = avgC;

    float y = (dC * dC - dB * dB) / (4.0f * TRI_HALF);
    float x = (2.0f * TRI_HA * TRI_HA - 2.0f * TRI_HALF * TRI_HALF -
               2.0f * dA * dA + dB * dB + dC * dC) /
              (4.0f * TRI_HA);

    if (fbInvert)
        x = -x;

    float ang = atan2f(y, x) * 180.0f / PI; // + = right of forward
    lastTriAngle = ang;

    float a = fabs(ang);
    if (a <= 22.5f)
        return DIR_FRONT;
    if (a <= 67.5f)
        return (ang > 0) ? DIR_FRONT_RIGHT : DIR_FRONT_LEFT;
    if (a <= 112.5f)
        return (ang > 0) ? DIR_RIGHT : DIR_LEFT;
    if (a <= 157.5f)
        return (ang > 0) ? DIR_BACK_RIGHT : DIR_BACK_LEFT;
    return DIR_BACK;
}

static Direction classify()
{
    lastWasTiebreak = false;

    if (useTrilateration)
        return classifyTrilateration();

    float frontness = ((avgB + avgC) * 0.5f) - avgA; // + = A closer = ahead
    if (fbInvert)
        frontness = -frontness;

    float rightness = avgC - avgB; // + = B closer = to the right

    bool isFront = frontness > compareMargin;
    bool isBack = frontness < -compareMargin;
    bool isRight = rightness > compareMargin;
    bool isLeft = rightness < -compareMargin;

    if (isFront)
    {
        if (isRight)
            return DIR_FRONT_RIGHT;
        if (isLeft)
            return DIR_FRONT_LEFT;
        return DIR_FRONT;
    }
    if (isBack)
    {
        if (isRight)
            return DIR_BACK_RIGHT;
        if (isLeft)
            return DIR_BACK_LEFT;
        // Front/back is confident (frontness cleared the margin), but left/
        // right didn't. That's not "no information" -- a real look at rightness
        // does exist, just too small to trust as an independent direction. It IS
        // trustworthy enough to break a tie: turning is going to happen either
        // way (that's what isBack already decided), so leaning it toward
        // whichever side the raw number already favours beats a fixed default.
        // But it's still ONLY a lean, not confident evidence -- beginTurn()
        // reads lastWasTiebreak and turns a small probe amount instead of the
        // full commit, and the next look either confirms it or corrects it.
        lastWasTiebreak = true;
        return (rightness >= 0.0f) ? DIR_BACK_RIGHT : DIR_BACK_LEFT;
    }
    // Left/right is confident, front/back isn't. Same move as the BACK case
    // above: lean the weak frontness reading toward FRONT-* or BACK-* instead
    // of throwing it away for a flat RIGHT/LEFT -- and same caveat, it's a
    // probe-sized turn until the next look confirms or corrects it.
    if (isRight)
    {
        lastWasTiebreak = true;
        return (frontness >= 0.0f) ? DIR_FRONT_RIGHT : DIR_BACK_RIGHT;
    }
    if (isLeft)
    {
        lastWasTiebreak = true;
        return (frontness >= 0.0f) ? DIR_FRONT_LEFT : DIR_BACK_LEFT;
    }

    // Nothing cleared the margin on either axis. This used to return DIR_FRONT
    // unconditionally — "can't tell, assume aligned, drive forward". That is
    // only safe when the weak reading at least LEANS forward. If frontness is
    // negative the beacon is behind, and driving forward actively increases
    // the range: observed in a real log as the robot walking away from a
    // stationary beacon sitting at 130-180 deg, cycle after cycle, because
    // every reading landed just inside the margin.
    //
    // So: respect the sign even when the magnitude is not trustworthy. A weak
    // forward lean still drives (worst case the beacon is off to a side and
    // forward motion still closes distance). A weak backward lean turns
    // instead — as a probe-sized step, since the evidence is weak.
    if (frontness >= 0.0f)
        return DIR_FRONT;

    lastWasTiebreak = true;
    return (rightness >= 0.0f) ? DIR_BACK_RIGHT : DIR_BACK_LEFT;
}

static void printLiveFix()
{
    if (millis() - debugTimer < DEBUG_INTERVAL_MS)
        return;
    debugTimer = millis();

    Serial.print("FOLLOW: raw=");
    Serial.print(rawA, 2);
    Serial.print("/");
    Serial.print(rawB, 2);
    Serial.print("/");
    Serial.print(rawC, 2);
    Serial.print(" R=");
    Serial.print(avgRange, 2);
    Serial.print(" dir=");
    Serial.print(dirName(lastDir));
    if (useTrilateration)
    {
        Serial.print(" ang=");
        Serial.print(lastTriAngle, 0);
    }
    Serial.print(" st=");
    Serial.println((int)state);
}

static void beginMeasure()
{
    motion(0);
    measureStart = millis();
    measureBraked = false;
    accA = accB = accC = 0.0f;
    accN = 0;
    state = FOLLOW_MEASURE;
}

static void beginTurn(Direction d)
{
    switch (d)
    {
    case DIR_FRONT_RIGHT:
        turnSign = 1;
        turnStepSize = TURN_NUDGE;
        break;
    case DIR_FRONT_LEFT:
        turnSign = -1;
        turnStepSize = TURN_NUDGE;
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
        turnStepSize = TURN_BACK;
        break;
    case DIR_BACK_LEFT:
        turnSign = -1;
        turnStepSize = TURN_BACK;
        break;
    case DIR_BACK:
    default:
        // Dead behind — no side information at all. Default to right; if the
        // operator is not exactly behind next check, one of the L/R buckets
        // will catch it and correct.
        turnSign = 1;
        turnStepSize = TURN_BACK;
        break;
    }

    // Weak (tiebreak) evidence only earns a probe-sized turn, not the full
    // commit -- see the note on PROBE_STEP for why this needs no separate
    // "was it right" check of its own.
    if (lastWasTiebreak && turnStepSize > PROBE_STEP)
        turnStepSize = PROBE_STEP;

    turnStartHeading = currentHeading;
    state = FOLLOW_TURN;
    consecutiveTurns++;

    if (consecutiveTurns == STUCK_TURN_WARNING)
    {
        Serial.println("FOLLOW: many turns without reaching FRONT — check followfb / followrev / followm");
        consecutiveTurns = 0;
    }

    Serial.print("FOLLOW: ");
    Serial.print(dirName(d));
    Serial.print(lastWasTiebreak ? " (probe) -> turn " : " -> turn ");
    Serial.print(turnSign > 0 ? "RIGHT " : "LEFT ");
    Serial.print(turnStepSize, 0);
    Serial.println(" deg");
}

// Called once a stationary averaged fix is in hand.
// Aim at the beacon and hold. There is no drive branch: reaching DIR_FRONT
// means "already pointing at them", and the response to that is to stop and
// keep watching, not to close distance.
static void decide()
{
    Direction d = classify();
    lastDir = d;

    if (d == DIR_FRONT)
    {
        motion(0);
        consecutiveTurns = 0;
        Serial.print("FOLLOW: aimed at beacon (R=");
        Serial.print(avgRange, 2);
        Serial.println(" m) — holding");
        // Keep measuring, so moving the beacon makes it re-aim.
        beginMeasure();
    }
    else
    {
        beginTurn(d);
    }
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

    float a = parseRangeField(u, ai + 2, bi);
    float b = parseRangeField(u, bi + 2, (ci >= 0) ? ci : u.length());
    float cc = (ci >= 0) ? parseRangeField(u, ci + 2, u.length()) : 0.0f;

    // Raw values are captured before any rejection: a reading too small to be a
    // plausible range is still evidence the operator is close, and the guard
    // that stops the robot must not be the thing that gets filtered out.
    if (a > 0.0f && a < 50.0f)
        rawA = a;
    if (b > 0.0f && b < 50.0f)
        rawB = b;
    if (cc > 0.0f && cc < 50.0f)
        rawC = cc;

    if (a <= 0.05f || a > 50.0f || b <= 0.05f || b > 50.0f || cc <= 0.05f || cc > 50.0f)
        return true; // addressed to us, just not a usable triple

    lastSampleMs = millis();

    // The real, stationary average used for actual turn/drive decisions: only
    // accumulated while genuinely stopped, never mid-turn -- rotating changes
    // the robot's heading between samples, which would blend readings taken
    // at different orientations into one meaningless average.
    if ((state == FOLLOW_MEASURE && measureBraked) || state == FOLLOW_HOLD)
    {
        accA += a;
        accB += b;
        accC += cc;
        accN++;
    }

    return true;
}

// ─── Public API ────────────────────────────────────────

void Follow_Init()
{
    state = FOLLOW_OFF;
    lastSampleMs = 0;
    rawA = rawB = rawC = 0.0f;
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
    // Drive_Cancel() also clears a stale pause flag, which would otherwise
    // force motion(0) every tick from Drive_Update().
    Drive_Cancel();
    Navigation_Cancel();

    proximityLatched = false;
    camStopReported = false;
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
    Serial.print("FOLLOW: anchor<");
    Serial.print(proximityStopRange, 2);
    Serial.print(" | margin ");
    Serial.print(compareMargin, 2);
    Serial.print(" m | turns ");
    Serial.print(turnInvert ? "INVERTED" : "normal");
    Serial.print(" | front/back ");
    Serial.print(fbInvert ? "INVERTED" : "normal");
    Serial.print(" | method ");
    Serial.print(useTrilateration ? "TRILATERATION" : "THRESHOLD");
    Serial.println(" | AIM-ONLY (this build never drives)");
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

    // Pause ('p', or a camera pause) must freeze follow-me too.  Drive_Update()
    // already forces motion(0) every tick while paused and runs BEFORE this, so
    // without freezing here the command issued below would overwrite that stop
    // on the same tick and the robot would keep going.
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

    // Hard proximity stop. Retained as a safety backstop even though this
    // build never drives: if it ever does latch, the robot stops, which is
    // always the safe outcome. It does NOT block turning, because turning in
    // place cannot close distance on the operator.
    if (proximityBlocked())
    {
        motion(0);
        if (state != FOLLOW_HOLD)
        {
            float nr = (rawA < rawB) ? rawA : rawB;
            if (rawC > 0.0f && rawC < nr)
                nr = rawC;
            Serial.print("FOLLOW: proximity stop — anchor ");
            Serial.print(nr, 2);
            Serial.println(" m");
            state = FOLLOW_HOLD;
            accA = accB = accC = 0.0f;
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

        // Let the PWM ramp-down finish before counting anything — readings
        // taken while still coasting are not stationary readings.
        if (!measureBraked)
        {
            if (millis() - measureStart < BRAKE_MS)
                break;
            measureBraked = true;
            measureStart = millis();
            accA = accB = accC = 0.0f;
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
        avgRange = (avgA + avgB + avgC) / 3.0f;

        decide();
        break;
    }

    // ── TURN: rotate the fixed amount for the current bucket ──
    case FOLLOW_TURN:
    {
        float turned = fabs(normalizeAngle(currentHeading - turnStartHeading));
        float remaining = turnStepSize - turned;

        if (remaining <= 0.0f)
        {
            motion(0);
            beginMeasure(); // always re-measure after a turn, never assume
        }
        else
        {
            motion(turnCode(remaining, turnSign));
        }
        break;
    }

    // ── DRIVE: one leg, roughly toward the beacon ──
    // ── HOLD: close enough — stay put until they walk away ──
    // ── HOLD: latched by the proximity stop ──
    // The proximity check at the top of Follow_Update returns early for as
    // long as it is blocked, so reaching this case at all means it has
    // released. Resume aiming immediately. (It previously waited on a range
    // threshold the proximity guard never consults, which could strand the
    // robot here: proximity releases at ~1.0 m but the old exit wanted 2.3 m,
    // so standing anywhere between the two left it frozen.)
    case FOLLOW_HOLD:
    {
        motion(0);
        Serial.println("FOLLOW: proximity clear — resuming aim");
        beginMeasure();
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

    if (line.equalsIgnoreCase("followfb"))
    {
        fbInvert = !fbInvert;
        Serial.print("FOLLOW: front/back sense ");
        Serial.println(fbInvert ? "INVERTED" : "normal");
        return true;
    }

    if (line.equalsIgnoreCase("followtri"))
    {
        useTrilateration = !useTrilateration;
        Serial.print("FOLLOW: direction method = ");
        Serial.println(useTrilateration ? "TRILATERATION (45deg sectors)" : "THRESHOLD (margin comparisons)");
        return true;
    }

    String upper = line;
    upper.toUpperCase();

    if (upper.startsWith("FOLLOWM "))
    {
        float v = line.substring(8).toFloat();
        // Capped at 0.21, not an arbitrary limit: past 0.216 there are angles
        // where neither frontness nor rightness can physically reach the
        // margin (their ceilings are only 0.311 and 0.300), and the beacon
        // goes invisible in a cone around +-135 deg. See the note on
        // compareMargin.
        if (v >= 0.03f && v <= 0.21f)
        {
            compareMargin = v;
            Serial.print("FOLLOW: compare margin set to ");
            Serial.print(compareMargin, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: margin must be 0.03-0.21 m (above 0.216 opens a blind cone at +/-135 deg)");
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
