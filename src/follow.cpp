#include "follow.h"
#include "drive.h"
#include "navigation.h"

#include <Arduino.h>
#include <math.h>
#include <var.h>

// ─── Geometry ──────────────────────────────────────────
//
// A is the beacon on the FRONT of the robot, B the one on the BACK, a fixed
// baseline `d` apart.  Both report their range to the single fixed "main"
// beacon the operator carries.  With R the range from the robot centre to the
// main beacon and theta the beacon's bearing off the robot's forward axis, the
// law of cosines gives
//
//     A^2 = R^2 + (d/2)^2 - R*d*cos(theta)
//     B^2 = R^2 + (d/2)^2 + R*d*cos(theta)
//
// which inverts exactly to
//
//     R          = sqrt( (A^2 + B^2)/2 - (d/2)^2 )
//     cos(theta) = (B^2 - A^2) / (2*R*d)
//
// theta therefore comes out as a MAGNITUDE in [0, 180]:
//     ~0    beacon dead ahead          (A much smaller than B)
//     ~90   beacon straight off a side (A == B)
//     ~180  beacon dead astern         (A much larger than B)
//
// Two ranges are symmetric about the robot's long axis, so they can never say
// whether the beacon is to the LEFT or the RIGHT.  That sign is resolved by
// rotating and checking which of the two hypotheses predicted the next reading
// better -- see the side-inference note further down -- and then held under a
// confidence counter so noise cannot keep reversing it.  Between fixes the
// bearing is carried forward on the IMU, so steering runs at loop rate off a
// 1 Hz sensor.

// ─── Tuning ────────────────────────────────────────────

static float beaconBaseline = 0.545f; // d — metres between beacon A and B (measured 54-55 cm)
static float standoffRange = 1.20f;  // m — stop this far from the operator

static const float RANGE_HYSTERESIS = 0.30f; // m — resume driving past standoff+this
static const float CRUISE_MARGIN = 1.00f;    // m — full speed only beyond standoff+this

// Bearing thresholds.  These are deliberately loose, and that is a property
// of the sensor, not a shortcut.  cos(theta) is flat near 0 deg, so range
// noise blows up into apparent bearing exactly where the robot is already
// aligned: with d = 0.5 m, 4 cm range noise and 8 averaged samples, a beacon
// that is truly dead ahead still reads ~7 deg +/- 8.  Off-axis the same setup
// is good to ~3 deg.  So: trust the bearing when it is large, and treat
// anything under ALIGN_TOL as "aligned" rather than hunting inside the noise.
// Mounting A and B further apart is what actually buys resolution here --
// the floor scales as 1/d, so d = 1.0 m roughly halves it (see `followd`).
static const float ALIGN_TOL = 20.0f;   // deg — close enough to drive forward
static const float REALIGN_TOL = 40.0f; // deg — drifted far enough to re-aim

// Arc steering.  Between ALIGN_TOL and ARC_TOL the robot curves toward the
// beacon WHILE driving instead of stopping to spin on the spot.  This is what
// removes the stop-turn-stop-turn shuffle: an operator walking ahead only ever
// drifts a few tens of degrees off the nose, and that is entirely correctable
// on the move.  Only a genuinely large error (or an unknown side) is worth
// stopping for.
static const float ARC_TOL = 70.0f; // deg — beyond this, spin in place instead

// Probe stepping — used ONLY when the side is unknown.
static const float PROBE_STEP = 25.0f; // deg — big enough to beat bearing noise

// ─── Which side is the beacon on? ──────────────────────
//
// Two ranges are symmetric about the robot's long axis, so the geometry yields
// |theta| but never the side.  The side is *inferred*: we know how far we just
// rotated (IMU) and which way (we commanded it), so each hypothesis makes a
// testable prediction for the next reading —
//
//     beacon on the right : theta_next = |theta_prev - rot|
//     beacon on the left  : theta_next = |theta_prev + rot|
//
// — and whichever prediction lands closer to the measurement wins.  That is a
// far stronger test than the old "did theta improve by 2 deg", which at 1 Hz
// flips on sensor noise alone and produced exactly the left-right-left shuffle
// this replaces.  The confidence counter then makes a locked-in side expensive
// to unseat: one bad fix costs a point, it does not reverse the robot.
static const float SIGN_MARGIN_DEG = 5.0f; // predictions must differ by this to count
static const int SIGN_CONF_MAX = 3;        // cap on accumulated confidence
static const float SIGN_MIN_ROT = 5.0f;    // deg — less rotation than this proves nothing

// Settling: after each step the motors are stopped and the range filter is
// flushed, so the next evaluation sees only readings taken while stationary.
// motion(0) ramps PWM down rather than cutting it, so the robot is still
// moving for ~300 ms afterwards — BRAKE_MS covers that coast before the
// filter is flushed, otherwise the "stopped" readings are anything but.
static const unsigned long BRAKE_MS = 350;
static const unsigned long SETTLE_MS = 350;
// Sized for the ~1 Hz beacon feed this rig actually produces. Every sample
// count here is a WALL-CLOCK cost: at 1 Hz, 8 samples is 8 seconds of the
// robot sitting still after every single turn step, which makes following a
// walking person impossible. If you raise the Jetson's send rate to 5-10 Hz,
// put this back to 8 and MIN_SAMPLES back to 6 — the bearing gets much
// quieter and none of the timing hurts any more.
static const int SETTLE_SAMPLES = 2;

// Filtering
// The EMA is what keeps the bearing usable -- more averaging buys a quieter
// theta -- but every averaged sample costs wall-clock time at 1 Hz. alpha 0.5
// averages ~3 readings, already a ~3 s lag on a walking operator; slower than
// that tracks where they were, not where they are. At 5-10 Hz, 0.25 is the
// better number (~7 readings, roughly +/-8 deg instead of +/-13 at d=0.5 m).
static const float FILTER_ALPHA = 0.50f; // EMA on each raw range
static const int MIN_SAMPLES = 2;        // samples before the filter is trusted

// Safety
// Must clear the beacon's send PERIOD with margin: at 1 Hz a 1500 ms timeout
// trips on any half-second of jitter and the robot reports "data lost".
static const unsigned long DATA_TIMEOUT_MS = 3500;  // no beacon line -> stop
static const unsigned long DEBUG_INTERVAL_MS = 500;

// ─── State ─────────────────────────────────────────────

enum FollowState
{
    FOLLOW_OFF,
    FOLLOW_ACQUIRE,   // waiting for usable beacon data
    FOLLOW_TURN_STEP, // rotating one step toward the beacon
    FOLLOW_SETTLE,    // stopped, refilling the filter before judging the step
    FOLLOW_DRIVE,     // aligned — driving at the beacon
    FOLLOW_HOLD       // inside the standoff — stopped, still facing the beacon
};

static FollowState state = FOLLOW_OFF;
static bool followDebug = true;
static bool followMonitor = false; // live readout while disarmed (bring-up)

// Beacon measurement state
static float filtA = 0.0f;
static float filtB = 0.0f;
static int sampleCount = 0;
static unsigned long lastSampleMs = 0;

static float beaconRange = 0.0f; // R, metres
static float beaconTheta = 0.0f; // |bearing| off the forward axis, 0..180 deg

// Turn state
static int turnSign = 1; // +1 = right, -1 = left
static float stepStartHeading = 0.0f;
static float stepSize = 0.0f; // deg planned for the current step

// Signed-bearing tracking.  beaconTheta is the raw magnitude from the last
// fix; bearingSigned is that magnitude carrying the inferred side, kept
// current between fixes by folding in the robot's own rotation.  At 1 Hz that
// dead-reckoning is what allows smooth control at loop rate instead of one
// jerk per second.
static float bearingSigned = 0.0f; // + = beacon to the right of the nose
static int signConfidence = 0;     // 0 = side unknown (probe); >0 = locked in
static float thetaAtLastFix = 0.0f;
static float rotSinceFix = 0.0f; // deg rotated since last fix, + = right
static bool haveLastFix = false;

// Rotation bookkeeping.  activeTurnSign is the direction we are *commanding*
// right now, so the IMU only ever has to supply a magnitude — this never
// depends on which way the BNO happens to count its heading.
static float prevHeading = 0.0f;
static bool headingTracked = false;
static int activeTurnSign = 0; // +1 right, -1 left, 0 = straight/stopped

static unsigned long settleStart = 0;
static bool settleJudgesStep = false;    // false when we settled without turning
static bool settleFilterCleared = false; // brake phase done, now sampling

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

static void resetFilter()
{
    sampleCount = 0;
}

// Forget everything inferred about the beacon's side. Used on arm/disarm and
// whenever the link drops, so a new follow never inherits a stale side.
static void resetTracking()
{
    bearingSigned = 0.0f;
    signConfidence = 0;
    thetaAtLastFix = 0.0f;
    rotSinceFix = 0.0f;
    haveLastFix = false;
    headingTracked = false;
    activeTurnSign = 0;
}

static bool dataValid()
{
    return (sampleCount >= MIN_SAMPLES);
}

// Motion code for an in-place turn, tiered by how much is left to rotate.
// Follow-me stays on the slow tiers: theta lags the true bearing by a filter
// time constant, so a fast spin overshoots before the estimate catches up.
static int turnCode(float remaining, int s)
{
    if (remaining <= 6.0f)
        return (s > 0) ? 11 : 21; // crawl, PWM 60
    if (remaining <= 20.0f)
        return (s > 0) ? 14 : 24; // slow, PWM 80
    return (s > 0) ? 17 : 27;     // mid, PWM 120
}

// Forward curve toward side `s`, tiered by how far off the nose the beacon is.
// motion.cpp drives 111-120 with the LEFT wheel faster (curves right) and
// 121-130 with the RIGHT wheel faster (curves left), matching the 1x=right /
// 2x=left convention the in-place turns use.  (The camera-block comment in
// main.cpp labels these two ranges the other way round; it blocks both
// together, so the mislabel has never had a functional effect.)
static int arcCode(float theta, int s)
{
    int base = (s > 0) ? 111 : 121;
    if (theta < 30.0f)
        return base + 1; // 112 / 122 — gentle
    if (theta < 50.0f)
        return base + 4; // 115 / 125 — medium
    return base + 7;     // 118 / 128 — sharp
}

// Every follow-me motion command goes through here so activeTurnSign can never
// fall out of step with what the motors are actually doing — the rotation
// tracker below depends on knowing which way we asked the robot to turn.
static void followMotion(int code, int turning)
{
    activeTurnSign = turning;
    motion(code);
}

// Fold the rotation since the previous tick into the signed bearing, and into
// the running total the side test consumes.  The IMU supplies only a
// magnitude; the direction comes from the command we issued, so none of this
// depends on which way the BNO counts its heading.
static void trackRotation()
{
    if (!headingTracked)
    {
        prevHeading = currentHeading;
        headingTracked = true;
        return;
    }

    float dmag = fabs(normalizeAngle(currentHeading - prevHeading));
    prevHeading = currentHeading;

    if (activeTurnSign == 0 || dmag < 0.01f)
        return;

    float rot = activeTurnSign * dmag;
    rotSinceFix += rot;
    bearingSigned = normalizeAngle(bearingSigned - rot);

    // Turning past the beacon puts it on the OTHER side. turnSign has to
    // follow the crossing, otherwise the next fix re-applies the stale side
    // (bearingSigned = turnSign * theta) and silently undoes the overshoot --
    // which reads as the robot refusing to correct back.
    turnSign = (bearingSigned >= 0.0f) ? 1 : -1;
}

// Rate-limited live readout of the current fix.  Shared by the running
// debug stream and by monitor mode.
static void printLiveFix()
{
    if (millis() - debugTimer < DEBUG_INTERVAL_MS)
        return;
    debugTimer = millis();

    Serial.print("FOLLOW: A=");
    Serial.print(filtA, 2);
    Serial.print(" B=");
    Serial.print(filtB, 2);
    Serial.print(" R=");
    Serial.print(beaconRange, 2);
    Serial.print(" theta=");
    Serial.print(beaconTheta, 0);
    Serial.print(" bearing=");
    Serial.print(bearingSigned, 0);
    Serial.print(" side=");
    Serial.print(signConfidence == 0 ? "?" : (turnSign > 0 ? "R" : "L"));
    Serial.print(" conf=");
    Serial.print(signConfidence);
    Serial.print(" state=");
    Serial.println((int)state);
}

static void beginSettle(bool judgeStep)
{
    followMotion(0, 0);
    settleStart = millis();
    settleJudgesStep = judgeStep;
    settleFilterCleared = false; // brake first, flush the filter after
    state = FOLLOW_SETTLE;
}

// A probe is a deliberate rotation taken purely to learn which side the beacon
// is on.  It is the ONLY thing that stops the robot to measure, and it runs
// only while signConfidence is 0 — normally once, at the start of a follow.
static void beginProbe()
{
    stepStartHeading = currentHeading;
    stepSize = PROBE_STEP;

    state = FOLLOW_TURN_STEP;

    if (followDebug)
    {
        Serial.print("FOLLOW: probing ");
        Serial.print(turnSign > 0 ? "right " : "left ");
        Serial.print(stepSize, 0);
        Serial.print(" deg to find the side (theta=");
        Serial.print(beaconTheta, 0);
        Serial.println(")");
    }
}

// Enter the continuous drive state.  Everything is decided tick by tick from
// there, so there is no longer a separate "turn" state in the normal path.
static void chooseNextAction()
{
    if (beaconRange <= standoffRange)
    {
        followMotion(0, 0);
        state = FOLLOW_HOLD;
        Serial.print("FOLLOW: holding at ");
        Serial.print(beaconRange, 2);
        Serial.println(" m");
        return;
    }

    state = FOLLOW_DRIVE;
    Serial.print("FOLLOW: driving (R=");
    Serial.print(beaconRange, 2);
    Serial.print(" m, theta=");
    Serial.print(beaconTheta, 0);
    Serial.println(")");
}

// ─── Continuous control law ────────────────────────────
//
// Runs every tick off the fused bearing, so steering updates at loop rate
// rather than once per beacon fix.  The ordering is the whole point:
//
//   aligned      -> drive straight
//   modest error -> CURVE while still driving forward   (no stopping)
//   large error  -> spin in place, but continuously, judged by the IMU
//
// The old code stopped, turned a step, stopped again and re-measured for every
// single correction, which at 1 Hz is where the left-right-left shuffle came
// from.  Here a spin ends the moment the dead-reckoned bearing falls inside
// ALIGN_TOL, which the IMU knows at 100 Hz — no waiting for the next fix.
static void steerToward()
{
    float theta = fabs(bearingSigned);
    int s = (bearingSigned >= 0.0f) ? 1 : -1;

    if (theta <= ALIGN_TOL)
    {
        // Aligned: motion(1)/motion(3) hold heading on the IMU PID.
        if (beaconRange > standoffRange + CRUISE_MARGIN)
            followMotion(1, 0);
        else
            followMotion(3, 0);
        return;
    }

    if (signConfidence > 0 && theta <= ARC_TOL)
    {
        followMotion(arcCode(theta, s), s);
        return;
    }

    // Too far off to fix on the move — rotate on the spot, but keep rotating
    // rather than stepping and re-measuring.
    followMotion(turnCode(theta, s), s);
}

// ─── Measurement intake ────────────────────────────────

// Pull the range out of one beacon's field, e.g. "A: TAG=3.13M  ".
// The module reports "A=A: TAG=3.13M", so the number lives after the TAG=
// label, not directly after "A=".  The bare form "A=3.35m" is still accepted
// as a fallback.  toFloat() stops at the first non-numeric character, so the
// trailing unit letter takes care of itself.
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
    // Work on an uppercased copy so the field/label search is case-insensitive.
    // toUpperCase() is 1:1 over ASCII, so indices still line up, and digits are
    // unaffected by the conversion.
    String u = line;
    u.toUpperCase();

    int ai = u.indexOf("A=");
    if (ai < 0)
        return false;

    // Search for B's field only after A's, so the "A" in "A=A:" can never be
    // mistaken for the start of the B field (or vice versa).
    int bi = u.indexOf("B=", ai + 2);
    if (bi < 0)
        return false;

    float a = parseRangeField(u, ai + 2, bi);
    float b = parseRangeField(u, bi + 2, u.length());

    // The line was addressed to us either way — a bad reading gets dropped
    // here, not handed back to the command parser as a stray number.
    if (a <= 0.05f || a > 50.0f || b <= 0.05f || b > 50.0f)
        return true;

    // |A - B| can never exceed the baseline geometrically; anything past it
    // is a multipath/dropout artefact, not a bearing.
    if (fabs(a - b) > beaconBaseline * 1.6f)
        return true;

    if (sampleCount == 0)
    {
        filtA = a;
        filtB = b;
    }
    else
    {
        filtA += FILTER_ALPHA * (a - filtA);
        filtB += FILTER_ALPHA * (b - filtB);
    }

    if (sampleCount < 1000)
        sampleCount++;
    lastSampleMs = millis();

    // Solve the triangle (see the geometry note at the top of this file).
    float halfD = beaconBaseline * 0.5f;
    float r2 = (filtA * filtA + filtB * filtB) * 0.5f - halfD * halfD;
    if (r2 < 0.0001f)
        r2 = 0.0001f;

    beaconRange = sqrtf(r2);

    float c = (filtB * filtB - filtA * filtA) / (2.0f * beaconRange * beaconBaseline);
    c = constrain(c, -1.0f, 1.0f);
    beaconTheta = acosf(c) * 180.0f / PI;

    // ── Infer which side the beacon is on ──
    // Only a rotation carries information: standing still, both hypotheses
    // predict the same next reading and the test is silent by construction.
    if (haveLastFix && fabs(rotSinceFix) >= SIGN_MIN_ROT)
    {
        float predRight = fabs(thetaAtLastFix - rotSinceFix);
        float predLeft = fabs(thetaAtLastFix + rotSinceFix);

        float errRight = fabs(beaconTheta - predRight);
        float errLeft = fabs(beaconTheta - predLeft);

        int evidence = 0;
        if (errRight < errLeft - SIGN_MARGIN_DEG)
            evidence = 1;
        else if (errLeft < errRight - SIGN_MARGIN_DEG)
            evidence = -1;

        if (evidence != 0)
        {
            if (signConfidence == 0)
            {
                turnSign = evidence;
                signConfidence = 1;
                if (followDebug)
                {
                    Serial.print("FOLLOW: side = ");
                    Serial.println(turnSign > 0 ? "RIGHT" : "LEFT");
                }
            }
            else if (evidence == turnSign)
            {
                // Agreeing evidence buys confidence, capped — so a locked-in
                // side survives the occasional bad fix instead of reversing.
                if (signConfidence < SIGN_CONF_MAX)
                    signConfidence++;
            }
            else
            {
                signConfidence--;
                if (signConfidence <= 0)
                {
                    turnSign = evidence;
                    signConfidence = 1;
                    if (followDebug)
                    {
                        Serial.print("FOLLOW: side flipped to ");
                        Serial.println(turnSign > 0 ? "RIGHT" : "LEFT");
                    }
                }
            }
        }
    }

    // Re-anchor the tracked bearing on the measurement: the sensor owns the
    // magnitude, the inference owns the sign, and trackRotation() carries it
    // forward between fixes.
    bearingSigned = turnSign * beaconTheta;
    thetaAtLastFix = beaconTheta;
    rotSinceFix = 0.0f;
    haveLastFix = true;

    return true;
}

// ─── Public API ────────────────────────────────────────

void Follow_Init()
{
    state = FOLLOW_OFF;
    resetFilter();
    resetTracking();
    lastSampleMs = 0;
    turnSign = 1;
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

    resetFilter();
    resetTracking();
    camStopReported = false;
    state = FOLLOW_ACQUIRE;

    Serial.println("FOLLOW: --- FOLLOW-ME ARMED --- waiting for beacon data");
    Follow_PrintConfig();
}

void Follow_Cancel()
{
    if (state == FOLLOW_OFF)
        return;

    followMotion(0, 0);
    state = FOLLOW_OFF;
    resetFilter();
    resetTracking();
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
    Serial.print("FOLLOW: baseline d=");
    Serial.print(beaconBaseline, 2);
    Serial.print(" m | standoff=");
    Serial.print(standoffRange, 2);
    Serial.print(" m | resume=");
    Serial.print(standoffRange + RANGE_HYSTERESIS, 2);
    Serial.print(" m | data ");
    if (dataValid())
    {
        Serial.print("A=");
        Serial.print(filtA, 2);
        Serial.print(" B=");
        Serial.print(filtB, 2);
        Serial.print(" R=");
        Serial.print(beaconRange, 2);
        Serial.print(" theta=");
        Serial.println(beaconTheta, 0);
    }
    else
    {
        Serial.println("(none yet)");
    }
}

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

    String upper = line;
    upper.toUpperCase();

    if (upper.startsWith("FOLLOWD "))
    {
        float v = line.substring(8).toFloat();
        if (v >= 0.05f && v <= 5.0f)
        {
            beaconBaseline = v;
            resetFilter();
            Serial.print("FOLLOW: baseline set to ");
            Serial.print(beaconBaseline, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: baseline must be 0.05-5.00 m");
        }
        return true;
    }

    if (upper.startsWith("FOLLOWR "))
    {
        float v = line.substring(8).toFloat();
        if (v >= 0.20f && v <= 10.0f)
        {
            standoffRange = v;
            Serial.print("FOLLOW: standoff set to ");
            Serial.print(standoffRange, 2);
            Serial.println(" m");
        }
        else
        {
            Serial.println("FOLLOW: standoff must be 0.20-10.00 m");
        }
        return true;
    }

    return false;
}

// ─── Update (called every loop tick) ───────────────────

void Follow_Update()
{
    if (state == FOLLOW_OFF)
    {
        // Monitor mode: live bearing readout with the motors provably idle,
        // because nothing below this point ever runs.  This is how you check
        // that A and B are not swapped before letting the robot drive.
        if (followMonitor && dataValid() &&
            (lastSampleMs != 0) && (millis() - lastSampleMs <= DATA_TIMEOUT_MS))
            printLiveFix();
        return;
    }

    // ── Pause ('p', or a camera pause) must stop follow-me too ──
    // Drive_Update() already forces motion(0) every tick while paused, but it
    // runs BEFORE Follow_Update() in loop(), so without this the follow-me
    // drive command issued here would simply overwrite that stop on the same
    // tick and the robot would keep going. Freezing here instead leaves the
    // state machine intact, so resuming picks up mid-step rather than
    // restarting the approach.
    if (Drive_IsPaused())
    {
        followMotion(0, 0);
        return;
    }

    // ── Beacon link lost: stop and wait, never coast blind ──
    bool stale = (lastSampleMs == 0) || (millis() - lastSampleMs > DATA_TIMEOUT_MS);
    if (stale)
    {
        followMotion(0, 0);
        if (state != FOLLOW_ACQUIRE)
        {
            Serial.println("FOLLOW: beacon data lost — stopping");
            resetFilter();
            state = FOLLOW_ACQUIRE;
        }
        return;
    }

    // Filter still refilling (fresh start, or flushed for a settle) — hold.
    if (!dataValid())
    {
        followMotion(0, 0);
        return;
    }

    // Carry the bearing forward through the robot's own rotation before any
    // decision is taken from it.
    trackRotation();

    // ── Periodic debug ──
    if (followDebug)
        printLiveFix();

    switch (state)
    {
    // ── ACQUIRE: first usable fix — decide what to do with it ──
    case FOLLOW_ACQUIRE:
    {
        Serial.print("FOLLOW: beacon acquired (R=");
        Serial.print(beaconRange, 2);
        Serial.print(" m, theta=");
        Serial.print(beaconTheta, 0);
        Serial.println(" deg)");

        // A bearing magnitude alone cannot say left from right. If the side is
        // not yet known, earn it once with a probe; otherwise go straight to
        // driving and never stop to measure again.
        if (signConfidence == 0 && beaconTheta > ALIGN_TOL)
            beginProbe();
        else
            chooseNextAction();
        break;
    }

    // ── PROBE: rotate a known amount so the side test has something to read ──
    // Reached only while the side is unknown.  The evidence itself is gathered
    // in Follow_IngestLine as fixes arrive, so there is nothing to judge here.
    case FOLLOW_TURN_STEP:
    {
        float turned = fabs(normalizeAngle(currentHeading - stepStartHeading));
        float remaining = stepSize - turned;

        if (remaining <= 0.0f)
        {
            followMotion(0, 0);
            beginSettle(true);
        }
        else
        {
            followMotion(turnCode(remaining, turnSign), turnSign);
        }
        break;
    }

    // ── SETTLE: motors off, clean stationary readings, then commit ──
    // A rotating robot smears the range EMA, so the probe's verdict is taken
    // from readings captured while genuinely stopped.  This is the only place
    // follow-me waits, and it is skipped entirely once the side is known.
    case FOLLOW_SETTLE:
    {
        followMotion(0, 0);

        // Phase 1: let the PWM ramp-down finish before trusting anything.
        if (!settleFilterCleared)
        {
            if (millis() - settleStart < BRAKE_MS)
                break;

            resetFilter(); // only readings taken while genuinely stopped count
            settleFilterCleared = true;
            settleStart = millis(); // restart the clock for the sampling window
            break;
        }

        // Phase 2: wait for a full window of stationary readings.
        if ((millis() - settleStart < SETTLE_MS) || (sampleCount < SETTLE_SAMPLES))
            break;

        if (settleJudgesStep && signConfidence == 0)
        {
            // The probe rotated but the two hypotheses still predict readings
            // too close to separate — usually the beacon is near dead ahead or
            // dead astern, where the geometry is genuinely least informative.
            // Probe again the other way rather than committing to a guess.
            turnSign = -turnSign;
            Serial.println("FOLLOW: side still unclear — probing the other way");
            beginProbe();
            break;
        }

        chooseNextAction();
        break;
    }

    // ── DRIVE: aligned — close on the beacon ──
    case FOLLOW_DRIVE:
    {
        // The front camera keeps its veto on forward motion.
        if (!camBypass && frontCamLocked)
        {
            followMotion(0, 0);
            if (!camStopReported)
            {
                camStopReported = true;
                Serial.println("FOLLOW: front camera blocked — waiting");
            }
            break;
        }
        camStopReported = false;

        if (beaconRange <= standoffRange)
        {
            followMotion(0, 0);
            state = FOLLOW_HOLD;
            Serial.print("FOLLOW: reached standoff (");
            Serial.print(beaconRange, 2);
            Serial.println(" m) — holding");
            break;
        }

        // Lost the side (a flip spent the last confidence point) — the only
        // thing worth stopping for.  Everything else is handled on the move.
        if (signConfidence == 0 && fabs(bearingSigned) > ALIGN_TOL)
        {
            Serial.println("FOLLOW: side lost — re-probing");
            beginProbe();
            break;
        }

        // Steer continuously: straight, curve, or spin, decided from the
        // IMU-tracked bearing every tick.  No stopping to re-measure.
        steerToward();
        break;
    }

    // ── HOLD: inside the standoff — stay put, stay facing the beacon ──
    case FOLLOW_HOLD:
    {
        if (beaconRange >= standoffRange + RANGE_HYSTERESIS)
        {
            Serial.println("FOLLOW: beacon moving away — resuming");
            chooseNextAction();
            break;
        }

        if (fabs(bearingSigned) >= REALIGN_TOL)
        {
            if (signConfidence == 0)
            {
                Serial.println("FOLLOW: re-facing beacon — side unknown, probing");
                beginProbe();
                break;
            }

            // Stopped and confident: rotate on the spot until the tracked
            // bearing comes back inside tolerance.
            int s = (bearingSigned >= 0.0f) ? 1 : -1;
            followMotion(turnCode(fabs(bearingSigned), s), s);
            break;
        }

        followMotion(0, 0);
        break;
    }

    default:
        break;
    }
}
