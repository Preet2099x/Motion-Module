#include "drive.h"
#include "distance.h"
#include "navigation.h"
#include "odometry.h"
#include <var.h>

// ─── Internal state ────────────────────────────────────

static const int QUEUE_MAX = 32;

static DriveCommand queue[QUEUE_MAX];
static int queueLength = 0;
static int queueIndex = 0;

enum DriveState
{
    DRIVE_IDLE,
    DRIVE_RUNNING,
    DRIVE_SETTLING,  // brief cooldown between non-continuous commands
    DRIVE_COMPLETE
};

static DriveState state = DRIVE_IDLE;

// ─── Pause state ──────────────────────────────────────
static bool drivePaused = false;
static int  pausedData  = 0;   // manual motion command saved at pause time

// Settling (cooldown between non-continuous commands)
static const unsigned long SETTLE_DURATION_MS = 300; // ms to hold motors at 0 between unlike commands
static unsigned long settleStartTime = 0;

// Per-command one-shot capture flag
static bool commandStarted = false;
static float startDistance = 0.0f;
static float startHeading = 0.0f;
static float plannedTurnDelta = 0.0f; // used by CMD_TURN_TO_ABS
static float turnAccum = 0.0f;        // signed degrees turned so far (CMD_LEFT/RIGHT)

// ─── Tuning ────────────────────────────────────────────

static const float TURN_TOLERANCE = 1.0f;  // degrees — stop within this of target (extreme precision)
static const float TURN_CRAWL_ZONE = 5.0f; // degrees — use crawl speed in this zone
static const float TURN_SLOW_ZONE  = 15.0f; // degrees — use slow speed in this zone

// ─── Helpers ───────────────────────────────────────────

// Normalize angle to [-180, +180]
static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

// Two consecutive commands are "continuous" (no settling needed) when they
// share the same motion category.  Categories:
//   linear   — CMD_FORWARD, CMD_BACKWARD  (motors already spinning the right way)
//   turn     — CMD_LEFT, CMD_RIGHT        (both are in-place rotations)
// Any other transition (turn→linear, linear→turn, or involving STOP/
// TURN_TO_ABS) is non-continuous and needs a brief motor cooldown so
// motion.cpp's start-ramp fires properly on the next command.
static bool isContinuousTransition(DriveCommandType prev, DriveCommandType next)
{
    // linear → linear
    if ((prev == CMD_FORWARD || prev == CMD_BACKWARD) &&
        (next == CMD_FORWARD || next == CMD_BACKWARD))
        return true;

    // turn → turn (includes CMD_TURN_TO_ABS)
    if ((prev == CMD_LEFT || prev == CMD_RIGHT || prev == CMD_TURN_TO_ABS) &&
        (next == CMD_LEFT || next == CMD_RIGHT || next == CMD_TURN_TO_ABS))
        return true;

    return false;
}

// Reset all per-command tracking state
static void resetCommandState()
{
    commandStarted = false;
    startDistance = 0.0f;
    startHeading = 0.0f;
    plannedTurnDelta = 0.0f;
    turnAccum = 0.0f;
}

// Degrees turned so far in the current L/R command, whichever way the IMU
// counts. Summing tick-to-tick changes means it never wraps at ±180, so
// L90 stops at 90 and L270 at 270. startHeading holds the previous tick.
static float turnedSoFar()
{
    turnAccum += normalizeAngle(currentHeading - startHeading);
    startHeading = currentHeading;
    return fabs(turnAccum);
}

// Advance to the next queued command.  If the upcoming command is a
// different motion category (non-continuous), enter a brief settling
// period first so the motors fully stop and motion.cpp's lastCommand
// resets to 0 — this lets the start-ramp fire for the next command
// instead of jumping straight to full PWM.
static void advanceQueue()
{
    DriveCommandType finishedType = queue[queueIndex].type;
    resetCommandState();
    queueIndex++;

    if (queueIndex >= queueLength)
    {
        // Task complete
        motion(0);
        state = DRIVE_IDLE;
        Serial.println("DRIVE: Task complete");
        return;
    }

    // Check if the transition to the next command is continuous
    DriveCommandType nextType = queue[queueIndex].type;

    if (!isContinuousTransition(finishedType, nextType))
    {
        // Non-continuous: settle motors before starting the next command
        motion(0);
        settleStartTime = millis();
        state = DRIVE_SETTLING;
        Serial.println("DRIVE: Settling between commands (300 ms)");
    }
}

// ─── Public API ────────────────────────────────────────

void Drive_Init()
{
    queueLength = 0;
    queueIndex = 0;
    state = DRIVE_IDLE;
    resetCommandState();
}

bool Drive_IsRunning()
{
    return (state == DRIVE_RUNNING || state == DRIVE_SETTLING);
}

void Drive_Cancel()
{
    if (state == DRIVE_RUNNING || state == DRIVE_SETTLING)
    {
        motion(0); // Ensure motors are stopped
        queueLength = 0;
        queueIndex = 0;
        state = DRIVE_IDLE;
        resetCommandState();
        Serial.println("DRIVE: Task cancelled (emergency)");
    }
    // Also clear pause so a fresh task can start cleanly
    drivePaused = false;
    pausedData  = 0;
}

// ─── Pause / Resume ──────────────────────────────────────

bool Drive_IsPaused()
{
    return drivePaused;
}

int Drive_GetSavedData()
{
    return pausedData;
}

// Pause: stop motors immediately, freeze task, save manual data
void Drive_Pause(int savedData)
{
    if (drivePaused)
        return; // already paused

    drivePaused = true;
    pausedData  = savedData;
    motion(0);  // kill motors immediately

    // commandStarted/startDistance/startHeading/plannedTurnDelta are left
    // untouched so the in-progress command keeps its original baseline —
    // on resume it continues counting from where it left off instead of
    // restarting (which would silently add the pre-pause distance/turn
    // on top of the target again).

    Serial.println("DRIVE: --- PAUSED --- (motors stopped, state saved)");
}

// Resume: un-freeze the task; caller restores the returned manual data
void Drive_Resume()
{
    if (!drivePaused)
        return; // not paused

    drivePaused = false;
    Serial.println("DRIVE: --- RESUMED ---");
}

// Toggle: one call pauses, next call resumes
void Drive_TogglePause(int currentData)
{
    if (drivePaused)
        Drive_Resume();
    else
        Drive_Pause(currentData);
}

bool Drive_SubmitTask(const DriveCommand *commands, int count)
{
    if (state == DRIVE_RUNNING)
        return false;

    if (count <= 0)
        return false;

    if (count > QUEUE_MAX)
        count = QUEUE_MAX;

    queueLength = count;
    queueIndex = 0;
    state = DRIVE_RUNNING;

    resetCommandState();

    for (int i = 0; i < count; i++)
        queue[i] = commands[i];

    Serial.print("DRIVE: Queued ");
    Serial.print(queueLength);
    Serial.println(" command(s)");
    

    return true;
}

// ─── Parse ─────────────────────────────────────────────

void Drive_ParseTask(const String &raw)
{
    // ── Reset queue before loading a new task ──
    queueLength = 0;
    queueIndex = 0;
    state = DRIVE_IDLE;
    resetCommandState();

    // Work on a mutable copy
    String input = raw;
    input.toUpperCase();
    input.trim();

    // Strip leading "T:"
    if (input.startsWith("T:"))
        input = input.substring(2);

    // Split by commas and parse each token
    int start = 0;
    while (start < (int)input.length() && queueLength < QUEUE_MAX)
    {
        int comma = input.indexOf(',', start);
        if (comma == -1)
            comma = input.length();

        String token = input.substring(start, comma);
        token.trim();

        if (token.length() >= 1)
        {
            char code = token.charAt(0);
            float value = 0.0f;

            if (token.length() > 1)
                value = token.substring(1).toFloat();

            DriveCommand cmd;
            bool valid = true;

            switch (code)
            {
            case 'F':
                cmd.type = CMD_FORWARD;
                cmd.value = value;
                break;
            case 'B':
                cmd.type = CMD_BACKWARD;
                cmd.value = value;
                break;
            case 'L':
                cmd.type = CMD_LEFT;
                cmd.value = value;
                break;
            case 'R':
                cmd.type = CMD_RIGHT;
                cmd.value = value;
                break;
            case 'S':
                cmd.type = CMD_STOP;
                cmd.value = 0;
                break;
            default:
                valid = false;
                Serial.print("DRIVE: Unknown token '");
                Serial.print(token);
                Serial.println("' — skipped");
                break;
            }

            if (valid)
                queue[queueLength++] = cmd;
        }

        start = comma + 1;
    }

    if (queueLength > 0)
    {
        Drive_SubmitTask(queue, queueLength);
    }
    else
    {
        Serial.println("DRIVE: No valid commands parsed");
    }
}

// ─── Update (called every loop tick) ───────────────────

void Drive_Update()
{
    if (drivePaused)
    {
        motion(0); // actively flush ramp state every tick
        return;
    }

    // ── SETTLING: hold motors at zero for SETTLE_DURATION_MS ──
    if (state == DRIVE_SETTLING)
    {
        motion(0); // keep flushing ramp state every tick

        if (millis() - settleStartTime >= SETTLE_DURATION_MS)
        {
            state = DRIVE_RUNNING;
            Serial.println("DRIVE: Settle complete, next command");
        }
        return;
    }

    if (state != DRIVE_RUNNING)
        return;

    if (queueIndex >= queueLength)
    {
        motion(0);
        state = DRIVE_IDLE;
        return;
    }

    DriveCommand &cmd = queue[queueIndex];

    switch (cmd.type)
    {
    // ── FORWARD ──────────────────────────────────────
    case CMD_FORWARD:
    {
        if (!commandStarted)
        {
            startDistance = distance.travelled;
            commandStarted = true;
            Serial.print("DRIVE: Forward ");
            Serial.print(cmd.value);
            Serial.println(" m");
        }

        float travelled = distance.travelled - startDistance;

        if (travelled >= cmd.value)
        {
            motion(0);
            Serial.println("DRIVE: Forward done");
            advanceQueue();
        }
        else
        {
            motion(1);
        }
        break;
    }

    // ── BACKWARD ─────────────────────────────────────
    case CMD_BACKWARD:
    {
        if (!commandStarted)
        {
            startDistance = distance.travelled;
            commandStarted = true;
            Serial.print("DRIVE: Backward ");
            Serial.print(cmd.value);
            Serial.println(" m");
        }

        float travelled = distance.travelled - startDistance;

        if (travelled >= cmd.value)
        {
            motion(0);
            Serial.println("DRIVE: Backward done");
            advanceQueue();
        }
        else
        {
            motion(2);
        }
        break;
    }

    // ── LEFT TURN ────────────────────────────────────
    
    case CMD_LEFT:
    {
        if (!commandStarted)
        {
            startHeading = currentHeading;
            commandStarted = true;
            Serial.print("DRIVE: Left ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }
        
        float delta = turnedSoFar();
        float remaining = cmd.value - delta;

        if (remaining <= TURN_TOLERANCE)
        {
            motion(0);
            Serial.print("DRIVE: Left done (delta=");
            Serial.print(delta, 1);
            Serial.println(")");
            advanceQueue();
        }
        else if (remaining <= TURN_CRAWL_ZONE)
        {
            motion(21); // crawl — slowest left turn
        }
        else if (remaining <= TURN_SLOW_ZONE)
        {
            motion(23); // slow left turn
        }
        else
        {
            motion(25); // mid-range left turn
        }
        break;
    }

    // ── RIGHT TURN ───────────────────────────────────

    case CMD_RIGHT:
    {
        if (!commandStarted)
        {
            startHeading = currentHeading;
            commandStarted = true;
            Serial.print("DRIVE: Right ");
            Serial.print(cmd.value);
            Serial.println(" deg");
        }

        float delta = turnedSoFar();
        float remaining = cmd.value - delta;

        if (remaining <= TURN_TOLERANCE)
        {
            motion(0);
            Serial.print("DRIVE: Right done (delta=");
            Serial.print(delta, 1);
            Serial.println(")");
            advanceQueue();
        }
        else if (remaining <= TURN_CRAWL_ZONE)
        {
            motion(11); // crawl — slowest right turn
        }
        else if (remaining <= TURN_SLOW_ZONE)
        {
            motion(13); // slow right turn
        }
        else
        {
            motion(15); // mid-range right turn
        }
        break;
    }

    // ── TURN TO ABSOLUTE HEADING ─────────────────────
    case CMD_TURN_TO_ABS:
    {
        if (!commandStarted)
        {
            // Compute signed delta in odometry space once, at execution time
            plannedTurnDelta = normalizeAngle(cmd.value - odometry.heading);
            commandStarted = true;
            Serial.print("DRIVE: TurnToAbs ");
            Serial.print(cmd.value, 1);
            Serial.print(" deg (delta=");
            Serial.print(plannedTurnDelta, 1);
            Serial.println(")");
        }

        // Track progress: positive delta = right, negative = left
        float current = normalizeAngle(odometry.heading - (cmd.value - plannedTurnDelta));
        float remaining = plannedTurnDelta - current;

        if (fabs(remaining) <= TURN_TOLERANCE)
        {
            motion(0);
            Serial.print("DRIVE: TurnToAbs done (remaining=");
            Serial.print(remaining, 1);
            Serial.println(")");
            advanceQueue();
        }
        else if (remaining > 0) // need to turn right
        {
            if (remaining <= TURN_CRAWL_ZONE)      motion(11);
            else if (remaining <= TURN_SLOW_ZONE)  motion(13);
            else                                    motion(15);
        }
        else // need to turn left
        {
            float absRem = -remaining;
            if (absRem <= TURN_CRAWL_ZONE)         motion(21);
            else if (absRem <= TURN_SLOW_ZONE)     motion(23);
            else                                   motion(25);
        }
        break;
    }

    // ── STOP ─────────────────────────────────────────
    case CMD_STOP:
    {
        motion(0);
        Serial.println("DRIVE: Stop");
        // Stop ends the task entirely — also cancel any running navigation
        // so that after an emergency unlatch the robot stays idle.
        queueLength = 0;
        queueIndex = 0;
        state = DRIVE_IDLE;
        resetCommandState();
        Navigation_Cancel();
        break;
    }
    }
}
