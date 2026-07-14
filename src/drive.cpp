#include "drive.h"
#include "distance.h"
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
    DRIVE_COMPLETE
};

static DriveState state = DRIVE_IDLE;

// Per-command one-shot capture flag
static bool commandStarted = false;
static float startDistance = 0.0f;
static float startHeading = 0.0f;



// ─── Tuning ────────────────────────────────────────────

static const float TURN_TOLERANCE  = 2.0f;   // degrees — stop within this of target
static const float TURN_SLOW_ZONE  = 20.0f;  // degrees — slow down when this close

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

// Reset all per-command tracking state
static void resetCommandState()
{
    commandStarted = false;
    startDistance = 0.0f;
    startHeading = 0.0f;
}

// Advance to the next queued command
static void advanceQueue()
{
    resetCommandState();
    queueIndex++;

    if (queueIndex >= queueLength)
    {
        // Task complete
        motion(0);
        state = DRIVE_IDLE;
        Serial.println("DRIVE: Task complete");
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
    return (state == DRIVE_RUNNING);
}

void Drive_Cancel()
{
    if (state == DRIVE_RUNNING)
    {
        motion(0); // Ensure motors are stopped
        queueLength = 0;
        queueIndex  = 0;
        state       = DRIVE_IDLE;
        resetCommandState();
        Serial.println("DRIVE: Task cancelled (emergency)");
    }
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
        state = DRIVE_RUNNING;
        Serial.print("DRIVE: Queued ");
        Serial.print(queueLength);
        Serial.println(" command(s)");
    }
    else
    {
        Serial.println("DRIVE: No valid commands parsed");
    }
}

// ─── Update (called every loop tick) ───────────────────

void Drive_Update()
{
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
    // On this robot, motion(11-20) causes BNO heading to increase.
    // delta = normalizeAngle(currentHeading - startHeading)
    // Positive delta = heading increased = turned left.
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

        float delta = normalizeAngle(startHeading - currentHeading);
        float remaining = cmd.value - delta;

        if (remaining <= TURN_TOLERANCE)
        {
            motion(0);
            Serial.print("DRIVE: Left done (delta=");
            Serial.print(delta, 1);
            Serial.println(")");
            advanceQueue();
        }
        else if (remaining <= TURN_SLOW_ZONE)
        {
            motion(21); // slowest left turn
        }
        else
        {
            motion(25); // mid-range left turn
        }
        break;
    }

    // ── RIGHT TURN ───────────────────────────────────
    // On this robot, motion(21-30) causes BNO heading to decrease.
    // delta = normalizeAngle(startHeading - currentHeading)
    // Positive delta = heading decreased = turned right.
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

        float delta = normalizeAngle(currentHeading - startHeading);
        float remaining = cmd.value - delta;

        if (remaining <= TURN_TOLERANCE)
        {
            motion(0);
            Serial.print("DRIVE: Right done (delta=");
            Serial.print(delta, 1);
            Serial.println(")");
            advanceQueue();
        }
        else if (remaining <= TURN_SLOW_ZONE)
        {
            motion(11); // slowest right turn
        }
        else
        {
            motion(15); // mid-range right turn
        }
        break;
    }

    // ── STOP ─────────────────────────────────────────
    case CMD_STOP:
    {
        motion(0);
        Serial.println("DRIVE: Stop");
        // Stop ends the task entirely
        queueLength = 0;
        queueIndex = 0;
        state = DRIVE_IDLE;
        resetCommandState();
        break;
    }
    }
}
