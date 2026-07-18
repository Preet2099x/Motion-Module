#include <Arduino.h>
#include <math.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"

// ─── State ─────────────────────────────────────────────

enum NavigationState
{
    NAV_IDLE,
    NAV_WAIT  // Drive is executing the submitted batch; we just wait
};

static NavigationState state = NAV_IDLE;

// ─── Helpers ───────────────────────────────────────────

static float normalizeAngle(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

static float calculateHeadingToHome()
{
    // Physical forward = heading + 180°, so to drive *toward* home the robot
    // must face the direction pointing *away* from home.
    // atan2(y, x) gives the angle of the vector FROM home TO robot — exactly
    // the heading to turn to so that physical forward goes toward (0, 0).
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
    state = NAV_IDLE;
}

void Navigation_ReturnHome()
{
    if (state != NAV_IDLE)
        return;

    float dx = -odometry.x;
    float dy = -odometry.y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist < 0.10f)
    {
        Serial.println("NAVIGATION: Already at home");
        return;
    }

    float targetHeading = calculateHeadingToHome();

    // Build the full command sequence and submit it in one shot.
    // CMD_TURN_TO_ABS values are resolved against odometry.heading at
    // execution time, so the return-to-0 turn is always correct even
    // after drive drift.
    DriveCommand cmds[3];

    cmds[0].type  = CMD_TURN_TO_ABS;
    cmds[0].value = targetHeading; // face home

    cmds[1].type  = CMD_FORWARD;
    cmds[1].value = dist;          // drive home

    cmds[2].type  = CMD_TURN_TO_ABS;
    cmds[2].value = 0.0f;          // restore start heading

    if (Drive_SubmitTask(cmds, 3))
    {
        Serial.println("NAVIGATION: Returning home");
        state = NAV_WAIT;
    }
}

void Navigation_Update()
{
    if (state == NAV_IDLE)
        return;

    // NAV_WAIT: nothing to do but watch for completion
    if (!Drive_IsRunning())
    {
        Serial.println("NAVIGATION: Home reached");
        state = NAV_IDLE;
    }
}
