#include <Arduino.h>
#include <math.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"

// ─── State ─────────────────────────────────────────────

enum NavigationState
{
    NAV_IDLE,
    NAV_WAIT_SEGMENT_1,
    NAV_WAIT_SEGMENT_2
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

    if (dist < 0.05f)
    {
        Serial.println("NAVIGATION: Already at home");
        return;
    }

    float targetHeading = calculateHeadingToHome();

    DriveCommand cmds[2];
    cmds[0].type  = CMD_TURN_TO_ABS;
    cmds[0].value = targetHeading; // face home

    cmds[1].type  = CMD_FORWARD;
    if (dist > 0.5f) {
        cmds[1].value = 0.5f;   // drive 0.5m segment
    } else {
        cmds[1].value = dist;   // drive remaining distance
    }

    if (Drive_SubmitTask(cmds, 2))
    {
        Serial.println("NAVIGATION: Returning home (Segment 1)");
        state = NAV_WAIT_SEGMENT_1;
    }
}

void Navigation_Update()
{
    if (state == NAV_IDLE)
        return;

    if (!Drive_IsRunning())
    {
        if (state == NAV_WAIT_SEGMENT_1)
        {
            float dx = -odometry.x;
            float dy = -odometry.y;
            float dist = sqrtf(dx * dx + dy * dy);
            
            if (dist > 0.05f)
            {
                float targetHeading = calculateHeadingToHome();
                
                DriveCommand cmds[2];
                cmds[0].type  = CMD_TURN_TO_ABS;
                cmds[0].value = targetHeading; // face home again
                
                cmds[1].type  = CMD_FORWARD;
                if (dist > 0.5f) {
                    cmds[1].value = 0.5f;
                } else {
                    cmds[1].value = dist;
                }

                if (Drive_SubmitTask(cmds, 2))
                {
                    Serial.println("NAVIGATION: Returning home (Next Segment)");
                    state = NAV_WAIT_SEGMENT_1; // Loop in this state
                }
                else
                {
                    state = NAV_IDLE;
                }
            }
            else // Reached home, now align
            {
                DriveCommand cmds[1];
                cmds[0].type  = CMD_TURN_TO_ABS;
                cmds[0].value = 0.0f;          // align to zero

                if (Drive_SubmitTask(cmds, 1))
                {
                    Serial.println("NAVIGATION: Home reached, aligning");
                    state = NAV_WAIT_SEGMENT_2;
                }
                else
                {
                    state = NAV_IDLE;
                }
            }
        }
        else if (state == NAV_WAIT_SEGMENT_2)
        {
            Serial.println("NAVIGATION: Return home complete");
            state = NAV_IDLE;
        }
    }
}
