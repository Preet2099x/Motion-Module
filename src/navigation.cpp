#include <Arduino.h>
#include <math.h>

#include "navigation.h"
#include "drive.h"
#include "odometry.h"

enum NavigationState
{
    NAV_IDLE,
    NAV_RETURN_HOME
};

static NavigationState state = NAV_IDLE;
static bool commandSent = false;

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

    Serial.println("NAVIGATION: Returning home");

    state = NAV_RETURN_HOME;
}

void Navigation_Update()
{
    if (state == NAV_IDLE)
        return;

    if (state == NAV_RETURN_HOME)
    {
        // Wait until the previous drive command finishes
        if (Drive_IsRunning())
            return;

        float dx = -odometry.x;
        float dy = -odometry.y;

        float distance = sqrtf(dx * dx + dy * dy);

        // Already at home?
        if (distance < 0.10f)
        {
            Serial.println("NAVIGATION: Home reached");
            state = NAV_IDLE;
            commandSent = false;
            return;
        }

        float targetHeading = atan2f(dy, dx);
        float headingError = targetHeading - odometry.heading;

        while (headingError > PI)
            headingError -= 2.0f * PI;

        while (headingError < -PI)
            headingError += 2.0f * PI;

        // If we're already facing home (within 3° leeway), don't issue another turn.
        if (fabsf(headingError) < (3.0f * PI / 180.0f))
        {
            Serial.println("NAVIGATION: Facing home");
            return;
        }

        DriveCommand cmd;

        if (headingError > 0)
            cmd.type = CMD_LEFT;
        else
            cmd.type = CMD_RIGHT;

        cmd.value = fabsf(headingError) * 180.0f / PI;

        Drive_SubmitTask(&cmd, 1);

        Serial.print("NAVIGATION: Turn ");
        Serial.print(cmd.value, 1);
        Serial.println(" deg");
    }
}
