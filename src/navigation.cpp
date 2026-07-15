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
        float dx = -odometry.x;
        float dy = -odometry.y;

        float distance = sqrtf(dx * dx + dy * dy);

        float targetHeading = atan2f(dy, dx);

        float headingError = targetHeading - odometry.heading;

        while (headingError > PI)
            headingError -= 2.0f * PI;

        while (headingError < -PI)
            headingError += 2.0f * PI;

        Serial.print("NAV: Dist=");
        Serial.print(distance, 3);

        Serial.print(" Target=");
        Serial.print(targetHeading * 180.0f / PI, 1);

        Serial.print(" Heading=");
        Serial.print(odometry.heading * 180.0f / PI, 1);

        Serial.print(" Error=");
        Serial.println(headingError * 180.0f / PI, 1);

        state = NAV_IDLE;
    }
}
