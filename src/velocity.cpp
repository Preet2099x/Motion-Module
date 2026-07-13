#include "velocity.h"

#include <Arduino.h>
#include <math.h>

#include "distance.h"
#include <var.h>

VelocityData velocity;

static float previousDistance = 0.0f;
static float previousHeading = 0.0f;

void Velocity_Init()
{
    Velocity_Reset();
}

void Velocity_Reset()
{
    velocity.linear = 0.0f;
    velocity.angular = 0.0f;

    previousDistance = distance.travelled;
    previousHeading = currentHeading;
}

void Velocity_Update(float dt)
{
    if (dt <= 0.0f)
        return;

    float deltaDistance = distance.travelled - previousDistance;

    float deltaHeading = currentHeading - previousHeading;

    while (deltaHeading > 180.0f)
        deltaHeading -= 360.0f;

    while (deltaHeading < -180.0f)
        deltaHeading += 360.0f;

    if (deltaDistance < 0.0005f)
    {
        velocity.linear = 0.0f;
        velocity.angular = 0.0f;
    }
    else
    {
        velocity.linear = deltaDistance / dt;

        velocity.angular =
            (deltaHeading * PI / 180.0f) / dt;
    }

    previousDistance = distance.travelled;
    previousHeading = currentHeading;
}