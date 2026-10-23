#include "odometry.h"

#include <Arduino.h>
#include <math.h>

#include "distance.h"
#include <var.h>

OdometryData odometry;

static float previousDistance = 0.0f;
static float headingOffset = 0.0f;

void Odometry_Init()
{
    Odometry_Reset();
}

void Odometry_Reset()
{
    odometry.x = 0.0f;
    odometry.y = 0.0f;
    odometry.heading = 0.0f;

    odometry.homeX = 0.0f;
    odometry.homeY = 0.0f;
    odometry.homeHeading = 0.0f;

    previousDistance = distance.travelled;
    headingOffset = currentHeading;
}

void Odometry_Update()
{
    float deltaDistance = distance.odometryTravelled - previousDistance;

    // Raw BNO heading (never changes)
    odometry.rawHeading = currentHeading;

    // Relative heading (0 after Home)
    // Navigation and Drive's TURN_TO_ABS expect this to INCREASE on the
    // 11-20 turn codes. currentHeading is mirrored by HEADING_INVERT (for the
    // straight-line PID), which makes it fall on those turns, so mirror it
    // back here. Only odometry/navigation see this; the PID is unaffected.
    odometry.heading = headingOffset - currentHeading;

    while (odometry.heading >= 360.0f)
        odometry.heading -= 360.0f;

    while (odometry.heading < 0.0f)
        odometry.heading += 360.0f;

    float headingRad = odometry.heading * PI / 180.0f;

    odometry.x += deltaDistance * cosf(headingRad);
    odometry.y += deltaDistance * sinf(headingRad);

    previousDistance = distance.odometryTravelled;
}

void Odometry_SetHome()
{
    odometry.x = 0.0f;
    odometry.y = 0.0f;
    odometry.heading = 0.0f;

    odometry.homeX = 0.0f;
    odometry.homeY = 0.0f;
    odometry.homeHeading = 0.0f;

    previousDistance = distance.odometryTravelled;
    headingOffset = currentHeading;

    Serial.println("ODOMETRY: Home set (x=0, y=0, heading=0)");
}