#include "odometry.h"

#include <Arduino.h>
#include <math.h>

#include "distance.h"
#include <var.h>

OdometryData odometry;

static float previousDistance = 0.0f;

void Odometry_Init()
{
    Odometry_Reset();
}

void Odometry_Reset()
{
    odometry.x = 0.0f;
    odometry.y = 0.0f;
    odometry.heading = 0.0f;

    previousDistance = distance.travelled;
}

void Odometry_Update()
{
    float deltaDistance = distance.travelled - previousDistance;

    float headingRad = currentHeading * PI / 180.0f;

    odometry.x += deltaDistance * cosf(headingRad);
    odometry.y += deltaDistance * sinf(headingRad);

    odometry.heading = headingRad;

    previousDistance = distance.travelled;
}