#include "distance.h"

#include <Arduino.h>
#include <stdlib.h>

DistanceData distance;

//======================================
// Robot Calibration (from old project)
//======================================

constexpr float WHEEL_DIAMETER = 0.15f; // 150 mm
constexpr float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

constexpr float TICKS_PER_ENCODER_REV = 4000.0f;
constexpr float MECH_RATIO = 1.6f;

constexpr float EFFECTIVE_TICKS_PER_WHEEL_REV =
    TICKS_PER_ENCODER_REV * MECH_RATIO; // 6400

//======================================

void Distance_Init()
{
    Distance_Reset();
}

void Distance_Reset()
{

    distance.left = 0.0f;
    distance.right = 0.0f;
    distance.travelled = 0.0f;
    distance.odometryTravelled = 0.0f;
}

void Distance_Update(long leftTicks, long rightTicks)
{

    // Absolute distance (for Drive)
    float leftDistance =
        (abs(leftTicks) / EFFECTIVE_TICKS_PER_WHEEL_REV) *
        WHEEL_CIRCUMFERENCE;

    float rightDistance =
        (abs(rightTicks) / EFFECTIVE_TICKS_PER_WHEEL_REV) *
        WHEEL_CIRCUMFERENCE;

    // Signed distance (for Odometry)
    float leftSignedDistance =
        (leftTicks / EFFECTIVE_TICKS_PER_WHEEL_REV) *
        WHEEL_CIRCUMFERENCE;

    float rightSignedDistance =
        (rightTicks / EFFECTIVE_TICKS_PER_WHEEL_REV) *
        WHEEL_CIRCUMFERENCE;

    distance.left += leftDistance;
    distance.right += rightDistance;

    distance.travelled =
        (distance.left + distance.right) * 0.5f;

    distance.odometryTravelled +=
    (leftSignedDistance + rightSignedDistance) * 0.5f;
}