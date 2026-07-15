#pragma once

struct DistanceData
{
    float left;                 // Total left wheel distance
    float right;                // Total right wheel distance
    float travelled;            // Always positive (used by Drive)
    float odometryTravelled;    // Signed (used only by Odometry)
};

extern DistanceData distance;

void Distance_Init();
void Distance_Reset();
void Distance_Update(long leftTicks, long rightTicks);