#pragma once

struct DistanceData
{
    float left;       // meters
    float right;      // meters
    float travelled;  // meters
};

extern DistanceData distance;

void Distance_Init();
void Distance_Reset();
void Distance_Update(long leftTicks, long rightTicks);