#pragma once

struct OdometryData
{
    float x;         // metres
    float y;         // metres
    float heading;   // radians
};

extern OdometryData odometry;

void Odometry_Init();
void Odometry_Reset();
void Odometry_Update();