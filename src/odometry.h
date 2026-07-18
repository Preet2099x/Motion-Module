#pragma once

struct OdometryData
{
    float x;
    float y;

    float heading;      // relative heading (0 after Home)
    float rawHeading;   // raw BNO heading (0-360)

    float homeX;
    float homeY;
    float homeHeading;
};

extern OdometryData odometry;

void Odometry_Init();
void Odometry_Reset();
void Odometry_Update();
void Odometry_SetHome();