#pragma once

struct OdometryData
{
    float x;
    float y;
    float heading;

    float homeX;
    float homeY;
    float homeHeading;
};

extern OdometryData odometry;

void Odometry_Init();
void Odometry_Reset();
void Odometry_Update();
void Odometry_SetHome();