#pragma once

struct VelocityData
{
    float linear;     // m/s
    float angular;    // rad/s
};

extern VelocityData velocity;

void Velocity_Init();
void Velocity_Reset();
void Velocity_Update(float dt);