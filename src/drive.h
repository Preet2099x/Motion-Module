#pragma once

#include <Arduino.h>

// ─── Drive command types ───────────────────────────────
enum DriveCommandType
{
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP,
    CMD_TURN_TO_ABS  // value = absolute target heading in odometry space (0–360 deg)
};

struct DriveCommand
{
    DriveCommandType type;
    float value; // metres (F/B) or degrees (L/R), ignored for S
};

// ─── Public API ────────────────────────────────────────

void Drive_Init();
void Drive_Update();

void Drive_ParseTask(const String &raw);
bool Drive_SubmitTask(const DriveCommand *commands, int count);

bool Drive_IsRunning();
void Drive_Cancel();

