#pragma once

#include <Arduino.h>

// ─── Drive command types ───────────────────────────────
enum DriveCommandType
{
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_STOP
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
bool Drive_IsRunning();
void Drive_Cancel(); // Abort the current task and reset the queue

