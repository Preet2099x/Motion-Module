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

// ─── Pause / Resume ────────────────────────────────────
// Pause: freezes the drive-mode task AND saves the caller's manual motion data.
// Resume: restores and returns the saved manual motion data (0 if nothing was running).
void Drive_Pause(int savedData);   // call with current `data` to preserve it
void Drive_Resume();               // re-arms drive; caller should restore returned data
bool Drive_IsPaused();
void Drive_TogglePause(int currentData); // flips pause↔resume, pass current `data`
int  Drive_GetSavedData();         // returns the data value saved at pause time

