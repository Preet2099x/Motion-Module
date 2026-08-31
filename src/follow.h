#pragma once

#include <Arduino.h>

// ─── Follow-Me mode ────────────────────────────────────
// AIM-ONLY: the robot turns to face the beacon and holds. It does not
// drive forward — that will be added later.
//
// Four UWB anchors on the robot, each reporting range to a beacon the
// operator carries:
//
//     A  FRONT   — mounted pointing forward
//     B  RIGHT   — mounted on the right side
//     C  LEFT    — mounted on the left side
//     D  BACK    — mounted pointing backward
//
// Direction logic (no trigonometry):
//   Front/back:  A vs D  →  whichever is closer, beacon is that way
//   Left/right:  B vs C  →  whichever is closer, beacon is that way
//   Combine → 8 cardinal directions → fixed turn angle per direction.
//
// Serial input format (one line per update):
//     A=3.35m  B=2.98m  C=3.10m  D=3.90m
//
// Serial commands:
//     follow         start follow-me
//     followstop     stop follow-me
//     followmon      live readout with the motors idle (bring-up)
//     followcfg      print current configuration
//     followdbg      toggle periodic debug print
//     followrev      invert which way "right"/"left" spins the motors
//     followm <m>    comparison noise margin in metres (default 0.15)
//     followp <m>    hard stop when any RAW anchor range drops below <m>

void Follow_Init();
void Follow_Update();

void Follow_Start();
void Follow_Stop();
void Follow_Cancel(); // silent stop, used by the emergency latch

bool Follow_IsRunning();

// Returns true if the line was a beacon reading and was consumed.
bool Follow_IngestLine(const String &line);

// Returns true if the line was a follow-me command and was consumed.
bool Follow_HandleCommand(const String &line);

void Follow_PrintConfig();
