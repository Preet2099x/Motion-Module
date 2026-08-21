#pragma once

#include <Arduino.h>

// ─── Follow-Me mode ────────────────────────────────────
// AIM-ONLY build: the robot turns to face the beacon and holds. It never
// drives -- there is no drive state and no command that can start one.
//
// Autonomous "walk in front of the robot and it follows you" mode driven by
// three UWB anchors mounted on the robot, each reporting its range to one
// fixed beacon carried by the operator:
//
//     A  FORWARD  — mounted pointing forward
//     B  RIGHT    — mounted on the right
//     C  LEFT     — mounted on the left
//
// Direction comes from plain comparisons between the three raw ranges — no
// trigonometry. Whichever anchor reads closer is the direction the beacon is
// more toward. See the derivation at the top of follow.cpp.
//
// Serial input format (one line per update, whitespace-flexible):
//     A=3.35m  B=2.98m  C=3.10m
//
// Serial commands:
//     follow         start follow-me
//     followstop     stop follow-me
//     followmon      live readout with the motors idle (bring-up)
//     followcfg      print current configuration + live beacon state
//     followdbg      toggle periodic debug print
//     followrev      invert which way "right"/"left" spins the motors
//     followfb       invert the front/back sense (if front reads as back)
//     followtri      toggle THRESHOLD vs TRILATERATION+45deg-sector method
//     followm <m>    comparison noise margin in metres (default 0.15)
//     followp <m>    hard stop when any RAW anchor range drops below <m>
//
// There is deliberately no drive/leg/standoff command: this build cannot
// translate, so range thresholds would have nothing to gate.

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
