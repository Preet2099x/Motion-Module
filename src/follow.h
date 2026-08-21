#pragma once

#include <Arduino.h>

// ─── Follow-Me mode ────────────────────────────────────
// Autonomous "walk in front of the robot and it follows you" mode driven by
// two ultrasonic/UWB beacons mounted on the robot (A = front, B = back) that
// each report their range to one fixed "main" beacon carried by the operator.
//
// Serial input format (one line per update, whitespace-flexible):
//     A=3.35m  B=2.98m
//
// Serial commands:
//     follow         start follow-me
//     followstop     stop follow-me
//     followcfg      print current configuration + live beacon state
//     followdbg      toggle periodic debug print
//     followd <m>    set the A↔B beacon baseline in metres
//     followr <m>    set the standoff (stop) range in metres

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
