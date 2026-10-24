#pragma once

#include <Arduino.h>

// ─── Camera lock codes ─────────────────────────────────
// The Jetson sends one code per camera whenever its state changes:
//
//     front  101 lock / 102 release   blocks 1, 111-130
//     back   201 lock / 202 release   blocks 2, 211-230
//     left   301 lock / 302 release   blocks 21-30, 121-130, 221-230
//     right  401 lock / 402 release   blocks 11-20, 111-120, 211-220
//
// A command is blocked if ANY locked side covers it. Emergency releases all.
// Lock codes are consumed here and never stored in `data`, so a camera
// message cannot overwrite or restart the current motion command.

// Returns true if `code` was one of the eight lock/release codes.
bool CamLock_HandleCode(int code);

// Returns true if `line` is exactly one of the eight codes (digits only).
bool CamLock_HandleLine(const String &line);

void CamLock_ClearAll();

// True if the (manual) motion command is blocked by a locked side.
bool CamLock_IsBlocked(int cmd);
