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

enum CamSide
{
    CAM_FRONT = 0,
    CAM_BACK = 1,
    CAM_LEFT = 2,
    CAM_RIGHT = 3
};

// Distances reported by the Jetson in "CAMD F=a/b B=a/b L=a/b R=a/b" lines
// (cm; a = dist1 = nearest thing in that camera's view, b = dist2 = ground
// strip; "--" = camera not live). Sent ~10x/s, so they go stale quickly.
// Returns dist1 for `side`, or -1 if it was never reported, is "--", or is
// older than CAMD_FRESH_MS.
float CamDist_Free(CamSide side);

// Returns true if `code` was one of the eight lock/release codes.
bool CamLock_HandleCode(int code);

// Returns true if `line` is exactly one of the eight codes (digits only),
// or a CAMD distance line. Neither touches `data`.
bool CamLock_HandleLine(const String &line);

void CamLock_ClearAll();

// True if the (manual) motion command is blocked by a locked side.
bool CamLock_IsBlocked(int cmd);
