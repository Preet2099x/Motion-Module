#include "camlock.h"

#include <Arduino.h>
#include <var.h>

// ── CAMD distance line ──
static const unsigned long CAMD_FRESH_MS = 600;

static float camD1[4];
static unsigned long camStamp[4];
static bool camValid[4];

static void parseCamd(const String &line)
{
    const char keys[4] = {'F', 'B', 'L', 'R'};
    const unsigned long now = millis();

    for (int i = 0; i < 4; i++)
    {
        String tag = String(" ") + keys[i] + "=";
        int at = line.indexOf(tag);
        if (at < 0)
            continue;

        int from = at + 3;
        int to = line.indexOf(' ', from);
        if (to < 0)
            to = line.length();

        String field = line.substring(from, to);
        if (field.length() == 0 || field.charAt(0) == '-')
        {
            camValid[i] = false;
            continue;
        }

        camD1[i] = field.toFloat(); // reads up to the '/'
        camStamp[i] = now;
        camValid[i] = true;
    }
}

float CamDist_Free(CamSide side)
{
    if (!camValid[side] || millis() - camStamp[side] > CAMD_FRESH_MS)
        return -1.0f;
    return camD1[side];
}

bool CamLock_HandleCode(int code)
{
    bool *flag = nullptr;
    const char *name = "";
    bool lock = false;

    switch (code)
    {
    case 101: flag = &frontCamLocked; name = "FRONT"; lock = true;  break;
    case 102: flag = &frontCamLocked; name = "FRONT"; lock = false; break;
    case 201: flag = &backCamLocked;  name = "BACK";  lock = true;  break;
    case 202: flag = &backCamLocked;  name = "BACK";  lock = false; break;
    case 301: flag = &leftCamLocked;  name = "LEFT";  lock = true;  break;
    case 302: flag = &leftCamLocked;  name = "LEFT";  lock = false; break;
    case 401: flag = &rightCamLocked; name = "RIGHT"; lock = true;  break;
    case 402: flag = &rightCamLocked; name = "RIGHT"; lock = false; break;
    default:
        return false;
    }

    // Bypass: still a camera code (so it never reaches `data`), just ignored.
    if (camBypass)
        return true;

    if (*flag != lock)
    {
        *flag = lock;
        Serial.print("CAM ");
        Serial.print(name);
        Serial.println(lock ? ": LOCKED" : ": FREE");
    }
    return true;
}

bool CamLock_HandleLine(const String &line)
{
    if (line.startsWith("CAMD"))
    {
        parseCamd(line);
        return true;
    }

    if (line.length() != 3)
        return false;

    for (unsigned int i = 0; i < 3; i++)
        if (!isDigit(line.charAt(i)))
            return false;

    return CamLock_HandleCode(line.toInt());
}

void CamLock_ClearAll()
{
    frontCamLocked = false;
    backCamLocked = false;
    leftCamLocked = false;
    rightCamLocked = false;
}

bool CamLock_IsBlocked(int cmd)
{
    if (camBypass)
        return false;

    // motion() remaps these D-pad codes to arcs; block by what actually runs
    if (cmd == 172) cmd = 115;
    if (cmd == 174) cmd = 125;
    if (cmd == 272) cmd = 215;
    if (cmd == 274) cmd = 225;

    if (frontCamLocked && (cmd == 1 || (cmd >= 111 && cmd <= 130)))
        return true;

    if (backCamLocked && (cmd == 2 || (cmd >= 211 && cmd <= 230)))
        return true;

    if (leftCamLocked &&
        ((cmd >= 21 && cmd <= 30) || (cmd >= 121 && cmd <= 130) || (cmd >= 221 && cmd <= 230)))
        return true;

    if (rightCamLocked &&
        ((cmd >= 11 && cmd <= 20) || (cmd >= 111 && cmd <= 120) || (cmd >= 211 && cmd <= 220)))
        return true;

    return false;
}
