#include "camlock.h"

#include <Arduino.h>
#include <var.h>

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
