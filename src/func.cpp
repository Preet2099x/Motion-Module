#include <var.h>
#include <string.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include "follow.h"
#include "camlock.h"

float calcAlphaEMA(float fn)
{
    if (fn <= 0)
        return 1;
    // α(fₙ) = cos(2πfₙ) - 1 + √( cos(2πfₙ)² - 4 cos(2πfₙ) + 3 )
    const float c = cos(2 * float(M_PI) * fn);
    return c - 1 + sqrt(c * c - 4 * c + 3);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double handletime(float data)
{
    if (data == 100)
    {
        return 10;
    }
    else if (data == 10)
    {
        return 100;
    }
    else
    {
        return 0;
    }
}

uint32_t getTeensySerial()
{
    uint32_t num;
    num = HW_OCOTP_MAC0 & 0xFFFFFF;
    if (num < 10000000)
    {
        num = num * 10;
        return num;
    }
    else
    {
        return num;
    }
}

void receiveEvent(int bytesReceived)
{

    String cmd = "";

    while (kire.available())
    {
        cmd += (char)kire.read();
    }

    cmd.trim();

    if (cmd.length() == 0)
    {
        return;
    }

    // Shortcut: 'c' over I2C starts follow-me, same as sending "follow"
    // over serial.
    if (cmd.length() == 1 && cmd.charAt(0) == 'c')
    {
        Follow_Start();
        return;
    }

    // Camera lock codes (101/102 ... 401/402) only update the lock flags;
    // they must not overwrite the current motion command in `data`.
    if (CamLock_HandleLine(cmd))
    {
        return;
    }

    data = cmd.toInt();
    // Serial.println(data);
    startTimeControlCounter = currentTimeControlCounter;
}