#include <var.h>
#include <Arduino.h>

static int lastCommand = 0;

static int currentPwmL = 0;
static int currentPwmR = 0;

// static unsigned long stopRampStartTime = 0;
// static bool stopRampActive = false;

// static int stopStartPwmL = 0;
// static int stopStartPwmR = 0;

static bool headingHoldActive = false;
static unsigned long headingCaptureStart = 0;

float targetHeading = 0;

float headingIntegral = 0;
float previousHeadingError = 0;
float pidError = 0;
float pidCorrection = 0;
unsigned long previousPidTime = 0;

static unsigned long boostCommandTimer = 0;

const float KpF = 2.4f;
const float KiF = 0.1f;
const float KdF = 0.08f;

const float KpB = 2.5f;
const float KiB = 0.35f;
const float KdB = 0.08f;

// const float KpB = 8.0f;
// const float KiB = 0.10f;
// const float KdB = 0.08f;

int debugPwmL = 0;
int debugPwmR = 0;

// Speed ramp for 1/2, keyed to the PWM actually on the motors, not to how long
// the command has been held. Each side's base speed is rate-limited toward the
// target, and it is (re)anchored to currentPwmL/R whenever 1/2 starts afresh
// (after 0, a turn, an arc, ...). A stick flick that sends 1,0,1,0... therefore
// resumes from wherever the motors are instead of restarting - or skipping -
// the ramp, which is what made it jump straight to full PWM.
static const float RAMP_START_PWM = 30.0f;     // floor when starting from rest
static const float RAMP_RATE = 0.15f;          // PWM per ms (~1.5 s for 30 -> 255)
static const float RAMP_RATE_ARC_EXIT = 0.85f; // PWM per ms (~300 ms for 0 -> 255)

static bool rampHeld = false;     // true while 1/2 is being driven continuously
static bool cameFromArc = false;  // last non-1/2 command was an arc (111-130)
static float rampL = 0;
static float rampR = 0;
static float rampRate = RAMP_RATE;
static unsigned long rampLastTick = 0;

static float approachTarget(float value, float target, float step)
{
  if (value < target)
    return min(value + step, target);
  return max(value - step, target);
}

// Outputs the pre-PID base PWM for each side (the heading PID adds its
// correction on top of these).
static void applySpeedRamp(int targetPWM, int &pwmL, int &pwmR)
{
  unsigned long now = millis();

  if (!rampHeld)
  {
    rampL = max((float)currentPwmL, RAMP_START_PWM);
    rampR = max((float)currentPwmR, RAMP_START_PWM);
    rampRate = cameFromArc ? RAMP_RATE_ARC_EXIT : RAMP_RATE;
    rampLastTick = now;
    rampHeld = true;
  }

  float step = rampRate * (float)(now - rampLastTick);
  rampLastTick = now;

  rampL = approachTarget(rampL, (float)targetPWM, step);
  rampR = approachTarget(rampR, (float)targetPWM, step);

  pwmL = (int)rampL;
  pwmR = (int)rampR;
}

static int applyRamp(int current, int target, int step)
{
  if (current > target)
  {
    current -= step;
    if (current < target)
      current = target;
  }
  return current;
}

float normalizeHeadingError(float error)
{
  while (error > 180.0f)
    error -= 360.0f;
  while (error < -180.0f)
    error += 360.0f;

  return error;
}

void motion(int _data)
{

  if (_data == 66)
  {
    if (millis() - boostCommandTimer >= 500)
    {
      boostCommandTimer = millis();

      turnBoost = !turnBoost;

      // Serial.print("Turn Boost: ");
      // Serial.println(turnBoost ? "ON" : "OFF");
    }

    // data = 0;
    return;
  }

  if (_data == 65)
  {
    if (millis() - boostCommandTimer >= 500)
    {
      boostCommandTimer = millis();

      straightBoost = !straightBoost;

      // Serial.print("Straight Boost: ");
      // Serial.println(straightBoost ? "ON" : "OFF");
    }

    // data = 0;
    return;
  }

  // D-PAD Commands:
  //  Remap new camera-aware commands to existing movement commands
  if (_data == 172)
    _data = 115;
  if (_data == 174)
    _data = 125;
  if (_data == 272)
    _data = 215;
  if (_data == 274)
    _data = 225;

  // Anything other than 1/2 ends a held ramp, so the next 1/2 re-anchors to the
  // PWM on the motors. 0 clears cameFromArc so arc -> release -> 1 uses the
  // normal rate, from the decayed PWM.
  if (_data != 1 && _data != 2)
  {
    rampHeld = false;
    cameFromArc = (_data >= 111 && _data <= 130);
  }

  // --- Kill motor on direction switch (1->2 or 2->1) ---
  if ((_data == 1 && lastCommand == 2) || (_data == 2 && lastCommand == 1))
  {
    currentPwmL = 0;
    currentPwmR = 0;

    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    rampHeld = false;
    cameFromArc = false;

    lastCommand = 0; // reset so next cycle treats it as a fresh start
    return;
  }

  if (_data == 0)
  {

    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    static unsigned long rampDownTimer = 0;
    if (millis() - rampDownTimer >= 5) // 5ms tick — tune this
    {
      rampDownTimer = millis();

      if (currentPwmL > 0)
      {
        if (currentPwmL > 180)
          currentPwmL = applyRamp(currentPwmL, 0, 8);
        else if (currentPwmL > 150)
          currentPwmL = applyRamp(currentPwmL, 0, 2);
        else if (currentPwmL > 90)
          currentPwmL = applyRamp(currentPwmL, 0, 8);
        else
        {
          currentPwmL = (int)(currentPwmL * 0.94f);

          if (currentPwmL < 5)
            currentPwmL = 0;
        }
      }

      if (currentPwmR > 0)
      {
        if (currentPwmR > 180)
          currentPwmR = applyRamp(currentPwmR, 0, 8);
        else if (currentPwmR > 150)
          currentPwmR = applyRamp(currentPwmR, 0, 2);
        else if (currentPwmR > 90)
          currentPwmR = applyRamp(currentPwmR, 0, 8);
        else
        {
          currentPwmR = (int)(currentPwmR * 0.94f);

          if (currentPwmR < 5)
            currentPwmR = 0;
        }
      }
    }

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);

    if (currentPwmL == 0 && currentPwmR == 0)
    {
      digitalWrite(dirPin_L, LOW);
      digitalWrite(dirPin_R, LOW);
      lastCommand = 0;
    }

    return;
  }

  if (_data == 1)
  {

    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);

    int pwmL, pwmR;
    applySpeedRamp(straightBoost ? 255 : 200, pwmL, pwmR);

    if (!headingHoldActive)
    {
      if (headingCaptureStart == 0)
        headingCaptureStart = millis();

      if (millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if (headingHoldActive)
    {
      unsigned long now = millis();
      float dt = (now - previousPidTime) / 1000.0f;

      if (dt >= 0.01f) // Run PID at ~100Hz (every 10ms)
      {
        float error = normalizeHeadingError(targetHeading - currentHeading);

        float derivative = (error - previousHeadingError) / dt;

        headingIntegral += error * dt;
        headingIntegral = constrain(headingIntegral, -15.0f, 15.0f);
        float correction = KpF * error +
                           KiF * headingIntegral +
                           KdF * derivative;
        pidError = error;
        pidCorrection = correction;

        currentPwmL = constrain(pwmL + correction, 0, 255);
        currentPwmR = constrain(pwmR - correction, 0, 255);

        debugPwmL = currentPwmL;
        debugPwmR = currentPwmR;

        previousHeadingError = error;
        previousPidTime = now; // Only reset when PID actually ran
      }
    }
    else
    {
      currentPwmL = pwmL;
      currentPwmR = pwmR;

      debugPwmL = currentPwmL;
      debugPwmR = currentPwmR;
    }

    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }

  else if (_data == 2)
  {
    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);

    int pwmL, pwmR;
    applySpeedRamp(straightBoost ? 255 : 200, pwmL, pwmR);

    if (!headingHoldActive)
    {
      if (headingCaptureStart == 0)
        headingCaptureStart = millis();

      if (millis() - headingCaptureStart >= 200)
      {
        targetHeading = currentHeading;
        headingHoldActive = true;
        headingIntegral = 0;
        previousHeadingError = 0;
        previousPidTime = millis();
      }
    }

    if (headingHoldActive)
    {
      unsigned long now = millis();
      float dt = (now - previousPidTime) / 1000.0f;

      if (dt >= 0.01f) // Run PID at ~100Hz (every 10ms)
      {
        float error = normalizeHeadingError(targetHeading - currentHeading);

        float derivative = (error - previousHeadingError) / dt;

        headingIntegral += error * dt;
        headingIntegral = constrain(headingIntegral, -15.0f, 15.0f);
        float correction = KpB * error +
                           KiB * headingIntegral +
                           KdB * derivative;
        pidError = error;
        pidCorrection = correction;

        currentPwmL = constrain(pwmL - correction, 0, 255);
        currentPwmR = constrain(pwmR + 2 + correction, 0, 255);

        debugPwmL = currentPwmL;
        debugPwmR = currentPwmR;

        previousHeadingError = error;
        previousPidTime = now; // Only reset when PID actually ran
      }
    }
    else
    {
      currentPwmL = pwmL;
      currentPwmR = pwmR;

      debugPwmL = currentPwmL;
      debugPwmR = currentPwmR;
    }

    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }

  else if (_data == 3)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    
    digitalWrite(dirPin_R, LOW);
    digitalWrite(dirPin_L, LOW);
    currentPwmL = straightBoost ? 150 : 120;
    currentPwmR = straightBoost ? 150 : 120;
    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }

  else if (_data == 4)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_R, HIGH);
    digitalWrite(dirPin_L, HIGH);
    currentPwmL = straightBoost ? 150 : 120;
    currentPwmR = straightBoost ? 150 : 120;
    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_R, currentPwmR);
    analogWrite(pwmPin_L, currentPwmL);
  }


  else if (_data >= 11 && _data <= 20)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 13)
    {
      currentPwmL = turnBoost ? 80 : 60;
      currentPwmR = turnBoost ? 80 : 60;
    }
    else if (_data <= 16)
    {
      currentPwmL = turnBoost ? 120 : 80;
      currentPwmR = turnBoost ? 120 : 80;
    }
    else
    {
      currentPwmL = turnBoost ? 180 : 120;
      currentPwmR = turnBoost ? 180 : 120;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 21 && _data <= 30)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 23)
    {
      currentPwmL = turnBoost ? 80 : 60;
      currentPwmR = turnBoost ? 80 : 60;
    }
    else if (_data <= 26)
    {
      currentPwmL = turnBoost ? 120 : 80;
      currentPwmR = turnBoost ? 120 : 80;
    }
    else
    {
      currentPwmL = turnBoost ? 180 : 120;
      currentPwmR = turnBoost ? 180 : 120;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  // Follow-me-only boosted turn codes. Mirror 11-30 exactly (same dir pins,
  // same 3-tier structure) but at higher PWM. Kept as their own code range,
  // never shared with drive.cpp/navigation.cpp, so speeding up follow-me
  // turns cannot change the speed of any other turn in the codebase.
  else if (_data >= 31 && _data <= 40)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 33)
    {
      currentPwmL = 90;
      currentPwmR = 90;
    }
    else if (_data <= 36)
    {
      currentPwmL = 130;
      currentPwmR = 130;
    }
    else
    {
      currentPwmL = 190;
      currentPwmR = 190;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 41 && _data <= 50)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 43)
    {
      currentPwmL = 90;
      currentPwmR = 90;
    }
    else if (_data <= 46)
    {
      currentPwmL = 130;
      currentPwmR = 130;
    }
    else
    {
      currentPwmL = 190;
      currentPwmR = 190;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 111 && _data <= 120)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 113)
    {
      currentPwmL = straightBoost ? 170 : 125;
      currentPwmR = straightBoost ? 120 : 90;
    }
    else if (_data <= 116)
    {
      currentPwmL = straightBoost ? 200 : 150;
      currentPwmR = straightBoost ? 150 : 115;
    }
    else
    {
      currentPwmL = straightBoost ? 240 : 180;
      currentPwmR = straightBoost ? 80 : 60;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 121 && _data <= 130)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);

    if (_data <= 123)
    {
      currentPwmL = straightBoost ? 120 : 90;
      currentPwmR = straightBoost ? 170 : 125;
    }
    else if (_data <= 126)
    {
      currentPwmL = straightBoost ? 150 : 115;
      currentPwmR = straightBoost ? 200 : 150;
    }
    else
    {
      currentPwmL = straightBoost ? 80 : 60;
      currentPwmR = straightBoost ? 240 : 180;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 221 && _data <= 230)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;

    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 223)
    {
      currentPwmL = straightBoost ? 170 : 125;
      currentPwmR = straightBoost ? 120 : 90;
    }
    else if (_data <= 226)
    {
      currentPwmL = straightBoost ? 200 : 150;
      currentPwmR = straightBoost ? 120 : 90;
    }
    else
    {
      currentPwmL = straightBoost ? 240 : 180;
      currentPwmR = straightBoost ? 80 : 60;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;
    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }

  else if (_data >= 211 && _data <= 220)
  {
    headingHoldActive = false;
    headingCaptureStart = 0;
    headingIntegral = 0;
    previousHeadingError = 0;
    previousPidTime = 0;
    pidError = 0;
    pidCorrection = 0;
    targetHeading = currentHeading;
    digitalWrite(dirPin_L, HIGH);
    digitalWrite(dirPin_R, HIGH);

    if (_data <= 213)
    {
      currentPwmL = straightBoost ? 120 : 90;
      currentPwmR = straightBoost ? 170 : 125;
    }
    else if (_data <= 216)
    {
      currentPwmL = straightBoost ? 120 : 90;
      currentPwmR = straightBoost ? 200 : 150;
    }
    else
    {
      currentPwmL = straightBoost ? 80 : 60;
      currentPwmR = straightBoost ? 240 : 180;
    }

    debugPwmL = currentPwmL;
    debugPwmR = currentPwmR;

    analogWrite(pwmPin_L, currentPwmL);
    analogWrite(pwmPin_R, currentPwmR);
  }
  lastCommand = _data;
}

// True instant kill for the emergency latch — unlike motion(0), which ramps
// currentPwmL/currentPwmR down gradually over several ticks. Callers used to
// hard-zero the pins directly with analogWrite() and skip motion() entirely,
// but that left currentPwmL/currentPwmR (this file's own ramp state) stale
// at whatever they were before the latch. The moment anything called
// motion(0) afterward — e.g. Drive_Cancel()/Follow_Cancel() on the latch's
// first tick — it re-applied that stale, still-large PWM (with the dir pins
// just forced LOW/LOW here, i.e. forward) before its own ramp had a chance
// to bring it down, producing a brief forward jerk right after the "stop".
// This zeroes the ramp state directly so any later motion(0) call is a
// no-op instead of an unintended re-drive.
void motion_EmergencyStop()
{
  currentPwmL = 0;
  currentPwmR = 0;

  headingHoldActive = false;
  headingCaptureStart = 0;
  headingIntegral = 0;
  previousHeadingError = 0;
  previousPidTime = 0;
  pidError = 0;
  pidCorrection = 0;
  targetHeading = currentHeading;

  rampHeld = false;
  cameFromArc = false;
  lastCommand = 0;

  digitalWrite(dirPin_L, LOW);
  digitalWrite(dirPin_R, LOW);
  analogWrite(pwmPin_L, 0);
  analogWrite(pwmPin_R, 0);
}
