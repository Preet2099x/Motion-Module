#include <var.h>
#include <maf.h>
#include <string.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <TeensyThreads.h>
#include <i2c_driver_wire.h>

#include "distance.h"
#include "velocity.h"
#include "odometry.h"
#include "drive.h"
#include "navigation.h"
#include "follow.h"
#include "camlock.h"

int SLAVE_ADDRESS = 0x72;

// uint8_t BNO_ADDR =       0x28;  // I2C address of BNO
// uint8_t ACC_DATA_X_LSB = 0x08;  // BNO register Acceleration Data X LSB
// uint8_t CALIB_STAT =     0x35;  // BNO register SYS Calib Status <7:6>, GYR Calib Status <5:4>, ACC Calib Status <3:2>, MAG Calib Status <1:0>
// uint8_t OPR_MODE   =     0x3D; // BNO register Operation Mode <3:0>
// uint8_t NDOF       =     0x0C;

// imuData imu;

int FLW = 0;
int FRW = 0;
int BLW = 0;
int BRW = 0;

int FLD = 0;
int FRD = 0;
int BLD = 0;
int BRD = 0;

int TRR = 0;
int TRL = 0;
int TLR = 0;
int TLL = 0;

int pwmPin_L = 23; // 7
int dirPin_L = 22; // 8
int pwmPin_R = 14; // 5
int dirPin_R = 20; // 6

unsigned int addressFLW = 0;
unsigned int addressFRW = 2;
unsigned int addressBLW = 4;
unsigned int addressBRW = 6;

unsigned int addressFLD = 8;
unsigned int addressFRD = 10;
unsigned int addressBLD = 12;
unsigned int addressBRD = 14;

unsigned int addressTRR = 16;
unsigned int addressTRL = 18;
unsigned int addressTLR = 20;
unsigned int addressTLL = 22;

int encoderPin_1_L = 9; // A/B swapped so the left count matches the right's direction
int encoderPin_2_L = 8;
int encoderPin_1_R = 5; // 14
int encoderPin_2_R = 6; // 20

bool turnBoost = false;
bool straightBoost = false;

bool frontCamLocked = false;
bool backCamLocked = false;
bool leftCamLocked = false;
bool rightCamLocked = false;
bool camBypass = false;

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
long lastencoderValue_L = 0;
// int lastMSB_L = 0;
// int lastLSB_L = 0;

volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;
long lastencoderValue_R = 0;

// Running signed tick totals (encoderValue_* is cleared every 100 ms tick)
long totalTicks_L = 0;
long totalTicks_R = 0;
// int lastMSB_R = 0;
// int lastLSB_R = 0;

// DIR and PWM Data
int delayBrake = 50;
int pin_Emergency = 15;

// Handle Print Vsriable
bool printAlter = false;
bool emergencyAlter = true;
bool systemCounter = false;

// Control Variable
int data = 0;
int rpmAlter_T = 0;
int rpmAlter = 0;

// Time Variable
float timeConstant = 100; // MilliSecond 100 | 10 ONLY -> Function-> handletime
float startTime, elaspedTime = 0, currentTime;

// Time Setup Control Counter
float timeConstantControlCounter = 1000; // Chnage This As per Trail
float startTimeControlCounter, elaspedTimeControlCounter = 0, currentTimeControlCounter;

void setup()
{
  // Serial Begin
  Serial.begin(115200);
  // Serial5.begin(115200);//TODO: For Simplified Serial
  kire.begin(SLAVE_ADDRESS);
  // initWire();

  // Motor Pin Setup
  pinMode(dirPin_L, OUTPUT);
  pinMode(pwmPin_L, OUTPUT);

  pinMode(dirPin_R, OUTPUT);
  pinMode(pwmPin_R, OUTPUT);

  // initializeBNO055();
  pinMode(pin_Emergency, INPUT_PULLDOWN);
  bnoStandaloneSetup();

  // Time Setup
  startTime = millis();

  // Time Setup Control Counter
  startTimeControlCounter = millis();

  digitalWrite(dirPin_L, LOW);
  digitalWrite(dirPin_R, LOW);
  digitalWrite(pwmPin_L, LOW);
  digitalWrite(pwmPin_R, LOW);

  // Encoder Setup
  pinMode(encoderPin_1_L, INPUT);
  pinMode(encoderPin_2_L, INPUT);

  digitalWrite(encoderPin_1_L, HIGH);
  digitalWrite(encoderPin_2_L, HIGH);
  attachInterrupt(encoderPin_1_L, updateEncoder_L, CHANGE);
  attachInterrupt(encoderPin_2_L, updateEncoder_L, CHANGE);

  pinMode(encoderPin_1_R, INPUT);
  pinMode(encoderPin_2_R, INPUT);

  digitalWrite(encoderPin_1_R, HIGH);
  digitalWrite(encoderPin_2_R, HIGH);
  attachInterrupt(encoderPin_1_R, updateEncoder_R, CHANGE);
  attachInterrupt(encoderPin_2_R, updateEncoder_R, CHANGE);

  // Read Permanent Data
  readEEPROM();

  Distance_Init();
  Velocity_Init();
  Odometry_Init();
  Drive_Init();
  Navigation_Init();
  Follow_Init();

  Serial5.write(0);
  Serial5.write(192);
  /*
  kire.setClock(400 * 1000);
  kire.begin();                // initialize I2C
  bno_write(BNO_ADDR, OPR_MODE, NDOF);
  delay(1000);

  threads.addThread(thread_func);
  */
}

void loop()
{

  kire.onReceive(receiveEvent);

  // Handle Fliter
  static SMA<20> filter_L;
  double rpm_L = 0;
  uint16_t avgRPM_L = 0;

  static SMA<20> filter_R;
  double rpm_R = 0;
  uint16_t avgRPM_R = 0;

  int emergency = analogRead(pin_Emergency); // TODO: Comment if emergency is removed

  // Handle  Time
  currentTime = millis();
  elaspedTime = currentTime - startTime;

  // Time Setup Control Counter
  currentTimeControlCounter = millis();
  elaspedTimeControlCounter = currentTimeControlCounter - startTimeControlCounter;

  // Handle Serial Commands
  if (Serial.available())
  {
    if (systemCounter == false)
    {
      // Read the first byte, then the rest of the line
      int _data = Serial.read();
      String rest = Serial.readStringUntil('\n');
      String fullLine = String((char)_data) + rest;
      fullLine.trim();

      // Echo everything that arrives on USB serial, so incoming data is
      // visible even when no command below recognises it.
      // (CAMD distance lines arrive ~10x/s; echoing them would drown the log.)
      if (!fullLine.startsWith("CAMD"))
      {
        Serial.print("RX[");
        Serial.print(fullLine.length());
        Serial.print("]: ");
        Serial.println(fullLine);
      }

      // ── Beacon ranges: "A=2.69M B=2.36M" ──
      // Must be checked before the numeric fallthrough at the bottom, where
      // toInt() would turn the whole line into a silent motion(0).
      if (Follow_IngestLine(fullLine))
      {
        // consumed — beacon data, not a command
      }
      // ── Follow-me commands: follow / followstop / followmon / ... ──
      else if (Follow_HandleCommand(fullLine))
      {
        data = 0;
      }
      // ── Camera lock codes: 101/102 front, 201/202 back, 301/302 left,
      // 401/402 right. Consumed without touching `data` or the control timer,
      // so a camera update never stops or restarts the current motion. ──
      else if (CamLock_HandleLine(fullLine))
      {
        // consumed — flags updated
      }
      // ── Task command: "T:..." ──
      else if (fullLine.startsWith("T:") || fullLine.startsWith("t:"))
      {
        Drive_ParseTask(fullLine);
        data = 0;
      }
      // ── Existing single-char commands ──
      else if (fullLine.length() == 1 && fullLine.charAt(0) == 'm')
      {
        Serial.print(getTeensySerial());
        Serial.print(" | ");
        Serial.println("Motion Module");
        data = 0;
      }
      else if (fullLine.length() == 1 && fullLine.charAt(0) == 's')
      {
        systemCounter = true;
        printAlter = false;
        data = 0;
        printSetting();
      }
      else if (fullLine.length() == 1 && fullLine.charAt(0) == 'p')
      {
        // Toggle drive pause via serial 'p'
        Drive_TogglePause(data);
        if (Drive_IsPaused())
        {
          data = 0; // zero immediately so no stale command persists
          Serial.println("PAUSE: Triggered by serial 'p'");
        }
        else
        {
          data = Drive_GetSavedData(); // restore saved motion command
          Serial.println("PAUSE: Resumed by serial 'p'");
        }
      }

      else if (fullLine.equalsIgnoreCase("home"))
      {
        Odometry_SetHome();
        data = 0;
      }
      else if (fullLine.equalsIgnoreCase("returnhome"))
      {
        Navigation_ReturnHome();
        data = 0;
      }

      // ── Numeric motion command ──
      else if (fullLine.length() >= 1 && fullLine.charAt(0) != '\n')
      {
        data = fullLine.toInt();
        startTimeControlCounter = currentTimeControlCounter;
      }
    }
    else
    {
      String _data = Serial.readString();
      _data = _data.remove(_data.length() - 1, 1);
      if (_data.length() == 1 && _data == "s")
      {
        Serial.println("Chaging to Control Mode");
        systemCounter = false;
      }
      else if (_data.length() >= 1)
      {
        updatedEEPROM(_data);
      }
    }
  }

  // Control-counter timeout: only when NOT in autonomous drive
  if (elaspedTimeControlCounter > timeConstantControlCounter)
  {
    if (!Drive_IsRunning() && !Navigation_IsRunning() && !Follow_IsRunning())
    {
      data = 0; // Commenting for Testing //TODO:
    }
    startTimeControlCounter = currentTimeControlCounter;
  }

  if (emergency > 900)
  {
    rpmAlter = false;
    // Serial5.write(0);
    // Serial5.write(128);

    // Also zeroes motion.cpp's own ramp state (currentPwmL/currentPwmR), not
    // just the pins — otherwise Drive_Cancel()/Follow_Cancel() below call
    // motion(0), which re-applies that stale nonzero PWM before its own
    // ramp-down can catch up, causing a brief forward jerk right after this.
    motion_EmergencyStop();
    data = 0; // TODO: Commenting for Testing'';

    // Cancel any running autonomous track on the first tick of the emergency latch
    // so that when the emergency is released the robot stays idle (free to drive manually).
    if (emergencyAlter)
    {
      Drive_Cancel();
      Navigation_Cancel();
      Follow_Cancel();
      CamLock_ClearAll();
      Serial.println("CAM: all locks released (emergency)");
      emergencyAlter = false;
    }
  }
  else if (emergency < 900)
  {
    emergencyAlter = true; // Rearm for the next latch event

    // ── Autonomous mode: suppress manual control ──
    // Navigation_IsRunning() covers pursuit/parking (navigation calls motion()
    // directly).  Drive_IsRunning() covers queued commands (drive calls motion()).
    if (Drive_IsRunning() || Navigation_IsRunning() || Follow_IsRunning())
    {
      // Camera locks reach autonomous modes through the front flag only, as
      // before: Navigation reads frontCamLocked for its sidestep, and a
      // queued "T:..." task (Drive) is paused while the front camera is
      // locked. Back/left/right locks are not applied to autonomous modes yet.
      //
      // Only pause/resume Drive's own queue when Drive actually owns the
      // motors right now (a manual "T:..." task, or Navigation's
      // FINAL_ALIGN handoff). Navigation's direct-drive phases react to
      // frontCamLocked themselves — Drive_Update() forces motion(0) every
      // tick while drivePaused is true, so pausing it while Navigation is
      // driving directly would freeze the robot.
      static bool camPausedDrive = false;

      if (!camBypass && Drive_IsRunning() && frontCamLocked && !Drive_IsPaused() && !camPausedDrive)
      {
        Drive_Pause(0);
        camPausedDrive = true;
        Serial.println("PAUSE: front camera locked");
      }
      else if (camPausedDrive && (!frontCamLocked || camBypass))
      {
        if (Drive_IsPaused())
        {
          Drive_Resume();
          Serial.println("RESUME: front camera free");
        }
        camPausedDrive = false;
      }

      // Manual data is ignored while an autonomous task is executing
      data = 0;
    }
    // ── Normal manual control ──
    else
    {
      static unsigned long camBypassTimer = 0;

      // 67 = toggle camera bypass
      if (data == 67)
      {
        if (millis() - camBypassTimer >= 500)
        {
          camBypassTimer = millis();
          camBypass = !camBypass;
          if (camBypass)
            CamLock_ClearAll();
          Serial.println(camBypass ? "CAM BYPASS: ON (ignoring cameras)" : "CAM BYPASS: OFF (cameras active)");
        }
        data = 0;
      }

      // While paused: actively drive motors to 0 every loop to flush any ramp state.
      // A command blocked by a camera lock is stopped instantly (ramp state
      // zeroed) but `data` is left alone, so motion resumes by itself when the
      // lock is released as long as the sender is still holding the command.
      if (Drive_IsPaused())
        motion(0);
      else if (CamLock_IsBlocked(data))
        motion_EmergencyStop();
      else
        motion(data);
    }
  }

  if (elaspedTime > timeConstant)
  {
    startTime = currentTime;

    // RPM Calculation
    rpm_L = ((abs(encoderValue_L) * 60 * handletime(timeConstant)) / 4000.00); // Change According to the time constant.
    rpm_R = ((abs(encoderValue_R) * 60 * handletime(timeConstant)) / 4000.00);
    // rpm = (No of Pluses/Total Pules) * 1sec
    // Total Pulse = Pulse * 4 where is changes in both the phases
    // SEC -> MilliSec 60*100 -> TimeConstant will be 100
    // SEC -> MilliSec 60*1000 -> TimeConstant will be 10

    avgRPM_L = filter_L(rpm_L);
    avgRPM_R = filter_R(rpm_R);

    Distance_Update(
        encoderValue_L,
        encoderValue_R);

    Velocity_Update(elaspedTime / 1000.0f);

    Odometry_Update();

    // encoderValue_* is zeroed every tick; keep running signed totals for the
    // debug print (ticks per wheel rev calibration, direction checks).
    totalTicks_L += encoderValue_L;
    totalTicks_R += encoderValue_R;

    encoderValue_L = encoderValue_R = 0;

    Navigation_Update();
    Drive_Update();

    // Print debug at 200ms to avoid serial flooding
    static unsigned long bnoPrintTimer = 0;

    if (millis() - bnoPrintTimer >= 200)
    {
      bnoPrintTimer = millis();

      // Serial.print(data);
      // Serial.print(" | ");

      // Serial.print(" | Dist=");
      // Serial.print(distance.travelled, 3);

      // Serial.print(" | Left =");
      // Serial.print(distance.left, 3);

      // Serial.print(" | Right =");
      // Serial.println(distance.right, 3);

      // Serial.print(" | V=");
      // Serial.print(velocity.linear, 3);

      // Serial.print(" m/s");

      // Serial.print(" | W=");
      // Serial.print(velocity.angular, 3);

      // Serial.println(" rad/s");

      // Serial.print(" D=");
      // Serial.print(distance.travelled, 3);

      // Serial.print(" OD=");
      // Serial.print(distance.odometryTravelled, 3);

      // Serial.print(" | X=");
      // Serial.print(odometry.x, 3);

      // Serial.print(" | Y=");
      // Serial.print(odometry.y, 3);

      // Serial.print(" | EncL=");
      // Serial.print(totalTicks_L);

      // Serial.print(" | EncR=");
      // Serial.println(totalTicks_R);

      // Serial.print(" | H=");
      // Serial.println(odometry.heading, 3);
      

      // Serial.println(" rad");

      // Serial.print(" Heading=");
      // Serial.print(currentHeading);

      // Serial.print(" | Target=");
      // Serial.print(targetHeading);

      // Serial.print(" | Error=");
      // Serial.print(pidError);

      // Serial.print(" | Corr=");
      // Serial.print(pidCorrection);

      // // // Serial.print(" | L=");
      // // // Serial.print(debugPwmL);

      // // // Serial.print(" | R=");
      // // // Serial.print(debugPwmR);

      // Serial.print(" | I=");
      // Serial.println(headingIntegral);
    }
  }

  // Read BNO heading at ~100Hz (every 10ms) so PID has fresh data
  // This MUST be outside the 100ms RPM timer block
  static unsigned long bnoReadTimer = 0;
  if (millis() - bnoReadTimer >= 10)
  {
    bnoReadTimer = millis();
    bnoStandaloneLoop();
  }

  // Send IMU and Odometry data to Jetson at 50Hz (every 20ms)
  static unsigned long sensorSendTimer = 0;
  if (millis() - sensorSendTimer >= 20)
  {
    sensorSendTimer = millis();
    
    // Format: SENSOR:qw,qx,qy,qz,liaX,liaY,liaZ,odomX,odomY,odomH,vLin,vAng
    // Serial.print("SENSOR:");
    // Serial.print(imuQw, 4); Serial.print(",");
    // Serial.print(imuQx, 4); Serial.print(",");
    // Serial.print(imuQy, 4); Serial.print(",");
    // Serial.print(imuQz, 4); Serial.print(",");
    
    // Serial.print(imuLiaX, 4); Serial.print(",");
    // Serial.print(imuLiaY, 4); Serial.print(",");
    // Serial.print(imuLiaZ, 4); Serial.print(",");
    
    // Serial.print(odometry.x, 4); Serial.print(",");
    // Serial.print(odometry.y, 4); Serial.print(",");
    // Serial.print(odometry.heading, 4); Serial.print(",");
    
    // Serial.print(velocity.linear, 4); Serial.print(",");
    // Serial.println(velocity.angular, 4);
  }



  // Autonomous drive — runs every loop() cycle for tight heading control
  Drive_Update();

  // Follow-me — MUST run after Drive_Update(): while paused Drive_Update()
  // forces motion(0) every tick, and Follow_Update() checks Drive_IsPaused()
  // itself so it freezes rather than overwriting that stop on the same tick.
  Follow_Update();
}
