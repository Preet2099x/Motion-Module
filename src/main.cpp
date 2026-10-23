#include <var.h>
#include <maf.h>
#include <string.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <i2c_driver.h>
#include <TeensyThreads.h>
#include <i2c_driver_wire.h>

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

int encoderPin_1_L = 7; // 23
int encoderPin_2_L = 8; // 22
int encoderPin_1_R = 5; // 14
int encoderPin_2_R = 4; // 20

bool turnBoost = false;
bool straightBoost = false;

bool frontCamLocked = false;
bool backCamLocked = false;
bool camBypass = false;

volatile int lastEncoded_L = 0;
volatile long encoderValue_L = 0;
long lastencoderValue_L = 0;
// int lastMSB_L = 0;
// int lastLSB_L = 0;

volatile int lastEncoded_R = 0;
volatile long encoderValue_R = 0;
long lastencoderValue_R = 0;
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
int _dirData = 0;

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
      int _data = Serial.read();
      if (_data == char('m'))
      {
        Serial.print(getTeensySerial());
        Serial.print(" | ");
        Serial.println("Motion Module");
        data = 0;
      }
      else if (_data == char('s'))
      {
        systemCounter = true;
        printAlter = false; // TODO: Can be Removed for fast testing
        data = 0;           // TODO: Can be Removed for fast testing
        printSetting();
      }
      else if (_data == char('p'))
      {
        printAlter = !printAlter;
      }
      else if (_data != 10)
      {
          // Read the rest of the line and parse as an integer
          String numStr = Serial.readStringUntil('\n');
          numStr = String((char)_data) + numStr;  // prepend the first byte already read
          data = numStr.toInt();
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

  if (elaspedTimeControlCounter > timeConstantControlCounter)
  {
    data = 0; // Commenting for Testing //TODO:
    startTimeControlCounter = currentTimeControlCounter;
  }

  if (emergency > 900)
  {
    rpmAlter = false;
    // Serial5.write(0);
    // Serial5.write(128);
    digitalWrite(dirPin_L, LOW);
    digitalWrite(dirPin_R, LOW);
    analogWrite(pwmPin_L, 0);
    analogWrite(pwmPin_R, 0);
    data = 0; // TODO: Commenting for Testing'';
  }
  else if (emergency < 900)
  {
    // --- Camera stop logic ---
    static unsigned long frontLockTime = 0;
    static unsigned long backLockTime = 0;
    static unsigned long camBypassTimer = 0;

    // 67 = toggle camera bypass
    if (data == 67)
    {
      if (millis() - camBypassTimer >= 500)
      {
        camBypassTimer = millis();
        camBypass = !camBypass;
        Serial.println(camBypass ? "CAM BYPASS: ON (ignoring cameras)" : "CAM BYPASS: OFF (cameras active)");
      }
      data = 0;
    }

    // Track last motion command (skip camera cmds so _dirData always holds the real motion)
    if (data != 101 && data != 102 && data != 201 && data != 202 && data != 67)
    {
      _dirData = data;
    }

    if (!camBypass)
    {
      // Front camera: 101 lock, 201 release
      if (data == 101)
      {
        frontCamLocked = true;
        frontLockTime = millis();
        Serial.println("FRONT CAM: LOCKED");
        // Only kill motors if current motion is a front-blocked command
        // Front-blocked: 1 (forward), 111-120 (fwd-left curve), 121-130 (fwd-right curve)
        if (_dirData == 1 || (_dirData >= 111 && _dirData <= 130))
        {
          analogWrite(pwmPin_L, 0);
          analogWrite(pwmPin_R, 0);
          digitalWrite(dirPin_L, LOW);
          digitalWrite(dirPin_R, LOW);
          data = 0;
        }
        else
        {
          // Not a front-moving command (e.g. turning) — keep current motion, just block future front cmds
          data = _dirData;
        }
      }
      else if (data == 201)
      {
        if (frontCamLocked && (millis() - frontLockTime >= 500))
        {
          frontCamLocked = false;
          Serial.println("FRONT CAM: FREE");
        }
        data = _dirData;
      }

      // Back camera: 102 lock, 202 release
      if (data == 102)
      {
        backCamLocked = true;
        backLockTime = millis();
        Serial.println("BACK CAM: LOCKED");
        // Only kill motors if current motion is a back-blocked command
        // Back-blocked: 2 (backward), 211-220 (bwd-left curve), 221-230 (bwd-right curve)
        if (_dirData == 2 || (_dirData >= 211 && _dirData <= 230))
        {
          analogWrite(pwmPin_L, 0);
          analogWrite(pwmPin_R, 0);
          digitalWrite(dirPin_L, LOW);
          digitalWrite(dirPin_R, LOW);
          data = 0;
        }
        else
        {
          // Not a back-moving command (e.g. turning) — keep current motion, just block future back cmds
          data = _dirData;
        }
      }
      else if (data == 202)
      {
        if (backCamLocked && (millis() - backLockTime >= 500))
        {
          backCamLocked = false;
          Serial.println("BACK CAM: FREE");
        }
        data = _dirData;
      }

      // Block commands when front camera locked: 1, 111-120, 121-130
      if (frontCamLocked)
      {
        if (data == 1 || (data >= 111 && data <= 130))
          data = 0;
      }

      // Block commands when back camera locked: 2, 211-220, 221-230
      if (backCamLocked)
      {
        if (data == 2 || (data >= 211 && data <= 230))
          data = 0;
      }
    }

    // Update _dirData after blocking so it reflects what motion actually runs
    if (data != 0)
      _dirData = data;

    motion(data);
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

    //   if(printAlter == true) {
    //     Serial.print(data);
    //     Serial.print(" | ");
    //     //Serial.print(rpmAlter);
    //     //Serial.print(" | ");
    //     //Serial.print(rpmAlter_T);
    //     //Serial.print(" | ");
    //     Serial.print(emergency);
    //     Serial.print(" | ");
    //     Serial.print(avgRPM_L);
    //     Serial.print(" | ");
    //     Serial.print(avgRPM_R);
    //     Serial.print(" | ");
    //     Serial.print(1.0/16*imu.eul_heading);
    //     Serial.print(" | ");
    //     Serial.print(1.0/16*imu.eul_roll);
    //     Serial.print(" | ");
    //     Serial.print(1.0/16*imu.eul_pitch);
    //     Serial.print(" | ");
    //     Serial.println(imu.temp);
    //     /*Serial.print(" | ");
    //     Serial.print((s.calib_stat >> 6) & 3);
    //     Serial.print(" | ");
    //     Serial.print((s.calib_stat >> 4) & 3);
    //     Serial.print(" | ");
    //     Serial.print((s.calib_stat >> 2) & 3);
    //     Serial.print(" | ");
    //     Serial.println((s.calib_stat >> 0) & 3); */
    //  }

    encoderValue_L = encoderValue_R = 0;

    // Print debug at 200ms to avoid serial flooding
    static unsigned long bnoPrintTimer = 0;

    if (millis() - bnoPrintTimer >= 200)
    {
      bnoPrintTimer = millis();

      Serial.print(data);
      Serial.print(" | ");

      Serial.print(" Heading=");
      Serial.print(currentHeading);

      Serial.print(" | Target=");
      Serial.print(targetHeading);

      Serial.print(" | Error=");
      Serial.print(pidError);

      Serial.print(" | Corr=");
      Serial.print(pidCorrection);

      Serial.print(" | L=");
      Serial.print(debugPwmL);

      Serial.print(" | R=");
      Serial.print(debugPwmR);

      Serial.print(" | I=");
      Serial.println(headingIntegral);
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
}
