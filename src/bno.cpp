#include <Wire.h>
#include <var.h>

#define BNO055_ADDR 0x28

// 1 = mirror the BNO heading (heading falls when the robot turns right).
// Set to 0 if a robot's IMU/motor wiring already counts the right way.
#define HEADING_INVERT 1

float currentHeading = 0;
float imuRoll = 0;
float imuPitch = 0;

float imuGx = 0;
float imuGy = 0;
float imuGz = 0;

float imuAx = 0;
float imuAy = 0;
float imuAz = 0;

float imuQw = 1.0;
float imuQx = 0;
float imuQy = 0;
float imuQz = 0;

float imuLiaX = 0;
float imuLiaY = 0;
float imuLiaZ = 0;

int16_t read16(uint8_t reg)
{
  Wire1.beginTransmission(BNO055_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission();
  Wire1.requestFrom(BNO055_ADDR, 2);

  uint8_t lsb = Wire1.read();
  uint8_t msb = Wire1.read();
  return (int16_t)(lsb | (msb << 8));
}

void write8(uint8_t reg, uint8_t val)
{
  Wire1.beginTransmission(BNO055_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  Wire1.endTransmission();
}

void bnoStandaloneSetup()
{
  // Serial.begin(115200);
  Wire1.begin();

  delay(700);

  // CONFIG mode
  write8(0x3D, 0x00);
  delay(20);

  // Normal power
  write8(0x3E, 0x00);
  delay(10);

  // Page 0
  write8(0x07, 0x00);

  // NDOF mode (fusion)
  write8(0x3D, 0x0C);
  delay(20);

  Serial.println("BNO055 ready");
}

void bnoStandaloneLoop()
{
  // Euler angles (deg * 16)
  float rawHeading = read16(0x1A) / 16.0;
  // Everything (heading PID, Drive turns, Navigation) assumes heading
  // INCREASES when the left wheel is faster / robot turns right (code 15).
  // On this robot the BNO counts the other way, so mirror it.
#if HEADING_INVERT
  currentHeading = (rawHeading > 0.0f) ? 360.0f - rawHeading : 0.0f;
#else
  currentHeading = rawHeading;
#endif
  float heading = currentHeading;
  float roll = read16(0x1C) / 16.0;
  float pitch = read16(0x1E) / 16.0;

  // Gyroscope (deg/s * 16)
  float gx = read16(0x14) / 16.0;
  float gy = read16(0x16) / 16.0;
  float gz = read16(0x18) / 16.0;
  
  imuGx = gx;
  imuGy = gy;
  imuGz = gz;

  // Quaternions (1 unit = 1/16384)
  imuQw = read16(0x20) / 16384.0;
  imuQx = read16(0x22) / 16384.0;
  imuQy = read16(0x24) / 16384.0;
  imuQz = read16(0x26) / 16384.0;

  // Linear Acceleration (1 unit = 1 m/s^2 / 100)
  imuLiaX = read16(0x28) / 100.0;
  imuLiaY = read16(0x2A) / 100.0;
  imuLiaZ = read16(0x2C) / 100.0;

  // Serial.print("H: "); Serial.print(heading);
  // Serial.print(" | P: "); Serial.print(pitch);
  // Serial.print(" | R: "); Serial.print(roll);

  // Serial.print(" || Gx: "); Serial.print(gx);
  // Serial.print(" Gy: "); Serial.print(gy);
  // Serial.print(" Gz: "); Serial.println(gz);

  // delay(200);
}