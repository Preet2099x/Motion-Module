#include <Wire.h>
#include <var.h>

#define BNO055_ADDR 0x28

float currentHeading = 0;

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
  currentHeading = read16(0x1A) / 16.0;
  float heading = currentHeading;
  float roll = read16(0x1C) / 16.0;
  float pitch = read16(0x1E) / 16.0;

  // Gyroscope (deg/s * 16)
  float gx = read16(0x14) / 16.0;
  float gy = read16(0x16) / 16.0;
  float gz = read16(0x18) / 16.0;

  // Serial.print("H: "); Serial.print(heading);
  // Serial.print(" | P: "); Serial.print(pitch);
  // Serial.print(" | R: "); Serial.print(roll);

  // Serial.print(" || Gx: "); Serial.print(gx);
  // Serial.print(" Gy: "); Serial.print(gy);
  // Serial.print(" Gz: "); Serial.println(gz);

  // delay(200);
}