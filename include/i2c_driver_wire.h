#ifndef I2C_DRIVER_WIRE_H
#define I2C_DRIVER_WIRE_H

#include <Arduino.h>
#include <Wire.h>

class I2CDriverWire
{
public:
  void begin()
  {
    Wire.begin();
  }

  void begin(uint8_t address)
  {
    Wire.begin(address);
  }

  void setClock(uint32_t clockHz)
  {
    Wire.setClock(clockHz);
  }

  void onReceive(void (*handler)(int))
  {
    Wire.onReceive(handler);
  }

  int available()
  {
    return Wire.available();
  }

  int read()
  {
    return Wire.read();
  }

  size_t write(uint8_t value)
  {
    return Wire.write(value);
  }
};

extern I2CDriverWire kire;

#endif