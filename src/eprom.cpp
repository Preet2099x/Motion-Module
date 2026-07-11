#include <var.h>
#include <EEPROM.h>
#include <Arduino.h>

void printSetting()
{
  Serial.println("Chaging to Setting Mode");
  Serial.print(" T -> Trun \n F -> Forward \n R -> Right \n L -> Left \n B -> Backward \n W -> Work \n D -> Drive \n");
  Serial.println("Enter With as shown with the range below.");
  Serial.println();
  Serial.print(" TRR : Values form 0 - 63    \n TRL : Values form 192 - 255\n");
  Serial.print(" TLR : Values form 64 - 127  \n TLL : Values form 128 - 191\n");
  Serial.println();
  Serial.print(" FRW : Values form 0 - 40    \n FLW : Values form 128 - 168\n");
  Serial.print(" BRW : Values form 64 - 104  \n BLW : Values form 192 - 232\n");
  Serial.println();
  Serial.print(" FRD : Values form 40 - 63    \n FLD : Values form 168 - 191\n");
  Serial.print(" BRD : Values form 104 - 127  \n BLD : Values form 232 - 255\n");
}

void updatedEEPROM(String _data)
{
  if (_data.substring(0, _data.indexOf(':')) == "TRR")
  {
    TRR = handleEEPROMwrite(_data, addressTRR);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "TRL")
  {
    TRL = handleEEPROMwrite(_data, addressTRL);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "TLR")
  {
    TLR = handleEEPROMwrite(_data, addressTLR);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "TLL")
  {
    TLL = handleEEPROMwrite(_data, addressTLL);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "FRW")
  {
    FRW = handleEEPROMwrite(_data, addressFRW);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "FLW")
  {
    FLW = handleEEPROMwrite(_data, addressFLW);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "BRW")
  {
    BRW = handleEEPROMwrite(_data, addressBRW);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "BLW")
  {
    BLW = handleEEPROMwrite(_data, addressBLW);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "FRD")
  {
    FRD = handleEEPROMwrite(_data, addressFRD);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "FLD")
  {
    FLD = handleEEPROMwrite(_data, addressFLD);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "BRD")
  {
    BRD = handleEEPROMwrite(_data, addressBRD);
  }
  else if (_data.substring(0, _data.indexOf(':')) == "BLD")
  {
    BLD = handleEEPROMwrite(_data, addressBLD);
  }
  else
  {
    Serial.println("Enter Correct Variable");
  }
}

int handleEEPROMwrite(String _data, unsigned int address)
{
  EEPROM.update(address, _data.substring(_data.indexOf(':')).remove(0, 1).toInt());
  Serial.print("Data Updated | ");
  Serial.print(_data.substring(0, _data.indexOf(':')));
  Serial.print(" : ");
  Serial.println(EEPROM.read(address));
  return EEPROM.read(address);
}

void readEEPROM()
{ // If Address id updated. Updated the same at Readme File!!
  FLW = EEPROM.read(addressFLW);
  FRW = EEPROM.read(addressFRW);
  BLW = EEPROM.read(addressBLW);
  BRW = EEPROM.read(addressBRW);

  FLD = EEPROM.read(addressFLD);
  FRD = EEPROM.read(addressFRD);
  BLD = EEPROM.read(addressBLD);
  BRD = EEPROM.read(addressBRD);

  TRR = EEPROM.read(addressTRR);
  TRL = EEPROM.read(addressTRL);
  TLR = EEPROM.read(addressTLR);
  TLL = EEPROM.read(addressTLL);
}