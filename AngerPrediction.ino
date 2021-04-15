#include <Arduino.h>
#include <avr/io.h>
#include <Wire.h>

#include "utils.h"
#include "lcd.h"
#include "MAX30100_PulseOximeter.h"

#ifndef LCDPinConfig
#define LCDPinConfig
#define RS 13
#define EN 12
#define D4 11
#define D5 10
#define D6  9
#define D7  8
#endif

#ifndef AlarmConfig
#define AlarmConfig
#define Alarm 7
#endif

#ifndef UpdateInterval
#define UpdateInterval
#define DataUpdateInterval 2000
#endif

#ifndef UserConfig
#define UserConfig
#define TempMin       30.00F
#define TempMax       40.00F
#define TempErrMin    25.00F
#define TempErrMax    50.00F
#define HeartMin      60.00F
#define HeartMax      100.00F
#define HeartErrMin   50.00F
#define HeartErrMax   120.00F
#define SPO2Min       85
#define SPO2Max       100
#define SPO2ErrMin    80
#define SPO2ErrMax    110
#endif

const char *Degree = "\x06\x09\x09\x06\x00\x00\x00\x00";
unsigned long DataTimer, WebTimer;
float Temperature, HeartRate;
unsigned int PulseOxiRange, ISStatusNormal = True;

PulseOximeter SPO2;

unsigned char readRegister(uint8_t address)
{
    Wire.beginTransmission(MAX30100_I2C_ADDRESS);
    Wire.write(address);
    Wire.endTransmission(false);
    Wire.requestFrom(MAX30100_I2C_ADDRESS, 1);
    return Wire.read();
}

unsigned char writeRegister(uint8_t address, uint8_t data)
{
    Wire.beginTransmission(MAX30100_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTransmission();
}

void SPO2_StartTemperatureSampling()
{
  unsigned char ModeConfig = readRegister(MAX30100_REG_MODE_CONFIGURATION);
  ModeConfig |= MAX30100_MC_TEMP_EN;
  writeRegister(MAX30100_REG_MODE_CONFIGURATION, ModeConfig);
}

char SPO2_isTemperatureReady()
{
  return !(readRegister(MAX30100_REG_MODE_CONFIGURATION) & MAX30100_MC_TEMP_EN);
}

float SPO2_retrieveTemperature()
{
  unsigned char TempInteger = readRegister(MAX30100_REG_TEMPERATURE_DATA_INT);
  unsigned char TempFrac = readRegister(MAX30100_REG_TEMPERATURE_DATA_FRAC);
  return TempFrac * 0.0625F + TempInteger;
}

void setup()
{
  pinMode(Alarm, OUTPUT);
  digitalWrite(Alarm, LOW);

  LCD_Initialize(RS, EN, D4, D5, D6, D7);
  LCD_Disp(0x80,"ANGER PREDICTION");
  LCD_Disp(0xC0,"USING DEEP LEARN");
  DelayMS(2500); LCDClear();

  SPO2.begin();
  SPO2.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  SPO2_StartTemperatureSampling();
}

void loop()
{
  SPO2.update();

  LCD_Disp(0x80, "P:");
  LCD_Float(0x82, HeartRate, 3, 2, NoSign);
  LCD_Disp(0x88, "BPM");
  LCD_Decimal(0x8C, PulseOxiRange, 3, DEC);
  LCD_Write(0x8F, '%');

  LCD_Disp(0xC0, "T:");
  LCD_Float(0xC2, Temperature, 2, 2, NoSign);
  LCD_CustomDisp(0xC7, 0, Degree);
  LCD_Write(0xC8, 'C');

  if(ISStatusNormal == 0) LCD_Disp(0xCA, "ERROR ");
  else if(ISStatusNormal == 1) LCD_Disp(0xCA, "ANGER ");
  else if(ISStatusNormal == 3) LCD_Disp(0xCA, "FEAR  ");
  else if(ISStatusNormal == 4) LCD_Disp(0xCA, "WALK  ");
  else if(ISStatusNormal == 5) LCD_Disp(0xCA, "NORMAL");

  digitalWrite(Alarm, (ISStatusNormal == 1 ?HIGH :LOW));

  if(millis() - DataTimer > DataUpdateInterval)
  {
    HeartRate = SPO2.getHeartRate();
    PulseOxiRange = SPO2.getSpO2();
    Temperature = SPO2_retrieveTemperature();
    SPO2_StartTemperatureSampling();
    DataTimer = millis();

    ISStatusNormal = True;

    if(Temperature < TempErrMin || Temperature > TempErrMax) ISStatusNormal = False;
    else if(HeartRate < HeartErrMin || HeartRate > HeartErrMax) ISStatusNormal = False;

    if(ISStatusNormal == 1)
    {
      if((Temperature < TempErrMax && Temperature > TempMax) && (HeartRate < HeartErrMax && HeartRate > HeartMax)) ISStatusNormal = 1;
      else if((Temperature > TempErrMin && Temperature < TempMin) && (HeartRate < HeartErrMax && HeartRate > HeartMax)) ISStatusNormal = 2;
      else if((Temperature > TempErrMin && Temperature < TempMin) && (HeartRate > HeartErrMin && HeartRate < HeartMin)) ISStatusNormal = 3;
      else ISStatusNormal = 4;
    }
  }
}

