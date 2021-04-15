#include <Arduino.h>
#include <avr/io.h>

#ifndef _UTILS_H
#include "utils.h"
#endif

#include "lcd.h"

char RegisterSelect;
char PinEnable;
char PinD4;
char PinD5;
char PinD6;
char PinD7;

void RollEnable()
{
  digitalWrite(PinEnable, LOW); delayMicroseconds(5);
  digitalWrite(PinEnable, HIGH); delayMicroseconds(5);
  digitalWrite(PinEnable, LOW); delayMicroseconds(5);
}

void LCDClear()
{
  LCD_Command(0x01);
  delay(20);
}

void LCD_Initialize(char RS, char EN, char D4, char D5, char D6, char D7)
{
  RegisterSelect = RS;
  PinEnable = EN;
  PinD4 = D4;
  PinD5 = D5;
  PinD6 = D6;
  PinD7 = D7;
  
  pinMode(PinD4, OUTPUT);
  pinMode(PinD5, OUTPUT);
  pinMode(PinD6, OUTPUT);
  pinMode(PinD7, OUTPUT);
  pinMode(RegisterSelect, OUTPUT);
  pinMode(PinEnable, OUTPUT);

  digitalWrite(PinD4, LOW);
  digitalWrite(PinD5, LOW);
  digitalWrite(PinD6, LOW);
  digitalWrite(PinD7, LOW);
  digitalWrite(RegisterSelect, LOW);
  digitalWrite(PinEnable, LOW);
  DelayMS(200);

  LCD_Command(0x03);
  _delay_ms(4.5);
  LCD_Command(0x03);
  _delay_ms(4.5);
  LCD_Command(0x03);
  _delay_us(150);
  LCD_Command(0x02);
  _delay_us(150);
  LCD_Command(0x28);
  _delay_ms(4.5);
  LCD_Command(0x28);
  _delay_ms(4.5);
  LCD_Command(0x28);
  _delay_us(150);
  LCD_Command(0x06);
  _delay_us(150);
  LCD_Command(0x0C);
  _delay_us(150);
  LCDClear();
}

void LCD_Command(unsigned char CommandData)
{
  digitalWrite(RegisterSelect, LOW);
  LCD_Allort(CommandData & 0xF0); RollEnable();
  LCD_Allort((CommandData << 4) & 0xF0); RollEnable();
}

void LCD_Write(unsigned char CommandData, unsigned char DisplayData)
{
  LCD_Command(CommandData);
  digitalWrite(RegisterSelect, HIGH);
  LCD_Allort(DisplayData & 0xF0); RollEnable();
  LCD_Allort((DisplayData << 4) & 0xF0); RollEnable();
}

void LCD_Disp(unsigned char DisplayAddress, const char *DisplayData)
{
  while(*DisplayData) LCD_Write(DisplayAddress++, *DisplayData++);
}

void LCD_Decimal(unsigned char DisplayAddress, unsigned long DecimalValue, char Length, const char Base)
{
  unsigned char LocalArray[10];
  dtoa(DecimalValue, (char*)LocalArray, Length, Base);
  LCD_Disp(DisplayAddress, (char*)LocalArray);
}

void LCD_Float(unsigned char DisplayAddress, float Value, char PostFix, char PreFix, const char Seg)
{
  unsigned char LocalArray[10];
  ftostra(Value, PostFix, PreFix, (char*)LocalArray, Seg);
  LCD_Disp(DisplayAddress, (char*)LocalArray);
}

void LCD_Allort(unsigned char Data)
{
  digitalWrite(PinD4, (Data & 0x10) ?HIGH :LOW);
  digitalWrite(PinD5, (Data & 0x20) ?HIGH :LOW);
  digitalWrite(PinD6, (Data & 0x40) ?HIGH :LOW);
  digitalWrite(PinD7, (Data & 0x80) ?HIGH :LOW);
}

void LCD_CustomChar(unsigned char Location, const char *DisplayData)
{
  Location *= 8;
  for(char k = 0; k < 8; k++) LCD_Write((0x40 + Location + k), DisplayData[k]);
}

void LCD_CustomDisp(unsigned char DisplayAddress, unsigned char T_Location, const char *DisplayData)
{
  LCD_CustomChar(T_Location, DisplayData); 
  LCD_Write(DisplayAddress, T_Location);
}
