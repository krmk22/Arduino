#ifndef _LCD_H
#define _LCD_H

void RollEnable();
void LCDClear();

void LCD_Initialize(char, char, char, char, char, char);
void LCD_Command(unsigned char);
void LCD_Write(unsigned char, unsigned char);
void LCD_Disp(unsigned char, const char*);
void LCD_Decimal(unsigned char, unsigned long, char, const char);
void LCD_Float(unsigned char, float, char, char, const char);
void LCD_Allort(unsigned char);
void LCD_CustomChar(unsigned char, const char*);
void LCD_CustomDisp(unsigned char, unsigned char, const char*);
#endif
