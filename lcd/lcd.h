#ifndef _LCD_H
#define _LCD_H

#define CLEAR_DISPLAY     0x01
#define RETURN_HOME       0x02

#define ENTRY_MODE        0x04
#define SHIFT_ENABLE      (ENTRY_MODE | (1 << 0))
#define SHIFT_DISABLE     (ENTRY_MODE &~(1 << 0))
#define CUR_INCREMENT     (ENTRY_MODE | (1 << 1))
#define CUR_DECREMENT     (ENRTY_MODE &~(1 << 1))

#define DISPLAY_CONTROL   0x08
#define DISPLAY_ON        (DISPLAY_CONTROL | (1 << 2))
#define DISPLAY_OFF       (DISPLAY_CONTROL &~(1 << 2))
#define CURSOR_ON         (DISPLAY_ON | (1 << 1))
#define CURSON_OFF        (DISPLAY_ON &~(1 << 1))
#define BLINK_ON          (DISPLAY_ON | (1 << 0))
#define BLINK_OFF         (DISPLAY_ON &~(1 << 0))

#define DISPLAY_SHIFT     0x10
#define ENTIRE_LINE       (DISPLAY_SHIFT &~(1 << 3))
#define CURRENT_LINE      (DISPLAY_SHIFT | (1 << 3))
#define SHIFT_LEFT        (DISPLAY_SHIFT &~(1 << 2))
#define SHIFT_RIGHT       (DISPLAY_SHIFT | (1 << 2))

#define FUNCTION_SET      0x20
#define EIGHT_BIT         (FUNCTION_SET | (1 << 4))
#define FOUR_BIT          (FUNCTION_SET &~(1 << 4))
#define TWO_LINE          (FUNCTION_SET | (1 << 3))
#define ONE_LINE          (FUNCTION_SET &~(1 << 3))
#define CHAR_SET2         (FUNCTION_SET | (1 << 2))
#define CHAR_SET1         (FUNCTION_SET &~(1 << 2))

#define CGRAM_ADDRESS     0x40

extern void LCD_Enable();
extern void LCD_Clear();
extern void LCD_Initialize(char, char, char, char, char, char);
extern void LCD_Command(unsigned char);
extern void LCD_Write(unsigned char, unsigned char);
extern void LCD_Disp(unsigned char, const char*);
extern void LCD_CustomChar(unsigned char, const char*);
extern void LCD_CustomDisp(unsigned char, unsigned char, const char*);
extern void LCD_Decimal(unsigned char, unsigned long, int, int);
extern void LCD_Float(unsigned char, float, int, int, int);
extern void LCD_Allort(unsigned char);
#endif
