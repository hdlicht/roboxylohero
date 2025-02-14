/*
 * LCD_GFX.h
 *
 * Created: 9/20/2021 6:54:37 PM
 *  Author: You
 */ 

#include <avr/io.h>
#include "ASCII_LUT.h"

#ifndef LCD_GFX_H_
#define LCD_GFX_H_

// colors

#define	BLACK     0x0000
#define WHITE     0xFFFF
#define	BLUE      ((((31*(255+4))/255)<<11)|(((63*(0+2))/255)<<5)|((31*(0+4))/255))
#define	ORANGE    ((((31*(0+4))/255)<<11)|(((63*(165+2))/255)<<5)|((31*(255+4))/255))
#define	RED       ((((31*(0+4))/255)<<11)|(((63*(0+2))/255)<<5)|((31*(255+4))/255))
#define	GREEN     ((((31*(0+4))/255)<<11)|(((63*(255+2))/255)<<5)|((31*(0+4))/255))
#define CYAN      ((((31*(255+4))/255)<<11)|(((63*(255+2))/255)<<5)|((31*(20+4))/255))
#define MAGENTA   ((((31*(255+4))/255)<<11)|(((63*(0+2))/255)<<5)|((31*(255+4))/255))
#define YELLOW    ((((31*(0+4))/255)<<11)|(((63*(255+2))/255)<<5)|((31*(255+4))/255))


uint16_t rgb565(uint8_t blue,  uint8_t green, uint8_t red);
void LCD_drawPixel(uint16_t x, uint16_t y, uint16_t color);
void LCD_drawChar(uint16_t x, uint16_t y, uint16_t character, uint16_t fColor, uint16_t bColor);
void LCD_drawCircle(uint16_t x0, uint16_t y0, uint16_t radius,uint16_t color, uint16_t bg);
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c);
void LCD_drawBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color);
void LCD_setScreen(uint16_t color);
void LCD_drawString(uint16_t x, uint16_t y, char* str, uint16_t fg, uint16_t bg);

// colors
/*#define	BLACK     0x0000
#define WHITE     0xFFFF
#define	BLUE      0x001F
#define	RED       0xF800
#define	GREEN     0x07E0
#define CYAN      0x07FF
#define MAGENTA   0xF81F
#define YELLOW    0xFFE0
*/

#endif /* LCD_GFX_H_ */