/*
 * LCD_GFX.c
 *
 * Created: 9/20/2021 6:54:25 PM
 *  Author: You
 */ 

#include "LCD_GFX.h"
#include "HX8357.h"
#include <string.h>

/******************************************************************************
* Local Functions
******************************************************************************/



/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			uint16_t rgb565(uint16_t red, uint16_t green, uint16_t blue)
* @brief		Convert RGB888 value to RGB565 16-bit color data
* @note
*****************************************************************************/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
	return ((((31*(red+4))/255)<<11) | (((63*(green+2))/255)<<5) | ((31*(blue+4))/255));
}

/**************************************************************************//**
* @fn			void LCD_drawPixel(uint16_t x, uint16_t y, uint16_t color)
* @brief		Draw a single pixel of 16-bit rgb565 color to the x & y coordinate
* @note
*****************************************************************************/
void LCD_drawPixel(uint16_t x, uint16_t y, uint16_t color) {
	LCD_setAddr(x,y,x,y);
	SPI_ControllerTx_16bit(color);
}

/**************************************************************************//**
* @fn			void LCD_drawChar(uint16_t x, uint16_t y, uint16_t character, uint16_t fColor, uint16_t bColor)
* @brief		Draw a character starting at the point with foreground and background colors
* @note
*****************************************************************************/
#define SCALE_FACTOR 3 // Set the scale factor for the characters

void LCD_drawChar(uint16_t x, uint16_t y, uint16_t character, uint16_t fColor, uint16_t bColor){
	uint16_t row = character - 0x20;		//Determine row of ASCII table starting at space
	int i, j;
	if ((LCD_WIDTH - x > 7 * SCALE_FACTOR) && (LCD_HEIGHT - y > 7 * SCALE_FACTOR)) {
		for(i=4;i>-1;i--){
			uint16_t pixels = ASCII[row][i]; //Go through the list of pixels
			for(j=0;j<8;j++){
				if ((pixels>>j)&1==1){
					LCD_drawBlock(x+i*SCALE_FACTOR,y-(j+1)*SCALE_FACTOR,x+(i+1)*SCALE_FACTOR-1,y-j*SCALE_FACTOR,fColor);
				}
				
				else {
					LCD_drawBlock(x+i*SCALE_FACTOR,y-(j+1)*SCALE_FACTOR,x+(i+1)*SCALE_FACTOR-1,y-j*SCALE_FACTOR,bColor);
				}
			}
		}
	}
}


/******************************************************************************
* LAB 4 TO DO. COMPLETE THE FUNCTIONS BELOW.
* You are free to create and add any additional files, libraries, and/or
*  helper function. All code must be authentically yours.
******************************************************************************/

/**************************************************************************//**
* @fn			void LCD_drawCircle(uint16_t x0, uint16_t y0, uint16_t radius,uint16_t color)
* @brief		Draw a colored circle of set radius at coordinates
* @note
*****************************************************************************/
void LCD_drawCircle(uint16_t x0, uint16_t y0, uint16_t radius,uint16_t color, uint16_t bg)
{
	// Fill this out
	int16_t x1 = x0 - radius;
	int16_t x2 = x0 + radius;
	int16_t y1 = y0 - radius;
	int16_t y2 = y0 + radius;
	LCD_setAddr(x1, y1, x2, y2);
	clear(LCD_TFT_CS_PORT, LCD_TFT_CS);
	for (int16_t i = x1; i <= x2; i++) {
		for (int16_t j = y1; j <= y2; j++) {
			int16_t dx = i - x0;
			int16_t dy = j - y0;
			if (dx*dx + dy*dy <= radius * radius)
			SPI_ControllerTx_16bit_stream(color);
			else
			SPI_ControllerTx_16bit_stream(bg);
		}
	}
	set(LCD_TFT_CS_PORT, LCD_TFT_CS);
}


/**************************************************************************//**
* @fn			void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
* @brief		Draw a line from and to a point with a color
* @note
*****************************************************************************/
void LCD_drawLine(short x0,short y0,short x1,short y1,uint16_t c)
{
	// Fill this out
	short m = (x1 - x0) / (y1 - y0);
	for (short i = x0; i <= x1; i++) {
		LCD_drawPixel(i, m*(i-x0) + i,c);
	}
}



/**************************************************************************//**
* @fn			void LCD_drawBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color)
* @brief		Draw a colored block at coordinates
* @note
*****************************************************************************/
void LCD_drawBlock(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,uint16_t color)
{
	if ((x1>0)&&(y1>0)){
		if(x0<0) x0 = 0;
		if(y0<0) y0 = 0;
		if(x1>LCD_WIDTH) x1 = LCD_WIDTH;
		if(y1>LCD_HEIGHT) y1 = LCD_HEIGHT;
		// Fill this out
		LCD_setAddr(x0,y0,x1,y1);
		clear(LCD_TFT_CS_PORT, LCD_TFT_CS);
		for (int i = x0; i < x1+1 ; i++){
			for (int j = y0; j < y1+1 ; j++){
				SPI_ControllerTx_16bit_stream(color);
			}
		}
		set(LCD_TFT_CS_PORT, LCD_TFT_CS);
	}
}

/**************************************************************************//**
* @fn			void LCD_setScreen(uint16_t color)
* @brief		Draw the entire screen to a color
* @note
*****************************************************************************/
void LCD_setScreen(uint16_t color) 
{
	// Fill this out
	LCD_setAddr(0,0,LCD_WIDTH-1,LCD_HEIGHT-1); //HX8357_TFTWIDTH
	clear(LCD_TFT_CS_PORT, LCD_TFT_CS);
	for (int i = 0; i < LCD_WIDTH ; i++){
		for (int j = 0; j < LCD_HEIGHT ; j++){
			SPI_ControllerTx_16bit_stream(color);
		}
	}
	set(LCD_TFT_CS_PORT, LCD_TFT_CS);
}

/**************************************************************************//**
* @fn			void LCD_drawString(uint16_t x, uint16_t y, char* str, uint16_t fg, uint16_t bg)
* @brief		Draw a string starting at the point with foreground and background colors
* @note
*****************************************************************************/
void LCD_drawString(uint16_t x, uint16_t y, char* str, uint16_t fg, uint16_t bg)
{
	// Fill this out
	uint16_t i = x;
	uint16_t j = y;
	for (int c = 0; c < strlen(str); c++) {
		if (i > LCD_WIDTH - 5){
			j -= 9*SCALE_FACTOR;
			i = 0;
		}
		LCD_drawChar(i, j, str[c], fg, bg);
		i += 5*SCALE_FACTOR;
	}
}