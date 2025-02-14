/**************************************************************************//**
* @file        HX8357.c
* @ingroup 	   HX8357
* @brief       Basic display driver for Adafruit 358 1.8" TFT LCD with ST7735R chip
*
* @details     Basic display driver for Adafruit 358 1.8" TFT LCD with ST7735R chip
*
*
* @copyright
* @author	   J. Ye
* @date        April 19, 2021
* @version		1.0
*****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "HX8357.h"
#include "uart.h"

char String[25]; 

/******************************************************************************
* Local Functions
******************************************************************************/

/**************************************************************************//**
* @fn			static void lcd_pin_init(void)
* @brief		Initialize SPI for LCD
* @note
*****************************************************************************/
static void lcd_pin_init(void)
{
	//Setup digital pins
	LCD_DC_DDR |= (1<<LCD_DC);
	LCD_RST_DDR |= (1<<LCD_RST);
	LCD_TFT_CS_DDR |= (1<<LCD_TFT_CS);
	LCD_MOSI_DDR |= (1<<LCD_MOSI);
	LCD_SCK_DDR |= (1<<LCD_SCK);	//Set up output pins
	LCD_LITE_DDR |= (1<<LCD_LITE);	//Set up output pins

	//Setup PWM for LCD Backlight
	TCCR0A |= (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);	//Fast PWM: clear OC0A on match, set at bottom
	TCCR0B |= (1<<CS02);	//clk/1024/256=244Hz
	OCR0A = 127;	//Set starting PWM value

	//Enable LCD by setting RST high
	_delay_ms(50);
	set(LCD_RST_PORT, LCD_RST);
}

/**************************************************************************//**
* @fn			static void SPI_Controller_Init(void)
* @brief		Initialize SPI for LCD
* @note
*****************************************************************************/
static void SPI_Controller_Init(void)
{
	SPCR = (1<<SPE) | (1<<MSTR);		//Enable SPI, Master, set clock rate fck/64
	SPSR = (1<<SPI2X);										//SPI 2X speed
}


/******************************************************************************
* Global Functions
******************************************************************************/

/**************************************************************************//**
* @fn			void Delay_ms(unsigned int n)
* @brief		Delay function using variables
* @note
*****************************************************************************/
void Delay_ms(unsigned int n)
{
	while (n--)
	{
		_delay_ms(1);
	}
}

/**************************************************************************//**
* @fn			void SPI_ControllerTx(uint8_t data)
* @brief		Send 8-bit SPI data to peripheral
* @note
*****************************************************************************/
void SPI_ControllerTx(uint8_t data)
{
	clear(LCD_TFT_CS_PORT, LCD_TFT_CS);	//CS pulled low to start communication

	SPI_ControllerTx_stream(data);

	set(LCD_TFT_CS_PORT, LCD_TFT_CS);	//set CS to high
}

/**************************************************************************//**
* @fn			void SPI_ControllerTx_stream(uint8_t stream)
* @brief		Send a command to LCD through SPI without setting CS or DC
* @note
*****************************************************************************/
void SPI_ControllerTx_stream(uint8_t stream)
{
	SPDR = stream;		//Place data to be sent on registers
	while(!(SPSR & (1<<SPIF)));	//wait for end of transmission
}

/**************************************************************************//**
* @fn			void SPI_ControllerTx_16bit(uint16_t data)
* @brief		Send 16-bit data to peripheral
* @note			Used for color information
*****************************************************************************/
void SPI_ControllerTx_16bit(uint16_t data)
{
	uint8_t temp = data >> 8;
	clear(LCD_TFT_CS_PORT, LCD_TFT_CS);	//CS pulled low to start communication
	
	SPDR = temp;		//Place data to be sent on registers
	while(!(SPSR & (1<<SPIF)));	//wait for end of transmission
	SPDR = data;		//Place data to be sent on registers
	while(!(SPSR & (1<<SPIF)));	//wait for end of transmission
	
	set(LCD_TFT_CS_PORT, LCD_TFT_CS);	//set CS to high
}

/**************************************************************************//**
* @fn			void SPI_ControllerTx_16bit_stream(uint16_t data)
* @brief		Send 16 bit data to LCD through SPI without setting CS or DC
* @note			Used for color information
*****************************************************************************/
void SPI_ControllerTx_16bit_stream(uint16_t data)
{
	uint8_t temp = data >> 8;

	SPDR = temp;		//Place data to be sent on registers
	while(!(SPSR & (1<<SPIF)));	//wait for end of transmission
	SPDR = data;		//Place data to be sent on registers
	while(!(SPSR & (1<<SPIF)));	//wait for end of transmission
}

/**************************************************************************//**
* @fn			void lcd_init(void)
* @brief		Initialize LCD settings
* @note
*****************************************************************************/
void lcd_init(void)
{
	lcd_pin_init();
	SPI_Controller_Init();
	_delay_ms(5);

	static uint8_t HX8357_cmds[] = {
		HX8357_SWRESET, 0, 100, // Soft reset, then delay 10 ms
		HX8357D_SETC, 0,255,//2, 0x83, 0x57, 300,          // No command, just delay 300 ms
		//HX8357D_SETC, 0, 300,
		HX8357_SETRGB, 4,
		0x80, 0x00, 0x06, 0x06, 0,   // 0x80 enables SDO pin (0x00 disables)
		HX8357D_SETCOM, 1,
		0x25, 0,                     // -1.52V
		HX8357_SETOSC, 1,
		0x68, 0,                     // Normal mode 70Hz, Idle mode 55 Hz
		HX8357_SETPANEL, 1,
		0x04, 0,                      // RGB, Gate direction swapped
		HX8357_SETPWR1, 6,
		0x00,                      // Not deep standby
		0x15,                      // BT
		0x1C,                      // VSPR
		0x1C,                      // VSNR
		0x83,                      // AP
		0xAA, 0,                     // FS
		HX8357D_SETSTBA, 6,
		0x50,                      // OPON normal
		0x50,                      // OPON idle
		0x01,                      // STBA
		0x3C,                      // STBA
		0x1E,                      // STBA
		0x08, 0,                     // GEN
		HX8357D_SETCYC, 7,
		0x02,                      // NW 0x02
		0x40,                      // RTN
		0x00,                      // DIV
		0x2A,                      // DUM
		0x2A,                      // DUM
		0x0D,                      // GDON
		0x78, 0,                     // GDOFF
		HX8357D_SETGAMMA, 34,
		0x02, 0x0A, 0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b,
		0x42, 0x3A, 0x27, 0x1B, 0x08, 0x09, 0x03, 0x02, 0x0A,
		0x11, 0x1d, 0x23, 0x35, 0x41, 0x4b, 0x4b, 0x42, 0x3A,
		0x27, 0x1B, 0x08, 0x09, 0x03, 0x00, 0x01, 0,
		HX8357_COLMOD, 1,
		0x55, 0,                      // 16 bit
		HX8357_MADCTL, 1,
		0xC0, 0,
		HX8357_TEON, 1,
		0x00, 0,                    // TW off
		HX8357_TEARLINE, 2,
		0x00, 0x02, 0,
		HX8357_SLPOUT, 0, 150, // Exit Sleep, then delay 150 ms
		HX8357_DISPON, 0, 50, // Main screen turn on, delay 50 ms
		                          // END OF COMMAND LIST
	};

	sendCommands(HX8357_cmds, 16);
	LCD_rotate(3);
}

/**************************************************************************//**
* @fn			void sendCommands (const uint8_t *cmds, uint8_t length)
* @brief		Parse and send array of commands thru SPI
* @note
*****************************************************************************/
void sendCommands (const uint8_t *cmds, uint8_t length)
{
	//Command array structure:
	//Command Code, # of data bytes, data bytes (if any), delay in ms
	uint8_t numCommands, numData, waitTime;

	numCommands = length;	// # of commands to send

	clear(LCD_TFT_CS_PORT, LCD_TFT_CS);	//CS pulled low to start communication

	while (numCommands--)	// Send each command
	{
		clear(LCD_TFT_CS_PORT, LCD_DC);	//D/C pulled low for command
		
		SPI_ControllerTx_stream(*cmds++);
		
		numData = *cmds++;	// # of data bytes to send

		set(LCD_TFT_CS_PORT, LCD_DC);	//D/C set high for data
		while (numData--)	// Send each data byte...
		{
			SPI_ControllerTx_stream(*cmds++);
			
		}

		waitTime = *cmds++;     // Read post-command delay time (ms)
		if (waitTime!=0)
		{	
			Delay_ms((waitTime==255 ? 500 : waitTime));
		}
	}

	set(LCD_TFT_CS_PORT, LCD_TFT_CS);	//set CS to high
}

/**************************************************************************//**
* @fn			void LCD_setAddr(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
* @brief		Set pixel memory address to write to
* @note
*****************************************************************************/
void LCD_setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{	
	uint8_t x0_h = (uint8_t) (x0 >> 8) & 0xFF;
	uint8_t x0_l = (uint8_t) (x0) & 0xFF;
	uint8_t x1_h = (uint8_t) (x1 >> 8) & 0xFF;
	uint8_t x1_l = (uint8_t) (x1) & 0xFF;
	uint8_t y0_h = (uint8_t) (y0 >> 8) & 0xFF;
	uint8_t y0_l = (uint8_t) (y0) & 0xFF;
	uint8_t y1_h = (uint8_t) (y1 >> 8) & 0xFF;
	uint8_t y1_l = (uint8_t) (y1) & 0xFF;
	uint8_t HX8357_cmds[]  =
	{
		HX8357_CASET, 4, x0_h, x0_l, x1_h, x1_l, 0,	// Column //0x00
		HX8357_PASET, 4, y0_h, y0_l, y1_h, y1_l, 0,	// Page
		HX8357_RAMWR, 0, 5				// Into RAM
	};
	sendCommands(HX8357_cmds, 3);
}

/**************************************************************************//**
* @fn			void LCD_brightness(uint8_t intensity)
* @brief		Changes the intensity of the LCD screen (max = 255)
* @note
*****************************************************************************/
void LCD_brightness(uint8_t intensity)
{
	OCR0A = intensity;	//Set PWM value
}

/**************************************************************************//**
* @fn			void LCD_rotate(uint8_t r)
* @brief		Rotate display to another orientation
* @note
*****************************************************************************/

void LCD_rotate(uint8_t r)
{
	uint8_t madctl = 0;
	uint8_t rotation = r % 4; // can't be higher than 3

	switch (rotation) {
		case 0:
			madctl = MADCTL_MX | MADCTL_MY | MADCTL_RGB;
			break;
		case 1:
			madctl = MADCTL_MY | MADCTL_MV | MADCTL_RGB;
			break;
		case 2:
			madctl = MADCTL_RGB;
			break;
		case 3:
			madctl = MADCTL_MX | MADCTL_MV | MADCTL_RGB;
			break;
		default:
			madctl = MADCTL_MX | MADCTL_MY | MADCTL_RGB;
			break;
	}
	
	uint8_t HX8357_cmds[]  =
	{
		HX8357_MADCTL, 1, madctl, 0
	};
	
	sendCommands(HX8357_cmds, 1);
}
