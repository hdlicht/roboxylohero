/**************************************************************************//**
* @file        HX8357.h
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

#ifndef HX8357_H_
#define HX8357_H_

#define LCD_MOSI_PORT	PORTB
#define LCD_MOSI_DDR	DDRB
#define LCD_MOSI		PORTB3 //purple

#define LCD_SCK_PORT	PORTB
#define LCD_SCK_DDR		DDRB
#define LCD_SCK			PORTB5 //grey

//LCD_LITE must be connected to pin 6 of Arduino Uno for PWM to change brightness (Otherwise, connect to 5V supply)
#define LCD_LITE_PORT	PORTD
#define LCD_LITE_DDR	DDRD
#define LCD_LITE		PORTD6 //orange

#define LCD_DC_PORT	    PORTD
#define LCD_DC_DDR	    DDRD
#define LCD_DC			PORTD5 //green

#define LCD_RST_PORT	PORTD
#define LCD_RST_DDR	    DDRD
#define LCD_RST			PORTD7 //yellow

#define LCD_TFT_CS_PORT	    PORTD
#define LCD_TFT_CS_DDR	    DDRD
#define LCD_TFT_CS		    PORTD4 //blue

#define LCD_WIDTH 480
#define LCD_HEIGHT 320

//! \name Return error codes
//! @{
#define ADAFRUIT358_SPI_NO_ERR                 0 //! No error
#define ADAFRUIT358_SPI_ERR                    1 //! General or an unknown error
#define ADAFRUIT358_SPI_ERR_RESP_TIMEOUT       2 //! Timeout during command
#define ADAFRUIT358_SPI_ERR_RESP_BUSY_TIMEOUT  3 //! Timeout on busy signal of R1B response
#define ADAFRUIT358_SPI_ERR_READ_TIMEOUT       4 //! Timeout during read operation
#define ADAFRUIT358_SPI_ERR_WRITE_TIMEOUT      5 //! Timeout during write operation
#define ADAFRUIT358_SPI_ERR_RESP_CRC           6 //! Command CRC error
#define ADAFRUIT358_SPI_ERR_READ_CRC           7 //! CRC error during read operation
#define ADAFRUIT358_SPI_ERR_WRITE_CRC          8 //! CRC error during write operation
#define ADAFRUIT358_SPI_ERR_ILLEGAL_COMMAND    9 //! Command not supported
#define ADAFRUIT358_SPI_ERR_WRITE             10 //! Error during write operation
#define ADAFRUIT358_SPI_ERR_OUT_OF_RANGE      11 //! Data access out of range
//! @}


// HX8357 registers
#define HX8357D                    0xD  ///< Our internal const for D type
#define HX8357B                    0xB  ///< Our internal const for B type

#define HX8357_TFTWIDTH            320  ///< 320 pixels wide
#define HX8357_TFTHEIGHT           480  ///< 480 pixels tall

#define HX8357_NOP                0x00  ///< No op
#define HX8357_SWRESET            0x01  ///< software reset
#define HX8357_RDDID              0x04  ///< Read ID
#define HX8357_RDDST              0x09  ///< (unknown)

#define HX8357_RDPOWMODE          0x0A  ///< Read power mode Read power mode
#define HX8357_RDMADCTL           0x0B  ///< Read MADCTL
#define HX8357_RDCOLMOD           0x0C  ///< Column entry mode
#define HX8357_RDDIM              0x0D  ///< Read display image mode
#define HX8357_RDDSDR             0x0F  ///< Read display signal mode

#define HX8357_SLPIN              0x10  ///< Enter sleep mode
#define HX8357_SLPOUT             0x11  ///< Exit sleep mode
#define HX8357B_PTLON             0x12  ///< Partial mode on
#define HX8357B_NORON             0x13  ///< Normal mode

#define HX8357_INVOFF             0x20  ///< Turn off invert
#define HX8357_INVON              0x21  ///< Turn on invert
#define HX8357_DISPOFF            0x28  ///< Display on
#define HX8357_DISPON             0x29  ///< Display off

#define HX8357_CASET              0x2A  ///< Column addr set
#define HX8357_PASET              0x2B  ///< Page addr set
#define HX8357_RAMWR              0x2C  ///< Write VRAM
#define HX8357_RAMRD              0x2E  ///< Read VRAm

#define HX8357B_PTLAR             0x30  ///< (unknown)
#define HX8357_TEON               0x35  ///< Tear enable on
#define HX8357_TEARLINE           0x44  ///< (unknown)
#define HX8357_MADCTL             0x36  ///< Memory access control
#define HX8357_COLMOD             0x3A  ///< Color mode

#define HX8357_SETOSC             0xB0  ///< Set oscillator
#define HX8357_SETPWR1            0xB1  ///< Set power control
#define HX8357B_SETDISPLAY        0xB2  ///< Set display mode
#define HX8357_SETRGB             0xB3  ///< Set RGB interface
#define HX8357D_SETCOM            0xB6  ///< Set VCOM voltage

#define HX8357B_SETDISPMODE       0xB4  ///< Set display mode
#define HX8357D_SETCYC            0xB4  ///< Set display cycle reg
#define HX8357B_SETOTP            0xB7  ///< Set OTP memory
#define HX8357D_SETC              0xB9  ///< Enable extension command

#define HX8357B_SET_PANEL_DRIVING 0xC0  ///< Set panel drive mode
#define HX8357D_SETSTBA           0xC0  ///< Set source option
#define HX8357B_SETDGC            0xC1  ///< Set DGC settings
#define HX8357B_SETID             0xC3  ///< Set ID
#define HX8357B_SETDDB            0xC4  ///< Set DDB
#define HX8357B_SETDISPLAYFRAME   0xC5  ///< Set display frame
#define HX8357B_GAMMASET          0xC8  ///< Set Gamma correction
#define HX8357B_SETCABC           0xC9  ///< Set CABC
#define HX8357_SETPANEL           0xCC  ///< Set Panel

#define HX8357B_SETPOWER          0xD0  ///< Set power control
#define HX8357B_SETVCOM           0xD1  ///< Set VCOM
#define HX8357B_SETPWRNORMAL      0xD2  ///< Set power normal

#define HX8357B_RDID1             0xDA  ///< Read ID #1
#define HX8357B_RDID2             0xDB  ///< Read ID #2
#define HX8357B_RDID3             0xDC  ///< Read ID #3
#define HX8357B_RDID4             0xDD  ///< Read ID #4

#define HX8357D_SETGAMMA          0xE0  ///< Set Gamma

#define HX8357B_SETGAMMA          0xC8 ///< Set Gamma
#define HX8357B_SETPANELRELATED   0xE9 ///< Set panel related

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

//Macro Functions
#define set(reg,bit) (reg) |= (1<<(bit))
#define clear(reg,bit) (reg) &= ~(1<<(bit))
#define toggle(reg,bit) (reg) ^= (1<<(bit))
#define loop_until_bit_is_set(sfr, bit) do { } while (bit_is_clear(sfr, bit))
#define loop_until_bit_is_clear(sfr, bit) do { } while (bit_is_set(sfr, bit))

void Delay_ms(unsigned int n);
void lcd_init(void);
void sendCommands (const uint8_t *cmds, uint8_t length);
void LCD_setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void SPI_ControllerTx(uint8_t data);
void SPI_ControllerTx_stream(uint8_t stream);
void SPI_ControllerTx_16bit(uint16_t data);
void SPI_ControllerTx_16bit_stream(uint16_t data);
void LCD_brightness(uint8_t intensity);
void LCD_rotate(uint8_t r);

#endif /* ST7735_H_ */
