/*
 * I2C.c
 *
 * Created: 11/29/2023 3:41:35 PM
 *  Author: student
 */ 
/**
 * \file
 *
 * \brief Empty user application template
 *
 */
// This Assumes that the arduino is the master and the LSM6DSO is the servent
// This runs on the Atmega328p and communicates with LSM6DSO IMU
//#include <asf.h>

#include "I2C.h"

// #include <stdlib.h>
// #include <stdio.h>
//#include <cstdint>

//#include <util/twi.h>

// These are useful configuration bits used for timers

#define TIMER_PSDIV_BITS 0b00000111
#define OVERFLOW_TICKS_8BIT 256 // 8-bit counter overflows every (2^8) ticks.

// Only uncomment MSB_FIRST if the I2C device you are communicating with stores the MSB of 16bit numbers in the first address of it. 
// #define MSB_FIRST 1

// Only define if using internal pullup resistors
//#define I2C_INTERNAL_PULLUPS // Comment if using external
// IO functions of PinC5 and PinC6 are disconnected
// from SDA and SCL pins when TWI is enabled
// TWI = 2 wire interface. It is compatible with Phillips I2C.
// TWI address packet format is 9 bits.
// The starting packet is 7 bit servant address. 1 Read/Write. 1 acknowledgment bit.
// following packets are 8-bit data frames and 1 bit acknowledgments.
// data frames are sent MSB first
// ACK (acknowledgments) is 0 if successful. NACK (not ACK) is 1.

#define LSM6DS0_ADDRESS 0xD6//0b01101100//0xD6 // Default I2C address with SA0 high

//#define CHECK_STATUS 1 // Uncomment to print Status Register
//#define DEBUG_PRINTS 1 // Uncomment to print Tx and Rx operations and timeouts. 

#define TW_WRITE 0
#define TW_READ 1

uint8_t Servant_Address; // 7 bit address of servant device.

uint8_t packet_delivery(void){
	uint16_t wait_counter = 0; // a busy counter to see if the ack signal has taken to long to be recieved.
	while(!(TWCR&(1<<TWINT))){
	}
#ifdef DEBUG_PRINTS
	sprintUART("Status = %02X  ",TWSR);
#endif
	return TW_STATUS;
}

void I2C_init(uint8_t servant_address) {
	// Control internal pullup resistors on the SDA and SCL pins (Pins A4 & A5)
	DDRC |= ((1<<PINC5)|(1<<PINC4)); //First set as output to disable pullup resistors.
	// IMPORTANT: comment out one of the following lines depending on the pullup resistors you are using. 
	PORTC &= ~((1<<PINC5)|(1<<PINC4)); // Disable to use external pullup resistors.
	//PORTC |= ((1<<PINC5)|(1<<PINC4)); // Enable to use internal pullup resistors.
	DDRC  &= ~((1<<PINC5)|(1<<PINC4)); // Set the pins as inputs for communicating
	// Set prescaler to 1 (no prescaling)
	TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
	// standard I2C has SCL at 100 kHz. Fast mode I2C has SCL at 400 kHz.
	// Set bit rate register so SCL operates at 100000Hz
	// SCL_freq = 16MHz/(16 + 2*TWBR*prescalar) */ 	// Prescalar is assumed to be 1. 

	TWBR = 72; //SCL_freq = 16MHz/(16 + 2*72*1) = 100kHz
	//TWBR = 24;// 	SCL_freq = 16MHz/(16 + 2*24*1) = 250KHz	
	//TWBR = 12;// 	SCL_freq = 16MHz/(16 + 2*12*1) = 400KHz	

	// Enable TWI, generate ack on received data, enable TWI interrupt
	TWCR = (1 << TWEN); // | (1<<TWEA) | (1<<TWIE);
	Servant_Address = servant_address;
}

void I2C_change_servant(uint8_t new_address){
	// send as a 7 bit number.
	Servant_Address = new_address;
}

void I2C_select_register(uint8_t reg){
	// reg is the register address.
	// Note that the register address is automatically incremented
	// in most servant devices after subsequent reads/writes.

	// Step 1: select LSM6DS0 servent address with write bit (write mode)
	//TWDR = (Servant_Address << 1) & ~(1<<TW_WRITE);
	
	// Step 2: start transmission with
	I2C_START_PACKET;
	TWDR = (Servant_Address << 1) & ~(1<<TW_WRITE);
	I2C_DATA_PACKET_ACK;
	// Step 3: select the register
	//UART_putstring("Started\n");
	TWDR = reg;
	// Step 4: transmit to servant to start operations at that register.
	I2C_DATA_PACKET;
	//UART_putstring("PACKET delivered");
}

uint8_t I2C_Tx_start(uint8_t servant_address, uint8_t reg){
	I2C_change_servant(servant_address);
	I2C_select_register(reg);
	return TWSR;
}
uint8_t I2C_Tx_stream(uint8_t data, uint8_t doStop){
	// Start stream by calling I2C_select_register(reg) before calling this function.
	// Writes data to the register and then increments to the next register.
	// End the stream with precompiler defintion: I2C_STOP_PACKET;
	TWDR = data;
	I2C_DATA_PACKET;
	if (doStop == 1){I2C_STOP_PACKET;}
	return TWCR;
}
uint8_t I2C_Tx16bit_stream(uint16_t data, uint8_t doStop){
	// Start stream by calling I2C_select_register(reg) before calling this function.
	// Writes data to the register and then increments to the next register.
	// End the stream with precompiler defintion: I2C_STOP_PACKET;
	uint8_t temp = data >> 8; data = (uint8_t) data;
	TWDR = data;
	I2C_DATA_PACKET;
	TWDR = temp;
	I2C_DATA_PACKET;
	if (doStop == 1){I2C_STOP_PACKET;}
	return TWCR;
}

uint8_t I2C_Rx_start(uint8_t servant_address, uint8_t reg){
	I2C_change_servant(servant_address);
	I2C_select_register(reg);
	//Send another START packet to initiate reading
	I2C_START_PACKET;
	TWDR = (Servant_Address<<1) | TW_READ;
	I2C_DATA_PACKET;
	//sprintUART("Status = %02X\n",TWCR);
	return TWCR;
}

uint8_t I2C_Rx_stream(uint8_t doStop){
	if (doStop == 1){
		I2C_DATA_PACKET_NACK;
		I2C_STOP_PACKET;
	}
	else{ I2C_DATA_PACKET_ACK;}
	return TWDR;
}

uint16_t I2C_Rx16bit_stream(uint8_t doStop){
	uint16_t data = 0; uint16_t msb_data = 0;
	I2C_DATA_PACKET_ACK;
	data |= TWDR;
	if (doStop == 1){
		I2C_DATA_PACKET_NACK;
		I2C_STOP_PACKET;
	}
	else{ I2C_DATA_PACKET_ACK;}
	msb_data |= TWDR;
	msb_data = msb_data << 8;
	return data|msb_data;
}

uint8_t I2C_read_8bit(uint8_t reg){
	uint8_t data;
	I2C_select_register(reg);
	//Send another START packet to initiate reading
	I2C_START_PACKET;
	// Send the servant address and read command.
	TWDR = (Servant_Address<<1) | TW_READ;
	I2C_DATA_PACKET;
	//sprintUART("Status = %02X\n",TWCR);
	I2C_DATA_PACKET_NACK;
	I2C_STOP_PACKET;
	data = TWDR;
	return data;
}

uint16_t I2C_read_16bit(uint8_t reg){
	// Note that for LSM6DSO, The Low register is first.
	uint16_t data = 0; uint16_t temp = 0;
	I2C_select_register(reg);
	I2C_START_PACKET;
	TWDR = (Servant_Address<<1) | TW_READ;
	I2C_DATA_PACKET; // send servant address + Read bit 
	//sprintUART("Status = %02X\n",TWCR);
	I2C_DATA_PACKET_ACK; // read first byte and send acknowledge 
	#ifdef MSB_FIRST
		data |= TWDR;
		I2C_DATA_PACKET_NACK; // read last byte and don't acknowledge to end data receiving. 
		data = (data<<8)|TWDR;
	#endif
	#ifndef MSB_FIRST
		data = TWDR;
		I2C_DATA_PACKET_NACK; // read last byte and don't acknowledge to end data receiving.
		temp = (uint16_t) TWDR;
		data |= (temp<<8);
	#endif
	//sprintUART("Status = %02X\n",TWCR);
	I2C_STOP_PACKET;
	return data;
}

uint8_t I2C_write_8bit(uint8_t reg, uint8_t data){
	// doStop is a boolean for whether to stop
	I2C_select_register(reg);
	// Write data to TWDR
	TWDR = data;
	// Send data to servant to write at selected address.
	I2C_DATA_PACKET;
	//UART_putstring("Packet sent\n");
	I2C_STOP_PACKET;
	return TWCR;
}

uint8_t I2C_write_16bit(uint8_t reg, uint16_t data){
	// Note that for LSM6DSO, The Low register is first.
	uint8_t temp = data >> 8;
	I2C_select_register(reg);
	#ifdef MSB_FIRST
	TWDR = temp;
	I2C_DATA_PACKET;
	TWDR = data;
	#endif
	#ifndef MSB_FIRST
	TWDR = data;
	I2C_DATA_PACKET;
	TWDR = temp;
	#endif
	I2C_DATA_PACKET;
	I2C_STOP_PACKET;
	return TWCR;
}

/*
void I2C_Rx_begin(uint8_t reg){
	// For continuous reading of consecutive registers starting at reg
	// without stopping and starting in between registers
	// Read &increment registers with I2C_Rx8bit_stream() or I2C_Rx8bit_stream()
	// End the stream with a stop packet with line: I2C_STOP_PACKET;
	I2C_select_register(reg);
	TWDR = (Servant_Address<<1) | TW_READ;
	I2C_START_PACKET;
}

uint8_t I2C_Rx8bit_stream(uint8_t starting_reg, uint8_t* data_array, uint8_t bytes){
	// Start stream by calling I2C_Rx_begin(reg) before calling this function.
	// Reads the register and then increments to the next register.
	// End the stream with precompiler defintion: I2C_STOP_PACKET;
	I2C_select_register(starting_reg);
	I2C_START_PACKET;
	TWDR = (Servant_Address<<1) | TW_READ;
	I2C_DATA_PACKET;
	for (int i = 0; i<bytes-1;i++){
		I2C_DATA_PACKET_ACK;
		data_array[i] = TWDR;
	}
	I2C_DATA_PACKET_NACK;
	data_array[bytes-1] = TWDR;
	return TWDR;
}
uint8_t I2C_Rx16bit_stream(uint8_t starting_reg, uint16_t* data_array, uint8_t words){
	
	// Start stream by calling I2C_Rx_begin(reg) before calling this function.
	// Reads the register and then increments to the next register.
	// End the stream with precompiler defintion: I2C_STOP_PACKET;
	uint16_t data = 0;
	I2C_DATA_PACKET;
	#ifdef MSB_FIRST
		data |= (TWDR<<8);
		I2C_DATA_PACKET;
		data |= TWDR;
	#endif
	#ifndef MSB_FIRST
		data |= TWDR;
		I2C_DATA_PACKET;
		data |= (TWDR<<8);
	#endif
	return data;
}
*/