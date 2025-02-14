/*
 * I2C.h
 *
 * Created: 11/29/2023 3:42:20 PM
 *  Author: student
 */ 


#ifndef I2C_H_
#define I2C_H_


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
#include <avr/io.h>
#include <avr/interrupt.h>

// Only define if using internal pullup resistors
// #define I2C_INTERNAL_PULLUPS // Comment if using external
// IO functions of PinC5 and PinC6 are disconnected
// from SDA and SCL pins when TWI is enabled
// TWI = 2 wire interface. It is compatible with Phillips I2C.
// TWI address packet format is 9 bits.
// The starting packet is 7 bit servant address. 1 Read/Write. 1 acknowledgment bit.
// following packets are 8-bit data frames and 1 bit acknowledgments.
// data frames are sent MSB first
// ACK (acknowledgments) is 0 if successful. NACK (not ACK) is 1.

#define LSM6DS0_ADDRESS 0xD6//0b01101100//0xD6 // Default I2C address with SA0 high

#define TW_WRITE 0
#define TW_READ 1

//#define MSB_FIRST 1
//TWCR needs to be written to only once. So this macro effectively masks it allowing for safe writing
#define CHANGE_CONDITION ((TWCR&~((1<<TWSTA)|(1<<TWSTO)))|((1<<TWEN)|(1<<TWINT)))

#define PACKET_DELIVERY while(!(TWCR&(1<<TWINT))) //Wait for TWINT flag to be set to know packet is done Tx/Rx.
// Process (Tx/Rx) the next packet, which will be in TWDR before Tx or after Rx.
// Tx only: Start communications with the servant who's 7-bit address is in TWDR. The LSb of TWDR is whether to read or write.
#define I2C_START_PACKET TWCR=CHANGE_CONDITION|(1<<TWSTA);PACKET_DELIVERY
// Tx only: Terminate communications with current servant. TWDR is irrelevant. Don't wait for delivery. 
// The TWINT flag remains cleared after STOP_PACKET until a new command is sent, so don't wait for it to set. 
#define I2C_STOP_PACKET TWCR=CHANGE_CONDITION|(1<<TWSTO) // DO not wait for packet_delivery
// Tx: send the packet currently in TWDR. Rx: store incoming packet in TWDR
// For Rx: Must send NACK for last byte expected to receive and ACK with all others. 
#define I2C_DATA_PACKET TWCR=CHANGE_CONDITION;PACKET_DELIVERY // Start next package Rx/Tx
#define I2C_DATA_PACKET_ACK TWCR=CHANGE_CONDITION|(1<<TWEA);PACKET_DELIVERY // Start next package Rx/Tx
#define I2C_DATA_PACKET_NACK TWCR=CHANGE_CONDITION&~(1<<TWEA);PACKET_DELIVERY // Start next package Rx/Tx

/*
// No longer being safe
#define PACKET_DELIVERY while(!(TWCR&(1<<TWINT))) //Wait for TWINT flag to be set to know packet is done Tx/Rx.
// Process (Tx/Rx) the next packet, which will be in TWDR before Tx or after Rx.
// Tx only: Start communications with the servant who's 7-bit address is in TWDR. The LSb of TWDR is whether to read or write.
#define I2C_START_PACKET TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTA);PACKET_DELIVERY
// Tx only: Terminate communications with current servant. TWDR is irrelevant. Don't wait for delivery.
// The TWINT flag remains cleared after STOP_PACKET until a new command is sent, so don't wait for it to set.
#define I2C_STOP_PACKET TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTO) // DO not wait for packet_delivery
// Tx: send the packet currently in TWDR. Rx: store incoming packet in TWDR
// For Rx: Must send NACK for last byte expected to receive and ACK with all others.
#define I2C_DATA_PACKET TWCR=(1<<TWEN)|(1<<TWINT);PACKET_DELIVERY // Start next package Rx/Tx
#define I2C_DATA_PACKET_ACK TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWEA);PACKET_DELIVERY // Start next package Rx/Tx
#define I2C_DATA_PACKET_NACK I2C_DATA_PACKET // Start next package Rx/Tx
*/

// Every command
uint8_t Servant_Address; // 7 bit address of servant device.
//uint8_t buffer
uint8_t packet_delivery(void);
uint8_t I2C_Rx_start(uint8_t servant_address, uint8_t reg);
uint8_t I2C_Rx_stream(uint8_t doStop);
uint16_t I2C_Rx16bit_stream(uint8_t doStop);

//uint8_t I2C_Rx8bit_stream(uint8_t starting_reg, uint8_t* data_array, uint8_t bytes);
//uint8_t I2C_Rx16bit_stream(uint8_t starting_reg, uint16_t* data_array, uint8_t words);
uint8_t I2C_Tx_start(uint8_t servant_address, uint8_t reg);
uint8_t I2C_Tx_stream(uint8_t data, uint8_t doStop);
uint8_t I2C_Tx16bit_stream(uint16_t data, uint8_t doStop);

void I2C_init(uint8_t servent_address);
void I2C_change_servant(uint8_t new_address);
void I2C_select_register(uint8_t reg);

//void I2C_Tx8bit_stream(uint8_t data);
//void I2C_Tx16bit_stream(uint16_t data);



uint8_t I2C_write_8bit(uint8_t reg, uint8_t data);
uint8_t I2C_write_16bit(uint8_t reg, uint16_t data);

uint8_t I2C_read_8bit(uint8_t reg);
uint16_t I2C_read_16bit(uint8_t reg);
void I2C_Rx_begin(uint8_t reg);
//uint8_t I2C_Rx8bit_stream(void);
//uint16_t I2C_Rx16bit_stream(void);


/* TWSR values and the status they indicate
  TW_MT_xxx - master transmitter
  TW_MR_xxx - master receiver
  TW_ST_xxx - slave transmitter
  TW_SR_xxx - slave receiver
  */

/* Master *///start condition transmitted */
#define TW_START		0x08
//repeated start condition transmitted */
#define TW_REP_START		0x10

/* Master Transmitter */
//SLA+W transmitted, ACK received */
#define TW_MT_SLA_ACK		0x18
//SLA+W transmitted, NACK received */
#define TW_MT_SLA_NACK		0x20
//data transmitted, ACK received */
#define TW_MT_DATA_ACK		0x28
//data transmitted, NACK received */
#define TW_MT_DATA_NACK		0x30
//arbitration lost in SLA+W or data */
#define TW_MT_ARB_LOST		0x38

/* Master Receiver */
//arbitration lost in SLA+R or NACK */
#define TW_MR_ARB_LOST		0x38
//SLA+R transmitted, ACK received */
#define TW_MR_SLA_ACK		0x40
//SLA+R transmitted, NACK received */
#define TW_MR_SLA_NACK		0x48
#define TW_MR_DATA_ACK		0x50 //data received, ACK returned 
#define TW_MR_DATA_NACK		0x58 //data received, NACK returned 

/* Slave Transmitter */
//SLA+R received, ACK returned */
#define TW_ST_SLA_ACK		0xA8
//arbitration lost in SLA+RW, SLA+R received, ACK returned */
#define TW_ST_ARB_LOST_SLA_ACK	0xB0
//data transmitted, ACK received */
#define TW_ST_DATA_ACK		0xB8
//data transmitted, NACK received */
#define TW_ST_DATA_NACK		0xC0
//last data byte transmitted, ACK received */
#define TW_ST_LAST_DATA		0xC8

/* Slave Receiver */
//SLA+W received, ACK returned */
#define TW_SR_SLA_ACK		0x60
//arbitration lost in SLA+RW, SLA+W received, ACK returned */
#define TW_SR_ARB_LOST_SLA_ACK	0x68
//general call received, ACK returned */
#define TW_SR_GCALL_ACK		0x70
//arbitration lost in SLA+RW, general call received, ACK returned */
#define TW_SR_ARB_LOST_GCALL_ACK 0x78
//data received, ACK returned */
#define TW_SR_DATA_ACK		0x80
//data received, NACK returned */
#define TW_SR_DATA_NACK		0x88
//general call data received, ACK returned */
#define TW_SR_GCALL_DATA_ACK	0x90
//general call data received, NACK returned */
#define TW_SR_GCALL_DATA_NACK	0x98
//stop or repeated start condition received while selected */
#define TW_SR_STOP		0xA0
//no state information available */
#define TW_NO_INFO		0xF8
//illegal start or stop condition */
#define TW_BUS_ERROR		0x00

// The lower 3 bits of TWSR are reserved on the ATmega163.
// The 2 LSB carry the prescaler bits on the newer ATmegas.
#define TW_STATUS_MASK	((1<<TWS7)|(1<<TWS6)|(1<<TWS5)|(1<<TWS4)|(1<<TWS3))
#define TW_STATUS	(TWSR & TW_STATUS_MASK) //TWSR, masked by TW_STATUS_MASK



#endif /* I2C_H_ */