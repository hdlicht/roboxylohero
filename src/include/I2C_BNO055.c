/*
 * I2C_BNO055.c
 *
 * Created: 12/6/2023 1:45:45 PM
 *  Author: student
 */ 

// This runs on the Atmega328p (master) and communicates with BNO055 IMU (servant)

#include "I2C_BNO055.h"

void BNO055_i2c_init(uint8_t operation_mode){
	// Note need an additional pin to pull ADR pin high according to the Adafruit open source library for it
	// All Setting the ADR pin should do is makes its servant address 0x29 instead of 0x28, but not setting it breaks I2C for some reason.

	int16_t yaw_offset=0; int16_t roll_offset=0; int16_t pitch_offset=0;

	//op_mode = OPERATION_MODE_NDOF;
	op_mode = operation_mode;
	//op_mode = OPERATION_MODE_AMG;

	I2C_write_8bit(BNO055_PAGE_ID_ADDR,0x00); // Set to page 0 (where most sensor outputs are)
	I2C_write_8bit(BNO055_UNIT_SEL_ADDR,0x00); // set units to m/s^2, degrees, and windows operation mode
	//MAG_CONFIG defaults to 0x0B.
	// Setting Op mode to any mode but Config mode prevents writing to other registers, so is done last.
	I2C_write_8bit(BNO055_OPR_MODE_ADDR,op_mode); // Put in Compass mode
}

void read_acc(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_ACCEL_DATA_X_LSB_ADDR);
	x_acc = (int16_t) I2C_Rx16bit_stream(0);
	y_acc = (int16_t) I2C_Rx16bit_stream(0);
	z_acc = (int16_t) I2C_Rx16bit_stream(1);
}
void set_acc_offset(){
	read_acc(0);
	I2C_write_8bit(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG); // Put in config mode
	I2C_Tx_start(BNO055_ADDRESS_A,ACCEL_OFFSET_X_LSB_ADDR);
	I2C_Tx16bit_stream(x_acc,0); I2C_Tx16bit_stream(y_acc,0); I2C_Tx16bit_stream(z_acc,1);
	I2C_write_8bit(BNO055_OPR_MODE_ADDR,op_mode); // return to original mode.
}

void read_mag(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_MAG_DATA_X_LSB_ADDR);
	x_mag = (int16_t) I2C_Rx16bit_stream(0);	
	y_mag = (int16_t) I2C_Rx16bit_stream(0);
	z_mag = (int16_t) I2C_Rx16bit_stream(1);
}
void set_mag_offset(){
	read_mag(0);
	I2C_write_8bit(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG); // Put in config mode
	I2C_Tx_start(BNO055_ADDRESS_A,MAG_OFFSET_X_LSB_ADDR);
	I2C_Tx16bit_stream(x_mag,0); I2C_Tx16bit_stream(y_mag,0); I2C_Tx16bit_stream(z_mag,1);
	I2C_write_8bit(BNO055_OPR_MODE_ADDR,op_mode); // return to original mode.
}

void read_gyr(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_GYRO_DATA_X_LSB_ADDR);
	x_gyr = (int16_t) I2C_Rx16bit_stream(0); 
	y_gyr = (int16_t) I2C_Rx16bit_stream(0);
	z_gyr = (int16_t) I2C_Rx16bit_stream(1);
}

void read_AccMagGry(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_ACCEL_DATA_X_LSB_ADDR);
	x_acc = (int16_t) I2C_Rx16bit_stream(0);y_acc = (int16_t) I2C_Rx16bit_stream(0); z_acc = (int16_t) I2C_Rx16bit_stream(0);
	x_mag = (int16_t) I2C_Rx16bit_stream(0);y_mag = (int16_t) I2C_Rx16bit_stream(0); z_mag = (int16_t) I2C_Rx16bit_stream(0);
	x_gyr = (int16_t) I2C_Rx16bit_stream(0);y_gyr = (int16_t) I2C_Rx16bit_stream(0); z_gyr = (int16_t) I2C_Rx16bit_stream(0);
}


void read_tilt(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_QUATERNION_DATA_W_LSB_ADDR);
	w_qua = (int16_t) I2C_Rx16bit_stream(0); x_qua = (int16_t) I2C_Rx16bit_stream(0);
	y_qua = (int16_t) I2C_Rx16bit_stream(0); z_qua = (int16_t) I2C_Rx16bit_stream(1);
}

void read_orientation(){
	I2C_Rx_start(BNO055_ADDRESS_A,BNO055_EULER_H_LSB_ADDR);
	yaw =( (int16_t) I2C_Rx16bit_stream(0))/((int16_t)EUL_UNIT);// - yaw_offset;
	roll =  ((int16_t)I2C_Rx16bit_stream(0))/((int16_t)EUL_UNIT);// - roll_offset;
	pitch = ((int16_t)I2C_Rx16bit_stream(1))/((int16_t)EUL_UNIT);//- pitch_offset;
}

void set_eul_offset(){
	read_orientation();
	yaw_offset=yaw; 
	roll_offset=roll; 
	pitch_offset=pitch;
}