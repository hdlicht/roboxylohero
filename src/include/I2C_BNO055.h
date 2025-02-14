/*
 * I2C_BNO055.h
 *
 * Created: 12/6/2023 1:48:47 PM
 *  Author: student
 */ 


#ifndef I2C_BNO055_H_
#define I2C_BNO055_H_

#include <uart.h>
#include <BNO055_Registers.h>
#include <I2C.h>
#include <TimerHelper.h>
#include <ServoControl.h>

#define ACC_UNIT 100 // 100 bits = 1m/s^2. Set to m/s^2 in UNIT_SEL register first.
#define MAG_UNIT 16 // 16 bits = 1 microTesla. 
#define GYR_UNIT 16 // 16 bits = 1 degree/sec. Set degrees in UNIT_SEL register first.
#define EUL_UNIT 16 // 16 bits = 1 degree. Set degrees in UNIT_SEL register first.
//#define DATA_POINTS 20

// Accelerometer output. Units are m/s^2
volatile short x_acc; volatile short y_acc; volatile short z_acc;
// magnetometer output. Units are microTesla
volatile short x_mag; volatile short y_mag; volatile short z_mag;
// gyroscope output. Units are degrees/s
volatile short x_gyr; volatile short y_gyr; volatile short z_gyr;

// Euler Orientation Information (fusion op_modes only)
volatile int16_t yaw; volatile int16_t roll; volatile int16_t pitch; //yaw is also called heading in datasheet
// Yaw goes from 0 to 360. Roll goes from -90 to 90. Pitch goes from -180 to 180. Degrees.
volatile int16_t yaw_offset; volatile int16_t roll_offset; volatile int16_t pitch_offset;

//int16_t eul_unit_conv = 16; // 16 bits = 1 degree. Set degrees in UNIT_SEL register first.
// Quaternion Tilt/Orientation Information (fusion op_modes only)
volatile int16_t w_qua; volatile int16_t x_qua; volatile int16_t y_qua; volatile int16_t z_qua;

//volatile uint8_t i_lia // index of linear acceleration pointers

volatile uint8_t op_mode; // current operation mode
void BNO055_i2c_init(uint8_t operation_mode);
void read_acc();
void read_mag();
void read_gyr();
void read_AccMagGry();
void set_acc_offset();
void set_mag_offset();

void read_tilt();
void read_orientation();
void set_eul_offset(); // euler angle orientation offset
//void read_All();

#endif /* I2C_BNO055_H_ */