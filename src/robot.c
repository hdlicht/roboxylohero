#include <avr/io.h>
#include <avr/interrupt.h>
#include "ServoControl.h"
#include <I2C_BNO055.h>
#include "robot.h"
#include "delay.h"

#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define ACC_STRIKE_THRESH 1500 // Z acc needed to strike

int16_t yaw_ref = 0;
volatile uint16_t servo1_reg = 0; 
volatile uint16_t servo2_reg = 0;
volatile char *String[20];

int16_t convert_yaw(int16_t raw){
	int16_t converted;
	converted = raw-yaw_ref+90;
	if(yaw_ref<90 && raw>180){
		converted = 90-(yaw_ref+(360-raw));
	}
	else if(yaw_ref>269 && raw<180){
		converted = 90+((360-yaw_ref)+raw);
	}
	return converted;
}



void update_servo(void){
		read_orientation(0);
		read_acc(0);
		int16_t base_angle = convert_yaw(yaw);
		if (z_acc < 0){
			//OCR1B = STRIKE_TOP; // recover from strike
		}
		else if ((z_acc > ACC_STRIKE_THRESH) ){
			strike(); //strike with striking servo
		}
		else if(base_angle < 180 || base_angle > 0){
			servo1_reg = setServo1Angle(base_angle); // Servo 1 angle: 90 degrees			
		}
}

void setup_robot(void)
{
	cli(); // disable interrupts
	PWM_init();  
	I2C_init(BNO055_ADDRESS_B);
	BNO055_i2c_init(OPERATION_MODE_NDOF);
	sei(); // enable interrupts
	_delay_ms(1000);
	OCR1A = BASE_SERVO_REG_MID;
	OCR1B = STRIKE_TOP;
	read_orientation();
}

void center_robot(void)
{
	yaw_ref = yaw;
}
