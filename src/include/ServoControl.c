/*
 * ServoControl.c
 *
 * Created: 12/5/2023 4:57:48 PM
 *  Author: student
 */ 
#include "ServoControl.h"

//uint16_t[5] keys = {KEY1_VAL, KEY2_VAL, KEY3_VAL, KEY4_VAL, KEY5_VAL};

// Function to initialize Timer1 for PWM
int16_t duty_50;
int16_t duty_res;
void PWM_init(void) {
	baseRef = 90;
	strike_time = 0;
	//keys[0] = KEY1_VAL;	keys[1] = KEY2_VAL; keys[2] = KEY3_VAL; keys[3] = KEY4_VAL; keys[4] = KEY5_VAL;
	// Set the mode for Timer1 (WGM13:0 = 1110 for Fast PWM mode, TOP = ICR1)
	TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);// | (1 << COM1A0) | (1 << COM1B0);
	TCCR1A &= ~(1<<WGM10);
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	TIMSK1 |= (1<<TOIE1); // enable overflwo interrupt
	// Set the prescaler to 8 to make it update every 0.5us (2MHz). 
	TCCR1B &= ~TIMER_PSDIV_BITS; TCCR1B |= TIMER_PSDIV_8;
	timer1_psdiv_print(TIMER_PSDIV_8);
	// Set the ICR1 register for a 5msms PWM period (200Hz)
	ICR1 = 10000; //5ms.			
	// Set the OC1A and OC1B pins as outputs
	DDRB |= (1 << DDB1) | (1 << DDB2);
	PORTB |= ((1 << DDB1) | (1 << DDB2));
	duty_50 = ICR1 / 2;
	duty_res = ICR1/500;
}

ISR(TIMER1_OVF_vect){
	if (OCR1B >= STRIKE_BOTTOM -100){
		strike_time++;
		if(strike_time > STRIKE_DUR){
			strike_time = 0;
			OCR1B = STRIKE_TOP;
		}		
	}
}

void strike(void){
	OCR1B = STRIKE_BOTTOM;
	strike_time=0;
}

void selectKey(uint8_t key){
	if (key < 0){ key =0;} else if (key > 4) {key = 4;}
	OCR1A = keys[key];	
}

void strikeKey(uint8_t key){
	selectKey(key);
	strike();
}

void rotateBase(int amount){
	
}

uint16_t setBaseRef(uint16_t ref_angle){
	baseRef = ref_angle;
}

// Function to set the servo angle for Servo 1 (OC1A)
uint16_t setServo1Angle(int angle) {
	uint16_t new_duty;
//	OCR1A = (int)(((float)angle / 180.0) * 0.12 * 39999); // Map angle to PWM pulse width	
	//if (angle > BASE_SERVO_ANGLE_MAX){angle = BASE_SERVO_ANGLE_MAX;}
	//else if (angle < BASE_SERVO_ANGLE_MIN) {BASE_SERVO_ANGLE_MIN;}
	new_duty = BASE_SERVO_REG_MID+16*(90-angle);//-baseRef);//(int)(((float)angle / 180.0) * 0.12 * ICR1); // Map angle to PWM pulse width
	if (new_duty > BASE_SERVO_REG_MAX) {new_duty = BASE_SERVO_REG_MAX;}
	else if (new_duty < BASE_SERVO_REG_MIN) {new_duty = BASE_SERVO_REG_MIN;}
	OCR1A = new_duty;
//	OCR1A = (int)(((float)angle / 180.0) * 0.12 * ICR1) + BASE_SERVO_REG_MIN;
	return new_duty;
}

// Function to set the servo angle for Servo 2 (OC1B)
uint16_t setServo2Angle(int angle) {
//	OCR1B = (int)(((float)angle / 180.0) * 0.12 * 39999);
	//if (angle > STRIKE_SERVO_ANGLE_MAX){angle = STRIKE_SERVO_ANGLE_MAX;}
	//else if (angle < STRIKE_SERVO_ANGLE_MIN) {STRIKE_SERVO_ANGLE_MIN;}
	OCR1B = (int)(((float)angle / 180.0) * 0.12 * ICR1);
	return OCR1B;
}

