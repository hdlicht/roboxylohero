/*
 * ServoControl.h
 *
 * Created: 12/5/2023 4:58:25 PM
 *  Author: student
 */ 

#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_
#include <avr/io.h>
#include "avr/interrupt.h"
#include "TimerHelper.h"


#define BASE_SERVO_ANGLE_MAX 180
#define BASE_SERVO_ANGLE_MIN 0
#define BASE_SERVO_REG_MIN 1000
#define BASE_SERVO_REG_MAX 3500
#define BASE_SERVO_REG_MID 2200
#define BASE_ERROR 100 
#define KEY4_VAL 1500
#define KEY3_VAL 2000
#define KEY2_VAL 2300
#define KEY1_VAL 2650
#define KEY0_VAL 3000
#define STRIKE_TOP 1200
#define STRIKE_BOTTOM 2100
#define STRIKE_DUR 50 // How many cycles it it takes to strike a key

#define STRIKE_SERVO_ANGLE_MAX 180
#define STRIKE_SERVO_ANGLE_MIN 0

static uint16_t keys[5] = {KEY0_VAL,KEY1_VAL, KEY2_VAL, KEY3_VAL, KEY4_VAL};
void selectKey(uint8_t key);
void strike(void);
void strikeKey(uint8_t key);
uint8_t strike_time;
volatile uint16_t baseRef;
uint16_t setBaseRef(uint16_t ref_angle);


void PWM_init(void);
uint16_t setServo1Angle(int angle); // Base motor (yaw)
uint16_t setServo2Angle(int angle); // Striking motor (pitch)

#endif /* SERVOCONTROL_H_ */