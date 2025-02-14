/**
 * \file
 *
 * \brief Empty user application template
 *
 */

#include "stdint.h"

volatile uint16_t servo1_reg; 
volatile uint16_t servo2_reg;

int16_t convert_yaw(int16_t raw);
void update_servo(void);
void setup_robot(void);
void center_robot(void);