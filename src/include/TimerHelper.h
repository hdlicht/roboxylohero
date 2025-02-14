/*
 * TimerHelper.h
 *
 * Created: 10/16/2023 3:08:11 PM
 *  Author: rotom
 */ 


#ifndef TIMERHELPER_H_
#define TIMERHELPER_H_

#include <math.h>
#include <uart.h>
#include <stdio.h>

#define F_SYS_CLK 16000000 //System Clock Ticks per second.
#define OVERFLOW_TICKS_8BIT 256 // 8-bit counter overflows every (2^8) ticks.
#define OVERFLOW_TICKS_16BIT 65536 // 16-bit counter overflows every (2^16) ticks.

// These are the configuration bits used change prescaler divider of timer0 and timer1
#define TIMER_PSDIV_BITS 0b00000111
#define TIMER_PSDIV_1 0x01
#define TIMER_PSDIV_8 0x02
#define TIMER_PSDIV_64 0x03
#define TIMER_PSDIV_256 0x04
#define TIMER_PSDIV_1024 0x05

// TIMER 0
#define TIMER0_PRESCALER 256 // Make sure to change this to match config bits used above.
#define F_TIMER0_CLK (F_SYS_CLK/TIMER0_PRESCALER) // Timer0 ticks per second.
#define F_TIMER0_OVERFLOW floor(F_TIMER0_CLK/OVERFLOW_TICKS_8BIT) // Frequency that the overflow triggers on timer0
#define TIMER0_TICK_USEC floor(1000000/F_TIMER0_CLK) // Number of microseconds between each tick
#define TIMER0_OVERFLOW_USEC floor(1000000*OVERFLOW_TICKS_8BIT/F_TIMER0_CLK) // microseconds it takes the 8-bit counter at Timer0 to overflow

// TIMER 1
#define TIMER1_PRESCALER 64 // Make sure to change this to match Actual config bits used above.
#define F_TIMER1_CLK (F_SYS_CLK/TIMER1_PRESCALER) // Timer1 ticks per second.
#define F_TIMER1_OVERFLOW floor(F_TIMER1_CLK/OVERFLOW_TICKS_16BIT) // Frequency that the overflow triggers on timer1
#define TIMER1_TICK_USEC floor(1000000/F_TIMER1_CLK) // Number of microseconds between each tick
#define TIMER1_OVERFLOW_USEC floor(1000000*OVERFLOW_TICKS_16BIT/F_TIMER1_CLK) // microseconds it takes the 16-bit counter at Timer1 to overflow

// global variables for timer0
float timer0_ticks_per_msec; // timer0 ticks per second
float timer0_f_overflow; // frequency that timer0 overflows. Or timer0 cycles per second
float timer0_tick_dur_usec; // amount of microseconds each tick takes for timer0.
unsigned int timer0_overflow_msec; // duration of each timer0 clock cycle in microseconds

// global variables for timer1
float timer1_ticks_per_msec; // timer1 ticks per second
float timer1_f_overflow; // frequency that timer1 overflows. Or timer1 cycles per second
float timer1_tick_dur_usec; // amount of microseconds each tick takes for timer1.
unsigned int timer1_overflow_msec; // duration of each timer1 clock cycle in microseconds

uint8_t freq_PWM_ticks(uint16_t f_desired); // returns ticks needed to produce f_desired
uint8_t timer0_make_freq(uint16_t f_desired);
void timer0_psdiv_print(uint16_t psdiv);
void timer1_psdiv_print(uint16_t psdiv);
void timer0_sweep_psdiv(void);
void timer1_sweep_psdiv(void);
void timer0_set_psdiv(uint16_t psdiv);
void timer1_set_psdiv(uint16_t psdiv);

#endif /* TIMERHELPER_H_ */