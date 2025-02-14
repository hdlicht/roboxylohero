/*
 * TimerHelper.c
 * Programming the ultrasonic distance sensor
 * Created: 10/16/2023 9:52:31 AM
 *  Author: Alexander
 */ 

#include "TimerHelper.h"
#include <avr/io.h>
#include "uart.h"


// frequency of interrupts as ticks = F_timer0_clk / freq_toggle. 
// but for square wave frequency of toggle = 2 * freq_squarewave
// For PWM we need to divide by 2 an additional time since it doubles the period. 
#define SONIC_SPEED 34// speed of sound in cm/msec
#define USEC_TO_CM (34/2000) //

char String[50];

uint8_t freq_PWM_ticks(uint16_t f_desired){ // returns ticks needed to produce f_desired
	uint16_t ticks_calculated;
	uint8_t ticks_between;
	int f_actual;
	ticks_calculated = (timer0_ticks_per_msec*1000/(2*f_desired)); // PWM
	if (ticks_calculated > OVERFLOW_TICKS_8BIT){
		sprintf(String,"Cannot generate %uHz in 1 cycle since it takes %u ticks \n", f_desired, ticks_calculated); UART_putstring(String);
		return ticks_calculated;
	}
	else{
		ticks_between = ticks_calculated;
		f_actual = floor(timer0_ticks_per_msec*1000/(2*ticks_between));
		sprintf(String,"Generate %uHz PWM signal by triggering every %u ticks \n", f_actual, ticks_between); UART_putstring(String);
		return ticks_between;
	}
}

// note cannot use sprintf to print floats. need to use dtostrf(float) first to make float a string. 
uint8_t timer0_find_PWM_freq(uint16_t f_desired){ // f_desired cannot exceed 65535
	uint16_t ticks_calculated;
	uint8_t ticks_between;
	int f_actual;
	ticks_calculated = (F_TIMER0_CLK/(4*f_desired)); // PWM
	if (ticks_calculated > OVERFLOW_TICKS_8BIT){
		sprintf(String,"Cannot generate %uHz in 1 cycle since it takes %u ticks \n", f_desired, ticks_calculated); UART_putstring(String); 
		return ticks_calculated;
	}
	else{
		ticks_between = ticks_calculated;
		f_actual = floor(F_TIMER0_CLK/(4*ticks_between));
		sprintf(String,"Generate %uHz PWM signal by triggering every %u ticks \n", f_actual, ticks_between); UART_putstring(String);
		return ticks_between;
	}
}

void timer0_psdiv_print(uint16_t psdiv){
	char str_ticks_per_msec[10]; char str_f_overflow[10]; char str_tick_usec[10];// needed to print float strings.
	float psdiv_fl = psdiv; // Had to make a float to prevent making results integers.
	float ticks_per_msec = (16000000/(psdiv_fl))/1000; // ticks per millisecond of the timer.
	float f_overflow = (ticks_per_msec*1000)/(OVERFLOW_TICKS_8BIT); // overflows (clock cycles) per second of timer.
	float tick_usec = (1000/ticks_per_msec); // time each tick takes in microseconds.
	unsigned int timer_overflow_msec = floor(1000/f_overflow); // milliseconds each timer cycle takes. Time to overflow.
	dtostrf(ticks_per_msec,6,3,str_ticks_per_msec); dtostrf(f_overflow,3,3,str_f_overflow); dtostrf(tick_usec,2,4,str_tick_usec);
	sprintf(String,"With prescaler division of %u \ntimer0 runs at %s ticks/msec and %s cycles/sec.\n", psdiv, str_ticks_per_msec,str_f_overflow); UART_putstring(String);
	sprintf(String,"One tick is %s usec. timer0 overflows every %u msec.\n", str_tick_usec, timer_overflow_msec); UART_putstring(String);
}

void timer1_psdiv_print(uint16_t psdiv){
    char str_ticks_per_msec[10]; char str_f_overflow[10]; char str_tick_usec[10];// needed to print float strings.
	float psdiv_fl = psdiv; // Had to make a float to prevent making results integers. 
	float ticks_per_msec = (16000000/(psdiv_fl))/1000; // ticks per millisecond of the timer.
	float f_overflow = (ticks_per_msec*1000)/(OVERFLOW_TICKS_16BIT); // overflows (clock cycles) per second of timer.
	float tick_usec = (1000/ticks_per_msec); // time each tick takes in microseconds.
	unsigned int timer_overflow_msec = floor(1000/f_overflow); // milliseconds each timer cycle takes. Time to overflow.
	dtostrf(ticks_per_msec,6,3,str_ticks_per_msec); dtostrf(f_overflow,3,3,str_f_overflow); dtostrf(tick_usec,2,4,str_tick_usec);
	sprintf(String,"With prescaler division of %u \nTimer1 runs at %s ticks/msec and %s cycles/sec.\n", psdiv, str_ticks_per_msec,str_f_overflow); UART_putstring(String);
	sprintf(String,"One tick is %s usec. Timer1 overflows every %u msec.\n", str_tick_usec, timer_overflow_msec); UART_putstring(String);
}

void timer0_sweep_psdiv(void){
	int i; // iterator.
	uint16_t psdivs[] = {1,8,64,256,1024}; // List of all prescaler dividers for timer0 and timer1
	for (i = 0;i<5;i++){
		timer0_psdiv_print(psdivs[i]);
	}
}

void timer1_sweep_psdiv(void){
	int i; // iterator.
	uint16_t psdivs[] = {1,8,64,256,1024}; 
	for (i = 0;i<5;i++){
		UART_putstring("\n");
		timer1_psdiv_print(psdivs[i]);
	}
}


void timer0_set_psdiv(uint16_t psdiv_config_bits){
	// input is the 3 bit code corresponding to the prescaler divider.
	float psdiv_fl[] = {1,8,64,256,1024}; // Had to make a float to prevent making results integers.
	TCCR0B &= ~TIMER_PSDIV_BITS; TCCR0B |= psdiv_config_bits; // set the prescaler for the timer
	timer0_ticks_per_msec = (16000000/(psdiv_fl[psdiv_config_bits-1]))/1000; // ticks per millisecond of the timer.
	timer0_f_overflow = (timer0_ticks_per_msec*1000)/(OVERFLOW_TICKS_8BIT)	; // overflows (clock cycles) per second of timer.
	timer0_tick_dur_usec = (1000/timer0_ticks_per_msec); // time each tick takes in microseconds.
	timer0_overflow_msec = floor(1000/timer0_f_overflow); // milliseconds each timer cycle takes. Time to overflow.
	timer0_psdiv_print(psdiv_fl[psdiv_config_bits-1]);
}

void timer1_set_psdiv(uint16_t psdiv_config_bits){
	// input is the 3 bit code corresponding to the prescaler divider.
	float psdiv_fl[] = {1,8,64,256,1024}; // Had to make a float to prevent making results integers.	
	TCCR1B &= ~TIMER_PSDIV_BITS; TCCR1B |= psdiv_config_bits; // set the prescaler for the timer
	timer1_ticks_per_msec = (16000000/(psdiv_fl[psdiv_config_bits-1]))/1000; // ticks per millisecond of the timer.
	timer1_f_overflow = (timer1_ticks_per_msec*1000)/(OVERFLOW_TICKS_16BIT); // overflows (clock cycles) per second of timer.
	timer1_tick_dur_usec = (1000/timer1_ticks_per_msec); // time each tick takes in microseconds.
	timer1_overflow_msec = floor(1000/timer1_f_overflow); // milliseconds each timer cycle takes. Time to overflow.
	timer1_psdiv_print(psdiv_fl[psdiv_config_bits-1]);
}