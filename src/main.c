#include <avr/io.h>
#include "stdio.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include "string.h"
#include "robot.h"
#include "I2C_BNO055.h"
#include "screen.h"
#include "LCD_GFX.h"
#include "uart.h"

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
char String[50];



void watch_pause(){
	if(PINC & (1<<PLAYPAUSE)){
		cli();
		write_big("Paused.");
		while(!(PINC & (1<<PLAYPAUSE)))
		{
			// nothing
		}
		sei();
		write_big("Unpaused.");
        center_robot();
	}
}

void play_game(Song s){
	sei();
	push_note(s);
	while(next_note < s.num_notes + VISIBLE){
		watch_pause();
        update_servo();
		if(flag==1){
			flag = 0;
			move_notes(s);
			_delay_ms(25);
		}
	}
	cli();
	next_note = 0;
}

int main(void)
{
    //setup_robot();
    UART_init(BAUD_PRESCALER);
    // setup_screen();
	// intro();
    while(1)
    {	
        read_orientation();
        sprintf(String, "%d \n\r", yaw);
        UART_putstring(String);
        _delay_ms(10);
		// LCD_setScreen(BG_COLOR);
		// Song x = select_song(NUM_SONGS);
		// get_ready(x);
		// init_game(x);
		// draw_board(x);
		// play_game(x);
	}
}