#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <avr/pgmspace.h>
#include "screen.h"
#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include "uart.h"
#include "HX8357.h"
#include "LCD_GFX.h"

char String[25];
char Lstr[75];


int bars[VISIBLE][2]; 
int tick = 0;
int flag = 0;
int next_note = 0;

const Song songs[NUM_SONGS] PROGMEM = {
	{
		"Mary Had a Little Lamb",
		{2,1,0,1,2,2,2,REST,1,1,1,REST,2,3,3,REST,2,1,0,1,2,2,2,2,1,1,2,1,0,REST,REST,REST,REST,REST,REST,REST,REST},
		33,
		"CDEG",
		{RED, ORANGE, YELLOW, CYAN}, 
		4
	}, 
	{
		"Row Row Row Your Boat",
		{0,0,0,1,2,2,1,2,3,4,REST,0,0,0,4,4,4,2,2,2,REST,REST,REST,REST,REST,REST,REST,REST},
		37,
		"CDEFG",
		{RED, ORANGE, RED, GREEN, CYAN}, 
		5
	},
	{
		"Hot Cross Buns",
		{2,1,0,REST,2,1,0,REST,0,0,0,0,1,1,1,1,2,1,0,REST,REST,REST,REST,REST,REST,REST,REST,REST},
		28,
		"GAB",
		{CYAN, BLUE, MAGENTA},
		3
	},
	{
		"Jingle Bells",
		{2,2,2,REST,2,2,2,REST,2,4,0,1,2,REST,REST,REST,3,3,3,3,3,2,2,2,2,1,1,2,1,REST,4,REST,REST,REST,REST,REST,REST},
		37,
		"CDEFG",
		{RED,ORANGE,YELLOW,GREEN,CYAN},
		5
	}
};

void init_pins(){
	DDRC &= ~(1 << PLAYPAUSE);
	DDRC &= ~(1 << TOGGLE);
}

ISR(TIMER2_OVF_vect) {

	flag = 1;

	TIFR2 |= (1 << TOV2);
}

void init_timer() {
	cli();
	
    // Set CTC mode (Clear Timer on Compare Match)
    //TCCR2A = (1 << WGM21);

    // Set prescaler to 1024
	TCCR2B = (1 << CS20);
	TCCR2B = (1 << CS21);
    TCCR2B = (1 << CS22);

    // Set the compare match value for a desired interrupt frequency
    //OCR2A = 250; // Example value for a 1 ms interrupt with a 16 MHz clock and prescaler of 64

    // Enable the compare match interrupt
    TIMSK2 |= (1 << TOIE2);
	
	//sei(); 
}

void intro(){
	LCD_setScreen(BG_COLOR);
	sprintf(Lstr,"Welcome to RoboXyloHero!");
	LCD_drawBlock(0, 200, 80, 220, CYAN);
	LCD_drawBlock(81, 240, 160, 260, RED);
	LCD_drawBlock(161, 260, 240, 280, MAGENTA);
	LCD_drawBlock(241, 200, 320, 220, YELLOW);
	LCD_drawBlock(321, 180, 400, 200, BLUE);
	LCD_drawBlock(401, 260, 480, 280, GREEN);
	LCD_drawString(50,LCD_HEIGHT-220, Lstr, WHITE, BG_COLOR);
	_delay_ms(5000);
	LCD_setScreen(BG_COLOR);
}

void draw_board(Song s){
	LCD_setScreen(BG_COLOR);
	for(int i=0; i<s.num_keys; ++i){
		LCD_drawBlock(i*SCREEN_W/s.num_keys, 0, (i+1)*SCREEN_W/s.num_keys-1, NOTE_SPACE, s.colors[i]);
		LCD_drawChar((i+0.5)*SCREEN_W/s.num_keys, (NOTE_SPACE*0.5)+8, s.keys[i], WHITE, s.colors[i]);
	}
}

void write_big(char * s)
{
	sprintf(Lstr,s);
	LCD_drawString(150,LCD_HEIGHT-100, Lstr, WHITE, BG_COLOR);
}

void get_ready(Song s){
	LCD_setScreen(BG_COLOR);
	sprintf(Lstr,"Place the keys in the console,");
	LCD_drawString(25,LCD_HEIGHT-30, Lstr, WHITE, BG_COLOR);
	sprintf(Lstr,"face the controller at the");
	LCD_drawString(25,LCD_HEIGHT-60, Lstr, WHITE, BG_COLOR);
	sprintf(Lstr,"screen, then press Play!");
	LCD_drawString(25,LCD_HEIGHT-90, Lstr, WHITE, BG_COLOR);
	for(int i=0; i<s.num_keys; ++i){
		LCD_drawBlock(150+i*50, 40, 150+i*50+40, 170, s.colors[i]);
		LCD_drawChar(150+i*50+10, 160, s.keys[i], WHITE, s.colors[i]);
	}
	while(!(PINC & (1<<PLAYPAUSE))){}
	
}

int min(int a, int b) {
	return (a < b) ? a : b;
}

void init_game(Song s){
	for(int i=0; i<VISIBLE; ++i){
		bars[i][0] = s.num_keys;
	}
}

void push_note(Song s){
	tick = 0;
	for(int i=0; i<VISIBLE-1; ++i){
		bars[i][0] = bars[i+1][0];
		bars[i][1] = bars[i+1][1];
	}
	if(next_note>=s.num_notes){}
	bars[VISIBLE-1][0] = (next_note<s.num_notes) ?  s.notes[next_note] : REST;
	bars[VISIBLE-1][1] = LCD_HEIGHT-1;
	next_note += 1;
}

void move_notes(Song s){
	for(int i=0; i<VISIBLE; ++i){
		bars[i][1] -= 1;
		if(bars[i][0]<s.num_keys || bars[i][0] == REST){
			if (!(bars[i][1]<=NOTE_SPACE)){
				LCD_drawBlock(bars[i][0]*SCREEN_W/s.num_keys, bars[i][1], (bars[i][0]+1)*SCREEN_W/s.num_keys,bars[i][1]+1, s.colors[bars[i][0]]);
			}
			if(bars[i][1]<LCD_HEIGHT-20){
				LCD_drawBlock(bars[i][0]*SCREEN_W/s.num_keys, bars[i][1]+NOTE_L+1, (bars[i][0]+1)*SCREEN_W/s.num_keys,bars[i][1]+NOTE_L+2, BG_COLOR);
			}
			if(bars[i][1]==NOTE_SPACE-10){
			}
			if(bars[i][1]==NOTE_SPACE-20){
			}
		}
	}

	++tick;
	if(tick == NOTE_SPACE-4){
		push_note(s);
	}
}

Song select_song(int size){
	int state = 0;
	sprintf(Lstr,"Choose your song!");
	LCD_drawString(25,LCD_HEIGHT-50, Lstr, WHITE, BG_COLOR);
	for(int i = 0; i<size; ++i){
		sprintf(Lstr,songs[i].title);
		int color = (state == i) ? BLUE : BG_COLOR;
		LCD_drawString(25,LCD_HEIGHT-100-30*i, Lstr, WHITE, color);
	}
	
	while(!(PINC & (1<<PLAYPAUSE))){
		if(PINC & (1<<TOGGLE)){
			++state;
			if(state == size){
				state = 0;
			}
			_delay_ms(100);
			for(int i = 0; i<size; ++i){
				sprintf(Lstr,songs[i].title);
				int color = (state == i) ? BLUE : BG_COLOR;
				LCD_drawString(25,LCD_HEIGHT-100-30*i, Lstr, WHITE, color);
			}
		}
	}
	return songs[state];
}

void setup_screen(void)
{
	lcd_init();
	init_pins();
	init_timer();
}