#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "LCD_GFX.h"


#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define F_CPU 16000000UL


#define SCREEN_W 480
#define SCREEN_L 320
#define VISIBLE 6
#define NUM_SONGS 4
#define NOTE_SPACE (SCREEN_L/VISIBLE)
#define NOTE_L (NOTE_SPACE-25) 
#define BG_COLOR BLACK
#define REST 10

#define PLAYPAUSE DDC2
#define TOGGLE DDC1

typedef struct 
{
	char title[50];
	int notes[50];
	int num_notes;
	char keys[5];
	int colors[5];
	int num_keys;
} Song;

extern const Song songs[NUM_SONGS];

extern int flag;
extern int next_note;

void init_pins();

void init_timer();

void intro();

void draw_board(Song s);

void get_ready(Song s);

int min(int a, int b);

void init_game(Song s);

void push_note(Song s);

void move_notes(Song s);

Song select_song(int size);

void write_big(char * s);

void setup_screen(void);