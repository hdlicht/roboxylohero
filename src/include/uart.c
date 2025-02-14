#include "uart.h"
#include <avr/io.h>

/*
For part a of the lab, I made a few changes to the uart.c file. 
1. Changing the way bits are set in the init method. 
2. Condensed 2 methods to 1. 
3. Replaced while loop with for loop. 
*/

void UART_init(int BAUD_PRESCALER)
{
	
	/*Set baud rate */
	UBRR0H = (unsigned char)(BAUD_PRESCALER>>8);
	UBRR0L = (unsigned char)BAUD_PRESCALER;
	//Enable receiver and transmitter
	UCSR0B |= (1<<RXEN0);
	UCSR0B |= (1<<TXEN0);
	/* Set frame format: 2 stop bits, 8 data bits */
	UCSR0C |= (1<<UCSZ01);
	UCSR0C |= (1<<UCSZ00);
	UCSR0C |= (1<<USBS0);
}

void UART_putstring(char* str)
{
	for (int i = 0; str[i] != 0; i++) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = str[i];
	}
}
