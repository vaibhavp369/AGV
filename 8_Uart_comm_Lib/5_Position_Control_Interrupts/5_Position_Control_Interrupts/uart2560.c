
#define F_CPU 14745600

#include<avr/interrupt.h>
#include <util/delay.h>
#include "uart2560.h"
#include "lcd.h"

#define FOSC 14745600


void USART_init(unsigned int baud)
{
	cli();
	unsigned int ubrr = FOSC/16/baud-1;
	UBRR2H = (unsigned char) (ubrr >> 8);		// set baud rate
	UBRR2L = (unsigned char) ubrr;				// set baud rate
	UCSR2B = ( (1<< TXEN2) | (1 << RXEN2) | (1 << RXCIE2) );	// enable TX,RX,RX complete interrupt 
	UCSR0C = ( (1 << UCSZ21) | (1 << UCSZ20) );  // 8 bit data frame and 1 stop bit
	sei();
}


char USART_Receive(void)
{
	while ( !( UCSR2A & ( 1<< RXC2) ) );
	return (char) UDR2;
}


void USART_Transmit(unsigned char data)
{
	lcd_wr_char(2,5,data);
	while( !( UCSR2A & (1<< UDRE2) ) );
	UDR2 = data;
	
}