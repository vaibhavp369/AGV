/*
 * UART_328.c
 *
 * Created: 06/12/2020 12:07:15 AM
 * Author : vaibhav
 */ 


#include <avr/io.h>
#define F_CPU 14745600
#define BAUD_RATE 9600
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "uart2560.h"


char Rx_buff[130];
int i=0;
char current_pos_x;
char current_pos_y;

char tx_buff[5]= {'2','5','E'};

char data = 'A';
/*
void USART_init(unsigned int ubrr)
{
	cli();
	UBRR2H = (unsigned char) (ubrr >> 8);		// set baud rate
	UBRR2L = (unsigned char) ubrr;				// set baud rate
	UCSR2B = ( (1<< TXEN2) | (1 << RXEN2) | (1 << RXCIE2) );	// enable TX,RX,RX complete interrupt 
	UCSR0C = ( (1 << UCSZ21) | (1 << UCSZ20) );  // 8 bit data frame and 1 stop bit
	sei();
}


void USART_Transmit(unsigned char data)
{
	lcd_wr_char(2,5,data);
	while( !( UCSR2A & (1<< UDRE2) ) );
	UDR2 = data;
	
}

char USART_Receive(void)
{
	while ( !( UCSR2A & ( 1<< RXC2) ) );
	return (char) UDR2;
}
*/
void print_lcd()
{
	i=0;
	while(Rx_buff[i] != '%')
	 {
		 lcd_wr_char(1,1,Rx_buff[i]);
		 _delay_ms(800);
		 i++;
	 }
	 
	 for(i=0;i<3;i++)
	 {
		 USART_Transmit(tx_buff[i]);
	 }
}


void send_fb()
{
	for(i=0;i<3;i++)
	 {
		 USART_Transmit(tx_buff[i]);
	 }
}

ISR(USART2_RX_vect)
{
	data = UDR2;
	if(data == '$')
	   {
		   current_pos_x = USART_Receive();
		   current_pos_y = USART_Receive();
		   for(i=0;i<130;i++)
		    {
			    Rx_buff[i] = USART_Receive();
				if(Rx_buff[i] == '%')
				  break;	
			}
		lcd_wr_char(2,1,current_pos_x);
		lcd_wr_char(2,2,current_pos_y);
		print_lcd();
	   }
	 
	else if(data == '#')
	 {
		 send_fb();
	 }
	
	
}

int main(void)
{
    USART_init(BAUD_RATE);
	char Rx_data;
	lcd_port_config();
	lcd_init();
    while (1) 
    {
	
	}
}

 


