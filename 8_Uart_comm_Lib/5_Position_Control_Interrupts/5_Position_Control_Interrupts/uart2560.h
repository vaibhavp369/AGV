/*

UART2560.h
Created : 3/1/2021
Author : Vaibhav

*/

#ifndef UART2560_H
#define UART2560_H

void USART_init(unsigned int baud); // intialize UART RX,TX, RX complete interrupt

char USART_Receive(void);  // To Receive a character

void USART_Transmit(unsigned char data); // To Send a character
 
#endif

/* End USART_2560 */