#ifndef _UART_H_
#define _UART_H_

#include "Common.h"

void uart0_init(void);

BYTE uart0_getchar(void);

void uart0_putchar(char ch);

void uart0_put(char *ptr_str);
//UART2 headers, and intializations. 

void uart2_init(void);

//Uart2- puts out the character on the screen.
void uart2_putchar(char u2ch);

//Uart2- gets the character from the queue.
BYTE uart2_getchar(void);

//Uart2- puts out the sentence on the screen 
void uart2_put(char *ptr_str_uart2);
BOOLEAN uart2_dataAvaliable(void);
#endif
