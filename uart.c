/*
 * File:        uart.c
 * Purpose:     Provide UART routines for serial IO
 *
 * Notes:		
 *
 */

#include "msp.h"
#include "uart.h"  // you need to create this file with the function prototypes
#include <math.h>
#include "CortexM.h"

#define BAUD_RATE 115200      //default baud rate 
extern uint32_t SystemCoreClock;  // clock rate of MCU
#define RX BIT2;
#define TX BIT3;
#define UART2_TX BIT3;
#define UART2_RX BIT2;
extern int displayMode;


void uart0_init() {

    //Set the UART to RESET state (set bit0 of EUSCI_A0->CTLW0 register to '1'
    EUSCI_A0->CTLW0 |= BIT0;

    // bit15=0,      no parity bits
    // bit14=x,      not used when parity is disabled
    // bit13=0,      LSB first
    // bit12=0,      8-bit data length
    // bit11=0,      1 stop bit
    // bits10-8=000, asynchronous UART mode
    // bits7-6=11,   clock source to SMCLK
    // bit5=0,       reject erroneous characters and do not set flag
    // bit4=0,       do not set flag for break characters
    // bit3=0,       not dormant
    // bit2=0,       transmit data, not address (not used here)
    // bit1=0,       do not transmit break (not used here)
    // bit0=1,       hold logic in reset state while configuring

    // set CTLW0 - hold logic and configure clock source to SMCLK
    EUSCI_A0->CTLW0 &= ~BIT1;
    EUSCI_A0->CTLW0 &= ~BIT2;
    EUSCI_A0->CTLW0 &= ~BIT3;
    EUSCI_A0->CTLW0 &= ~BIT4;
    EUSCI_A0->CTLW0 &= ~BIT5;
    EUSCI_A0->CTLW0 |= BIT6;
    EUSCI_A0->CTLW0 |= BIT7;
    EUSCI_A0->CTLW0 &= ~BIT8;
    EUSCI_A0->CTLW0 &= ~BIT9;
    EUSCI_A0->CTLW0 &= ~BITA;
    EUSCI_A0->CTLW0 &= ~BITB;
    EUSCI_A0->CTLW0 &= ~BITC;
    EUSCI_A0->CTLW0 &= ~BITD;
    EUSCI_A0->CTLW0 &= ~BITE;
    EUSCI_A0->CTLW0 &= ~BITF;

    // baud rate
    // N = clock/baud rate = clock_speed/BAUD_RATE
    // set BRW register
    EUSCI_A0->BRW = (SystemCoreClock / BAUD_RATE);


    // clear first and second modulation stage bit fields
    // MCTLW register;
    EUSCI_A0->MCTLW &= 0xf;


    // P1.3 = TxD
    // P1.2 = RxD
    // we will be using P1.2, P1.3 for RX and TX but not in I/O mode, so we set Port1 SEL1=0 and SEL0=1
    // set SEL0, SEL1 appropriately

    P1->SEL0 |= RX;
    P1->SEL1 &= ~RX;
    P1->SEL0 |= TX;
    P1->SEL1 &= ~TX;

    // CTLW0 register - release from reset state
    EUSCI_A0->CTLW0 &= ~BIT0;

    // disable interrupts (transmit ready, start received, transmit empty, receive full)
    // IE register;
    EUSCI_A0->IE &= ~BIT0;
    EUSCI_A0->IE &= ~BIT1;
    EUSCI_A0->IE &= ~BIT2;
    EUSCI_A0->IE &= ~BIT3;

}

BYTE uart0_getchar() {
    BYTE inChar;

    // Wait for data
    // IFG register
    int val = is_set(EUSCI_A0->IFG, BIT0);
    while ((EUSCI_A0->IFG & BIT0) == 0) {
    }

    // read character and store in inChar variable
    // RXBUF register
    inChar = EUSCI_A0->RXBUF;

    //Return the 8-bit data from the receiver
    return (inChar);

}

void uart0_putchar(char ch) {
    // Wait until transmission of previous bit is complete
    // IFG register
    while ((EUSCI_A0->IFG & BIT1) != BIT1) {
    }

    // send ch character to uart
    // TXBUF register
    EUSCI_A0->TXBUF = ch;


}

void uart0_put(char *ptr_str) {
    while (*ptr_str != 0)
        uart0_putchar(*ptr_str++);
}


void uart2_init() {

    //The initalization of the UART2 port pins for the communication
    //over the HM-10 Bluetooth module device

    EUSCI_A2->CTLW0 |= UCSWRST;


    // bit15=0,      no parity bits
    // bit14=x,      not used when parity is disabled
    // bit13=0,      LSB first
    // bit12=0,      8-bit data length
    // bit11=0,      1 stop bit
    // bits10-8=000, asynchronous UART mode
    // bits7-6=11,   clock source to SMCLK
    // bit5=0,       reject erroneous characters and do not set flag
    // bit4=0,       do not set flag for break characters
    // bit3=0,       not dormant
    // bit2=0,       transmit data, not address (not used here)
    // bit1=0,       do not transmit break (not used here)
    // bit0=1,       hold logic in reset state while configuring

    EUSCI_A2->CTLW0 &= ~BITF;
    EUSCI_A2->CTLW0 &= ~BITE;
    EUSCI_A2->CTLW0 &= ~BITD;
    EUSCI_A2->CTLW0 &= ~BITC;
    EUSCI_A2->CTLW0 &= ~BITB;
    EUSCI_A2->CTLW0 &= ~BITA;

    EUSCI_A2->CTLW0 &= ~BIT9;
    EUSCI_A2->CTLW0 &= ~BIT8;

    EUSCI_A2->CTLW0 |= BIT7;
    EUSCI_A2->CTLW0 |= BIT6;

    EUSCI_A2->CTLW0 &= ~BIT5;
    EUSCI_A2->CTLW0 &= ~BIT4;
    EUSCI_A2->CTLW0 &= ~BIT3;
    EUSCI_A2->CTLW0 &= ~BIT2;
    EUSCI_A2->CTLW0 &= ~BIT1;

    EUSCI_A2->CTLW0 |= BIT0;

    // set CTLW0 - hold logic and configure clock source to SMCLK
    ;


    // baud rate


    // N = clock/baud rate = clock_speed/BAUD_RATE

    EUSCI_A2->BRW = 0x1A;
    // set BRW register
    ;

    // clear first and second modulation stage bit fields
    // MCTLW register;

    EUSCI_A2->MCTLW |= 0x00006F01;


    //Setting up the port pins for the UART2 serial driver for the
    //bluetooth communication.

    P3->SEL0 |= UART2_RX;
    P3->SEL1 &= ~UART2_RX;
    P3->SEL0 |= UART2_TX;
    P3->SEL1 &= ~UART2_TX;;

    EUSCI_A2->CTLW0 &= ~UCSWRST;
    EUSCI_A2->IE &= ~BIT0;
    EUSCI_A2->IE &= ~BIT1;
    EUSCI_A2->IE &= ~BIT2;
    EUSCI_A2->IE &= ~BIT3;


}


void uart2_putchar(char u2ch) {


//Waiting until the data is recieved in the queue.
    while ((EUSCI_A2->IFG & BIT1) != BIT1) {

    }

    //Storing it in the variable to transmit
    EUSCI_A2->TXBUF = u2ch;


}


BYTE uart2_getchar() {

    BYTE inChar2;
    // Wait for data
    // IFG register
    while ((EUSCI_A2->IFG & BIT0) == 0) {
		if((displayMode % 6) != 1){
			return (0x0);
		}
    }

    inChar2 = EUSCI_A2->RXBUF;
    // read character and store in inChar variable
    // RXBUF register
    ;
    //Returning the character recieved
    return (inChar2);

}

void uart2_put(char *ptr_str_uart2) {
    while (*ptr_str_uart2 != 0)
        uart2_putchar(*ptr_str_uart2++);

}


BOOLEAN uart2_dataAvaliable(){
	
//This is a peek function that allows to check for the character in a queue.
BOOLEAN go = FALSE;
	
//if a character is avaliable in UART2, set go = True	
	if( (EUSCI_A2 -> IFG & BIT0) != 0){
			go = TRUE;
			}
	;
return go;
}

static int is_set(uint32_t address, uint16_t mask) {
    int shft = log2(mask);
    int masked = address & mask;
    int val = masked >> shft;
    return val;
}
