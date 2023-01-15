#include "msp.h"
#include "Common.h"

#define SW1 BIT1;
#define SW2 BIT4;

void Switch1_Init(void) {
    // configure PortPin for Switch 1
    P1->SEL0 &= ~SW1;
    P1->SEL1 &= ~SW1;
    // configure as input
    P1->DIR &= ~SW1;
    // enable pullup resistor
    P1->REN |= SW1;
    P1->OUT |= SW1;
}

void Switch2_Init(void) {
    // configure PortPin for Switch 2
    P1->SEL0 &= ~SW2;
    P1->SEL1 &= ~SW2;
    // configure as input
    P1->DIR &= ~SW2;
    // enable pullup resistor
    P1->REN |= SW2;
    P1->OUT |= SW2;
}

BOOLEAN Switch1_Pressed(void) {
    BOOLEAN retVal = FALSE;
    int val = P1->IN & SW1;
    if (val) {
        retVal = FALSE;
    } else {
        retVal = TRUE;
    }
    // check if pressed

    return (retVal);              // return TRUE(pressed) or FALSE(not pressed)
}

BOOLEAN Switch2_Pressed(void) {
    BOOLEAN retVal = FALSE;
    int val = P1->IN & SW2;
    if (val) {
        retVal = FALSE;
    } else {
        retVal = TRUE;
    }
    // check if pressed

    return (retVal);              // return TRUE(pressed) or FALSE(not pressed)
}
