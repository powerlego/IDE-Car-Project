#include "msp.h"
#include "Common.h"

#define LED1 BIT0;
#define LED2_RED BIT0;
#define LED2_GREEN BIT1;
#define LED2_BLUE BIT2;

// Main Program
// 1. Init LEDs
// 2. Init Switch
// 3. init state=0
// 4. If SW1 pressed, light LED1
// 5. if SW2 pressed, cycle next color of R,G,B,C,M,Y,W
// 5. If button released
//		Turn off the LED
//
//

static BOOLEAN LED1_Status;

void LED1_Init(void) {
    // configure PortPin for LED1 as port I/O
    P1->SEL0 &= ~LED1;
    P1->SEL1 &= ~LED1;
    // make built-in LED1 LED high drive strength
    P1->DS |= LED1;
    // make built-in LED1 out
    P1->DIR |= LED1;
    // turn off LED
    P1->OUT &= ~LED1;
    LED1_Status = FALSE;
}

void LED2_Init(void) {
    // configure PortPin for LED2 as port I/O
    P2->SEL0 &= ~LED2_RED;
    P2->SEL1 &= ~LED2_RED;
    P2->SEL0 &= ~LED2_GREEN;
    P2->SEL1 &= ~LED2_GREEN;
    P2->SEL0 &= ~LED2_BLUE;
    P2->SEL1 &= ~LED2_BLUE;

    // make built-in LED2 LEDs high drive strength
    P2->DS |= LED2_RED;
    P2->DS |= LED2_GREEN;
    P2->DS |= LED2_BLUE;
    // make built-in LED2 out
    P2->DIR |= LED2_RED;
    P2->DIR |= LED2_GREEN;
    P2->DIR |= LED2_BLUE;
    // turn off LED
    P2->OUT &= ~LED2_RED;
    P2->OUT &= ~LED2_GREEN;
    P2->OUT &= ~LED2_BLUE;

}


void LED1_On(void) {
    P1->OUT |= LED1;
    LED1_Status = TRUE;
}

void LED1_Off(void) {
    P1->OUT &= ~LED1;
    LED1_Status = FALSE;
}

BOOLEAN LED1_State(void) {
    return LED1_Status;
}
