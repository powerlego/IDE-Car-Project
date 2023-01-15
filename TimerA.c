// TimerA.c

/*  LJBeato
    2021
    TimerA functionality to drive DC motor
    and Servo Motor
 */


#include "msp.h"
#include <stdint.h>
#include <stdio.h>
#include "TimerA.h"
#include "uart.h"


static uint32_t DEFAULT_PERIOD_A0[5] = {0, 0, 0, 0, 0};
static uint32_t DEFAULT_PERIOD_A2[5] = {0, 0, 0, 0, 0};
//***************************PWM_Init*******************************
// PWM output on P2.4, P2.5, P2.6, P2.7
// Inputs:  period 
//          percentDutyCycle (0 -> 1.0)
//          pin number (1,2,3,4)
// Outputs: none


int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin) {

    // Timer A0.1
    if (pin == 1) {
        P2->SEL0 |= A0_PIN1;
        P2->SEL1 &= ~A0_PIN1;
        P2->DIR |= A0_PIN1;
    }
        // Timer A0.2
    else if (pin == 2) {
        P2->SEL0 |= A0_PIN2;
        P2->SEL1 &= ~A0_PIN2;
        P2->DIR |= A0_PIN2;
    }
        // Timer A0.3
    else if (pin == 3) {
        P2->SEL0 |= A0_PIN3;
        P2->SEL1 &= ~A0_PIN3;
        P2->DIR |= A0_PIN3;
    }
        // Timer A0.4
    else if (pin == 4) {
        P2->SEL0 |= A0_PIN4;
        P2->SEL1 &= ~A0_PIN4;
        P2->DIR |= A0_PIN4;
    } else return -2;


    // TIMER_A0->CCTL[0]
    TIMER_A0->CCTL[0] |= 0xC0;

    // TIMER_A0->CCR[0]
    TIMER_A0->CCR[0] = period;

    // save the period for this timer instance
    // DEFAULT_PERIOD_A0[pin] where pin is the pin number
    DEFAULT_PERIOD_A0[pin] = period;

    // set the duty cycle
    // TIMER_A0->CCTL[pin]
    TIMER_A0->CCTL[pin] |= 0xC0;

    uint16_t dutyCycle = (uint16_t) (percentDutyCycle * (double) DEFAULT_PERIOD_A0[pin]);

    // CCR1 duty cycle is duty1/period
    //TIMER_A0->CCR[pin]
    TIMER_A0->CCR[pin] = dutyCycle;

    // Timer CONTROL register
    // TIMER_A0->CTL
    TIMER_A0->CTL |= 0x0214;
    return 0;
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on pin
// Inputs:  dutycycle, pin
// Outputs: none
// percentDutyCycle is a number between 0 and 1  (ie. 0.5 = 50%)
void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin) {
    uint16_t dutyCycle = (uint16_t) (percentDutyCycle * (double) DEFAULT_PERIOD_A0[pin]);
    TIMER_A0->CCR[pin] = dutyCycle;
}

//***************************PWM_Init*******************************
// PWM output on P5.6
// Inputs:  period 
//          duty cycle
//          pin number (1,2,3,4), but always 1
// Outputs: none
int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin) {

    // NOTE: Timer A2 only exposes 1 PWM pin
    // TimerA2.1
    if (pin == 1) {
        P5->SEL0 |= A2_PIN1;
        P5->SEL1 &= ~A2_PIN1;
        P5->DIR |= A2_PIN1;
    } else return -2;

    // NOTE: Setup similar to TimerA0
    // TIMER_A2->CCTL[0]
    TIMER_A2->CCTL[0] |= 0xC0;

    // TIMER_A2->CCR[0]
    TIMER_A2->CCR[0] = period;

    // save the period for this timer instance
    // DEFAULT_PERIOD_A2[pin] where pin is the pin number
    DEFAULT_PERIOD_A2[pin] = period;

    // set the duty cycle
    // TIMER_A2->CCTL[pin]
    TIMER_A2->CCTL[pin] |= 0xC0;

    uint16_t dutyCycle = (uint16_t) (percentDutyCycle * (double) DEFAULT_PERIOD_A2[pin]);

    // CCR1 duty cycle is duty1/period
    //TIMER_A2->CCR[pin]
    TIMER_A2->CCR[pin] = dutyCycle;

    // Timer CONTROL register
    // TIMER_A2->CTL
    TIMER_A2->CTL |= 0x02D4;
    TIMER_A2->EX0 |= 0x1;
    // You will have to use the prescaler (clock divider) to get down to 20ms
    return 0;
}

//***************************PWM_Duty1*******************************
// change duty cycle of PWM output on P5.6
// Inputs:  percentDutyCycle, pin  (should always be 1 for TimerA2.1)
//         
// Outputs: none
// 
void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin) {
    uint16_t dutyCycle = (uint16_t) (percentDutyCycle * (double) DEFAULT_PERIOD_A2[pin]);
    TIMER_A2->CCR[pin] = dutyCycle;
}

