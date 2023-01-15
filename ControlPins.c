#include "msp.h"
#include "Timer32.h"
#include "Common.h"
#include "CortexM.h"
#include "SysTickTimer.h"
#include "ControlPins.h"
#include "ADC14.h"


//#include "Leds.h"
extern uint32_t SystemCoreClock;

// default SI integration time is 7.5ms = 133Hz
//
extern float INTEGRATION_TIME;



// default CLK frequency of the camera 180KHz (assume 48MHz clock)
// NOTE: we have to double 50000, because we need a clock for the rising edge and one for the falling edge.
//#define HIGH_CLOCK_SPEED 48000000

#define CLK_PERIOD ((double)SystemCoreClock/180000.0)

// ADC_In() gets the latest value from the ADC
// ADC will be P4.7 A6
// SI Pin will be P5.5 A0
// CLK Pin will be P5.4 A1
#define ADC BIT7


unsigned long tempCounter = 0;
static long pixelCounter = 0;

extern uint16_t line[128];
extern BOOLEAN g_sendData;


////////////////////////////////////////////
//
// this is the handler for the SI signal.
//
////////////////////////////////////////////
void SI_Handler(void) {
    // the clock could stop when its high, so check on that
    if ((P5->OUT & CLK) != 0)
        P5->OUT &= ~CLK; // set the clock low in case it was high.
    // This is the SI,CLK sequencing to start the data transfer process
    // SI HIGH
    P5->OUT |= SI;
    // CLK HIGH
    P5->OUT |= CLK;
    // SI LOW
    P5->OUT &= ~SI;
    // CLK LOW
    P5->OUT &= ~CLK;
    // OK, Data should be ready to clock out, so start the clock
    // Start the clock after we issues a SI pulse.
    EnableSysTickTimer();
}

// ADC_In() gets the latest value from the ADC
// ADC will be P4.7 
// SI Pin will be P5.5 
// CLK Pin will be P5.4 
//////////////////////////////////////////
//
// Init the SI timer
// using the Timer32-1 timer
// going to use pin P5.5
//
//////////////////////////////////////////
void ControlPin_SI_Init() {
    // frequency of 133 Hz works OK, but could use more light
    // so try 50Hz?
    // Go with 50Hz for now - integration period of 20ms
    unsigned long period = CalcPeriodFromFrequency(2.0 / (double) (INTEGRATION_TIME));
    // initialize P5.5 and make it output (P5.5 SI Pin)

    // configure built-in LED1 as GPIO
    P5->SEL0 &= ~SI;
    P5->SEL1 &= ~SI;
    // make SI high drive strength
    P5->DS |= SI;
    // make P5.5 out
    P5->DIR |= SI;
    // turn off SI
    P5->OUT &= ~SI;
    // start Timer
    Timer32_1_Init(*SI_Handler, period, T32DIV1);
}

//////////////////////////////////////////
//
// Init the CLK timer
// using the SysTick base ARM timer
// going to use pin P5.4
//
//////////////////////////////////////////
void ControlPin_CLK_Init() {
    // use 200000 to make a 100K clock, 1 interrupt for each edge
    unsigned long period = CalcPeriodFromFrequency(200000);
    // initialize P5.4 and make it output (P5.4 CLK Pin)

    // configure P5.4 as GPIO
    P5->SEL0 &= ~CLK;
    P5->SEL1 &= ~CLK;
    // make CLK high drive strength
    P5->DS |= CLK;
    // make P5.5 out
    P5->DIR |= CLK;
    // turn off CLK
    P5->OUT &= ~CLK;
    // if the period is based on a 48MHz clock, each tick would be 20.83 ns
    // i want a 100KHz clock
    SysTickTimer_Init(*CLK_Handler, period);
    // stop the clock for now...
    DisableSysTickTimer();


}

// this is the handler for the CLK signal.
// on every clock pulse, we need to:
// 1. Toggle the CLK pin
// 2. If the CLK is high, read the value at the ADC A6
// 3. If I have read 128 pixels, 
//			set the g_sendData flag so the main program can send the data to the computer
//			disable the CLK timer
//
void CLK_Handler(void) {
    unsigned int ADC_val;
    // toggle the clock
    P5->OUT ^= CLK;
    // check to see if clock is high
    if ((P5->OUT & CLK) != 0) {
        // read data from ADC
        ADC_val = ADC_In();
        // save into the line buffer
        line[pixelCounter] = ADC_val;
        // increment the pixelCounter
        pixelCounter = (pixelCounter + 1);
        //are we done??
        if (pixelCounter == 128) {
            // set the flag to indicate its OK to send data out uart
            g_sendData = TRUE;
            // stop the clock until the next SI pulse
            DisableSysTickTimer();    // stop the clock
            pixelCounter = 0;            // reset the counter
        }
    }
}
