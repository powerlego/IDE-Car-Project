#ifndef LEDS_H_
#define LEDS_H_

#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define CYAN 0x6
#define MAGENTA 0x5
#define WHITE 0x7

void LED1_Init(void);

void LED2_Init(void);

void LED1_On(void);

void LED1_Off(void);

BOOLEAN LED1_State(void);

#endif
