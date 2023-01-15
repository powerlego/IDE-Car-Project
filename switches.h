#ifndef SWITCHES_H_
#define SWITCHES_H_

#include "Common.h"

void Switch1_Init(void);

void Switch2_Init(void);

BOOLEAN Switch1_Pressed(void);

BOOLEAN Switch2_Pressed(void);

#define SW1 BIT1
#define SW2 BIT4
#endif
