/*
 * i2c.h
 *
 *  Created on: 9/4/21
 *      Author: LJBeato
 */

#ifndef I2C_H_
#define I2C_H_

#include "Common.h"

void i2c0_Init(int listenerAddress);

void i2c0_put(BYTE *data, unsigned int len);

void i2c0_putchar(BYTE ch);

#define START()\
{\
    EUSCI_B0->CTLW0 |= BIT1;\
    while(is_set(EUSCI_B0->IFG, BIT1) == 0){\
    }\
}
#define STOP()\
{\
    EUSCI_B0->CTLW0 |= BIT2;\
    while(is_set(EUSCI_B0->IFG, BIT3) == 0){\
    }\
}

#endif /* I2C_H_ */
