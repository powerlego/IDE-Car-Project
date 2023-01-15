#ifndef TIMER_A_H_
#define TIMER_A_H_

#define A0_PIN1 BIT4
#define A0_PIN2 BIT5
#define A0_PIN3 BIT6
#define A0_PIN4 BIT7
#define A2_PIN1 BIT6

int TIMER_A0_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);

void TIMER_A0_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);

int TIMER_A2_PWM_Init(uint16_t period, double percentDutyCycle, uint16_t pin);

void TIMER_A2_PWM_DutyCycle(double percentDutyCycle, uint16_t pin);

#endif
