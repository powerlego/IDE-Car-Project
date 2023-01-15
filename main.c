/*
 * Rochester Institute of Technology
 * Department of Computer Engineering
 * CMPE 460  Interfacing Digital Electronics
 * LJBeato
 * 1/14/2021
 *
 * Filename: main_timer_template.c
 */
#include <ctype.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ADC14.h"
#include "Common.h"
#include "ControlPins.h"
#include "CortexM.h"
#include "Timer32.h"
#include "TimerA.h"
#include "msp.h"
#include "oled.h"
#include "switches.h"
#include "uart.h"

uint16_t    line[128];
BOOLEAN     g_sendData;
static char str[1024];

extern unsigned char OLED_clr_line[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];

#define DIVIDEND   0.075
#define MAX_COUNT  100
#define CHAR_COUNT 80
int     kpDivisor = 50;
int     kiDivisor = 27000;
int     kdDivisor = 100;
double  Kp        = 0.01; // 100
double  Ki        = 0.00000375;
double  Kd        = 0.001; // 100
double  maxSpeed  = 0.24;
double  straightSpeed;
double  diffGain = 1.125;
int     MINVAL   = 0;
BOOLEAN stopped  = FALSE;
BOOLEAN running  = FALSE;
BYTE    chr;
float   INTEGRATION_TIME                   = 0.02f;
int     displayMode                        = 0;
int     runMode                            = 0;
char    array_char_uart2[(CHAR_COUNT + 1)] = {0}; // array for the UART2 for phone side
char  **split_string;
int     offTrackCounter = 0;
double  servoPos, pid, speedOldL, speedL, speedR, speedOldR, servoPosOld;
double  err, errOld1, errOld2;
int     THRESHOLD = 5;
double  MaxError  = 15.0;
double  centerAverageVal;
int     speedCount    = 0;
int     maxSpeedCount = 3;
int     averager      = 27;
int     edge          = 3;
int     CENTER        = 64;
double  speed         = 0.24;
double  ramp          = 0.01;
BOOLEAN jumpstart     = TRUE;

void centerArray(double src[], int srcLen, int destLen) {
    double temp[destLen];
    int    tempCounter = 0;
    for(int i = ((srcLen / 2) - (destLen / 2)); i < ((srcLen / 2) + (destLen / 2)); i++) {
        temp[tempCounter] = src[i];
        tempCounter++;
    }
    memset(src, 0, sizeof(double) * srcLen);
    for(int i = 0; i < srcLen; i++) {
        src[i] = temp[i];
    }
    free(temp);
}
unsigned long minFunc(unsigned long a, unsigned long b) {
    return (a < b) ? a : b;
}

unsigned long binomialCoefficients(int n, int k) {
    unsigned long C[k + 1];
    memset(C, 0, sizeof(C));

    C[0] = 1; // nC0 is 1

    for(unsigned long i = 1; i <= n; i++) {
        // Compute next row of pascal triangle using
        // the previous row
        for(unsigned long j = minFunc(i, k); j > 0; j--) C[j] = C[j] + C[j - 1];
    }
    return C[k];
}

double clip(double pos, double lowerBound, double upperBound) {
    if(pos < lowerBound) {
        pos = lowerBound;
    }
    if(pos > upperBound) {
        pos = upperBound;
    }
    return pos;
}

void PORT1_IRQHandler(void) {
    if(P1->IFG & SW1) {
        DisableInterrupts();
        P1->IFG &= ~SW1;
        runMode++;
        if((runMode % 4) == 5) {
            stopped         = FALSE;
            offTrackCounter = 0;
            speedOldL = speedOldR = speedL = speedR = maxSpeed;
            err = errOld1 = errOld2 = pid = 0;
            servoPos = servoPosOld = 0.075;
            jumpstart              = TRUE;
        }
        EnableInterrupts();
    }
    if(P1->IFG & SW2) {
        DisableInterrupts();
        P1->IFG &= ~SW2;
        displayMode++;
        EnableInterrupts();
    }
}

void initializeMotors(void) {
    P3->SEL0 &= ~BIT6;
    P3->SEL1 &= ~BIT6;
    P3->DIR |= BIT6;
    P3->DS |= BIT6;
    P3->OUT |= BIT6;
    P3->SEL0 &= ~BIT7;
    P3->SEL1 &= ~BIT7;
    P3->DIR |= BIT7;
    P3->DS |= BIT7;
    P3->OUT |= BIT7;
    TIMER_A0_PWM_Init(SystemCoreClock / 10000, 0, 1);
    TIMER_A0_PWM_Init(SystemCoreClock / 10000, 0, 2);
    TIMER_A0_PWM_Init(SystemCoreClock / 10000, 0, 3);
    TIMER_A0_PWM_Init(SystemCoreClock / 10000, 0, 4);
    TIMER_A2_PWM_Init(SystemCoreClock / 50 / 16, 0, 1);
}

void msdelay(int delay) {
    int i, j;
    for(i = 0; i < delay; i++)
        for(j = 0; j < 16000; j++)
            ;
}

void Switch1_Interrupt_Init(void) {
    DisableInterrupts();
    Switch1_Init();
    P1->IFG &= ~SW1;
    P1->IE |= SW1;
    P1->IES |= SW1;
    NVIC_IPR8 = (NVIC_IPR8 & 0x00FFFFFF) | 0x40000000;
    NVIC_ISER1 |= 0x00000008;
    EnableInterrupts();
}

void Switch2_Interrupt_Init(void) {
    DisableInterrupts();
    Switch2_Init();
    P1->IES |= SW2;
    P1->IFG &= ~SW2;
    P1->IE |= SW2;
    NVIC_IPR8 = (NVIC_IPR8 & 0x00FFFFFF) | 0x40000000; // priority 2
    NVIC_ISER1 |= 0x00000008;
    EnableInterrupts();
}

BOOLEAN equalStrings(char *str1, char *str2) {
    if(strcmp(str1, str2) == 0) {
        return TRUE;
    }
    else {
        return FALSE;
    }
}

void INIT_Camera(void) {
    g_sendData = FALSE;
    ControlPin_SI_Init();
    ControlPin_CLK_Init();
    ADC0_InitSWTriggerCh6();
}

void stopMotors(void) {
    stopped = TRUE;
    TIMER_A0_PWM_DutyCycle(0, 1);
    TIMER_A0_PWM_DutyCycle(0, 2);
    TIMER_A0_PWM_DutyCycle(0, 3);
    TIMER_A0_PWM_DutyCycle(0, 4);
}

void updateServo() {
    TIMER_A2_PWM_DutyCycle(servoPos, 1);
}

int main(void) {
    int      counter = 0;
    // initializations
    uint16_t camera[128];
    int      char_counter = 0;
    char     char_com     = 0;
    char    *token;

    unsigned long coeff[averager];
    memset(coeff, '\0', sizeof(coeff));
    for(int i = 0; i < averager; i++) {
        coeff[i] = binomialCoefficients(averager - 1, i);
    }
    unsigned long lcm = 0;
    for(int i = 1; i < averager; i++) {
        lcm += coeff[i];
    }

    double h1[128];
    memset(h1, '\0', sizeof(h1));
    for(int i = 0; i < averager; i++) {
        h1[i] = 3 * ((double)coeff[i]) / lcm;
    }

    double h2[128] = {-1, 0, 1};
    memset(array_char_uart2, '\0', CHAR_COUNT);
    split_string    = malloc(16 * sizeof(char *));
    int      length = sizeof(camera) / sizeof(camera[0]);
    double   conv1[length + averager];
    uint16_t data1[length];
    uint16_t data2[length];
    double   halfway, halfway2, halfway3;
    halfway  = 0;
    halfway3 = 0;
    double   conv2[length + edge];
    uint16_t conv3[length + averager + edge];
    double   center = 64;
    double   lower, upper, max, min, lcor, ucor;
    err = errOld1 = errOld2 = 0;
    servoPosOld = servoPos = 0.075;
    speedOldL = speedOldR = speedL = speedR = maxSpeed;
    min = lcor = INT_MAX;
    max = ucor = INT_MIN;
    lower = upper = 0;
    DisableInterrupts();
    Timer32_2_Init(updateServo, SystemCoreClock / 50, T32DIV1);
    uart2_init();
    OLED_Init();
    INIT_Camera();
    initializeMotors();
    Switch1_Interrupt_Init();
    Switch2_Interrupt_Init();
    EnableInterrupts();
    TIMER32_CONTROL1 |= BIT7;
    TIMER32_CONTROL2 |= BIT7;
    uart2_put("Everything is intitialized\r\n");
    BOOLEAN inThresh = FALSE;

    while(1) {
        straightSpeed = 0.27;
        inThresh      = FALSE;
        memcpy(camera, line, sizeof(camera));
        if((displayMode % 6) >= 2) {
            sprintf(str,
                    "\r\npid: %f\r\ncenter: %.2f\r\nstopped: %d\r\nhalfway: %.2f\r\nservo pos: %f\r\nspeedL: "
                    "%f\r\nspeedR: %f\r\nerror: %f\r\n------------------------",
                    pid, center, stopped, halfway3, servoPos, speedL, speedR, err);
            uart2_put(str);
        }
        if((runMode % 4) == 1) {
            TIMER_A2_PWM_DutyCycle(0.075, 1);
            stopped         = FALSE;
            offTrackCounter = 0;
        }
        if((runMode % 4) == 2) {
            displayMode = 0;
            if(!stopped) {
                if(halfway3 <= MINVAL) {
                    //                    sprintf(str,
                    //                            "\r\npid: %f\r\ncenter: %.2f\r\nlower: %.2f\r\nupper: %.2f\r\nstopped:
                    //                            %d\r\nhalfway: %.2f\r\n------------------------", pid, center, lower,
                    //                            upper, stopped, halfway3);
                    //                    uart2_put(str);
                    offTrackCounter++;
                }
                else {
                    counter++;
                }
            }
            if(counter >= MAX_COUNT) {
                offTrackCounter = 0;
                counter         = 0;
            }
            if(offTrackCounter >= 3) {
                stopMotors();
            }
            if(jumpstart) {
                jumpstart = FALSE;
                TIMER_A2_PWM_DutyCycle(0.075, 1);
                TIMER_A0_PWM_DutyCycle(0.35, 1);
                TIMER_A0_PWM_DutyCycle(0, 2);
                TIMER_A0_PWM_DutyCycle(0, 3);
                TIMER_A0_PWM_DutyCycle(0.35, 4);
                msdelay(100);
            }
        }
        else {
            TIMER_A0_PWM_DutyCycle(0, 1);
            TIMER_A0_PWM_DutyCycle(0, 2);
            TIMER_A0_PWM_DutyCycle(0, 3);
            TIMER_A0_PWM_DutyCycle(0, 4);
        }

        if(g_sendData == TRUE) {
            min = lcor = DBL_MAX;
            max = ucor = -DBL_MAX;
            lower = upper = 0;
            for(int i = 0; i < length + averager; i++) {
                conv1[i] = 0;
                for(int j = 0; j < length; ++j) {
                    if(i - j >= 0) {
                        conv1[i] = conv1[i] + (camera[j] * h1[i - j]);
                    }
                }
                if(conv1[i] > ucor) {
                    ucor = conv1[i];
                }
                if(conv1[i] < lcor) {
                    lcor = conv1[i];
                }
            }

            centerArray(conv1, length + averager, length);
            if((displayMode % 6) >= 1) {
                for(int i = 0; i < length; i++) {
                    data1[i] = (uint16_t)(conv1[i]);
                }
            }
            halfway  = (ucor + lcor) / 2;
            halfway2 = (halfway + ucor) / 2;
            halfway2 = (halfway2 + ucor) / 2;
            for(int i = 0; i < length + averager; i++) {
                if(conv1[i] < halfway2) {
                    conv1[i] = 0;
                }
                else if(conv1[i] >= halfway2) {
                    conv1[i] = 16383;
                }
            }

            if((displayMode % 6) >= 1) {
                for(int i = 0; i < length; i++) {
                    data2[i] = (uint16_t)(conv1[i]);
                }
            }
            halfway3 = (halfway + lcor) / 2;
            for(int i = 0; i < length + edge; i++) {
                conv2[i] = 0;
                for(int j = 0; j < length; ++j) {
                    if(i <= edge) {
                        conv2[i] = 0;
                    }
                    if(i >= length - edge) {
                        conv2[i] = 0;
                    }
                    if(i - j >= 0) {
                        conv2[i] = conv2[i] + (conv1[j] * h2[i - j]);
                    }
                }
                if(conv2[i] > max) {
                    max   = conv2[i];
                    upper = i;
                }
                if(conv2[i] < min) {
                    min   = conv2[i];
                    lower = i;
                }
            }
            if((displayMode % 6) >= 1) {
                for(int i = 0; i < length + edge; i++) {
                    conv3[i] = (uint16_t)((15000 / max) * fabs(conv2[i]));
                }
            }
            // send the array over uart
            if((displayMode % 6) == 1) {
                OLED_Print(1, 1, "Tweaker");
                OLED_write_display(OLED_TEXT_ARR);
                uart2_put("\r\nTweaking values\r\nPlease enter continue to finish tweaking\r\n");
                while(1) {
                    if((displayMode % 6) != 1) {
                        break;
                    }
                    if(uart2_dataAvaliable() == TRUE) {
                        if((runMode % 4) >= 2) {
                            runMode = 1;
                            TIMER_A0_PWM_DutyCycle(0, 1);
                            TIMER_A0_PWM_DutyCycle(0, 2);
                            TIMER_A0_PWM_DutyCycle(0, 3);
                            TIMER_A0_PWM_DutyCycle(0, 4);
                        }
                        char_counter = 0;
                        while(char_counter < CHAR_COUNT) {
                            if((displayMode % 6) != 1) {
                                break;
                            }
                            char_com = uart2_getchar();
                            if((displayMode % 6) != 1) {
                                break;
                            }
                            array_char_uart2[char_counter] = char_com;
                            uart2_putchar(char_com);
                            char_counter++;
                            if(char_com == '\n' || char_com == '\r') {
                                array_char_uart2[char_counter - 1] = 0x0;
                                break;
                            }
                            else if(char_com == '\b' || char_com == 0x8 || char_com == 127) {
                                if(char_counter < 0) {
                                    break;
                                }
                                char_counter--;
                                array_char_uart2[char_counter] = '\0';
                                char_counter--;
                                array_char_uart2[char_counter] = '\0';
                                uart2_putchar('\0');
                            }
                            else {
                                if(char_counter > 78) {
                                    break;
                                }
                            }
                        }
                        if((displayMode % 6) != 1) {
                            break;
                        }
                        char_com = 0;
                        uart2_put("\r\n");
                        int index = 0;
                        token     = strtok(array_char_uart2, " ");
                        while(token != NULL) {
                            for(int i = 0; i < strlen(token); i++) {
                                token[i] = tolower((unsigned char)token[i]);
                            }
                            split_string[index] = token;
                            token               = strtok(NULL, " ");
                            index++;
                        }

                        if(equalStrings("kp", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Kp = %f/%d\r\n", DIVIDEND, kpDivisor);
                            }
                            else {
                                kpDivisor = atoi(split_string[1]);
                                Kp        = DIVIDEND / kpDivisor;
                                sprintf(str, "Set Kp to %f/%d\r\n", DIVIDEND, kpDivisor);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("ki", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Ki = %f/%d\r\n", DIVIDEND, kiDivisor);
                            }
                            else {
                                kiDivisor = atoi(split_string[1]);
                                Ki        = DIVIDEND / kiDivisor;
                                sprintf(str, "Set Ki to %f/%d\r\n", DIVIDEND, kiDivisor);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("kd", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Kd = %f/%d\r\n", DIVIDEND, kdDivisor);
                            }
                            else {
                                kdDivisor = atoi(split_string[1]);
                                Kd        = DIVIDEND / kdDivisor;
                                sprintf(str, "Set Kd to %f/%d\r\n", DIVIDEND, kdDivisor);
                            }
                            uart2_put(str);
                        }
                        else if((equalStrings("spd", split_string[0])) || (equalStrings("speed", split_string[0]))) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "speed = %f\r\n", maxSpeed);
                            }
                            else {
                                maxSpeed = atof(split_string[1]);
                                sprintf(str, "Set Max Speed to %f\r\n", maxSpeed);
                            }
                            uart2_put(str);
                        }
                        else if((equalStrings("go", split_string[0])) || (equalStrings("continue", split_string[0]))) {
                            OLED_display_clear();
                            displayMode = 0;
                            break;
                        }
                        else if(equalStrings("stop", split_string[0]) || equalStrings("minval", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Stop Value = %d\r\n", MINVAL);
                            }
                            else {
                                MINVAL = atoi(split_string[1]);
                                sprintf(str, "Set Stop Value to %d\r\n", MINVAL);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("integ", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Integration Time = %f\r\n", INTEGRATION_TIME);
                            }
                            else {
                                INTEGRATION_TIME = (float)atof(split_string[1]);
                                sprintf(str, "Set Integration Time to %f\r\n", INTEGRATION_TIME);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("?", split_string[0])) {
                            uart2_put("kp #: sets Kp divisor\r\nkp ?: querrys current Kp divisor\r\n");
                            uart2_put("ki #: sets Ki divisor\r\nkp ?: querrys current Ki divisor\r\n");
                            uart2_put("kd #: sets Kd divisor\r\nkd ?: querrys current Kd divisor\r\n");
                            uart2_put(
                                "(spd or speed) #: sets max speed\r\n(spd or speed) ?: querrys current max speed\r\n");
                            uart2_put("(stop or minval) #: sets stop value\r\n(stop or minval) ?: querrys current stop "
                                      "value\r\n");
                            uart2_put(
                                "integ #: sets integration time\r\ninteg ?: querrys current integration time\r\n");
                            uart2_put("(go or continue): exits the value tweaker\r\n");
                            uart2_put("?: displays this menu\r\n");
                        }
                        else if(equalStrings("diff", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Diff Gain = %f\r\n", diffGain);
                            }
                            else {
                                diffGain = atof(split_string[1]);
                                sprintf(str, "Set Diff Gain to %f\r\n", diffGain);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("thresh", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Threshold = %d\r\n", THRESHOLD);
                            }
                            else {
                                THRESHOLD = atoi(split_string[1]);
                                sprintf(str, "Set Threshold to %d\r\n", THRESHOLD);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("merr", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Max Error = %f\r\n", MaxError);
                            }
                            else {
                                MaxError = atof(split_string[1]);
                                sprintf(str, "Set Max Error to %f\r\n", MaxError);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("straight", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Straight Speed = %f\r\n", straightSpeed);
                            }
                            else {
                                straightSpeed = atof(split_string[1]);
                                sprintf(str, "Set Straight Speed to %f\r\n", straightSpeed);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("spdCount", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Straight Speed Count = %d\r\n", maxSpeedCount);
                            }
                            else {
                                maxSpeedCount = atoi(split_string[1]);
                                sprintf(str, "Set Straight Speed Count to %d\r\n", maxSpeedCount);
                            }
                            uart2_put(str);
                        }
                        else if(equalStrings("center", split_string[0])) {
                            if(equalStrings("?", split_string[1])) {
                                sprintf(str, "Center = %d\r\n", CENTER);
                            }
                            else {
                                CENTER = atoi(split_string[1]);
                                sprintf(str, "Set Center to %d\r\n", CENTER);
                            }
                            uart2_put(str);
                        }
                        memset(array_char_uart2, '\0', CHAR_COUNT);
                        char_counter = 0;
                    }
                }
            }
            if((displayMode % 6) == 2) {
                OLED_DisplayCameraData(data2);
            }
            else if((displayMode % 6) == 3) {
                OLED_DisplayCameraData(data1);
            }
            else if((displayMode % 6) == 4) {
                OLED_DisplayCameraData(conv3);
            }
            else if((displayMode % 6) == 5) {
                OLED_DisplayCameraData(line);
            }
            g_sendData = FALSE;
        }
        center = (upper + lower) / 2;
        err    = CENTER - center;

        pid = (((Kp) * (err - errOld1)) + (Ki * (err + errOld1) / 2) + (Kd * (err - 2 * errOld1 + errOld2)));
        if(fabs(err) <= THRESHOLD) {
            inThresh = TRUE;
            pid      = 0;
            err      = 0;
        }
        if((runMode % 4) >= 2) {
            servoPos = servoPosOld + pid;
            if(inThresh) {
                // servoPos = 0.075;
                speed  = clip(speed + ramp, maxSpeed, straightSpeed);
                speedL = speed;
                speedR = speed;
                speedCount++;
            }
            else {
                // servoPos = servoPosOld + pid;
                speedL = speedOldL - (diffGain * pid);
                speedR = speedOldR + (diffGain * pid);
                if((speed > maxSpeed + 0.05)) {
                    TIMER_A0_PWM_DutyCycle(0, 1);
                    TIMER_A0_PWM_DutyCycle(speed, 2);
                    TIMER_A0_PWM_DutyCycle(speed, 3);
                    TIMER_A0_PWM_DutyCycle(0, 4);
                    msdelay(25);
                    TIMER_A0_PWM_DutyCycle(maxSpeed, 1);
                    TIMER_A0_PWM_DutyCycle(0, 2);
                    TIMER_A0_PWM_DutyCycle(0, 3);
                    TIMER_A0_PWM_DutyCycle(maxSpeed, 4);
                }
                speed      = maxSpeed;
                speedCount = 0;
            }
            servoPos = clip(servoPos, 0.05, 0.1);

            if(((runMode % 4) == 2)) {
                if((fabs(err) >= MaxError)) {
                    if(err < 0) {
                        if(speedL < 0) {
                            TIMER_A0_PWM_DutyCycle(0, 1);
                            TIMER_A0_PWM_DutyCycle(0, 2);
                            TIMER_A0_PWM_DutyCycle(0, 4);
                            TIMER_A0_PWM_DutyCycle(fabs(speedL), 3);
                        }
                        else {
                            TIMER_A0_PWM_DutyCycle(0, 1);
                            TIMER_A0_PWM_DutyCycle(0, 2);
                            TIMER_A0_PWM_DutyCycle(0, 3);
                            TIMER_A0_PWM_DutyCycle(speedL, 4);
                        }
                    }
                    else {
                        if(speedR < 0) {
                            TIMER_A0_PWM_DutyCycle(0, 1);
                            TIMER_A0_PWM_DutyCycle(fabs(speedR), 2);
                            TIMER_A0_PWM_DutyCycle(0, 3);
                            TIMER_A0_PWM_DutyCycle(0, 4);
                        }
                        else {
                            TIMER_A0_PWM_DutyCycle(speedR, 1);
                            TIMER_A0_PWM_DutyCycle(0, 2);
                            TIMER_A0_PWM_DutyCycle(0, 3);
                            TIMER_A0_PWM_DutyCycle(0, 4);
                        }
                    }
                }
                else {
                    TIMER_A0_PWM_DutyCycle(speed, 1);
                    TIMER_A0_PWM_DutyCycle(0, 2);
                    TIMER_A0_PWM_DutyCycle(0, 3);
                    TIMER_A0_PWM_DutyCycle(speed, 4);
                }
            }

            servoPosOld = servoPos;
            speedOldL   = speedL;
            speedOldR   = speedR;
            errOld2     = errOld1;
            errOld1     = err;
        }
    }
}
