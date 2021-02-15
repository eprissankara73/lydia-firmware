/*****************************************************************************************************************
 * LCD Explorer Exploration program
 ******************************************************************************************************************
 * FileName:        Main.c
 * Dependencies:    Exploration.h
 * Processor:       PIC24FJ128GA310
 * Compiler:        XC16, v1.33
 * Programmer:      ICD3, SN: 
 * Linker:          MPLAB LINK30
 * Board:           DM240314, LCD Explorer XLP Development Board
 * Company:         Electric Power Research Institute (EPRI)
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author           Date             Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Evan Giarta      2018-05-03       Code for controlling the LED RD8 is success. It is actually on F6.
 ******************************************************************************************************************/

#include "Exploration.h"
_CONFIG1(FWDTEN_WDT_DIS & ICS_PGx3 & JTAGEN_OFF);
_CONFIG2(POSCMD_NONE &OSCIOFCN_ON & FCKSM_CSDCMD & FNOSC_FRC & IESO_OFF); // FRC Oscillator with PLL
_CONFIG3(SOSCSEL_ON & BOREN_ON);

#define LIMIT_10_BIT    1023    // 2^10 - 1

unsigned long DurationLED = 0;

int main(void) {
    InitLED();
    InitTimer();
    InitSwtiches();
    ADCInit();
    SWxWaitRel();
    
    printf("Hello World!\n");
    while (1) {
        if (SW2 == 0) SetLED(LED_OFF);
        else if (SW1 == 0) SetLED(LED_ON);
        else {
            if (DurationLED >= LIMIT_10_BIT) DurationLED = 0;
            if (DurationLED <= GetVoltageADC()) SetLED(LED_ON);
            else SetLED(LED_OFF);
        }
    }
    
}

void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
    ++DurationLED;
    IFS0bits.T1IF = 0;           /* reset Timer 1 interrupt flag */
}
