/*****************************************************************************************************************
*
* FileName:        Timer.c
* Dependencies:    Exploration.h
* Processor:       PIC24
* Compiler:        MPLAB C30 v3.30C or later
* Linker:          MPLAB LINK30
* Company:         Electric Power Research Institute (EPRI)
*
******************************************************************************************************************/

#include "Exploration.h"
/*****************************************************************************
* Function: InitTimer
*
* Preconditions: None.
*
* Overview: This function initiates Timer
*
* Input: None.
*
* Output: None.
*
******************************************************************************/

void InitTimer(void)
{
    IPC0bits.T1IP = TIMER_INTERRUPT_PRIORITY;   /* set priority level */
    IFS0bits.T1IF = 0;                          /* clear interrupt flag */
    IEC0bits.T1IE = 1;                          //Enable Timer1 interrupts
    TMR1 = 0;                                   /* clear timer1 register */
    PR1 = TMR1_PERIOD;                          /* set period1 register */
    T1CON = TIMER_ON | TIMER_SOURCE_INTERNAL | GATED_TIME_DISABLED | TIMER_16BIT_MODE | TIMER_PRESCALER_1;
}
