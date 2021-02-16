/*****************************************************************************************************************
*
* FileName:        LED.c
* Dependencies:    Exploration.h
* Processor:       PIC24
* Compiler:        MPLAB C30 v3.30C or later
* Linker:          MPLAB LINK30
* Company:         Electric Power Research Institute (EPRI)
*
******************************************************************************************************************/

#include "Exploration.h"
/*****************************************************************************
* Function: InitLED
*
* Preconditions: None.
*
* Overview: This function initiates LED
*
* Input: None.
*
* Output: None.
*
******************************************************************************/

unsigned char CountLED;

void InitLED(void)
{
    CountLED = 0;
    LED_OUT_LAT = LED_OFF;
	LED_OUT_TRIS = LED_SET_OUTPUT;
}

void SetLED(unsigned char set)
{
	LED_OUT_LAT = set;
}

void ToggleLED(void)
{
	if (LED_OUT == LED_OFF)   LED_OUT_LAT = LED_ON;
    else                LED_OUT_LAT = LED_OFF;
}

void LimitLED(unsigned char limit)
{
    CountLED = (++CountLED) % limit;
	if (CountLED == LED_OFF)   LED_OUT_LAT = LED_ON;
    else                LED_OUT_LAT = LED_OFF;
}
