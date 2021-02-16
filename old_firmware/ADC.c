/*****************************************************************************************************************
* FileName:        ADC.c
* Dependencies:    Exploration.h
* Processor:       PIC24
* Compiler:        MPLAB C30 v3.30C or later
* Linker:          MPLAB LINK30
* Company:         Microchip Technology Incorporated
*
* Copyright 2011 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right use, modify, copy, and distribute the accompanying 
* Microchip demo code only when used with or embedded on a Microchip microcontroller or Microchip 
* digital signal controller that is integrated into your product or a third party product.  
* Any redistributions of Microchip’s demo code in compliance with the foregoing must include a copy 
* of this entire notice.  *\
* 
* 
* THE MICROCHIP DEMO CODE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER 
* EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT, 
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED 
* UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE 
* THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, 
* INDIRECT, OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
* TECHNOLOGY, SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), 
* OR OTHER SIMILAR COSTS.
* 
******************************************************************************************************************/





#include "Exploration.h"


// Reference voltage, mV
#define AVCC                3300
//
/*****************************************************************************
* Function: ADCInit
*
* Preconditions: None.
*
* Overview: Initialize ADC
*
* Input: None.
*
* Output: None.
*
******************************************************************************/

void ADCInit(){
	ADC_POT_TRIS=1;
	ADC_TEMP_TRIS=1;
	ADC_POT_CHANNEL=1; 			//Analog POT           //AN11
	ADC_TEMP_CHANNEL=1;			// temp sensor //AN20


 	AD1CSSL = 0x0800; // AN11
    AD1CSSH = 0xD010; // Include 4 channels in scan (VBG, VSS, VDD and VBAT), AN20
    AD1CON1 = 0x0070; // Internal counter triggers conversion
    AD1CON3 = 0x1F1F; // Sample time = 15Tad, Tad = Tcy

    AD1CON2 = 0x040C; // Set AD1IF after every 4 samples, enable scanning
    AD1CON5 = 0x1000; // Enable Band Gap
    AD1CON1bits.ADON = 1; // turn ADC ON
//BUF0=Voltmeter reading
//BUF1=Temperature sensor
//BUF2=Bandgap
//BUF3=VDD

        IFS0bits.AD1IF = 0; 
        AD1CON1bits.ASAM = 1; 		// auto start sampling 
        while (!IFS0bits.AD1IF){}; 	// conversion done
        AD1CON1bits.ASAM = 0; 		// yes then stop sample/convert
}

/*****************************************************************************
* Function: GetVoltageADC
*
* Preconditions: None.
*
* Overview: Returns the voltage off the trim potentiometer scaled from 0 to 2^10
*
* Input: None.
* 
* Output: ADC Voltage scale from 0 to 2^10 as long variable
*
******************************************************************************/


unsigned long GetVoltageADC() {
    IFS0bits.AD1IF = 0; 
    AD1CON1bits.ASAM = 1; 		// auto start sampling 
    while (!IFS0bits.AD1IF){}; 	// conversion done
    AD1CON1bits.ASAM = 0; 		// yes then stop sample/convert
    
    return (long) ADC1BUF0;
}

