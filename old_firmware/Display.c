/*****************************************************************************************************************
* FileName:        Display.c
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
* Any redistributions of Microchip�s demo code in compliance with the foregoing must include a copy 
* of this entire notice.  *\
* 
* 
* THE MICROCHIP DEMO CODE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER 
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
unsigned char DIG2,DIG1,DIG0;
unsigned char TCDIG2,TCDIG1,TCDIG0;
unsigned char TFDIG2,TFDIG1,TFDIG0;
unsigned char SEC0,SEC1,MIN0,MIN1,WK,HRS1,HRS0;
unsigned short MINSEC0,WKHRS;
/*****************************************************************************
* Function: ContrastDisplay
*
* Preconditions: None.
*
* Overview: Displays biasing and contrast
*
* Input: None.
*
* Output: None.
*
******************************************************************************/

void ContrastDisplay(void)
{

	X1C18=0;X2C18=0;X3C18=0;X4C18=0;X5C18=0;X6C18=0;X7C18=0;
	X1C17=0;X2C17=0;X3C17=0;X4C17=0;X5C17=0;X6C17=1;X7C17=0;
	X1C16=0;X2C16=0;X3C16=1;X4C16=0;X5C16=1;X6C16=0;X7C16=1;
	X1C15=0;X2C15=0;X3C15=1;X4C15=0;X5C15=1;X6C15=0;X7C15=1;
	X1C14=0;X2C14=0;X3C14=0;X4C14=1;X5C14=0;X6C14=0;X7C14=1;

	X1C13=0;X2C13=0;X3C13=0;X4C13=0;X5C13=0;X6C13=0;X7C13=0;
	X1C12=0;X2C12=0;X3C12=0;X4C12=1;X5C12=1;X6C12=1;X7C12=1;
	X1C11=0;X2C11=0;X3C11=1;X4C11=0;X5C11=1;X6C11=0;X7C11=1;
	X1C10=0;X2C10=0;X3C10=1;X4C10=0;X5C10=1;X6C10=0;X7C10=1;
	X1C9=0;X2C9=0;X3C9=0;X4C9=0;X5C9=0;X6C9=1;X7C9=0;

	X1C8=0;X2C8=0;X3C8=0;X4C8=0;X5C8=0;X6C8=0;X7C8=0;
	X1C7=0;X2C7=0;X3C7=1;X4C7=0;X5C7=1;X6C7=1;X7C7=1;
	X1C6=0;X2C6=0;X3C6=0;X4C6=0;X5C6=0;X6C6=0;X7C6=0;

	X1C5=0;X2C5=0;X3C5=1;X4C5=0;X5C5=1;X6C5=1;X7C5=0;
	X1C4=0;X2C4=1;X3C4=0;X4C4=1;X5C4=0;X6C4=0;X7C4=1;
	X1C3=0;X2C3=1;X3C3=0;X4C3=1;X5C3=0;X6C3=0;X7C3=1;
	X1C2=0;X2C2=1;X3C2=0;X4C2=1;X5C2=0;X6C2=0;X7C2=1;
	X1C1=0;X2C1=1;X3C1=1;X4C1=1;X5C1=1;X6C1=1;X7C1=1;

///if ChargePUMP
		if(VDD_RES2<2700)
		{
				switch(SEC0)
				{
				case 0:LCDREGbits.BIAS=0;	Digit2A(0);Digit1(0);Digit0(0);break;
				case 1:LCDREGbits.BIAS=1;	Digit2A(0);Digit1(0);Digit0(1);break;
				case 2:LCDREGbits.BIAS=2;	Digit2A(0);Digit1(1);Digit0(0);break;
				case 3:LCDREGbits.BIAS=3;	Digit2A(0);Digit1(1);Digit0(1);break;
				case 4:LCDREGbits.BIAS=4;	Digit2A(1);Digit1(0);Digit0(0);break;
				case 5:LCDREGbits.BIAS=5;	Digit2A(1);Digit1(0);Digit0(1);break;
				case 6:LCDREGbits.BIAS=6;	Digit2A(1);Digit1(1);Digit0(0);break;
				case 7:LCDREGbits.BIAS=7;	Digit2A(1);Digit1(1);Digit0(1);break;
				case 8:LCDREGbits.BIAS=0;	Digit2A(0);Digit1(0);Digit0(0);break;
				case 9:LCDREGbits.BIAS=0;	Digit2A(0);Digit1(0);Digit0(0);break;
					
				}
				switch(SEC0)
				{
				case 9:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=0;break;
				case 8:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=0;WHEEL8=0;break;
				case 7:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 6:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 5:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 4:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 3:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 2:WHEEL9=1;WHEEL10=1;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 1:WHEEL9=1;WHEEL10=0;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 0:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=1;break;
				}
		}
		else //internal resistor ladder
		{
				switch(SEC0)
				{
				case 0:LCDREFbits.LCDCST=0;	Digit2A(0);Digit1(0);Digit0(0);break;
				case 1:LCDREFbits.LCDCST=1;	Digit2A(0);Digit1(0);Digit0(1);break;
				case 2:LCDREFbits.LCDCST=2;	Digit2A(0);Digit1(1);Digit0(0);break;
				case 3:LCDREFbits.LCDCST=3;	Digit2A(0);Digit1(1);Digit0(1);break;
				case 4:LCDREFbits.LCDCST=4;	Digit2A(1);Digit1(0);Digit0(0);break;
				case 5:LCDREFbits.LCDCST=5;	Digit2A(1);Digit1(0);Digit0(1);break;
				case 6:LCDREFbits.LCDCST=6;	Digit2A(1);Digit1(1);Digit0(0);break;
				case 7:LCDREFbits.LCDCST=7;	Digit2A(1);Digit1(1);Digit0(1);break;
				case 8:LCDREFbits.LCDCST=0;	Digit2A(0);Digit1(0);Digit0(0);break;
				case 9:LCDREFbits.LCDCST=0;	Digit2A(0);Digit1(0);Digit0(0);break;
					
				}
				switch(SEC0)
				{
				case 9:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=0;break;
				case 8:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=0;WHEEL8=0;break;
				case 7:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 6:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 5:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 4:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 3:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 2:WHEEL9=1;WHEEL10=1;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 1:WHEEL9=1;WHEEL10=0;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
				case 0:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=1;break;
				}
	   }
		MICROCHIP=1;
}
/*****************************************************************************
* Function: DisplayRTCC 
*
* Preconditions: None.
*
* Overview: Displays RTCC values 
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
void DisplayRTCC(void)
{
	if(SEC0>=8)
	{
	UpdateHrsWeek();
	}
	else
	{
		
	Digit3(MIN1);
	Digit2(MIN0);
	Col1(2);
	Digit1(SEC1);
	Digit0(SEC0);

  		X1C11=0;X2C11=0;X3C11=0;X4C11=0;X5C11=0;X6C11=0;X7C11=0;
   		X1C10=0;X2C10=0;X3C10=0;X4C10=0;X5C10=0;X6C10=0;X7C10=0;
		X1C9=0;X2C9=0;X3C9=0;X4C9=0;X5C9=0;X6C9=0;X7C9=0;
		X1C8=0;X2C8=0;X3C8=0;X4C8=0;X5C8=0;X6C8=0;X7C8=0;
		X1C7=0;X2C7=0;X3C7=0;X4C7=0;X5C7=0;X6C7=0;X7C7=0;
		X1C6=0;X2C6=0;X3C6=0;X4C6=0;X5C6=0;X6C6=0;X7C6=0;
		X1C5=0;X2C5=0;X3C5=0;X4C5=0;X5C5=0;X6C5=0;X7C5=0;
		X1C4=0;X2C4=0;X3C4=0;X4C4=0;X5C4=0;X6C4=0;X7C4=0;
		X1C3=0;X2C3=0;X3C3=0;X4C3=0;X5C3=0;X6C3=0;X7C3=0;
		X1C2=0;X2C2=0;X3C2=0;X4C2=0;X5C2=0;X6C2=0;X7C2=0;
		X1C1=0;X2C1=0;X3C1=0;X4C1=0;X5C1=0;X6C1=0;X7C1=0;
	



	}
		switch(SEC0)
		{
		case 9:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=0;break;
		case 8:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=0;WHEEL8=0;break;
		case 7:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 6:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 5:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 4:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 3:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 2:WHEEL9=1;WHEEL10=1;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 1:WHEEL9=1;WHEEL10=0;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;WHEEL8=0;break;
		case 0:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;WHEEL8=1;break;
		}
	
	
		MICROCHIP=1;


}

/*****************************************************************************
* Function: UpdateHrsWeek
*
* Preconditions: None.
*
* Overview: Displays RTCC values 
*
* Input: None.
*
* Output: None.
*
******************************************************************************/

void UpdateHrsWeek(void)
{
	Digit3(HRS1);
	Digit2(HRS0);
	Col1(2);
	Digit1(MIN1);
	Digit0(MIN0);
if(Set==10)// to indicate entry during RTCC time configuration
{
  		X1C11=0;X2C11=0;X3C11=0;X4C11=0;X5C11=0;X6C11=0;X7C11=0;
	X1C10=0;X2C10=0;X3C10=0;X4C10=0;X5C10=0;X6C10=0;X7C10=0;
	X1C9=0;X2C9=0;X3C9=0;X4C9=0;X5C9=0;X6C9=0;X7C9=0;
	X1C8=0;X2C8=0;X3C8=0;X4C8=0;X5C8=0;X6C8=0;X7C8=0;
	X1C7=0;X2C7=0;X3C7=0;X4C7=0;X5C7=0;X6C7=0;X7C7=0;
	X1C6=0;X2C6=0;X3C6=0;X4C6=0;X5C6=0;X6C6=0;X7C6=0;	
	X1C5=0;X2C5=0;X3C5=0;X4C5=0;X5C5=0;X6C5=0;X7C5=0;
	X1C4=0;X2C4=0;X3C4=0;X4C4=0;X5C4=0;X6C4=0;X7C4=0;	
	X1C3=0;X2C3=0;X3C3=0;X4C3=0;X5C3=0;X6C3=0;X7C3=0;	
	X1C2=0;X2C2=0;X3C2=0;X4C2=0;X5C2=0;X6C2=0;X7C2=0;
	X1C1=0;X2C1=0;X3C1=0;X4C1=0;X5C1=0;X6C1=0;X7C1=0;

}

		switch(WK)
		{
		case 0://Mo
   		X5C10=1;X6C10=1;X4C9=1;X7C9=1;X4C8=1;X7C8=1;X5C7=1;X6C7=1;X4C5=1;X5C5=1;X6C5=1;
		X7C5=1;X3C4=1;X4C3=1;X5C3=1;;X3C2=1;;X4C1=1;X5C1=1;X6C1=1;X7C1=1;break;
		case 1://Tu
  		X3C10=1;X4C10=1;X5C10=1;X6C10=1;;X7C9=1;;X7C8=1;X3C7=1;X4C7=1;X5C7=1;X6C7=1;X6C5=1;X7C4=1;X3C3=1;X7C3=1;
		X1C2=1;X2C2=1;X3C2=1;X4C2=1;X5C2=1;X6C2=1;X3C1=1;break;
		case 2://Wd
		X2C10=1;X3C10=1;X4C10=1;X5C10=1;X6C10=1;X7C10=1;X4C9=1;X7C9=1;X4C8=1;X7C8=1;X5C7=1;X6C7=1;
		X3C5=1;X4C5=1;X5C5=1;X6C5=1;X7C4=1;X4C3=1;X5C3=1;X6C3=1;X7C2=1;X3C1=1;X4C1=1;X5C1=1;X6C1=1;break;
		case 3://th
  		X5C10=1;X6C10=1;X7C10=1;X4C9=1;X4C8=1;;X2C7=1;X3C7=1;X4C7=1;X5C7=1;X6C7=1;X7C7=1;
		X6C5=1;X7C4=1;X3C3=1;X7C3=1;X1C2=1;X2C2=1;X3C2=1;X4C2=1;X5C2=1;X6C2=1;X3C1=1;break;
		case 4://fr
		X5C10=1;X4C9=1;X4C8=1;X3C7=1;X4C7=1;X5C7=1;X6C7=1;X7C7=1;X1C5=0;X2C5=1;
		X1C4=1;X1C3=1;X4C3=1;X2C2=1;X3C2=1;X4C2=1;X5C2=1;X6C2=1;X7C2=1;X4C1=1;break;
		case 5://st
 		X4C10=1;X5C10=1;X6C10=1;X7C10=1;X3C9=1;X5C9=1;X7C9=1;X3C8=1;X5C8=1;X6C8=0;X7C8=1;X3C7=1;X6C7=1;
		X6C5=1;X3C4=1;X5C4=1;X7C4=1;X3C3=1;X5C3=1;X7C3=1;X3C2=1;X5C2=1;X7C2=1;X4C1=1;X7C1=1;break;
		case 6://Su
 		X3C10=1;X4C10=1;X5C10=1;X6C10=1;X7C9=1;X7C8=1;X3C7=1;X4C7=1;X5C7=1;X6C7=1;
		X6C5=1;X3C4=1;X5C4=1;X7C4=1;X3C3=1;X5C3=1;X7C3=1;X3C2=1;X5C2=1;X7C2=1;X4C1=1;X7C1=1;
		}

}



/*****************************************************************************
* Function: DisplayVoltage ( Analog Pot)
*
* Preconditions: None.
*
* Overview: Displays Analog Pot Voltage
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
void DisplayVoltage(void)
{

	X1C17=0;X2C17=0;X3C17=1;X4C17=0;X5C17=1;X6C17=0;X7C17=0;HEART=0;
	X1C16=0;X2C16=0;X3C16=1;X4C16=0;X5C16=1;X6C16=0;X7C16=0;CLOUD=0;
	X1C15=0;X2C15=0;X3C15=1;X4C15=0;X5C15=1;X6C15=0;X7C15=0;ZIG=0;

	X1C14=0;X2C14=0;X3C14=0;X4C14=0;X5C14=0;X6C14=0;X7C14=0;WIFI=0;
	X1C13=1;X2C13=1;X3C13=1;X4C13=1;X5C13=1;X6C13=0;X7C13=0;PAN=0;
	X1C12=0;X2C12=0;X3C12=0;X4C12=0;X5C12=0;X6C12=1;X7C12=0;SUN=0;
	X1C11=0;X2C11=0;X3C11=0;X4C11=0;X5C11=0;X6C11=0;X7C11=1;
	X1C10=0;X2C10=0;X3C10=0;X4C10=0;X5C10=0;X6C10=1;X7C10=0;
	X1C9=1; X2C9=1;  X3C9=1; X4C9=1; X5C9=1; X6C9=0; X7C9=0;
	X1C8=0; X2C8=0;  X3C8=0; X4C8=0; X5C8=0; X6C8=0; X7C8=0;

	Digit2(DIG2);
	Col1(1);
	Digit1(DIG1);
	Digit0(DIG0);

	switch(Result1)
	{
	case 9:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=1;break;
	case 8:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=1;WHEEL7=0;break;
	case 7:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=1;WHEEL6=0;WHEEL7=0;break;
	case 6:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=1;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 5:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=1;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 4:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=1;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 3:WHEEL9=1;WHEEL10=1;WHEEL1=1;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 2:WHEEL9=1;WHEEL10=1;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 1:WHEEL9=1;WHEEL10=0;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;
	case 0:WHEEL9=0;WHEEL10=0;WHEEL1=0;WHEEL2=0;WHEEL3=0;WHEEL4=0;WHEEL5=0;WHEEL6=0;WHEEL7=0;break;


	}


	MICROCHIP=1;

}
/*****************************************************************************
* Function: DisplayTempC
*
* Preconditions: None.
*
* Overview: Displays temperature in Celcius
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
void DisplayTempC(void)
{
	
	X1C37=0;X2C37=0;X3C37=0;X4C37=0;X5C37=0;X6C37=0;X7C37=0;
	X1C36=0;X2C36=0;X3C36=0;X4C36=0;X5C36=0;X6C36=0;X7C36=0;
	X1C35=0;X2C35=1;X3C35=0;X4C35=0;X5C35=0;X6C35=0;X7C35=1;
	X1C34=0;X2C34=1;X3C34=0;X4C34=0;X5C34=0;X6C34=0;X7C34=1;
	X1C33=0;X2C33=1;X3C33=0;X4C33=0;X5C33=0;X6C33=0;X7C33=1;
	X1C32=0;X2C32=1;X3C32=0;X4C32=0;X5C32=0;X6C32=0;X7C32=1;
	X1C31=0;X2C31=0;X3C31=1;X4C31=1;X5C31=1;X6C31=1;X7C31=0;X2C37=0;

	X1C30=0;X2C30=0;X3C30=0;X4C30=0;X5C30=0;X6C30=0;X7C30=0;X1C37=0;
	X1C29=0;X2C29=1;X3C29=1;X4C29=0;X5C29=0;X6C29=0;X7C29=0;RMB=0;
	X1C28=1;X2C28=0;X3C28=0;X4C28=1;X5C28=0;X6C28=0;X7C28=0;CENT=0;
	X1C27=1;X2C27=0;X3C27=0;X4C27=1;X5C27=0;X6C27=0;X7C27=0;MICROCHIP=1;
	X1C26=0;X2C26=1;X3C26=1;X4C26=0;X5C26=0;X6C26=0;X7C26=0;FAR=0;
	X1C25=0;X2C25=0;X3C25=0;X4C25=0;X5C25=0;X6C25=0;X7C25=0;CEL=0;


	if(TFDIG2>0)
	{
		Digit4(TCDIG2);
	}	
	else
	{
		Digit4(10);
	}
	Digit3(TCDIG1);
	Digit2(TCDIG0);
	SUN=1;
	CLOUD=1;

	MICROCHIP=1;
}
/*****************************************************************************
* Function: DisplayTempF
*
* Preconditions: None.
*
* Overview: Displays temperature in Far max up to 90
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
void DisplayTempF(void)
{
	
	X1C37=0;X2C37=0;X3C37=0;X4C37=0;X5C37=0;X6C37=0;X7C37=0;
	X1C36=0;X2C36=0;X3C36=0;X4C36=0;X5C36=0;X6C36=0;X7C36=0;
	X1C35=0;X2C35=1;X3C35=0;X4C35=1;X5C35=0;X6C35=0;X7C35=0;
	X1C34=0;X2C34=1;X3C34=0;X4C34=1;X5C34=0;X6C34=0;X7C34=0;
	X1C33=0;X2C33=1;X3C33=0;X4C33=1;X5C33=0;X6C33=0;X7C33=0;
	X1C32=0;X2C32=1;X3C32=0;X4C32=1;X5C32=0;X6C32=0;X7C32=0;
	X1C31=0;X2C31=1;X3C31=1;X4C31=1;X5C31=1;X6C31=1;X7C31=1;X2C37=0;

	X1C30=0;X2C30=0;X3C30=0;X4C30=0;X5C30=0;X6C30=0;X7C30=0;X1C37=0;
	X1C29=0;X2C29=1;X3C29=1;X4C29=0;X5C29=0;X6C29=0;X7C29=0;RMB=0;
	X1C28=1;X2C28=0;X3C28=0;X4C28=1;X5C28=0;X6C28=0;X7C28=0;CENT=0;
	X1C27=1;X2C27=0;X3C27=0;X4C27=1;X5C27=0;X6C27=0;X7C27=0;MICROCHIP=1;
	X1C26=0;X2C26=1;X3C26=1;X4C26=0;X5C26=0;X6C26=0;X7C26=0;FAR=0;
	X1C25=0;X2C25=0;X3C25=0;X4C25=0;X5C25=0;X6C25=0;X7C25=0;CEL=0;

	if(TFDIG2>0)
	{
		Digit4(TFDIG2);
	}	
	else
	{
		Digit4(10);
	}
	Digit3(TFDIG1);
	Digit2(TFDIG0);
	SUN=1;
	CLOUD=1;

	MICROCHIP=1;
}
