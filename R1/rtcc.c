
/**
  RTCC Generated Driver API Header File

  @Company:
    Microchip Technology Inc.

  @File Name:
    rtcc.c

  @Summary:
    This is the generated header file for the RTCC driver using MPLAB(c) Code Configurator

  @Description:
    This header file provides APIs for driver for RTCC.
    Generation Information :
        Product Revision  :  MPLAB(c) Code Configurator - v3.00
        Device            :  PIC24FJ256GB406
        Driver Version    :  0.5
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.26
        MPLAB 	          :  MPLAB X 3.20
*/

/*
Copyright (c) 2013 - 2015 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 */


/**
 Section: Included Files
*/

#include <xc.h>
#include "main.h"
#include "rtcc.h"
#include "memory.h"

/**
// Section: Static function
*/

static void RTCC_Unlock(void);
static void RTCC_Lock(void);
bool rtccTimeInitialized;
static bool RTCCTimeInitialized(void);
static uint8_t ConvertHexToBCD(uint8_t hexvalue);
static uint8_t ConvertBCDToHex(uint8_t bcdvalue);

/**
// Section: Driver Interface Function Definitions
*/

void RTCC_Initialize(void)
{
  // Turn on the secondary oscillator
  __builtin_write_OSCCONL(0x02);

   RTCCON1Lbits.RTCEN = 0;
   
   RTCC_Unlock();
   
   //if(!RTCCTimeInitialized())
   if ( DATEH < 0x1901 )
   {
       // set 2019-01-01 00-00-00
       DATEH = 0x1901;    // Year/Month
       DATEL = 0x0103;    // Date/Wday
       TIMEH = 0x0000;    // hours/minutes
       TIMEL = 0x0000;    // seconds
       
       //setting up the alarm time
       ALMDATEH = 0x1901;    // Year/Month
       ALMDATEL = 0x0101;    // Date/Wday
       ALMTIMEH = 0x0000;    // hours/minutes
       ALMTIMEL = 0x0000;    // seconds
        
       //rtccTimeInitialized = true; // temporary time setup, actual time to be setup over BLE connection
                                     // all write to buffers will happen only after rtcc time has been set   
   }
   // PWCPS disabled; PS 1:1; CLKSEL SOSC; FDIV 0; 
   RTCCON2L = 0x0000;
   // DIV for 32.768KHz crystal; 
   RTCCON2H = 0x3FFF;
   // PWCSTAB 0; PWCSAMP 0; 
   RTCCON3L = 0x0000;

   // RTCEN enabled; OUTSEL unused; PWCPOE disabled; TSBEN disabled; 
   // PWCEN disabled; WRLOCK disabled; PWCPOL disabled; TSAEN disabled; RTCOE disabled; 
   RTCCON1L = 0x8000; 
   
   RTCC_Lock();

}

static void RTCC_Unlock(void)
{
    asm volatile("DISI #6");
    asm volatile("MOV #NVMKEY, W1");
    asm volatile("MOV #0x55, W2");
    asm volatile("MOV W2, [W1]");
    asm volatile("MOV #0xAA, W3");
    asm volatile("MOV W3, [W1]");
    asm volatile("BCLR RTCCON1L, #11");
}

static void RTCC_Lock(void)
{
    asm volatile("DISI #6");
    asm volatile("MOV #NVMKEY, W1");
    asm volatile("MOV #0x55, W2");
    asm volatile("MOV W2, [W1]");
    asm volatile("MOV #0xAA, W3");
    asm volatile("MOV W3, [W1]");
    asm volatile("BSET RTCCON1L, #11");
}

bool RTCC_TimeGet(struct tm *currentTime)
{
    uint16_t register_value;
    if(RTCSTATLbits.SYNC){
        return false;
    }

    RTCC_Unlock();
   
    register_value = DATEH;
    currentTime->tm_year = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_mon = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = DATEL;
    currentTime->tm_mday = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_wday = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TIMEH;
    currentTime->tm_hour = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_min = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TIMEL;
    currentTime->tm_sec = ConvertBCDToHex((register_value & 0xFF00) >> 8);
   
    RTCC_Lock();

    return true;
}

void RTCC_TimeSet(struct tm *initialTime)
{
   RTCC_Unlock();

   RTCCON1Lbits.RTCEN = 0;
   
   IFS3bits.RTCIF = false;
   IEC3bits.RTCIE = 0;

   // set RTCC initial time
   DATEH = (ConvertHexToBCD(initialTime->tm_year) << 8) | ConvertHexToBCD(initialTime->tm_mon) ;  // YEAR/MONTH-1
   DATEL = (ConvertHexToBCD(initialTime->tm_mday) << 8) | ConvertHexToBCD(initialTime->tm_wday) ;  // /DAY-1/WEEKDAY
   TIMEH = (ConvertHexToBCD(initialTime->tm_hour) << 8)  | ConvertHexToBCD(initialTime->tm_min); // /HOURS/MINUTES
   TIMEL = (ConvertHexToBCD(initialTime->tm_sec) << 8) ;   // SECOND
           
   // Enable RTCC, clear RTCWREN         
   RTCCON1Lbits.RTCEN = 1;  
   RTCC_Lock();

   //Enable RTCC interrupt
   IEC3bits.RTCIE = 1;
}

bool RTCC_BCDTimeGet(bcdTime_t *currentTime)
{
    uint16_t register_value;
    if(RTCSTATLbits.SYNC){
        return false;
    }

    RTCC_Unlock();
   
    register_value = DATEH;
    currentTime->tm_year = (register_value & 0xFF00) >> 8;
    currentTime->tm_mon = register_value & 0x00FF;
    
    register_value = DATEL;
    currentTime->tm_mday = (register_value & 0xFF00) >> 8;
    currentTime->tm_wday = register_value & 0x00FF;
    
    register_value = TIMEH;
    currentTime->tm_hour = (register_value & 0xFF00) >> 8;
    currentTime->tm_min = register_value & 0x00FF;
   
    register_value = TIMEL;
    currentTime->tm_sec = (register_value & 0xFF00) >> 8;
   
    RTCC_Lock();

    return true;
}

void RTCC_BCDTimeSet(bcdTime_t *initialTime)
{
   RTCC_Unlock();

   RTCCON1Lbits.RTCEN = 0;
   
   IFS3bits.RTCIF = false;
   IEC3bits.RTCIE = 0;

   // set RTCC initial time
   DATEH = (initialTime->tm_year << 8) | (initialTime->tm_mon) ;  // YEAR/MONTH-1
   DATEL = (initialTime->tm_mday << 8) | (initialTime->tm_wday) ;  // /DAY-1/WEEKDAY
   TIMEH = (initialTime->tm_hour << 8) | (initialTime->tm_min); // /HOURS/MINUTES
   TIMEL = (initialTime->tm_sec << 8);   // SECONDS   
           
   // Enable RTCC, clear RTCWREN         
   RTCCON1Lbits.RTCEN = 1;  
   RTCC_Lock();

   //Enable RTCC interrupt
   IEC3bits.RTCIE = 1;
}

///**
// This function implements RTCC_TimeReset.This function is used to
// used by application to reset the RTCC value and reinitialize RTCC value.
//*/
//void RTCC_TimeReset(bool reset)
//{
//    rtccTimeInitialized = reset;
//}
//
//static bool RTCCTimeInitialized(void)
//{
//    return(rtccTimeInitialized);
//}

void RTCC_TimestampAEventManualSet(void)
{
    RTCSTATLbits.TSAEVT = 1;
}

bool RTCC_TimestampADataGet(struct tm *currentTime)
{
    uint16_t register_value;
    if(!RTCSTATLbits.TSAEVT){
        return false;
    }
  
    register_value = TSADATEH;
    currentTime->tm_year = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_mon = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSADATEL;
    currentTime->tm_mday = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_wday = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSATIMEH;
    currentTime->tm_hour = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_min = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSATIMEL;
    currentTime->tm_sec = ConvertBCDToHex((register_value & 0xFF00) >> 8);
   
    RTCSTATLbits.TSAEVT = 0;

    return true;
}



bool RTCC_TimestampA_BCDDataGet(bcdTime_t *currentTime)
{
    uint16_t register_value;
    if(!RTCSTATLbits.TSAEVT){
        return false;
    }
  
    register_value = TSADATEH;
    currentTime->tm_year = (register_value & 0xFF00) >> 8;
    currentTime->tm_mon = (register_value & 0x00FF);
    
    register_value = TSADATEL;
    currentTime->tm_mday = (register_value & 0xFF00) >> 8;
    currentTime->tm_wday = (register_value & 0x00FF);
    
    register_value = TSATIMEH;
    currentTime->tm_hour = (register_value & 0xFF00) >> 8;
    currentTime->tm_min = (register_value & 0x00FF);
    
    register_value = TSATIMEL;
    currentTime->tm_sec = (register_value & 0xFF00) >> 8;
   
    RTCSTATLbits.TSAEVT = 0;

    return true;
}

void RTCC_TimestampBEventManualSet(void)
{
    RTCSTATLbits.TSBEVT = 1;
}

bool RTCC_TimestampBDataGet(struct tm *currentTime)
{
    uint16_t register_value;
    if(!RTCSTATLbits.TSBEVT){
        return false;
    }
  
    register_value = TSBDATEH;
    currentTime->tm_year = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_mon = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSBDATEL;
    currentTime->tm_mday = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_wday = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSBTIMEH;
    currentTime->tm_hour = ConvertBCDToHex((register_value & 0xFF00) >> 8);
    currentTime->tm_min = ConvertBCDToHex(register_value & 0x00FF);
    
    register_value = TSBTIMEL;
    currentTime->tm_sec = ConvertBCDToHex((register_value & 0xFF00) >> 8);

    RTCSTATLbits.TSBEVT = 0;

    return true;
}

bool RTCC_TimestampB_BCDDataGet(bcdTime_t *currentTime)
{
    uint16_t register_value;
    if(!RTCSTATLbits.TSBEVT){
        return false;
    }
  
    register_value = TSBDATEH;
    currentTime->tm_year = (register_value & 0xFF00) >> 8;
    currentTime->tm_mon = (register_value & 0x00FF);
    
    register_value = TSBDATEL;
    currentTime->tm_mday = (register_value & 0xFF00) >> 8;
    currentTime->tm_wday = (register_value & 0x00FF);
    
    register_value = TSBTIMEH;
    currentTime->tm_hour = (register_value & 0xFF00) >> 8;
    currentTime->tm_min = (register_value & 0x00FF);
    
    register_value = TSBTIMEL;
    currentTime->tm_sec = (register_value & 0xFF00) >> 8;

    RTCSTATLbits.TSBEVT = 0;

    return true;
}

void RTCC_AlarmEnable(uint8_t interval)
{
    //0=0.5s, 1= 1s, 2 = 10s, 3 = 1min, 4=10min, 5=1hr, 6=1day, 7=1week
    //8=1month, 9=1year, 
    if (interval >=0 && interval <= 9)
    {
        RTCC_Unlock();
        RTCCON1Lbits.RTCEN = 0;
        IFS3bits.RTCIF = false;
        IEC3bits.RTCIE = 0;

        RTCCON1Hbits.AMASK = interval;
        RTCCON1Hbits.CHIME = 1; //makes the alarm repeat
        RTCCON1Hbits.ALMRPT = 0; //
        RTCCON1Hbits.ALRMEN = 1;

        RTCCON1Lbits.RTCEN = 1;  
        RTCC_Lock();        
        IEC3bits.RTCIE = 1;//Enable RTCC interrupt
    }    
}

void RTCC_AlarmDisable(void)
{
    RTCCON1Hbits.ALRMEN = 0;
}

static uint8_t ConvertHexToBCD(uint8_t hexvalue)
{
    uint8_t bcdvalue;
    bcdvalue = (hexvalue / 10) << 4;
    bcdvalue = bcdvalue | (hexvalue % 10);
    return (bcdvalue);
}

static uint8_t ConvertBCDToHex(uint8_t bcdvalue)
{
    uint8_t hexvalue;
    hexvalue = (((bcdvalue & 0xF0) >> 4)* 10) + (bcdvalue & 0x0F);
    return hexvalue;
}


/* Function:
  void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _RTCCInterrupt( void )

  Summary:
    Interrupt Service Routine for the RTCC Peripheral

  Description:
    This is the interrupt service routine for the RTCC peripheral. Add in code if 
    required in the ISR. 
*/

/**
 End of File
*/
