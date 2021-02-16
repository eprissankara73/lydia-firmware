/* 
 * File:   memory.h
 * Author: Viswanath
 *
 * Created on January 22, 2019, 5:34 AM
 */

#include <xc.h> // include processor files - each processor file is guarded.  
#include "main.h"

#define MemoryWidth 256  // of the FRAM banks 

 //******** MEMORY MAP BANK 0****************
 /* BANK 0 FRAM LOCATIONS 256K = 262143 bytes
  * 
  * RTCInitialized:             1 BYTE:  0x00000000
  * RTCC SETUP date             5 BYTES: 0x00000001- 0x00000005
  * none schedule heat setpoint 1 BYTE   0x00000006
  * Setpoint cool:              1 BYTE   0x00000007
  * Setpoint heat:              1 BYTE   0x00000008
  * App Initialized 1:          1 BYTE   0x00000009
  * screen refresh count        2 BYTE   0x0000000A, 0x0000000B
  * App initialized  2:         1 BYTE   0x0000000C
  * buffers initialized         1 BYTE   0x0000000D
  * none schedule cool setpoint 1 BYTE   0x0000000E
  * previous schedule           1 BYTE   0x0000000F  
  * away mode                   1 BYTE   0s00000010 // HEAT (1),  COOL(2) OR Heat+Cool(3); value of 0x4C => Home schedule
  * away mode cool setpoint     1 BYTE   0x00000011 // cool setpoint if away mode is cool or Heat+Cool
  * away mode heat setpoint     1 BYTE   0x00000012 // cool setpoint if away mode is heat or Heat_cool
  * hysteresis                  1 BYTE   0x00000013 // range of 1 to 4
  * thermostat mode             1 BYTE   0x00000014 // heat+cool, cool only , heat only
  * previous thermostat mode    1 BYTE   0x00000015
  * schedule                    1 BYTE   0x00000016
  * Reserve for further updates : 2 bytes
  * TOTAL BYTES this section  = 25 BYTES
  *  
  *    *********************
  * 
  * SCHEDULE : 5 schedules per day; for 7 days => assume schedule received is ordered, starts on Sunday
  *         ends on Saturday, and within each day, it is ordered by time SCHEDULE AT 7:30AM comes before
  *         schedule at 11:00 AM etc....
  *   Need to store 35 schedules
  *     Bitmap of stored schedules are: 1 at position => it is set, 0 => not set
  *                                     Bit positions
  *         first byte (of 5 bytes):    7  6  5  4  3  2  1  0  maps to (don't care), (don't care), (don't care),  (don't care), (DR sch), (Sun, 5th), (Sun, 4th), (Sun, 3rd)   
  *         second byte (of 5 bytes):   7  6  5  4  3  2  1  0  maps to (Sun, 2nd), (Sun, 1st), (Mon, 5th), (Mon, 4th), (Mon, 3rd), (Mon, 2nd), (Mon, 1st), (Tue, 5th)
  *         third byte (of 5 bytes):    7  6  5  4  3  2  1  0  maps to (Tue, 4th), (Tue, 3rd), (Tue, 2nd), (Tue, 1st), (Wed, 5th), (Wed, 4th), (Wed, 3rd), (Wed, 2nd)
  *         fourth byte (of 5 bytes):   7  6  5  4  3  2  1  0  maps to (Wed, 1st), (Thu, 5th), (Thu, 4th), (Thu, 3rd), (Thu, 2nd), (Thu, 1st), (Fri, 5th), (Fri, 4th)
  *         fifth byte (of 5 bytes):    7  6  5  4  3  2  1  0  maps to (Fri, 3rd), (Fri, 2nd), (Fri, 1st), (Sat, 5th), (Sat, 4th), (Sat, 3rd), (Sat, 2nd), (Sat, 1st)
  *                 NOTE: (Sat, 2nd) => second schedule for Saturday (assuming all are ordered), (Sat, 1st) => first schedule of Saturday (assuming ordered schedules)
  *     Each schedule consists of 
  *        <Start time>             2 BYTES // first byte HH (0 - 24), second byte MM (0-60)
  *        <Cool set point>         1 BYTE
  *        <Heat set point>         1 BYTE  
  *    <first 5 bytes = bitmap of stored schedules> // 
  *    <first 20 bytes = SUNDAY SCHEDULE>
  *    <next  20 bytes = MONDAY SCHEDULE>
  *    <next  20 bytes = TUESDAY SCHEDULE>
  *    <next  20 bytes = WEDNESDAY SCHEDULE>
  *    <next  20 bytes = THURSDAY SCHEDULE>
  *    <next  20 bytes = FRIDAY SCHEDULE>
  *    <last  20 bytes = SATURDAY SCHEDULE>
  *    <DR Event> = 8 bytes
  *         <month(0-12)><day(0-31)> <hour (0-24)> <minute (0-60)><duration hours><duration minutes><heat set point> <cool set point>
  *    TOTAL BYTES this section = 5 + 4 * 35 + 8 =  5 + 140 + 8 = 153
  * SCHEDULE STARTS AT LOCATION : 0x00000019 
  * TODO: READADJUST ADDRESSES
  *    *********************
  * CIRCULAR BUFFER FOR SETPOINT DATA
  * Head                            2 BYTES
  * Tail                            2 BYTES
  * capacity of buffer              2 BYTES
  * number of elements              2 BYTES
  * Overwritten count               2 BYTES
  * RTC TIME base for buffer        5 BYTES // ALL ZEROS IMPLY NO TIME SET
  * < REPEAT DATA STRUCT>
  *     <diff in minutes from 
  *         RTC TIME base>          2 BYTES // uint16 (max of 65535 minutes)
  *     <Cool Set point>            1 BTYE  // MSBit is 1 => set from phone app, 0 => set from HW interface
  *                                         //      remaining 7 bits cool setpoint 
  *     <Heat Set point>            1 BTYE  // MSBit is 1 => set from phone app, 0 => set from HW interface
  *                                         //      remaining 7 bits heat setpoint 
  * TOTAL BYTES this section = 15 + 4 *x, where x is number of samples stored
  *                          = 15 + 4 * 5000 = 20015
  * SETPOINT BUFFER STARTS AT LOCATION : 25 + 153 = 178 = 0x000000B2
  *   
  *    *********************
  * CIRCULAR BUFFER FOR RUNTIME DATA
  * Head                            2 BYTES
  * Tail                            2 BYTES
  * capacity of buffer              2 BYTES
  * number of elements              2 BYTES
  * Overwritten count               2 BYTES
  * RTC TIME base for buffer        5 BYTES // ALL ZEROS IMPLY NO TIME SET
  * < REPEAT DATA STRUCT>
  *     <diff in minutes from 
  *         RTC TIME base>          2 BYTES // uint16 (max of 65535 minutes)
  *     <Cool runtime info>         1 BTYE  // MSBit is 1 => Cool turned on 0 => cool turned off
  *                                         //      remaining 7 bits current temp
  *     <Heat runtime info>         1 BTYE  // MSBit is 1 => Heat turned on, 0 => heat turned off
  *                                         //      remaining 7 bits current temp 
  * TOTAL BYTES this section = 15 + 4 *y, where y is number of samples stored
  *                          = 15 + 4 * 15000 = 60015
  * RUNTIME BUFFER STARTS AT LOCATION 178 + 20015 = 20193 = 0x00004EE1
  * 
  *    *********************
  * CIRCULAR BUFFER FOR Temperature DATA
  * Head                            2 BYTES
  * Tail                            2 BYTES
  * capacity of buffer              2 BYTES
  * number of elements (count)      2 BYTES
  * Overwritten count               2 BYTES
  * RTC TIME base for buffer        5 BYTES // ALL ZEROS IMPLY NO TIME SET
  * < REPEAT DATA STRUCT>
  *     <diff in minutes from 
  *         RTC TIME base>          2 BYTES // uint16 
  *     <Temperaure>                1 BTYE  // current temperature in Integer
  *     <Relative Humidity>         1 BTYE  // current temperature in Integer
  *                                         //      remaining 7 bits current temp 
  * TOTAL BYTES this section = 15 + 4 *z, where z is number of samples stored
  *                          = 15 + 4 * 45480 = 181935
  * TEMPERATURE BUFFER STARTS AT LOCATION 178 + 20015 + 60015 = 80208 = 0x00013950
  * 
  *    *********************
  * 
  *     Calculations: see excel sheet
  *    *********************
  * 
 */
 

#define memLocRTCInitialized            0x00000000 // 1 or 0
#define memLocRTCSetupDate              0x00000001 // 5 bytes year (2 digits), month, date, hour, minute
#define memLocNoneSchHeatSetpoint       0x00000006 
#define memLocCoolSetpoint              0x00000007 // value range 75F - 90F 
#define memLocHeatSetpoint              0x00000008 // value range 50F - 74F 
#define memLocAppInitializedOne         0x00000009 // first byte indicating app has executed at least once should contain 0xB5
#define memLocPartialScreenUpdateCount  0x0000000A // uint16
#define memLocAppInitializedTwo         0x0000000C // second byte indicating app has executed at least once should contain 0x4A
                                                   // sum of first byte  + second byte must be 0xFF
#define memLocBufferInitialized         0x0000000D // if value is 6D the buffers have been initialized once
#define memLocNoneSchCoolSetpoint       0x0000000E
#define BufferInitializedWriteValue     0x6D
#define memLocPreviousSchedule          0x0000000F
#define memLocAwayThermostatMode        0x00000010 // 1 => heat only, 2 => cool only, 3=> heat + cool
#define memLocAwayCoolSetpoint          0x00000011
#define memLocAwayHeatSetpoint          0x00000012
#define memLocHysteresis                0x00000013
#define memLocThermostatMode            0x00000014
#define memLocPreviousThermostatMode    0x00000015
#define memLocSchedule                  0x00000016

#define memLocScheduleStart             0x00000019 
#define memLocSetpointBufferStart       0x000000B2
#define memLocRuntimeBufferStart        0x00004EE1
#define memLocTemperatureBufferStart    0x00013950
#define SetpointBufferMaxCapacity       0x1388 // 5000        
#define RuntimeBufferMaxCapacity        0x3A98 // 15000
#define TemperatureBufferMaxCapacity    0xB1A8 //45480



#ifndef MEMORY_H
#define	MEMORY_H

#ifdef	__cplusplus
extern "C" {
#endif

void  ReadRTCCInitializedMemory(void);
void WriteRTCCInitializedMemory(void);
void ClearRTCCInitializedMemory(void);
void WriteRTCSetupTime(struct tm *setupTime);
void ReadRTCCSetupTime(struct tm *setupTime);
void ReadCoolSetpoint(void);
void WriteCoolSetpoint(void);
void ReadHeatSetpoint(void);
void WriteHeatSetpoint(void);
uint8_t FRAMInitialized(void);
void ReadPartialScreenUpdateCount(void);
void WritePartialScreenUpdateCount(void);
void ReadHysteresis(void);
void WriteHysteresis(void);
void ReadThermostatMode(void);
void WriteThermostatMode(void);
void ScheduleInit();
void ReadSchedule(int dayOfWeek);
void WriteSingleSchedule(int dayOfWeek, int schedule_number,
        uint8_t hhour, uint8_t mminute, uint8_t cool_sp, uint8_t heat_sp);
void ClearDRSchedule(void);
void ReadDRSchedule(void);
void WriteDRSchedule();
void WriteSchedule(int dayOfWeek);
void WriteBuffersInitialized();
uint8_t ReadBuffersInitialized();
void WriteTemperatureBuffer();
int ReadTemperatureBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx);
void TemperatureBufferInit();
void DeleteEntriesTemperatureBuffer(int num_of_items);
uint16_t GetItemCountTemperatureBuffer();
uint16_t GetCapacityTemperatureBuffer();
void WriteScheduleReceived();
uint8_t ScheduleAvailable();
uint8_t DRScheduleAvailable();
void WriteAwayScheduleMode(uint8_t th_mode, uint8_t cool_sp, uint8_t heat_sp);
void ReadAwayScheduleMode();
void RuntimeBufferInit();
void WriteRuntimeBuffer();
int ReadRuntimeBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx);
void DeleteEntriesRuntimeBuffer(int num_of_items);
uint16_t GetItemCountRuntimeBuffer();
uint16_t GetCapacityRuntimeBuffer();
void SetpointBufferInit();
void WriteSetpointBuffer();
int ReadSetpointBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx);
void DeleteEntriesSetpointBuffer(int num_of_items);
uint16_t GetItemCountSetpointBuffer();
uint16_t GetCapacitySetpointBuffer();
void ParametersInit();
void ClearBuffersInitialized();
void FRAMWriteInitialization();
void FRAMClearInitialization();
void LoadScheduleFromFRAM();

#ifdef	__cplusplus
}
#endif

#endif	/* MEMORY_H */

