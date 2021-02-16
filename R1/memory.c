/* 
 * File:   memory.c
 * Author: Viswanath
 *
 * Created on January 22, 2019, 5:34 AM
 */


#include <xc.h>
#include "main.h"
#include "hal.h"
#include "system.h"
#include "rtcc.h"
#include "FRAM.h"
#include "memory.h"


/**
 * RTC Initialize bit is used only by 
 * rtcc.h during RTCC initialize. If 
 * the time has already been set, then
 * the rtcc initialize will not overwrite
 * the DATE and TIME registers
 */
void  ReadRTCCInitializedMemory(void)
{
    uint8_t data = 0;
    data = FRAMReadByte(memLocRTCInitialized, 0);
    if (data == 1)
    {
        rtccTimeInitialized = true;
    }
    else
    {
        rtccTimeInitialized = false;
    }    
}

void WriteRTCCInitializedMemory(void)
{
    uint8_t data = 0;
    data = 1;
    FRAMWriteByte(data,memLocRTCInitialized, 0 );
}

void ClearRTCCInitializedMemory(void)
{
    uint8_t data = 0;
    FRAMWriteByte(data,memLocRTCInitialized, 0 );
}


void ReadRTCCSetupTime(struct tm *setupTime) {
    uint8_t tempval;
    //date and time stamp stored in 5 bytes: year, month, day, hour, minutes
    tempval = FRAMReadByte(memLocRTCSetupDate, 0);
    setupTime->tm_year = tempval;
    tempval = FRAMReadByte(memLocRTCSetupDate+1, 0);
    setupTime->tm_mon = tempval;
    tempval = FRAMReadByte(memLocRTCSetupDate+2, 0);
    setupTime->tm_mday = tempval;
    tempval = FRAMReadByte(memLocRTCSetupDate+3, 0);
    setupTime->tm_hour = tempval;
    tempval = FRAMReadByte(memLocRTCSetupDate+4, 0);
    setupTime->tm_min = tempval;
}

void WriteRTCSetupTime(struct tm *setupTime) {
    uint8_t tempval;
    //date and time stamp stored in 5 bytes: year, month, day, hour, minutes
    tempval = (uint8_t)setupTime->tm_year;
    FRAMWriteByte(tempval,memLocRTCSetupDate,0);
    tempval = (uint8_t)setupTime->tm_mon;
    FRAMWriteByte(tempval,(memLocRTCSetupDate+1),0);
    tempval = (uint8_t)setupTime->tm_mday;
    FRAMWriteByte(tempval,(memLocRTCSetupDate+2),0);
    tempval = (uint8_t)setupTime->tm_hour;
    FRAMWriteByte(tempval,(memLocRTCSetupDate+3),0);
    tempval = (uint8_t)setupTime->tm_min;
    FRAMWriteByte(tempval,(memLocRTCSetupDate+4),0);     
    
}

/**
 * u8CoolSetpoint is the cool setpoint for the thermostat
 * this performs the cooling when temperature goes above setpoint 
 * Reads and Writes cool set point from global variable u8CoolSetpoint
 */
void ReadCoolSetpoint(void)
{
    uint8_t data = 0xFF;
    data = FRAMReadByte(memLocCoolSetpoint, 0);
    u8CoolSetpoint = data;
}
void WriteCoolSetpoint(void)
{
    uint8_t data = u8CoolSetpoint;
    FRAMWriteByte(data,memLocCoolSetpoint, 0 );
}

/**
 * u8HeatSetpoint is the heat setpoint for the thermostat
 * this performs the heat when temperature goes below setpoint 
 * Reads and Writes heat set point from global variable u8HeatSetpoint
 */
void ReadHeatSetpoint(void)
{
    uint8_t data = 0xFF;
    data = FRAMReadByte(memLocHeatSetpoint, 0);
    u8HeatSetpoint = data;
}
void WriteHeatSetpoint(void)
{
    uint8_t data = u8HeatSetpoint;
    FRAMWriteByte(data,memLocHeatSetpoint, 0 );
}

/**
 * Identifies if the FRAM has been initialized
 *    implying the app has run at least once
 *    and all initializations are completed
 * Reads the first byte at memLocAppInitializedOne, and 
 *  second byte at memLocAppInitializedTwo.
 *  
 * @return 1 if sum of first byte + second byte = 0xFF
 *      else return 0
 */
uint8_t FRAMInitialized()
{
    uint8_t data1, data2;
    data1 = FRAMReadByte(memLocAppInitializedOne, 0);
    data2 = FRAMReadByte(memLocAppInitializedTwo, 0);
    if ( ((data1 + data2) == 0xFF) && ( (data1 | data2) == 0xFF) )
    {
        return 1;
    } else
    {
        return 0;
    }
}


/**
 * Write FRAM initialization bytes
 * Write 0xB5 at first byte at memLocAppInitializedOne, and 
 *  0x4A at second byte at memLocAppInitializedTwo.
 *  
 */
void FRAMWriteInitialization()
{
    FRAMWriteByte(0xB5, memLocAppInitializedOne, 0);
    FRAMWriteByte(0x4A, memLocAppInitializedTwo, 0);
}



/**
 * Clears FRAM initialization
 * Write 0 at first byte at memLocAppInitializedOne, and 
 *  0 at second byte at memLocAppInitializedTwo.
 *  
 */
void FRAMClearInitialization()
{
    FRAMWriteByte(0x00, memLocAppInitializedOne, 0);
    FRAMWriteByte(0x00, memLocAppInitializedTwo, 0);
}

/**
 * u16PartialScreenUpdateCount is the count that tracks number of partial updates
 * after a certain number of partial updates, a full screen update would be necessary
 * Reads and Writes screen update count from global variable u16PartialScreenUpdateCount
 */
void ReadPartialScreenUpdateCount(void)
{
    uint16_t data = 0xFF;
    data = FRAMReadWord(memLocPartialScreenUpdateCount, 0);
    u16PartialScreenUpdateCount = data;
}
void WritePartialScreenUpdateCount(void)
{
    uint16_t data = u16PartialScreenUpdateCount;
    FRAMWriteWord(data,memLocPartialScreenUpdateCount, 0 );
}


/**
 * u8Hysteresis contains the current hysteresis value set
 *   allowed values are : 1 to 4
 * Reads and Writes mode from global variable u8Hysteresis
 */
void ReadHysteresis(void)
{
    uint8_t data = 0xFF;
    data = FRAMReadByte(memLocHysteresis, 0);
    u8Hysteresis = data;
}
void WriteHysteresis(void)
{
    uint8_t data = u8Hysteresis;
    FRAMWriteByte(data,memLocHysteresis, 0 );
}


/**
 * u8ThermostatMode has the current mode
 *   allowed values are : heat+cool, cool only, heat only
 * Reads and Writes mode from global variable u8ThermostatMode
 */
void ReadThermostatMode(void)
{
    uint8_t data = 0xFF;
    data = FRAMReadByte(memLocThermostatMode, 0);
    u8ThermostatMode = data;
}
void WriteThermostatMode(void)
{
    uint8_t data = u8ThermostatMode;
    FRAMWriteByte(data,memLocThermostatMode, 0 );
}

/**
 * initialzes the schedule bitmap to 0's 
 * this is called the first time the app executes
 */
void ScheduleInit()
{
    FRAMWriteByte(0x00, memLocScheduleStart, 0);
    FRAMWriteByte(0x00, memLocScheduleStart+1, 0);
    FRAMWriteByte(0x00, memLocScheduleStart+2, 0);
    FRAMWriteByte(0x00, memLocScheduleStart+3, 0);
    FRAMWriteByte(0x00, memLocScheduleStart+4, 0);
  
}

/**
 * Read and write schedules from/to FRAM 
 * global variable uint8_t u8arrTodaySchedule[16] contains the following
 * byte 0: hour
 * byte 1: minute
 * byte 2: cool setpoint
 * byte 3: heat setpoint
 * each day consists of 5 schedules, hence array is 20 bytes long
 * These functions read/ write a schedule for a given day (5 sections per day)
 * Therefore to write the schedules for a week, the 
 * variable todaySchedule must be correctly populated, and the Write function called to store it in FRAM
 * input: dayOfWeek, 0 represents Sunday, 1 represents Monday, 2 represents Tuesday, .... 6 represents Saturday
 */
void ReadSchedule(int dayOfWeek)
{
    uint8_t data = 0xFF;
    int offset = 5 + 20*dayOfWeek; // 5 is the schedule bitmap
    data = FRAMReadByte(memLocScheduleStart+offset, 0);
    u8arrTodaySchedule[0] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+1, 0);
    u8arrTodaySchedule[1] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+2, 0);
    u8arrTodaySchedule[2] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+3, 0);
    u8arrTodaySchedule[3] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+4, 0);
    u8arrTodaySchedule[4] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+5, 0);
    u8arrTodaySchedule[5] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+6, 0);
    u8arrTodaySchedule[6] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+7, 0);
    u8arrTodaySchedule[7] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+8, 0);
    u8arrTodaySchedule[8] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+9, 0);
    u8arrTodaySchedule[9] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+10, 0);
    u8arrTodaySchedule[10] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+11, 0);
    u8arrTodaySchedule[11] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+12, 0);
    u8arrTodaySchedule[12] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+13, 0);
    u8arrTodaySchedule[13] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+14, 0);
    u8arrTodaySchedule[14] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+15, 0);
    u8arrTodaySchedule[15] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+16, 0);
    u8arrTodaySchedule[16] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+17, 0);
    u8arrTodaySchedule[17] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+18, 0);
    u8arrTodaySchedule[18] = data;
    data = FRAMReadByte(memLocScheduleStart+offset+19, 0);
    u8arrTodaySchedule[19] = data;
}

/**
 * Writes a 1 at the bitmap corresponding to the schedule at dayOfWeek, and schedule number
 *  this function assumes that dayOfWeek is in range 0-6, and schedule_num is in range 0-4
 *    for writing a  bit for DR event, dayOfWeek should be 0x0F, and schedule number should be 0
*/
void WriteSingleScheduleBitmap(int dayOfWeek, int schedule_num)
{
    uint8_t data;
    uint8_t tmpu8_loc;
    uint8_t tmpu8_bit;
    tmpu8_loc = 0;
    switch (dayOfWeek)
    {
        case 0:
            tmpu8_loc = 2;
            tmpu8_bit = 6;
            break;
        case 1:
            tmpu8_loc = 2;
            tmpu8_bit = 1;
            break;
        case 2:
            tmpu8_loc = 3;
            tmpu8_bit = 4;
            break;
        case 3:
            tmpu8_loc = 4;
            tmpu8_bit = 7;
            break;
        case 4:
            tmpu8_loc = 4;
            tmpu8_bit = 2;
            break;
        case 5:
            tmpu8_loc = 5;
            tmpu8_bit = 5;
            break;
        case 6:
            tmpu8_loc = 5;
            tmpu8_bit = 0;
            break;
        case 0x0F:
            tmpu8_loc = 1;
            tmpu8_bit = 3;
    }
    // if schedule_num is zero, the values are already set
    if ( (tmpu8_loc > 0) && (schedule_num > 0) )
    {
        tmpu8_bit = tmpu8_bit + schedule_num;
        if (tmpu8_bit > 7)
        {
            tmpu8_bit = tmpu8_bit % 8;
            tmpu8_loc -=1;
        }
    }
    data = FRAMReadByte(memLocScheduleStart + tmpu8_loc - 1, 0);
    data = data | (1 << tmpu8_bit);
    FRAMWriteByte(data, memLocScheduleStart + tmpu8_loc - 1, 0);
}

/**
 * The bluetooth will be sending individual schedules, and hence this 
 * function will write a single schedule to FRAM, i.e
 *      each schedule contains start time (hh(0-24), mm(0-60), cool and heat set points
 * @param dayOfWeek
 * @param schedule_number
 * @param hhour
 * @param mminute
 * @param cool_sp
 * @param heat_sp
 */
void WriteSingleSchedule(int dayOfWeek, int schedule_number,
        uint8_t hhour, uint8_t mminute, uint8_t cool_sp, uint8_t heat_sp)
{
    int offset = 5 + 20*dayOfWeek + schedule_number*4; // 5 is the schedule bitmap
#ifndef testble
    FRAMWriteByte(hhour, memLocScheduleStart+offset, 0);
    FRAMWriteByte(mminute, memLocScheduleStart+offset+1, 0);
    FRAMWriteByte(cool_sp, memLocScheduleStart+offset+2, 0);
    FRAMWriteByte(heat_sp, memLocScheduleStart+offset+3, 0);
    WriteSingleScheduleBitmap(dayOfWeek, schedule_number);
#endif
    offset = 20*dayOfWeek + schedule_number*4;
    u8arrSchedule[offset] = hhour;
    u8arrSchedule[offset+1] = mminute;
    u8arrSchedule[offset+2] = cool_sp;
    u8arrSchedule[offset+3] = heat_sp;
}

/**
 * Write DR Schedule to FRAM
 * DR schedule in u8arrDrEvent, and format is same as the following parameters
 *      month
 *      dayOfMonth
 *      hhour
 *      mminute
 *      duration_hh
 *      duration_mm
 *      cool_sp
 *      heat_sp
 */
void WriteDRSchedule()
{
    int offset = 5 + 20*7; // 5 is schedule bitmap
    FRAMWriteByte(u8arrDrEvent[0], memLocScheduleStart+offset, 0);
    FRAMWriteByte(u8arrDrEvent[1], memLocScheduleStart+offset+1, 0);
    FRAMWriteByte(u8arrDrEvent[2], memLocScheduleStart+offset+2, 0);
    FRAMWriteByte(u8arrDrEvent[3], memLocScheduleStart+offset+3, 0);
    FRAMWriteByte(u8arrDrEvent[4], memLocScheduleStart+offset+4, 0);
    FRAMWriteByte(u8arrDrEvent[5], memLocScheduleStart+offset+5, 0);
    FRAMWriteByte(u8arrDrEvent[6], memLocScheduleStart+offset+6, 0);
    FRAMWriteByte(u8arrDrEvent[7], memLocScheduleStart+offset+7, 0);
    WriteSingleScheduleBitmap(0x0F, 0);
}

/**
 * Read the DR schedule from FRAM TO u8arrDrEvent array
 */
void ReadDRSchedule()
{
    int offset = 5 + 20*7; // 5 is schedule bitmap
    u8arrDrEvent[0] = FRAMReadByte(memLocScheduleStart+offset, 0);
    u8arrDrEvent[1] = FRAMReadByte( memLocScheduleStart+offset+1, 0);
    u8arrDrEvent[2] = FRAMReadByte( memLocScheduleStart+offset+2, 0);
    u8arrDrEvent[3] = FRAMReadByte( memLocScheduleStart+offset+3, 0);
    u8arrDrEvent[4] = FRAMReadByte( memLocScheduleStart+offset+4, 0);
    u8arrDrEvent[5] = FRAMReadByte( memLocScheduleStart+offset+5, 0);
    u8arrDrEvent[6] = FRAMReadByte( memLocScheduleStart+offset+6, 0);
    u8arrDrEvent[7] = FRAMReadByte( memLocScheduleStart+offset+7, 0);
}

/**
 * clears the DRSchedule in FRAM 
 */
void ClearDRSchedule()
{
    uint8_t tmpu8;
    // delete the DR event from memory
    FRAMWriteLong(0x00000000, memLocScheduleStart + 5 + 7*20, 0);
    FRAMWriteLong(0x00000000, memLocScheduleStart + 5 + 7*20+4, 0);
    // clear the DR event bit 
    tmpu8 = FRAMReadByte(memLocScheduleStart, 0);
    tmpu8 = (uint8_t)(tmpu8 & (~0x08));
    FRAMWriteByte(tmpu8, memLocScheduleStart, 0);
}

/**
 * Stores a single day's schedule to FRAM
 * todaySchedule must be correctly populated, and the Write function called to store it in FRAM
 * @param: dayOfWeek, 0 represents Sunday, 1 represents Monday, 2 represents Tuesday, .... 6 represents Saturday
 */
void WriteSchedule(int dayOfWeek)
{
    uint8_t data;
    int offset = 5 + 20*dayOfWeek; // 5 is schedule bitmap
    data = u8arrTodaySchedule[0];
    FRAMWriteByte(data,memLocScheduleStart + offset, 0 );
    data = u8arrTodaySchedule[1];
    FRAMWriteByte(data,memLocScheduleStart + offset+1, 0 );
    data = u8arrTodaySchedule[2];
    FRAMWriteByte(data,memLocScheduleStart + offset+2, 0 );
    data = u8arrTodaySchedule[3];
    FRAMWriteByte(data,memLocScheduleStart + offset+3, 0 );
    data = u8arrTodaySchedule[4];
    FRAMWriteByte(data,memLocScheduleStart + offset+4, 0 );
    data = u8arrTodaySchedule[5];
    FRAMWriteByte(data,memLocScheduleStart + offset+5, 0 );
    data = u8arrTodaySchedule[6];
    FRAMWriteByte(data,memLocScheduleStart + offset+6, 0 );
    data = u8arrTodaySchedule[7];
    FRAMWriteByte(data,memLocScheduleStart + offset+7, 0 );
    data = u8arrTodaySchedule[8];
    FRAMWriteByte(data,memLocScheduleStart + offset+8, 0 );
    data = u8arrTodaySchedule[9];
    FRAMWriteByte(data,memLocScheduleStart + offset+9, 0 );
    data = u8arrTodaySchedule[10];
    FRAMWriteByte(data,memLocScheduleStart + offset+10, 0 );
    data = u8arrTodaySchedule[11];
    FRAMWriteByte(data,memLocScheduleStart + offset+11, 0 );
    data = u8arrTodaySchedule[12];
    FRAMWriteByte(data,memLocScheduleStart + offset+12, 0 );
    data = u8arrTodaySchedule[13];
    FRAMWriteByte(data,memLocScheduleStart + offset+13, 0 );
    data = u8arrTodaySchedule[14];
    FRAMWriteByte(data,memLocScheduleStart + offset+14, 0 );
    data = u8arrTodaySchedule[15];
    FRAMWriteByte(data,memLocScheduleStart + offset+15, 0 );
}


/*
 * Read/ Write the location memLocBufferInitialized to indicate if the system is executing for the 
 *      first time, or has some data from earlier state
 *      if first time write BufferInitializedWriteValue (0x6D) to location
 *          else do not do any initialization
 */
void WriteBuffersInitialized()
{
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != (uint8_t)BufferInitializedWriteValue) {
        data = (uint8_t)BufferInitializedWriteValue;
        FRAMWriteByte(data, memLocBufferInitialized, 0);
    }
}
uint8_t ReadBuffersInitialized()
{
    uint8_t data;
    data = FRAMReadByte(memLocBufferInitialized, 0);
    return data;
}

void ClearBuffersInitialized()
{
    FRAMWriteByte(0, memLocBufferInitialized, 0);
    FRAMWriteWord(0, memLocTemperatureBufferStart + 6, 0);
    FRAMWriteWord(0, memLocSetpointBufferStart + 6, 0);
    FRAMWriteWord(0, memLocRuntimeBufferStart + 6, 0);
}

/*
 * Buffer for temperature data stored in FRAM
 * initialize:
 *     this function initializes the buffer for setpoint data storage
 *     - set head to 0, 
 *     - set tail to 0, 
 *     - set capacity to TemperatureBufferCapacity (max based on allocated storage)
 *     - set count to 0,
 *     - set overtide count to 0,
 *     - set all rtc time base bytes to 0
 *    This function is called to initialize buffers and  WriteBuffersInitialized() is called after calling this function
 */
void TemperatureBufferInit()
{
//    uint8_t data;
//    data = ReadBuffersInitialized();
//    if (data == (uint8_t)BufferInitializedWriteValue ) 
//    {
        // initialize the temperature buffer
        // set head to zero
        FRAMWriteWord(0x0000, memLocTemperatureBufferStart, 0);
        // set tail to zero
        FRAMWriteWord(0x0000, memLocTemperatureBufferStart+2, 0);
        // set capacity to TemperatureBufferCapacity
        FRAMWriteWord((uint16_t)TemperatureBufferMaxCapacity, memLocTemperatureBufferStart+4, 0);
        // set count to zero
        FRAMWriteWord(0x0000, memLocTemperatureBufferStart+6, 0);
        // set override count to zero
        FRAMWriteWord(0x0000, memLocTemperatureBufferStart+8, 0);
        // set all rtc time base bytes to zero
        FRAMWriteByte(0x00, memLocTemperatureBufferStart+10, 0);
        FRAMWriteByte(0x00, memLocTemperatureBufferStart+11, 0);
        FRAMWriteByte(0x00, memLocTemperatureBufferStart+12, 0);
        FRAMWriteByte(0x00, memLocTemperatureBufferStart+13, 0);
        FRAMWriteByte(0x00, memLocTemperatureBufferStart+14, 0);
//    }
}

/*
 * u8CurrentTemperature stores the current temperature
 * write the value in u8CurrentTemperature to temperature buffer 
 * This function writes the temperature stored in u8CurrentTemperature to the tmeperature buffer
 */
void WriteTemperatureBuffer()
{
    uint8_t data;
    uint32_t u32data;
    uint16_t count_in_tempbuf;
    double diff_time;
    time_t base_t, curr_t;
    int offset;
    uint16_t tmpu16;
    struct tm base_time;
    struct tm curr_time;
    uint16_t head;
    uint16_t capacity;
    if (rtccTimeInitialized == false)
    {
        return;
    }
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return;
    }
    count_in_tempbuf = FRAMReadWord(memLocTemperatureBufferStart + 6, 0);
    // TODO: create circular buffer
    // for now if we reach max capacity, we stop writing
    head = FRAMReadWord(memLocTemperatureBufferStart, 0 );
    capacity = FRAMReadWord(memLocTemperatureBufferStart + 4, 0);
    // reached max capacity return
    if (count_in_tempbuf >= capacity) {
        return;
    }
    if (count_in_tempbuf == 0)
    {
        // write the current timestamp to rtc time base
        RTCC_TimeGet(&base_time);
        data = base_time.tm_year;
        FRAMWriteByte(data, memLocTemperatureBufferStart + 10, 0);
        data = base_time.tm_mon;
        FRAMWriteByte(data, memLocTemperatureBufferStart + 11, 0);
        data = base_time.tm_mday;
        FRAMWriteByte(data, memLocTemperatureBufferStart + 12, 0);
        data = base_time.tm_hour;
        FRAMWriteByte(data, memLocTemperatureBufferStart + 13, 0);
        data = base_time.tm_min;
        FRAMWriteByte(data, memLocTemperatureBufferStart + 14, 0);
        u32data = 0x00;
        
    }
    else 
    {
        RTCC_TimeGet(&curr_time);
        // read the rtc time base from buffer
        data = FRAMReadByte(memLocTemperatureBufferStart + 10, 0);
        base_time.tm_year = (int)data;
        data = FRAMReadByte(memLocTemperatureBufferStart + 11, 0);
        base_time.tm_mon = (int)data;
        data = FRAMReadByte(memLocTemperatureBufferStart + 12, 0);
        base_time.tm_mday = (int)data;
        data = FRAMReadByte(memLocTemperatureBufferStart + 13, 0);
        base_time.tm_hour = (int)data;
        data = FRAMReadByte(memLocTemperatureBufferStart + 14, 0);
        base_time.tm_min = (int)data;
        base_time.tm_sec = 0;
        base_t = mktime(&base_time);
        curr_t = mktime(&curr_time);
        diff_time = difftime(curr_t, base_t);
        if (diff_time < 0)
        {
            diff_time = -diff_time;
        }
        diff_time = diff_time / 60.0; // convert to minutes;
        if (diff_time > (double)0xFFFF)
        {
            u32data = 0;
        }
        else 
        {
            u32data =(int) diff_time;
        }
    }
    if ( (u32data == 0) && ( count_in_tempbuf  > 0) )
    {
        // cannot report data since not enough space to store time
        return;
    }
    //offset = count_in_tempbuf * 4 + 15; // memLocTemperatureBufferStart;
    offset = head * 4 + 15;
    tmpu16 = (head + 1) % capacity;
    data = (uint8_t)(u32data & 0xFF); // Little endian format
    FRAMWriteByte(data, memLocTemperatureBufferStart + offset, 0 );
    data = (uint8_t)( (u32data & 0xFF00) >> 8); // Little endian format
    FRAMWriteByte(data, memLocTemperatureBufferStart + offset + 1, 0);
    data = u8CurrentTemperature;
    FRAMWriteByte(data, memLocTemperatureBufferStart + offset + 2, 0);
    data = u8RelativeHumidity;
#ifdef debugboardtemp
    // testing on unit between middle and edge temperature
    //u8BoardTemperature = ReadBoardTemperature(BoardMiddle);
    // TODO: NEXT LINE ONLY FOR DEBUGGYNG.... ???????????????          ????????????? **************** TODO remove after debug
    data = (uint8_t)ReadBoardTemperature(BoardMiddle);
#endif
    FRAMWriteByte(data, memLocTemperatureBufferStart + offset + 3, 0);
    // write number_of_items + 1 to fram
    FRAMWriteWord(tmpu16, memLocTemperatureBufferStart, 0);
    tmpu16 = (uint16_t)(count_in_tempbuf + 1);
    FRAMWriteWord(tmpu16, memLocTemperatureBufferStart + 6, 0);
}

/**
 * ReadTemperatureBuffer
 *  arguments 
 *      number_of_samples to read
 *      data_buf(uint8_t *) for storing return elements;this must be allocated before the call
 *      data_buf_len: length of buffer created
 * 
 *  return value:
 *      int: 
 *          number of samples written to data_buf if successful
 *          -1 if data_buf_len is less than the required length for number of elements
 *          -2 if temperature buffer is empty
 *          -3 if buffer in FRAM has not been initialized
 * 
 *  format of data_buf returned
 *      first 5 bytes base timestamp as stored in FRAM
 *      4 BYTES per sample each
 *               
 */
int ReadTemperatureBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx)
{
    uint8_t data;
    int buf_ret_count = 0;
    uint16_t count_in_tempbuf;
    int ii;
    uint16_t tail;
    uint16_t capacity;
    int offset;
    int tmp_count_pos;
    
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return -3;
    }
    if ( (number_of_samples * 4 + 6) > ( data_buf_len - start_idx ) ) 
    {
        // insufficient space to return data
        return -1;
    }
    count_in_tempbuf = FRAMReadWord(memLocTemperatureBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return -2;
    }
    tail = FRAMReadWord(memLocTemperatureBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocTemperatureBufferStart + 4, 0);
    
    buf_ret_count = 0;
    data = FRAMReadByte(memLocTemperatureBufferStart + 10, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocTemperatureBufferStart + 11, 0);
    data_buf[start_idx + buf_ret_count] = data; 
    buf_ret_count ++;
    data = FRAMReadByte(memLocTemperatureBufferStart + 12, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocTemperatureBufferStart + 13, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocTemperatureBufferStart + 14, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    tmp_count_pos = buf_ret_count;
    buf_ret_count ++;
    ii = 0;
    while ( (ii < number_of_samples) && (count_in_tempbuf > 0) )
    {
        offset = tail * 4 + 15;
        data = FRAMReadByte(memLocTemperatureBufferStart + offset, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocTemperatureBufferStart + offset+1, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocTemperatureBufferStart + offset+2, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocTemperatureBufferStart + offset+3, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        tail = (tail + 1) % capacity;
        count_in_tempbuf --;
        ii++;
    }
    data_buf[start_idx + tmp_count_pos] = (uint8_t)ii;
    return ii; // count of samples that 
}

/**
 * 
 * @param num_of_items
 * Delete from FRAM num_of_items from Temperature Buffer
 *      since they were successfully transmitted
 */
void DeleteEntriesTemperatureBuffer(int num_of_items)
{
    uint16_t count_in_tempbuf;
    uint16_t tail;
    uint16_t capacity;
    count_in_tempbuf = FRAMReadWord(memLocTemperatureBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return;
    }
    tail = FRAMReadWord(memLocTemperatureBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocTemperatureBufferStart + 4, 0);
    if (count_in_tempbuf < num_of_items)
    {
        tail = (tail + count_in_tempbuf) % capacity;
        count_in_tempbuf = 0;
    }
    else 
    {
        count_in_tempbuf -= num_of_items;
        tail = (tail + num_of_items) % capacity;
    }
    // write new tail to fram
    FRAMWriteWord(count_in_tempbuf, memLocTemperatureBufferStart + 6,    0);
    FRAMWriteWord(tail, memLocTemperatureBufferStart + 2, 0);
    if (count_in_tempbuf == 0)
    {
        // clear the timestamp in temperature buffer as all elements are empty
        FRAMWriteByte(0, memLocTemperatureBufferStart + 10, 0);
        FRAMWriteByte(0, memLocTemperatureBufferStart + 11, 0);
        FRAMWriteByte(0, memLocTemperatureBufferStart + 12, 0);
        FRAMWriteByte(0, memLocTemperatureBufferStart + 13, 0);
        FRAMWriteByte(0, memLocTemperatureBufferStart + 14, 0);
    }
    
}

/**
 * 
 * @return the nubmer of elements stored in temperature buffer
 */
uint16_t GetItemCountTemperatureBuffer() 
{
    uint16_t count_in_tempbuf;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    count_in_tempbuf = FRAMReadWord(memLocTemperatureBufferStart + 6, 0);
    return count_in_tempbuf;
}

/**
 * 
 * @return the capacity of temperature buffer
 */
uint16_t GetCapacityTemperatureBuffer() 
{
    uint16_t capacity;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    capacity = FRAMReadWord(memLocTemperatureBufferStart + 4, 0);
    return capacity;
}


/*
 * Buffer for runtime data stored in FRAM
 * initialize:
 *     this function initializes the buffer for runtime data storage
 *     - set head to 0, 
 *     - set tail to 0, 
 *     - set capacity to RuntimeBufferMaxCapacity (max based on allocated storage)
 *     - set count to 0,
 *     - set overtide count to 0,
 *     - set all rtc time base bytes to 0
 *    This function is called initialize buffers, and  WriteBuffersInitialized() is called after calling this function
 */
void RuntimeBufferInit()
{
//    uint8_t data;
//    data = ReadBuffersInitialized();
//    if (data == (uint8_t)BufferInitializedWriteValue ) 
//    {
        // initialize the setpoint buffer
        // set head to zero
        FRAMWriteWord(0x0000, memLocRuntimeBufferStart, 0);
        // set tail to zero
        FRAMWriteWord(0x0000, memLocRuntimeBufferStart+2, 0);
        // set capacity to TemperatureBufferCapacity
        FRAMWriteWord((uint16_t)RuntimeBufferMaxCapacity, memLocRuntimeBufferStart+4, 0);
        // set count to zero
        FRAMWriteWord(0x0000, memLocRuntimeBufferStart+6, 0);
        // set override count to zero
        FRAMWriteWord(0x0000, memLocRuntimeBufferStart+8, 0);
        // set all rtc time base bytes to zero
        FRAMWriteByte(0x00, memLocRuntimeBufferStart+10, 0);
        FRAMWriteByte(0x00, memLocRuntimeBufferStart+11, 0);
        FRAMWriteByte(0x00, memLocRuntimeBufferStart+12, 0);
        FRAMWriteByte(0x00, memLocRuntimeBufferStart+13, 0);
        FRAMWriteByte(0x00, memLocRuntimeBufferStart+14, 0);
//    }
}


/*
 * u8CurrentTemperature stores the current temperature
 * u8CurrentThermostatAction stores the current action of thermostat : ACTION_HEATING, ACTION_COOLING, ACTION_NONE
 * write the value in u8CurrentTemperature and u8Thermostat Action into cool info and heat info bytes 
 * This function writes the temperature stored in u8CurrentTemperature and u8Thermostat Action into 2 bytes:
 * one byte for Cool info and one byte for heat info
 */
void WriteRuntimeBuffer()
{
    uint8_t data;
    uint32_t u32data;
    uint16_t count_in_tempbuf;
    double diff_time;
    time_t base_t, curr_t;
    int offset;
    uint16_t tmpu16;
    struct tm base_time;
    struct tm curr_time;
    uint16_t head;
    uint16_t capacity;
    if (rtccTimeInitialized == false)
    {
        return;
    }
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return;
    }
    count_in_tempbuf = FRAMReadWord(memLocRuntimeBufferStart + 6, 0);
    // for now if we reach max capacity, we stop writing
    head = FRAMReadWord(memLocRuntimeBufferStart, 0 );
    capacity = FRAMReadWord(memLocRuntimeBufferStart + 4, 0);
    // reached max capacity return
    if (count_in_tempbuf >= capacity) {
        return;
    }
    if (count_in_tempbuf == 0)
    {
        // write the current timestamp to rtc time base
        RTCC_TimeGet(&base_time);
        data = base_time.tm_year;
        FRAMWriteByte(data, memLocRuntimeBufferStart + 10, 0);
        data = base_time.tm_mon;
        FRAMWriteByte(data, memLocRuntimeBufferStart + 11, 0);
        data = base_time.tm_mday;
        FRAMWriteByte(data, memLocRuntimeBufferStart + 12, 0);
        data = base_time.tm_hour;
        FRAMWriteByte(data, memLocRuntimeBufferStart + 13, 0);
        data = base_time.tm_min;
        FRAMWriteByte(data, memLocRuntimeBufferStart + 14, 0);
        u32data = 0x00;
        
    }
    else 
    {
        RTCC_TimeGet(&curr_time);
        // read the rtc time base from buffer
        data = FRAMReadByte(memLocRuntimeBufferStart + 10, 0);
        base_time.tm_year = (int)data;
        data = FRAMReadByte(memLocRuntimeBufferStart + 11, 0);
        base_time.tm_mon = (int)data;
        data = FRAMReadByte(memLocRuntimeBufferStart + 12, 0);
        base_time.tm_mday = (int)data;
        data = FRAMReadByte(memLocRuntimeBufferStart + 13, 0);
        base_time.tm_hour = (int)data;
        data = FRAMReadByte(memLocRuntimeBufferStart + 14, 0);
        base_time.tm_min = (int)data;
        base_time.tm_sec = 0;
        base_t = mktime(&base_time);
        curr_t = mktime(&curr_time);
        diff_time = difftime(curr_t, base_t);
        if (diff_time < 0)
        {
            diff_time = -diff_time;
        }
        diff_time = diff_time / 60.0; // convert to minutes;
        if (diff_time > (double)0xFFFF)
        {
            u32data = 0;
        }
        else 
        {
            u32data =(int) diff_time;
        }
    }
    if ( (u32data == 0) && ( count_in_tempbuf  > 0) )
    {
        // cannot report data since not enough space to store time
        return;
    }
    //offset = count_in_tempbuf * 4 + 15; // memLocTemperatureBufferStart;
    offset = head * 4 + 15;
    tmpu16 = (head + 1) % capacity;
    data = (uint8_t)(u32data & 0xFF); // Little endian format
    FRAMWriteByte(data, memLocRuntimeBufferStart + offset, 0 );
    data = (uint8_t)( (u32data & 0xFF00) >> 8); // Little endian format
    FRAMWriteByte(data, memLocRuntimeBufferStart + offset + 1, 0);
    data = u8CurrentTemperature;
    if (u8CurrentThermostatAction == (uint8_t) ACTION_COOLING )
    {
        data = data | 0x80;
    }
    FRAMWriteByte(data, memLocRuntimeBufferStart + offset + 2, 0);
    data = u8CurrentTemperature;
    if (u8CurrentThermostatAction == (uint8_t) ACTION_HEATING) 
    {
        data = data | 0x80;
    }
    FRAMWriteByte(data, memLocRuntimeBufferStart + offset + 3, 0);
    // write number_of_items + 1 to fram
    FRAMWriteWord(tmpu16, memLocRuntimeBufferStart, 0);
    tmpu16 = (uint16_t)(count_in_tempbuf + 1);
    FRAMWriteWord(tmpu16, memLocRuntimeBufferStart + 6, 0);
}

/**
 * ReadRuntimeBuffer
 *  arguments 
 *      number_of_samples to read
 *      data_buf(uint8_t *) for storing return elements;this must be allocated before the call
 *      data_buf_len: length of buffer created
 * 
 *  return value:
 *      int: 
 *          number of uint8_t bytes written to data_buf if successful
 *          -1 if data_buf_len is less than the required length for number of elements
 *          -2 if temperature buffer is empty
 *          -3 if buffer in FRAM has not been initialized
 * 
 *  format of data_buf returned
 *      first 5 bytes base timestamp as stored in FRAM
 *      4 BYTES per sample each
 *               
 */
int ReadRuntimeBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx)
{
    uint8_t data;
    int buf_ret_count = 0;
    uint16_t count_in_tempbuf;
    int ii;
    uint16_t tail;
    uint16_t capacity;
    int offset;
    int tmp_count_idx;
    
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return -3;
    }
    if ( (number_of_samples * 4 + 6) > (data_buf_len - start_idx) ) 
    {
        // insufficient space to return data
        return -1;
    }
    count_in_tempbuf = FRAMReadWord(memLocRuntimeBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return -2;
    }
    tail = FRAMReadWord(memLocRuntimeBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocRuntimeBufferStart + 4, 0);
    
    buf_ret_count = 0;
    data = FRAMReadByte(memLocRuntimeBufferStart + 10, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocRuntimeBufferStart + 11, 0);
    data_buf[start_idx + buf_ret_count] = data; 
    buf_ret_count ++;
    data = FRAMReadByte(memLocRuntimeBufferStart + 12, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocRuntimeBufferStart + 13, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocRuntimeBufferStart + 14, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    tmp_count_idx = buf_ret_count;
    buf_ret_count ++;
    ii = 0;
    while ( (ii < number_of_samples) && (count_in_tempbuf > 0) )
    {
        offset = tail * 4 + 15;
        data = FRAMReadByte(memLocRuntimeBufferStart + offset, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocRuntimeBufferStart + offset+1, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocRuntimeBufferStart + offset+2, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocRuntimeBufferStart + offset+3, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        tail = (tail + 1) % capacity;
        count_in_tempbuf --;
        ii++;
    }
    data_buf[start_idx + tmp_count_idx] = (uint8_t) ii;
    return ii;
}


/**
 * 
 * @param num_of_items
 * Delete from FRAM num_of_items from Runtime Buffer
 *      since they were successfully transmitted
 */
void DeleteEntriesRuntimeBuffer(int num_of_items)
{
    uint16_t count_in_tempbuf;
    uint16_t tail;
    uint16_t capacity;
    count_in_tempbuf = FRAMReadWord(memLocRuntimeBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return;
    }
    tail = FRAMReadWord(memLocRuntimeBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocRuntimeBufferStart + 4, 0);
    if (count_in_tempbuf < num_of_items)
    {
        tail = (tail + count_in_tempbuf) % capacity;
        count_in_tempbuf = 0;
    }
    else 
    {
        count_in_tempbuf -= num_of_items;
        tail = (tail + num_of_items) % capacity;
    }
    // write new tail to fram
    FRAMWriteWord(count_in_tempbuf, memLocRuntimeBufferStart + 6,    0);
    FRAMWriteWord(tail, memLocRuntimeBufferStart + 2, 0);
    if (count_in_tempbuf == 0)
    {
        // clear the timestamp in temperature buffer as all elements are empty
        FRAMWriteByte(0, memLocRuntimeBufferStart + 10, 0);
        FRAMWriteByte(0, memLocRuntimeBufferStart + 11, 0);
        FRAMWriteByte(0, memLocRuntimeBufferStart + 12, 0);
        FRAMWriteByte(0, memLocRuntimeBufferStart + 13, 0);
        FRAMWriteByte(0, memLocRuntimeBufferStart + 14, 0);
    }
    
}

/**
 * 
 * @return the number of elements stored in temperature buffer
 */
uint16_t GetItemCountRuntimeBuffer() 
{
    uint16_t count_in_tempbuf;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    count_in_tempbuf = FRAMReadWord(memLocRuntimeBufferStart + 6, 0);
    return count_in_tempbuf;
}

/**
 * 
 * @return the number of elements stored in temperature buffer
 */
uint16_t GetCapacityRuntimeBuffer() 
{
    uint16_t capacity;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    capacity = FRAMReadWord(memLocRuntimeBufferStart + 4, 0);
    return capacity;
}



/*
 * Buffer for setpoint data stored in FRAM
 * initialize:
 *     this function initializes the buffer for setpoint data storage
 *     - set head to 0, 
 *     - set tail to 0, 
 *     - set capacity to SetpointBufferMaxCapacity (max based on allocated storage)
 *     - set count to 0,
 *     - set overtide count to 0,
 *     - set all rtc time base bytes to 0
 *    This function is called to initialize buffers, and  WriteBuffersInitialized() is called after calling this function
 */
void SetpointBufferInit()
{
//    uint8_t data;
//    data = ReadBuffersInitialized();
//    if (data == (uint8_t)BufferInitializedWriteValue ) 
//    {
        // initialize the set point buffer
        // set head to zero
        FRAMWriteWord(0x0000, memLocSetpointBufferStart, 0);
        // set tail to zero
        FRAMWriteWord(0x0000, memLocSetpointBufferStart+2, 0);
        // set capacity to TemperatureBufferCapacity
        FRAMWriteWord((uint16_t)SetpointBufferMaxCapacity, memLocSetpointBufferStart+4, 0);
        // set count to zero
        FRAMWriteWord(0x0000, memLocSetpointBufferStart+6, 0);
        // set override count to zero
        FRAMWriteWord(0x0000, memLocSetpointBufferStart+8, 0);
        // set all rtc time base bytes to zero
        FRAMWriteByte(0x00, memLocSetpointBufferStart+10, 0);
        FRAMWriteByte(0x00, memLocSetpointBufferStart+11, 0);
        FRAMWriteByte(0x00, memLocSetpointBufferStart+12, 0);
        FRAMWriteByte(0x00, memLocSetpointBufferStart+13, 0);
        FRAMWriteByte(0x00, memLocSetpointBufferStart+14, 0);
//    }
}


/*
 * u8NewCoolSetpoint stores the cool setpoint, and u8NewHeatSetpoint stores the heat setpoint
 * write the value in u8NewCoolSetpoint and u8NewHeatSetpoint into the buffer 
 * one byte for Cool setpoint and one byte for heat setpoint
 * this is triggered even if one of them has changed
 */
void WriteSetpointBuffer()
{
    uint8_t data;
    uint32_t u32data;
    uint16_t count_in_tempbuf;
    double diff_time;
    time_t base_t, curr_t;
    int offset;
    uint16_t tmpu16;
    struct tm base_time;
    struct tm curr_time;
    uint16_t head;
    uint16_t capacity;
    if (rtccTimeInitialized == false)
    {
        return;
    }
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return;
    }
    count_in_tempbuf = FRAMReadWord(memLocSetpointBufferStart + 6, 0);
    // for now if we reach max capacity, we stop writing
    head = FRAMReadWord(memLocSetpointBufferStart, 0 );
    capacity = FRAMReadWord(memLocSetpointBufferStart + 4, 0);
    // reached max capacity return
    if (count_in_tempbuf >= capacity) {
        return;
    }
    if (count_in_tempbuf == 0)
    {
        // write the current timestamp to rtc time base
        RTCC_TimeGet(&base_time);
        data = base_time.tm_year;
        FRAMWriteByte(data, memLocSetpointBufferStart + 10, 0);
        data = base_time.tm_mon;
        FRAMWriteByte(data, memLocSetpointBufferStart + 11, 0);
        data = base_time.tm_mday;
        FRAMWriteByte(data, memLocSetpointBufferStart + 12, 0);
        data = base_time.tm_hour;
        FRAMWriteByte(data, memLocSetpointBufferStart + 13, 0);
        data = base_time.tm_min;
        FRAMWriteByte(data, memLocSetpointBufferStart + 14, 0);
        u32data = 0x00;
        
    }
    else 
    {
        RTCC_TimeGet(&curr_time);
        // read the rtc time base from buffer
        data = FRAMReadByte(memLocSetpointBufferStart + 10, 0);
        base_time.tm_year = (int)data;
        data = FRAMReadByte(memLocSetpointBufferStart + 11, 0);
        base_time.tm_mon = (int)data;
        data = FRAMReadByte(memLocSetpointBufferStart + 12, 0);
        base_time.tm_mday = (int)data;
        data = FRAMReadByte(memLocSetpointBufferStart + 13, 0);
        base_time.tm_hour = (int)data;
        data = FRAMReadByte(memLocSetpointBufferStart + 14, 0);
        base_time.tm_min = (int)data;
        base_time.tm_sec = 0;
        base_t = mktime(&base_time);
        curr_t = mktime(&curr_time);
        diff_time = difftime(curr_t, base_t);
        if (diff_time < 0)
        {
            diff_time = -diff_time;
        }
        diff_time = diff_time / 60.0; // convert to minutes;
        if (diff_time > (double)0xFFFF)
        {
            u32data = 0;
        }
        else 
        {
            u32data =(int) diff_time;
        }
    }
    if ( (u32data == 0) && ( count_in_tempbuf  > 0) )
    {
        // cannot report data since not enough space to store time
        return;
    }
    //offset = count_in_tempbuf * 4 + 15; // memLocTemperatureBufferStart;
    offset = head * 4 + 15;
    tmpu16 = (head + 1) % capacity;
    data = (uint8_t)(u32data & 0xFF); // Little endian format
    FRAMWriteByte(data, memLocSetpointBufferStart + offset, 0 );
    data = (uint8_t)( (u32data & 0xFF00) >> 8); // Little endian format
    FRAMWriteByte(data, memLocSetpointBufferStart + offset + 1, 0);
    data = u8NewCoolSetpoint;
    FRAMWriteByte(data, memLocSetpointBufferStart + offset + 2, 0);
    data = u8NewHeatSetpoint;
    FRAMWriteByte(data, memLocSetpointBufferStart + offset + 3, 0);
    // write number_of_items + 1 to fram
    FRAMWriteWord(tmpu16, memLocSetpointBufferStart, 0);
    tmpu16 = (uint16_t)(count_in_tempbuf + 1);
    FRAMWriteWord(tmpu16, memLocSetpointBufferStart + 6, 0);
}

/**
 * ReadSetpointBuffer
 *  arguments 
 *      number_of_samples to read
 *      data_buf(uint8_t *) for storing return elements;this must be allocated before the call
 *      data_buf_len: length of buffer created
 * 
 *  return value:
 *      int: 
 *          number of uint8_t bytes written to data_buf if successful
 *          -1 if data_buf_len is less than the required length for number of elements
 *          -2 if temperature buffer is empty
 *          -3 if buffer in FRAM has not been initialized
 * 
 *  format of data_buf returned
 *      first 5 bytes base timestamp as stored in FRAM
 *      4 BYTES per sample each
 *               
 */
int ReadSetpointBuffer(int number_of_samples, uint8_t * data_buf, int data_buf_len, int start_idx)
{
    uint8_t data;
    int buf_ret_count = 0;
    uint16_t count_in_tempbuf;
    int ii;
    uint16_t tail;
    uint16_t capacity;
    int offset;
    int tmp_count_idx;
    
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return -3;
    }
    if ( (number_of_samples * 4 + 6) > ( data_buf_len - start_idx ) ) 
    {
        // insufficient space to return data
        return -1;
    }
    count_in_tempbuf = FRAMReadWord(memLocSetpointBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return -2;
    }
    tail = FRAMReadWord(memLocSetpointBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocSetpointBufferStart + 4, 0);
    
    buf_ret_count = 0;
    data = FRAMReadByte(memLocSetpointBufferStart + 10, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocSetpointBufferStart + 11, 0);
    data_buf[start_idx + buf_ret_count] = data; 
    buf_ret_count ++;
    data = FRAMReadByte(memLocSetpointBufferStart + 12, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocSetpointBufferStart + 13, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    data = FRAMReadByte(memLocSetpointBufferStart + 14, 0);
    data_buf[start_idx + buf_ret_count] = data;
    buf_ret_count ++;
    tmp_count_idx = buf_ret_count;
    buf_ret_count ++;
    ii = 0;
    while ( (ii < number_of_samples) && (count_in_tempbuf > 0) )
    {
        offset = tail * 4 + 15;
        data = FRAMReadByte(memLocSetpointBufferStart + offset, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocSetpointBufferStart + offset+1, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocSetpointBufferStart + offset+2, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        data = FRAMReadByte(memLocSetpointBufferStart + offset+3, 0);
        data_buf[start_idx + buf_ret_count] = data;
        buf_ret_count ++;
        tail = (tail + 1) % capacity;
        count_in_tempbuf --;
        ii++;
    }
    data_buf[start_idx + tmp_count_idx] = (uint8_t) ii;
    return ii;
}


/**
 * 
 * @param num_of_items
 * Delete from FRAM num_of_items from setpoint Buffer
 *      since they were successfully transmitted
 */
void DeleteEntriesSetpointBuffer(int num_of_items)
{
    uint16_t count_in_tempbuf;
    uint16_t tail;
    uint16_t capacity;
    count_in_tempbuf = FRAMReadWord(memLocSetpointBufferStart + 6, 0);
    if ( count_in_tempbuf == 0)
    {
        return;
    }
    tail = FRAMReadWord(memLocSetpointBufferStart + 2, 0);
    capacity = FRAMReadWord(memLocSetpointBufferStart + 4, 0);
    if (count_in_tempbuf < num_of_items)
    {
        tail = (tail + count_in_tempbuf) % capacity;
        count_in_tempbuf = 0;
    }
    else 
    {
        count_in_tempbuf -= num_of_items;
        tail = (tail + num_of_items) % capacity;
    }
    // write new tail to fram
    FRAMWriteWord(count_in_tempbuf, memLocSetpointBufferStart + 6,    0);
    FRAMWriteWord(tail, memLocSetpointBufferStart + 2, 0);
    if (count_in_tempbuf == 0)
    {
        // clear the timestamp in temperature buffer as all elements are empty
        FRAMWriteByte(0, memLocSetpointBufferStart + 10, 0);
        FRAMWriteByte(0, memLocSetpointBufferStart + 11, 0);
        FRAMWriteByte(0, memLocSetpointBufferStart + 12, 0);
        FRAMWriteByte(0, memLocSetpointBufferStart + 13, 0);
        FRAMWriteByte(0, memLocSetpointBufferStart + 14, 0);
    }
    
}

/**
 * 
 * @return the number of elements stored in temperature buffer
 */
uint16_t GetItemCountSetpointBuffer() 
{
    uint16_t count_in_tempbuf;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    count_in_tempbuf = FRAMReadWord(memLocSetpointBufferStart + 6, 0);
    return count_in_tempbuf;
}

/**
 * 
 * @return the number of elements stored in temperature buffer
 */
uint16_t GetCapacitySetpointBuffer() 
{
    uint16_t capacity;
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data != BufferInitializedWriteValue)
    {
        return 0;
    }
    capacity = FRAMReadWord(memLocSetpointBufferStart + 4, 0);
    return capacity;
}



void ParametersInit()
{
    FRAMWriteByte(0x01, memLocHysteresis, 0);
    FRAMWriteByte(MODE_HEATCOOL, memLocThermostatMode, 0);
    FRAMWriteByte(COOL_SETPOINT_DEFAULT_DEGF, memLocCoolSetpoint, 0);
    FRAMWriteByte(HEAT_SETPOINT_DEFAULT_DEGF, memLocHeatSetpoint, 0);
    FRAMWriteByte(SCHEDULE_NONE, memLocPreviousSchedule, 0);
}

/**
 * Reads the FRAM to check if schedule has been received by reading the 
 *  bitmaps at specific locations
 * @return 1 if all required bitmpas are set
 * @return 0 otherwise
 */
uint8_t ScheduleAvailable()
{
    uint8_t tmpu8;
    uint32_t tmpu32;
    tmpu8 = FRAMReadByte(memLocScheduleStart, 0);
    if ( (tmpu8 & 0x07 ) == 0x07 )
    {
        tmpu32 = FRAMReadLong(memLocScheduleStart+1, 0);
        if (tmpu32 == 0xFFFFFFFF)
        {
            return 1;
        }
        else 
        {
            return 0;
        }
    }
    else 
    {
        return 0;
    }
}

/**
 * loads the schedule to u8arrSchedule from FRAM
 * This function does not check if schedule is available in FRAM or not
 * Use the Function "ScheduleAvailable" to check that a schedule is available and 
 * then call this function to load the schedule into the u8arrSchedule array
 */
void LoadScheduleFromFRAM()
{
    FRAMReadByteArray(memLocScheduleStart + 5, 0, u8arrSchedule, 140);
} 

/**
 * Reads the FRAM to check if DRSchedule has been received by reading the 
 *  bitmap at the specified location
 * @return 1 if if available
 * @return 0 otherwise
 */
uint8_t DRScheduleAvailable()
{
    uint8_t tmpu8;
    tmpu8 = FRAMReadByte(memLocScheduleStart, 0);
    if ( (tmpu8 & 0x08 ) == 0x08 )
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}



/**
 * Writes the away schedule to FRAM
 * @param th_mode
 * @param cool_sp
 * @param heat_sp
 */
void WriteAwayScheduleMode(uint8_t th_mode, uint8_t cool_sp, uint8_t heat_sp)
{
    FRAMWriteByte(th_mode, memLocAwayThermostatMode, 0);
    FRAMWriteByte(cool_sp, memLocAwayCoolSetpoint, 0);
    FRAMWriteByte(heat_sp, memLocAwayHeatSetpoint, 0);
    
}

/**
 * Reads the away schedule mode and setpoints
 * and populates the appropriate variable 
 *      u8ThermostatMode, u8CoolSetpoint, u8HeatSetpoint
 */
void ReadAwayScheduleMode()
{
    uint8_t tmpu8;
    tmpu8 = FRAMReadByte(memLocAwayThermostatMode, 0);
    u8ThermostatMode = tmpu8;
    tmpu8 = FRAMReadByte(memLocAwayCoolSetpoint, 0);
    u8CoolSetpoint = tmpu8;
    tmpu8 = FRAMReadByte(memLocAwayHeatSetpoint, 0);
    u8HeatSetpoint = tmpu8;
}
