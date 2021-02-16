/*
 * File:   system.c
 * Author: Madhu
 *
 * Created on January 17, 2019, 8:05 AM
 */


#include <xc.h>
#include "main.h"
#include "hal.h"
#include "bme680.h"
#include "memory.h"
#include "system.h"
#include "FRAM.h"
#include "rtcc.h"


/******************************************************************************
 *          BME680 related
 ******************************************************************************/
struct bme680_dev gas_sensor;
uint16_t meas_period;    
struct bme680_field_data data;

void ConfigureBME680(void)
{
    uint8_t set_required_settings;
    int8_t rslt = BME680_OK;
    
    gas_sensor.dev_id = 0;
    gas_sensor.intf = BME680_SPI_INTF;
    gas_sensor.read = BME680_spi_read;
    gas_sensor.write = BME680_spi_write;
    gas_sensor.delay_ms = BME680_delay_ms;
    gas_sensor.amb_temp = 25; //can also be set after getting a few readings
    
    rslt = bme680_init(&gas_sensor);
    
    //Setting up the TPHG configuration registers:
    /* Set the temperature, pressure and humidity settings */
    gas_sensor.tph_sett.os_hum = BME680_OS_2X;
    gas_sensor.tph_sett.os_pres = BME680_OS_4X;
    gas_sensor.tph_sett.os_temp = BME680_OS_8X;
    gas_sensor.tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS; //BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor.gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor.gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor.power_mode = BME680_FORCED_MODE; 

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL;

    
    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings,&gas_sensor);

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(&gas_sensor);
    
    bme680_get_profile_dur(&meas_period, &gas_sensor);
}

void BME680_delay_ms(uint32_t period)
{
    __delay_ms(period);
}

int8_t BME680_spi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    uint16_t i = 0;
   
    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (reg_data[0])       | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   | (reg_data[len - 1]) | LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */

    if (dev_id == 0) //if we have more than one BME680, the CS line will be selected here
    {
        CS_BME680_SetLow();
    }
    
    SPI1_Write8bit(reg_addr);
    
    for (i=0;i<len;i++)
    {
        reg_data[i] = SPI1_Exchange8bit(0);
    }
    
    if (dev_id == 0) //if we have more than one BME680, the CS line will be selected here
    {
        CS_BME680_SetHigh();
    }
    return rslt;
}

int8_t BME680_spi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
     int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
     uint16_t i = 0;
     
    /*
     * Data on the bus should be like
     * |---------------------+--------------+-------------|
     * | MOSI                | MISO         | Chip Select |
     * |---------------------+--------------|-------------|
     * | (don't care)        | (don't care) | HIGH        |
     * | (reg_addr)          | (don't care) | LOW         |
     * | (reg_data[0])       | (don't care) | LOW         |
     * | (....)              | (....)       | LOW         |
     * | (reg_data[len - 1]) | (don't care) | LOW         |
     * | (don't care)        | (don't care) | HIGH        |
     * |---------------------+--------------|-------------|
     */
    if (dev_id == 0) //if we have more than one BME680, the CS line will be selected here
    {
        CS_BME680_SetLow();
    }
    
    SPI1_Write8bit(reg_addr);
    
    for (i=0;i<len;i++)
    {
        SPI1_Write8bit(reg_data[i]);
    }
    
    if (dev_id == 0) //if we have more than one BME680, the CS line will be selected here
    {
        CS_BME680_SetHigh();
    }
    return rslt;
    
}
   
    
/**
 * Reads Temperature from U4 : AN10 OR U6: AN11
 * Board edge is location 0x0A, board center is 0x0B
 * @return 
 */
float ReadBoardTemperature(uint8_t location)
{
    uint16_t adcData = 0;
    float voltage = 0;
    float temperature = 0;
    uint8_t i = 0;
    //uint8_t j = 0; // unused variable
//    uint16_t adcData_arr[10];
//    uint16_t minAdcData = 0xFFFF;
//    uint16_t maxAdcData = 0;
//    uint8_t maxAdcCount, minAdcCount;
    //Turning on LMT70 power and enabling ADC:
    // AD1CON1bits.ADON=1;
    // _LATC15 = 1;
    // __delay_us(500); //for LMT70 power to stabilize
    //0x0A is T1ADC - board edge, 0x0B is T2ADC 
    for (i=0; i<= 9; i++) 
    {
//        adcData_arr[i] = ADC1_ReadChannel(location);
//        if (adcData_arr[i] > maxAdcData)
//        {
//            maxAdcData = adcData_arr[i];
//            maxAdcCount = 1;
//        }
//        else if (adcData_arr[i] == maxAdcData)
//        {
//            maxAdcCount ++;
//        }
//        if (adcData_arr[i] < minAdcData)
//        {
//            minAdcData = adcData_arr[i];
//            minAdcCount = 1;
//        }
//        else if (adcData_arr[i] == minAdcCount)
//        {
//            minAdcCount ++;
//        }
        adcData = adcData + ADC1_ReadChannel(location);
    }
    adcData = adcData / 10;
//    if (minAdcCount > 1) 
//    {
//        minAdcData = 0; // do not discard minADCvalues
//    }
//    if (maxAdcCount > 1) {
//        maxAdcData = 0; // do not discard maxAdcValue
//    }
//            
//    j = 0;
//    adcData = 0;
//    for (i=0; i <= 9; i++)
//    {
//        if ( (adcData_arr[i] != minAdcData) && (adcData_arr[i] != maxAdcData) )
//        {
//            adcData += adcData_arr[i];
//            j++;
//        }
//    }
//    adcData = adcData / j;
    
    
    
    //converting to voltage:
    //voltage = ( (float)adcData * 3300 ) / 4096.0; //in mv
    //in second batach of board voltage is 3.350
    voltage = ( (float)adcData * 3350 ) / 4096.0; //in mv
    
    //Using the third order equation (d.s page # 13)
    
    temperature = -0.000007857923 * voltage * voltage -
                  0.1777501 * voltage + 
                  204.6398;
    //getting rid of the decimal
    //temperature = floor(temperature);
    //converting Centigrade to deg F:
    temperature = (temperature * 9 / 5.0) + 32;
    
    //turning off  ADC and LMT70:
    //AD1CON1bits.ADON = 0;
    // _LATC15 = 0;
    return temperature;    
   
}

/**
 * Reads battery voltage sensed on AN13
 * 
 * @return voltage in mv
 */
uint16_t ReadBatteryVoltage (void)
{
    uint16_t adcData = 0;
    uint8_t i = 0;
    float battVoltage = 0;
    __delay_ms(1); //for voltage to stabilize after wakeup
    //AD1CON1bits.ADON=1;
    for (i=0; i<= 9; i++) 
    {
        adcData = adcData + ADC1_ReadChannel(0x0D);
    }
    adcData = adcData / 10;

    //AD1CON1bits.ADON=0;
    battVoltage = ((float)adcData / 4096.0 ) * 3300 ; //in mv
    
    return (uint16_t)battVoltage;    
}

/**
 * This function reads the current time from RTCC 
 * and populates the u8NewHeatSetpoint and u8NewCoolSetpoint based on schedule
 * and sets the i32DurationMins variable based on time remaining from now to next schedule
 * This function assumes that schedules are available and does not check
 * The calling function must check that before calling this function
 */
void SetScheduleForNow()
{
    struct tm curr_time;
    int ii;
    int tmpi;
    int tmpj;
    int tmpk;
    int tmpl;
    int start_idx;
    int last_idx;
    int32_t tmpi32;
    bool done = false;
    if (rtccTimeInitialized == false) 
    {
        // since rtcc has not been initialized, we cannot set schedule
        return;
    }
    RTCC_TimeGet(&curr_time);
    //ReadSchedule(curr_time.tm_wday);
    start_idx = curr_time.tm_wday*20;
    ii = CountSchedulesInWday(curr_time.tm_wday*20);
    if (ii == 0)
    {
        // Set CSP and HSP from an available previous schedule
        // Set duration from now to next available schedule
        tmpi = FindPreviousSchedule(start_idx);
        if (tmpi > 0) // found valid schedule
        {
            u8NewCoolSetpoint = u8arrSchedule[tmpi+2];
            u8NewHeatSetpoint = u8arrSchedule[tmpi+3];
        }
        tmpi = (24*60) - (curr_time.tm_hour*60 + curr_time.tm_min);
        tmpi32 = ComputeDurationNextDaySchedule(tmpi, start_idx);
        if (tmpi32 > 0)
        {
            i32DurationMins = tmpi32;
        }
        else
        {
            // this should never happen with valid schedules, it it happens, wierd case, set it to 3 hours
            i32DurationMins = 3 * 60;
        }
    }
    else if (ii == 1)
    {
        /*
        If current time is before schedule begins, then
            set CSP and HSP from an available previous schedule
            set duration is from now to the only existing schedule
        Else // implies current time is after or at the current schedule
            set CSP and HSP from only existing schedule
            set duration from now to next available schedule
         */
        tmpi = (curr_time.tm_hour * 60) + curr_time.tm_min;
        tmpj = (u8arrSchedule[start_idx]*60) + u8arrSchedule[start_idx + 1];
        if (tmpi < tmpj)
        {
            tmpi = FindPreviousSchedule(start_idx);
            if (tmpi > 0) // found valid schedule
            {
                u8NewCoolSetpoint = u8arrSchedule[tmpi+2];
                u8NewHeatSetpoint = u8arrSchedule[tmpi+3];
            }
            i32DurationMins = tmpj - tmpi;
        }
        else
        {
            u8NewCoolSetpoint = u8arrSchedule[start_idx+2];
            u8NewHeatSetpoint = u8arrSchedule[start_idx+3];
            tmpi = (24*60) - (curr_time.tm_hour*60 + curr_time.tm_min);
            tmpi32 = ComputeDurationNextDaySchedule(tmpi, start_idx);
            if (tmpi32 > 0)
            {
                i32DurationMins = tmpi32;
            }
            else
            {
                // this should never happen with valid schedules, if it happens, wierd case, set it to 3 hours
                i32DurationMins = 3 * 60;
            }
        }

    }
    else
    {
        // there can be 2 to 5 schedules
        tmpi = (curr_time.tm_hour * 60) + curr_time.tm_min;
        tmpj = (u8arrSchedule[start_idx]*60) + u8arrSchedule[start_idx + 1];
        last_idx = start_idx + (ii-1) * 4;
        tmpk = (u8arrSchedule[last_idx]*60) + u8arrSchedule[last_idx + 1];
        if (tmpi < tmpj)
        {
            tmpi = FindPreviousSchedule(start_idx);
            if (tmpi > 0) // found valid schedule
            {
                u8NewCoolSetpoint = u8arrSchedule[tmpi+2];
                u8NewHeatSetpoint = u8arrSchedule[tmpi+3];
            }
            i32DurationMins = tmpj - tmpi;
        }
        else if (tmpi >= tmpk)
        {
            u8NewCoolSetpoint = u8arrSchedule[last_idx+2];
            u8NewHeatSetpoint = u8arrSchedule[last_idx+3];
            tmpi = (24*60) - (curr_time.tm_hour*60 + curr_time.tm_min);
            tmpi32 = ComputeDurationNextDaySchedule(tmpi, start_idx);
            if (tmpi32 > 0)
            {
                i32DurationMins = tmpi32;
            }
            else
            {
                // this should never happen with valid schedules, if it happens, wierd case, set it to 2 hours
                i32DurationMins = 3 * 60;
            }

        }
        else
        {
            tmpj = start_idx;
            while ( (tmpj < last_idx) && (!done) )
            {
                tmpk = u8arrSchedule[tmpj]*60 + u8arrSchedule[tmpj+1]; // time for tmpj index (schedule)
                tmpl = u8arrSchedule[tmpj+4]*60 + u8arrSchedule[tmpj+4+1]; // time for next tmpj index (schedule))
                if ( (tmpi >= tmpk) && (tmpi < tmpl) )
                {
                    u8NewCoolSetpoint = u8arrSchedule[tmpj+2];
                    u8NewHeatSetpoint = u8arrSchedule[tmpj+3];
                    i32DurationMins = tmpl - tmpi;
                    done = true;
                }
                tmpj += 4;
            }
            if (done == false)
            {
                // data in schedule could be wrong
                // keep the current set points
                // set the duration to 6 hours
                i32DurationMins = 60*6;
            }
        }
    }
    
}

void ReadParametersFromFRAM()
{
    // TODO: Implement the perameters that need to be read from FRAM on power up/ battery change
    // if RTCC is set, 
    //      if u8Schedule is home, read the schedule from FRAM, else use default values
    //      else u8Schedule is Away, read the setpoints from FRAM, and set them up
    // if setpoints are not in range, check and set them up appropriately
    uint8_t tmpu8;
    ReadRTCCInitializedMemory();
    //if (rtccTimeInitialized == true)
    //{
        // if schedule is SCHEDULE_AWAY, the u8CoolSetpoint and u8HeatSetpoint 
        //  are already set by the ReadSchedule() function as it is 
        //  read from FRAM
    //    if (u8Schedule == SCHEDULE_HOME)
    //    {
            // need to read and populate 
            tmpu8 = ScheduleAvailable();
            if (tmpu8 == 1)
            {
                // schedule is available in FRAM, read and populate 
                LoadScheduleFromFRAM();
    //            SetScheduleForNow();
            }
    //    }
    //    else if (u8Schedule == SCHEDULE_DREVENT)
    //    {
            ReadDRSchedule();
    //    }
    //}
}


/**
 * Initialize variables for first time initialization
 * 
 */
void Application_Initialize(void) 
{
    uint8_t data;
    data = ReadBuffersInitialized();
    if (data == (uint8_t)BufferInitializedWriteValue)
    {
        ReadThermostatMode();
        if ( (u8ThermostatMode < (uint8_t)MODE_HEATCOOL ) || ( u8ThermostatMode > (uint8_t)MODE_COOL_ONLY ) )
        {
            // default value is heat and cool
            u8ThermostatMode = (uint8_t) MODE_HEATCOOL;
        }
        
        
        ReadHysteresis();
        if ( (u8Hysteresis > 4) || (u8Hysteresis < 1) ) {
            u8Hysteresis = 1;
        }
        u8TimerMinutesToOff = 0;
    }
    else
    {
        // default values
        u8ThermostatMode = (uint8_t) MODE_HEATCOOL;
        u8Hysteresis = 1;
    }
    u8TimerMinutesToOff = 0;
    u8CoolSetpoint = (uint8_t) COOL_SETPOINT_DEFAULT_DEGF;
    u8NewCoolSetpoint = COOL_SETPOINT_DEFAULT_DEGF;
    u8SchNoneCoolSetpoint = COOL_SETPOINT_DEFAULT_DEGF;
    u8HeatSetpoint = (uint8_t) HEAT_SETPOINT_DEFAULT_DEGF;
    u8NewHeatSetpoint = HEAT_SETPOINT_DEFAULT_DEGF;
    u8SchNoneHeatSetpoint = HEAT_SETPOINT_DEFAULT_DEGF;
    // default in none
    // algorithm will set it up to heat/ cool
    u8CurrentThermostatAction = (uint8_t) ACTION_NONE;
    u8ApplicationState = (uint8_t)APP_STATE_ON;
    u16PartialScreenUpdateCount = 0;
    u8TimerOnScreen = 0;
    u8OffIconOnScreen = 0;
    for (data = 0; data < TEMPERATURE_AVERAGE_MAX_ELEMENTS; data++)
    {
        fTempArr[data] = 0.0f;
    }
    fTempSum = 0;
    fTempAvg = 0.0;
    fTempAvgPrev = 0.0;
    u8TempArrIndex = 0;
    u8TempArrCount = 0;
    u8RelativeHumidity = 0;
    u8Schedule = SCHEDULE_NONE;
    u8GotoSchedule = 0xFF;
    u8PlusMinusOnScreen = (uint8_t)INCDEC_STATE_NONE;
    u8PlusMinusDispTimeoutMins = 0; //(uint8_t)INCDEC_DISP_TIMEOUT_MINS;
    fKalman_xkp = 0;
    fKalman_pkp = 1;
    fKalman_gain = 1;
    u8UseMovAverage = 1;
    bDRScheduleRead = false;
}

void Heat_off(void)
{
    HEAT_RESET_SetHigh();
    __delay_ms(100);
    HEAT_RESET_SetLow();
}

void Heat_on(void)
{
    HEAT_SET_SetHigh();
    __delay_ms(100);
    HEAT_SET_SetLow();
}

void Cool_off(void) 
{
    COOL_RESET_SetHigh();
    __delay_ms(100);
    COOL_RESET_SetLow();
}

void Cool_on(void)
{
    COOL_SET_SetHigh();
    __delay_ms(100);
    COOL_SET_SetLow();
}

void Fan_off(void)
{
    FAN_RESET_SetHigh();
    __delay_ms(100);
    FAN_RESET_SetLow();
}

void Fan_on(void)
{
    FAN_SET_SetHigh();
    __delay_ms(100);
    FAN_SET_SetLow();
}

uint8_t Temperature_average(float current_read_temperature)
{
    uint8_t tmpu8;
    float tmpf;
    fTempSum = fTempSum - fTempArr[u8TempArrIndex];
    fTempArr[u8TempArrIndex] = current_read_temperature;
    fTempSum = fTempSum + current_read_temperature;
    
    u8TempArrIndex ++;
    if (u8TempArrIndex >= TEMPERATURE_AVERAGE_MAX_ELEMENTS)
    {
        u8TempArrIndex = 0;
    }
    if (u8TempArrCount < TEMPERATURE_AVERAGE_MAX_ELEMENTS)
    {
        u8TempArrCount++;
    }
    //fTempAvgPrev = fTempAvg;
    fTempAvg = ( fTempSum  / (1.0 * u8TempArrCount ) );
    // if new average is in dead band of <int>.35 to <int>.65 use the previous tempavg for computation
    // and do not store the computed value in the previous value
    tmpf = fTempAvg - (int)fTempAvg;
    if ( (fTempAvgPrev > 12.0) && ( (tmpf > 0.35) && (tmpf < 0.65 ) ) )
    {
        tmpu8 = (uint8_t) (fTempAvgPrev + 0.5);
    }
    else
    {
        tmpu8 = (uint8_t)(fTempAvg + 0.5); // simple round function
        fTempAvgPrev = fTempAvg;
    }
    // TODO: TO FIND THIS VALUE AFTER CALIBRATION
//    if (tmpu8 > 10)
//    {
//        //tmpu8 = tmpu8 - 1; // offset for open boards
//        tmpu8 = tmpu8 -2; // offset for units with enclosure
//    }
    
//    if (movAvg2ArrCount < MOVAVG_2_MAX_ELEMENTS)
//    {
//        movAvg2Arr[movAvg2ArrIndex] = tmpu8;
//        movAvg2ArrIndex += 1;
//        movAvg2Sum = movAvg2Sum + tmpu8;
//        tmpf = ((float)(movAvg2Sum * 1.0))/ movAvg2ArrIndex;
//        tmpu8 = (uint8_t)(tmpf + 0.5);
//        movAvg2ArrCount ++;
//        if (movAvg2ArrIndex >= MOVAVG_2_MAX_ELEMENTS)
//        {
//            movAvg2ArrIndex = 0;
//        }
//        
//    }
//    else 
//    {
//        movAvg2Sum = movAvg2Sum - movAvg2Arr[movAvg2ArrIndex];
//        movAvg2Sum = movAvg2Sum + tmpu8;
//        movAvg2Arr[movAvg2ArrIndex] = tmpu8;
//        movAvg2ArrIndex += 1;
//        tmpf = ((float)(movAvg2Sum * 1.0))/ 3.0;
//        tmpu8 = (uint8_t)(tmpf + 0.5);
//        if (movAvg2ArrIndex >= MOVAVG_2_MAX_ELEMENTS)
//        {
//            movAvg2ArrIndex = 0;
//        }
//    }
    
    return tmpu8;
}

uint8_t Temperature_average_with_kalman(float current_read_temperature)
{
    uint8_t tmpu8;
    float tmpf1, tmpf2;
    fTempSum = fTempSum - fTempArr[u8TempArrIndex];
    fTempArr[u8TempArrIndex] = current_read_temperature;
    fTempSum = fTempSum + current_read_temperature;
    
    if (u8UseMovAverage == 1)
    {
        u8TempArrIndex ++;
        if (u8TempArrIndex >= TEMPERATURE_AVERAGE_MAX_ELEMENTS)
        {
            u8TempArrIndex = 0;
        }
        if (u8TempArrCount < TEMPERATURE_AVERAGE_MAX_ELEMENTS)
        {
            u8TempArrCount++;
        }
        fTempAvg = ( fTempSum  / (1.0 * u8TempArrCount ) );

        tmpu8 = (uint8_t)(fTempAvg + 0.5); // simple round function
        if (tmpu8 > 10)
        {
            tmpu8 = tmpu8 - 1; // offset
        }
        tmpf1 = Temperature_estimate_kalman(current_read_temperature);
        tmpf2 = tmpf1 - fTempAvg;
        if (tmpf2 < 0)
        {
            tmpf2 = (-1) * tmpf2;
        }
        if (tmpf2 < 0.1)
        {
            u8UseMovAverage = 0;
            tmpu8 = (uint8_t)(tmpf1 + 0.5); // simple round function
        }
    }
    else 
    {
        tmpf1 = Temperature_estimate_kalman(current_read_temperature);
        tmpu8 = (uint8_t)(tmpf1 + 0.5); // simple round function
    }
    return tmpu8;
}



float Temperature_estimate_kalman(float current_read_temperature)
{
    float tmpxp_f;
    float tmppk_f;
    fKalman_gain = fKalman_pkp / (fKalman_pkp + KALMAN_R_FACTOR);
    tmpxp_f = fKalman_xkp + fKalman_gain * (current_read_temperature - fKalman_xkp);
    tmppk_f = (1 - fKalman_gain)* fKalman_pkp;

    fKalman_xkp = tmpxp_f;
    fKalman_pkp = tmppk_f;
    return tmpxp_f;
}


/**
 * Actions to be taken on entry to SCHEDULE_NONE state
 */
void OnEntryScheduleNone()
{
    u8NewCoolSetpoint = u8SchNoneCoolSetpoint;
    u8NewHeatSetpoint = u8SchNoneHeatSetpoint;
}

/**
 * Actions to be taken on exiting the SCHEDULE_NONE state
 */
void OnExitScheduleNone()
{
    u8SchNoneCoolSetpoint = u8CoolSetpoint;
    u8SchNoneHeatSetpoint = u8HeatSetpoint;
}

void DuringScheduleNone()
{

}

/**
 * Actions to be taken on entering the SCHEDULE_HOME state
 */
void OnEntryScheduleHome()
{
    i32DurationMins = -1;
}

void OnExitScheduleHome()
{
    // nothing needs to be done here
}

void DuringScheduleHome()
{
    if (i32DurationMins == -1)
    {
        SetScheduleForNow();
    }
    else
    {
        i32DurationMins --;
    }
}


void OnEntryScheduleDREvent()
{
    uint8_t tmpu8;
    u8PrevSchedule = u8Schedule;
    u8PrevCoolSetpoint = u8CoolSetpoint;
    u8PrevHeatSetpoint = u8HeatSetpoint;
//    u8NewCoolSetpoint = FRAMReadByte(memLocScheduleStart + 5 + 7*20 + 7, 0);
//    u8NewHeatSetpoint = FRAMReadByte(memLocScheduleStart + 5 + 7 * 20 + 6, 0);
    if (u8arrDrEvent[6] != 0) // heat setpoint
    {
        tmpu8 = u8arrDrEvent[6];
        if ( (tmpu8 & 0x80) == 0x80 )
        {
            tmpu8 = tmpu8 - 0x80;
            u8NewHeatSetpoint = u8HeatSetpoint - tmpu8;
        }
        else
        {
            u8NewHeatSetpoint = u8HeatSetpoint + tmpu8;
        }
    }
    if (u8arrDrEvent[7] != 0) // cool setpoint
    {
        tmpu8 = u8arrDrEvent[7];
        if ( (tmpu8 & 0x80) == 0x80 )
        {
            tmpu8 = tmpu8 - 0x80;
            u8NewCoolSetpoint = u8CoolSetpoint - tmpu8;
        }
        else
        {
            u8NewCoolSetpoint = u8CoolSetpoint + tmpu8;
        }
    }
    i32DurationMins = u8arrDrEvent[4] * 60 + u8arrDrEvent[5] + 1; // during is called right away, so i need to add a 1 here
}

void OnExitScheduleDREvent()
{
    u8GotoSchedule = u8PrevSchedule;
    u8NewCoolSetpoint = u8PrevCoolSetpoint;
    u8NewHeatSetpoint = u8PrevHeatSetpoint;
    bDRScheduleRead = false;
    ClearDRSchedule();
}


void DuringScheduleDREvent()
{
    if (i32DurationMins > 0)
    {
        i32DurationMins -= 1;
        if (i32DurationMins <= 0)
        {
            OnExitScheduleDREvent();
        }
    }
}

void OnEntryScheduleAway()
{
    u8PrevThermostatMode = u8ThermostatMode;
    u8ThermostatMode = u8AwayThermostatMode;
    u8NewHeatSetpoint = u8AwayHeatSetpoint;
    u8NewCoolSetpoint = u8AwayCoolSetpoint;
}

void OnExitScheduleAway()
{
    u8ThermostatMode = u8PrevThermostatMode;
}

void DuringScheduleAway()
{
    // nothing needs to be done for now
}

/**
 *
 * @param start_idx: start search backwards at this point
 * @return index of a valid prevous schedule, or
 *        -1 if no valid schedule
 */
int FindPreviousSchedule(int start_idx)
{
    int tmpi, tmpj, tmpk; // tmpk keeps track of schedules visited so that it does not go into an infinite loop
    tmpj = 0;
    tmpk = 0;
    tmpi = start_idx;
    if (start_idx >= 140) {
        return -1;
    }
    while ( (tmpj == 0) && (tmpk < 140) ) // if tmpk == 140 => all schedules have been searched.
    {
        tmpi = tmpi - 4;
        if (tmpi < 0)
        {
            tmpi = tmpi + 140;
        }
        if (tmpi != start_idx)
        {
            tmpj = u8arrSchedule[tmpi];
            tmpj += u8arrSchedule[tmpi+1];
            tmpj += u8arrSchedule[tmpi+2];
            tmpj += u8arrSchedule[tmpi+3];
            tmpk += 4;
        }
        else
        {
            break;
        }
    }
    if (tmpi == start_idx)
    {
        return -1;
    }
    else
    {
        return tmpi;
    }
}

/**
 * Find the duration to the next valid schedule which happens to be
 *    on the next day(s)
 * @param partial_mins : amount of minutes left in today's time
 * @param start_idx : index of wday(0-6) in the schedule array where
 *      the current processing is happening. So start searching for the next
 *      schedule starting at start_idx + 20, and ending at start_idx - 4
 * @return duration in minutes that include the value of partial_mins if a valid schedule is found
 *      return -1 if no valid schedule is found
 */
int32_t ComputeDurationNextDaySchedule(int32_t partial_mins, int start_idx)
{
    int32_t tmpi32;
    int tmpi, tmpj;
    tmpi32 = partial_mins;
    if (start_idx >= 140) {
        return -1;
    }
    tmpi = (start_idx + 20 ) % 140;
    tmpj = 0;
    while (tmpj == 0)
    {
        if (tmpi != start_idx)
        {
            tmpj = u8arrSchedule[tmpi];
            tmpj += u8arrSchedule[tmpi+1];
            tmpj += u8arrSchedule[tmpi+2];
            tmpj += u8arrSchedule[tmpi+3];
            if (tmpj == 0)
            {
                tmpi = (tmpi + 4) % 140;
                if (tmpi % 20 == 0)
                {
                    tmpi32 += (24*60);
                }
            }
        }
        else
        {
            break;
        }
    }
    if (tmpi == start_idx)
    {
        return -1;
    }
    else
    {
        tmpi32 += (u8arrSchedule[tmpi]*60 + u8arrSchedule[tmpi+1]);
        return tmpi32;
    }
}

/**
 * Search the set of 5 schedules (4 bytes each) starting at start_idx
 * @param start_idx
 * @return the number of non-zero schedules
 */
int CountSchedulesInWday(int start_idx)
{
    int tmpi, tmpj;
    int count;
    bool done = false;
    count = 0;

    if (start_idx >= 140)
    {
        return 0;
    }

    tmpi = 0;
    while (  ( tmpi < 20) && (!done) )
    {
        tmpj = u8arrSchedule[start_idx+tmpi];
        tmpj += u8arrSchedule[start_idx+tmpi+1];
        tmpj += u8arrSchedule[start_idx+tmpi+2];
        tmpj += u8arrSchedule[start_idx+tmpi+3];
        if (tmpj > 0)
        {
            count ++;
        }
        else
        {
            done = true;
        }
        tmpi += 4;
    }
    return count;
}

void ExecuteSchedule()
{
    struct tm curr_time;
    int tmp_curr_min;
    int tmp_start_min;
    int tmp_dur_min;
    if ( (u8GotoSchedule == 0xFF) && (u8Schedule != SCHEDULE_DREVENT) )
    {
        // check to see if there is a DR schedule pending, and if it is time, execute the schedule
        if (DRScheduleAvailable() == 1)
        {
            // check to see if it is to run now
            if (bDRScheduleRead == false)
            {
                ReadDRSchedule();
                bDRScheduleRead = true;
            }
            // is it time for schedule to run
            RTCC_TimeGet(&curr_time);
            if ( (curr_time.tm_mon == (int)u8arrDrEvent[0]) && 
                 (curr_time.tm_mday == (int)u8arrDrEvent[1])  
               )
            {
                tmp_curr_min = curr_time.tm_hour * 60 + curr_time.tm_min;
                tmp_start_min = u8arrDrEvent[2] * 60 + u8arrDrEvent[3];
                tmp_dur_min = u8arrDrEvent[4]* 60 + u8arrDrEvent[5];
                if ( (tmp_curr_min >= tmp_start_min) && (tmp_curr_min < (tmp_start_min + tmp_dur_min)))
                {
                    u8GotoSchedule = SCHEDULE_DREVENT;
                }
            }
        }
    }
    if (u8GotoSchedule != 0xFF)
    {
        // need to change schedule
        if (u8Schedule == SCHEDULE_NONE)
        {
            OnExitScheduleNone();
        }
        else if (u8Schedule == SCHEDULE_AWAY)
        {
            OnExitScheduleAway();
        }
        else if (u8Schedule == SCHEDULE_HOME)
        {
            OnExitScheduleHome();
        }
        else if (u8Schedule == SCHEDULE_DREVENT)
        {
            // OnExitSchduleDREvent has already been called, so need not call again
        }

        if (u8GotoSchedule == SCHEDULE_NONE)
        {
            OnEntryScheduleNone();
        }
        else if (u8GotoSchedule == SCHEDULE_AWAY)
        {
            OnEntryScheduleAway();
        }
        else if (u8GotoSchedule == SCHEDULE_HOME)
        {
            OnEntryScheduleHome();
        }
        else if (u8GotoSchedule == SCHEDULE_DREVENT)
        {
            OnEntryScheduleDREvent();
        }
        u8Schedule = u8GotoSchedule;
        u8GotoSchedule = 0xFF;
    }
    if (u8Schedule == SCHEDULE_NONE)
    {
        DuringScheduleNone();
    }
    else if (u8Schedule == SCHEDULE_AWAY)
    {
        DuringScheduleAway();
    }
    else if (u8Schedule == SCHEDULE_HOME)
    {
        DuringScheduleHome();
    }
    else if (u8Schedule == SCHEDULE_DREVENT)
    {
        DuringScheduleDREvent();
    }
}

void PopulateDebugBuffers(uint8_t * t_buf, uint8_t * r_buf, uint8_t * s_buf, int buf_len, int transmit_buf_len)
{
    // max capacity of buffer is buf_len
    // each buffer is transmitted over a characteristic of length 20, so number of transmissions is buf_len/ 20
    int num_trans = buf_len / transmit_buf_len;
    if ( (num_trans * transmit_buf_len) < buf_len )
    {
        num_trans += 1;
    }
    int num_elements = 0;//(transmit_buf_len - 6) / 4;
    int ii;
    //int jj; // unused variable
    int tmprand;
    int temp_start  = 10;
    int hum_start = 1;
    int t_idx, r_idx, s_idx;
    int element_idx = 0;
    uint8_t tm_yr = 119;
    uint8_t tm_mo = 3;
    uint8_t tm_dy = 19;
    uint8_t tm_hr = 16;
    uint8_t tm_mi = 36;
    t_idx = 0;
    while (t_idx < buf_len)
    {
        if ( (buf_len - t_idx) >= 10 )
        {
            tmprand = rand();
            t_buf[t_idx] = tm_yr;
            t_buf[t_idx+1] = tm_mo;
            t_buf[t_idx+2] = tm_dy;
            t_buf[t_idx+3] = tm_hr;
            t_buf[t_idx+4] = tm_mi;
            element_idx = t_idx + 5;
            //t_buf[t_idx+5] = (uint8_t)num_elements;
            t_idx += 6;
            ii = 0;
            num_elements = 0;
            while ( ( ( transmit_buf_len - (ii + 6) ) >= 4) && (t_idx < buf_len) )
            {
                t_buf[t_idx] = (uint8_t) (tmprand & 0xFF);
                t_buf[t_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
                tmprand = tmprand + 1;
                t_buf[t_idx+2] = (uint8_t) (temp_start);
                t_buf[t_idx+3] = (uint8_t) ( hum_start );
                t_idx += 4;
                ii += 4;
                num_elements ++;
            }
            t_buf[element_idx] = num_elements;
        }
        temp_start += 6;
        hum_start += 2;
    }

//    for (ii=0; ii < num_trans; ii++ ) {
//        // fill 20 byte buffer
//        tmprand = rand();
//        t_buf[t_idx] = tm_yr;
//        t_buf[t_idx+1] = tm_mo;
//        t_buf[t_idx+2] = tm_dy;
//        t_buf[t_idx+3] = tm_hr;
//        t_buf[t_idx+4] = tm_mi;
//        t_buf[t_idx+5] = (uint8_t)num_elements;
//        t_idx += 6;
//        for (jj = 0; jj < num_elements; jj++)
//        {
//            t_buf[t_idx] = (uint8_t) (tmprand & 0xFF);
//            t_buf[t_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
//            tmprand = tmprand + 1;
//            t_buf[t_idx+2] = (uint8_t) (temp_start);
//            t_buf[t_idx+3] = (uint8_t) ( hum_start );
//            t_idx += 4;
//        }
//        temp_start += 6;
//        hum_start += 2;
//        if (t_idx < transmit_buf_len)
//        {
//            for (jj=0; jj<(transmit_buf_len - t_idx); jj++)
//            {
//                t_buf[t_idx + jj] = 0;
//            }
//        }
//    }
    temp_start = 12;
    hum_start = 1;
    r_idx = 0;
    r_idx = 0;
    while (r_idx < buf_len)
    {
        if ( (buf_len - r_idx) >= 10 )
        {
            tmprand = rand();
            r_buf[r_idx] = tm_yr;
            r_buf[r_idx+1] = tm_mo;
            r_buf[r_idx+2] = tm_dy+1;
            r_buf[r_idx+3] = tm_hr;
            r_buf[r_idx+4] = tm_mi;
            element_idx = r_idx + 5;
            //t_buf[r_idx+5] = (uint8_t)num_elements;
            r_idx += 6;
            ii = 0;
            num_elements = 0;
            while ( ( ( transmit_buf_len - (ii + 6) ) >= 4) && (r_idx < buf_len) )
            {
                r_buf[r_idx] = (uint8_t) (tmprand & 0xFF);
                r_buf[r_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
                tmprand = tmprand + 1;
                r_buf[r_idx+2] = (uint8_t) (temp_start);
                r_buf[r_idx+3] = (uint8_t) ( hum_start );
                r_idx += 4;
                ii += 4;
                num_elements ++;
            }
            t_buf[element_idx] = num_elements;
        }
        temp_start += 6;
        hum_start += 2;
    }
//    for (ii=0; ii < num_trans; ii++ ) {
//        // fill 20 byte buffer
//        tmprand = rand();
//        r_buf[r_idx] = tm_yr;
//        r_buf[r_idx+1] = tm_mo;
//        r_buf[r_idx+2] = tm_dy+1;
//        r_buf[r_idx+3] = tm_hr;
//        r_buf[r_idx+4] = tm_mi;
//        r_buf[r_idx+5] = (uint8_t)num_elements; // only three elements
//        r_idx += 6;
//        for (jj=0; jj < num_elements; jj++)
//        {
//            r_buf[r_idx] = (uint8_t) (tmprand & 0xFF);
//            r_buf[r_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
//            tmprand = tmprand + 1;
//            r_buf[r_idx+2] = (uint8_t) (temp_start);
//            r_buf[r_idx+3] = (uint8_t) ( hum_start );
//            r_idx += 4;
//        }
//        temp_start += 6;
//        hum_start += 2;
//        if (r_idx < transmit_buf_len)
//        {
//            for (jj=0; jj<(transmit_buf_len - r_idx); jj++)
//            {
//                r_buf[r_idx + jj] = 0;
//            }
//        }
//    }
    temp_start = 15;
    hum_start = 1;
    s_idx = 0;
    s_idx = 0;
    while (s_idx < buf_len)
    {
        if ( (buf_len - s_idx) >= 10 )
        {
            tmprand = rand();
            s_buf[s_idx] = tm_yr;
            s_buf[s_idx+1] = tm_mo;
            s_buf[s_idx+2] = tm_dy+1;
            s_buf[s_idx+3] = tm_hr;
            s_buf[s_idx+4] = tm_mi;
            element_idx = s_idx + 5;
            //s_buf[s_idx+5] = (uint8_t)num_elements;
            s_idx += 6;
            ii = 0;
            num_elements = 0;
            while ( ( ( transmit_buf_len - (ii + 6) ) >= 4) && (s_idx < buf_len) )
            {
                s_buf[s_idx] = (uint8_t) (tmprand & 0xFF);
                s_buf[s_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
                tmprand = tmprand + 1;
                s_buf[s_idx+2] = (uint8_t) (temp_start);
                s_buf[s_idx+3] = (uint8_t) ( hum_start );
                s_idx += 4;
                ii += 4;
                num_elements ++;
            }
            s_buf[element_idx] = num_elements;
        }
        temp_start += 6;
        hum_start += 2;
    }
//    for (ii=0; ii < num_trans; ii++ ) {
//        // fill 20 byte buffer
//        tmprand = rand();
//        s_buf[s_idx] = tm_yr;
//        s_buf[s_idx+1] = tm_mo;
//        s_buf[s_idx+2] = tm_dy+1;
//        s_buf[s_idx+3] = tm_hr;
//        s_buf[s_idx+4] = tm_mi;
//        s_buf[s_idx+5] = num_elements; // only three elements
//        s_idx += 6;
//        for (jj=0; jj<num_elements; jj++)
//        {
//            s_buf[s_idx] = (uint8_t) (tmprand & 0xFF);
//            s_buf[s_idx+1] = (uint8_t) ( (tmprand & 0xFF00) >> 8);
//            tmprand = tmprand + 1;
//            s_buf[s_idx+2] = (uint8_t) (temp_start);
//            s_buf[s_idx+3] = (uint8_t) ( hum_start );
//            s_idx += 4;
//        }
//        if (s_idx < transmit_buf_len)
//        {
//            for (jj=0; jj<(transmit_buf_len - s_idx); jj++)
//            {
//                s_buf[s_idx + jj] = 0;
//            }
//        }
//        temp_start += 6;
//        hum_start += 2;
//    }

}

/**
 * Process schedules in the array and return number of schedules received
 * If 0xFF or end of schedule is received return 0xFF, else the count of schedules received
 * @param data
 * @param start_idx
 */
uint8_t ProcessSchedules(uint8_t * data, int start_idx)
{
    uint8_t tmpu8_count;
    uint8_t tmpu8_idx;
    uint8_t tmpu8_dow, tmpu8_num;
    bool exit_loop = false;
//#ifdef testble
//    int tmpi1;
//    tmpu8_idx = start_idx;
//    tmpu8_count = 0;
//    while ( ( tmpu8_idx < data[0] ) && ( exit_loop == false ) )
//    {
//        if (    (data[tmpu8_idx] <= 0x64) ||
//                (data[tmpu8_idx] == 0x0F) ||
//                (data[tmpu8_idx] == 0xFF) ||
//                (data[tmpu8_idx] == 0xFF)
//            )
//        {
//            if (data[tmpu8_idx] == 0x0F)
//            {
//                dbgSchedulesBLE[145] = data[tmpu8_idx+1];
//                dbgSchedulesBLE[146] = data[tmpu8_idx+2];
//                dbgSchedulesBLE[147] = data[tmpu8_idx+3];
//                dbgSchedulesBLE[148] = data[tmpu8_idx+4];
//
//            }
//            else if (data[tmpu8_idx] == 0xF0)
//            {
//                dbgSchedulesBLE[149] = data[tmpu8_idx+1];
//                dbgSchedulesBLE[150] = data[tmpu8_idx+2];
//                dbgSchedulesBLE[151] = data[tmpu8_idx+3];
//                dbgSchedulesBLE[152] = data[tmpu8_idx+4];
//            }
//            else if (data[tmpu8_idx] == 0xFF)
//            {
//                exit_loop = true;
//                tmpu8_count = 0xFF;
//            }
//            else
//            {
//                // valid schedule received
//                tmpu8_dow = (data[tmpu8_idx] & 0xF0) >> 4;
//                tmpu8_num = data[tmpu8_idx] & 0x0F;
//                tmpi1 = 5 + 20*tmpu8_dow + tmpu8_num * 4;
//                dbgSchedulesBLE[tmpi1] = data[tmpu8_idx+1];
//                dbgSchedulesBLE[tmpi1+1] = data[tmpu8_idx+2];
//                dbgSchedulesBLE[tmpi1+2] = data[tmpu8_idx+3];
//                dbgSchedulesBLE[tmpi1+3] = data[tmpu8_idx+4];
//                tmpu8_count += 1;
//            }
//        }
//        tmpu8_idx += 5;
//    }
//    return tmpu8_count;
//#else
    tmpu8_idx = start_idx;
    tmpu8_count = 0;
    while ( ( tmpu8_idx < data[0] ) && ( exit_loop == false ) )
    {
        if (    (data[tmpu8_idx] <= 0x64) ||
                (data[tmpu8_idx] == 0x0F) ||
                (data[tmpu8_idx] == 0xF0) ||
                (data[tmpu8_idx] == 0xFF)
            )
        {
            if (data[tmpu8_idx] == 0x0F)
            {
                u8arrDrEvent[0] = data[tmpu8_idx+1];
                u8arrDrEvent[1] = data[tmpu8_idx+2];
                u8arrDrEvent[2] = data[tmpu8_idx+3];
                u8arrDrEvent[3] = data[tmpu8_idx+4];
                tmpu8_count ++;
            }
            else if (data[tmpu8_idx] == 0xF0)
            {
                u8arrDrEvent[4] = data[tmpu8_idx+1];
                u8arrDrEvent[5] = data[tmpu8_idx+2];
                u8arrDrEvent[6] = data[tmpu8_idx+3];
                u8arrDrEvent[7] = data[tmpu8_idx+4];
#ifndef testble
                WriteDRSchedule();
#endif
                tmpu8_count ++;
            }
            else if (data[tmpu8_idx] == 0xFF)
            {
                exit_loop = true;
                tmpu8_count = 0xFF;
            }
            else
            {
                // valid schedule received
                tmpu8_dow = (data[tmpu8_idx] & 0xF0) >> 4;
                tmpu8_num = data[tmpu8_idx] & 0x0F;
                WriteSingleSchedule(tmpu8_dow, tmpu8_num,
                       data[tmpu8_idx+1], data[tmpu8_idx+2],
                        data[tmpu8_idx+3], data[tmpu8_idx+4]);
                tmpu8_count ++;
            }
        }
        tmpu8_idx += 5;
    }
    return tmpu8_count;
//#endif
}


/**
 * Load the temperature from the temperature buffer into the buffer for transmitting on BLE
 * @param data
 * @param data_len
 * @return num of data elements populated, and 0 if no data was populated
 */
uint8_t LoadTemperatureBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx)
{
    int tmpi2, tmpi1;
    int avail_data_len;
    avail_data_len = data_len - start_idx;
//#ifdef testble
//    // no more data avaiLable
//    if (dbgTemperatureBufIndex >= DBG_BUF_SIZE)
//    {
//        return 0;
//    }
//    
//    tmpi1 = dbgTemperatureBuffer[dbgTemperatureBufIndex + 5];
//    avail_data_len = 6 + tmpi1 * 4;
//    
//    if ( (dbgTemperatureBufIndex + avail_data_len) <= DBG_BUF_SIZE)
//    {
//        memcpy(&data[start_idx], &dbgTemperatureBuffer[dbgTemperatureBufIndex], avail_data_len);
//        dbgTemperatureBufIndex += avail_data_len;
//        return avail_data_len;
//    }
//    else
//    {
//        return 0;
//    }
//#else
    tmpi1 = (avail_data_len - 6) / 4; // number of samples that can be sent
    tmpi2 = GetItemCountTemperatureBuffer();
    if (tmpi2 > 0)
    {
        if (tmpi1 > tmpi2)
        {
            tmpi1 = tmpi2; // only tmpi2 samples are available
        }
        // else only tmpi1 items can be transfered in a single packet
        tmpi2 = ReadTemperatureBuffer(tmpi1, data, data_len, start_idx);    
        // return number of samples transmitted which is returned from ReadTemperatureBuffer
    }
    else
    {
        // no samples to return
        tmpi2 = 0;
    }
    return tmpi2; 
//#endif
}

/**
 * Load the runtime from the runtime buffer into the buffer for transmitting on BLE
 * @param data
 * @param data_len
 * @return num of data elements populated, and 0 if no data was populated
 */
uint8_t LoadRuntimeBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx)
{
    int tmpi2, tmpi1;
    int avail_data_len;
    avail_data_len = data_len - start_idx;
//#ifdef testble
//    // no more data avaiLable
//    if (dbgRuntimeBufIndex >= DBG_BUF_SIZE)
//    {
//        return 0;
//    }
//
//    tmpi1 =  dbgRuntimeBuffer[dbgRuntimeBufIndex + 5];
//    avail_data_len = 6 + tmpi1 * 4;
//    if ( (dbgRuntimeBufIndex + avail_data_len) <= DBG_BUF_SIZE )
//    {
//        memcpy(&data[start_idx], &dbgRuntimeBuffer[dbgRuntimeBufIndex], avail_data_len);
//        dbgRuntimeBufIndex += avail_data_len;
//        return avail_data_len;
//    }
//    else
//    {
//        return 0;
//    }
//#else
    tmpi1 = (avail_data_len - 6) / 4; // number of samples that can be sent
    tmpi2 = GetItemCountRuntimeBuffer();
    if (tmpi2 > 0)
    {
        if (tmpi1 > tmpi2)
        {
            tmpi1 = tmpi2; // only tmpi2 samples are available
        }
        tmpi2 = ReadRuntimeBuffer(tmpi1, data, data_len, start_idx);    
        // return number of samples transmitted which is returned from ReadSetpointBuffer
    }
    else
    {
        // no samples to return
        tmpi2 = 0;
    }
    return tmpi2; 
//#endif
}

/**
 * Load the set points from the setpoint buffer into the buffer for transmitting on BLE
 * @param data
 * @param data_len
 * @return num of data elements populated, and 0 if no data was populated
 */
uint8_t LoadSetpointBufferForBLE(uint8_t * data, int data_len, uint8_t start_idx)
{
    int tmpi2, tmpi1;
    int avail_data_len;
    avail_data_len = data_len - start_idx;
//#ifdef testble
//    // no more data avaiLable
//    if (dbgSetpointBufIndex >= DBG_BUF_SIZE)
//    {
//        return 0;
//    }
//    
//    tmpi1 = dbgSetpointBuffer[dbgSetpointBufIndex + 5];
//    avail_data_len = 6 + tmpi1 * 4;
//    if ( (dbgSetpointBufIndex + avail_data_len) <= DBG_BUF_SIZE )
//    {
//        memcpy(&data[start_idx], &dbgSetpointBuffer[dbgSetpointBufIndex], avail_data_len);
//        dbgSetpointBufIndex += avail_data_len;
//        return avail_data_len;
//    }
//    else
//    {
//        return 0;
//    }
//#else
    tmpi1 = (avail_data_len - 6) / 4; // number of samples that can be sent
    tmpi2 = GetItemCountSetpointBuffer();
    if (tmpi2 > 0)
    {
        if (tmpi1 > tmpi2)
        {
            tmpi1 = tmpi2; // only tmpi2 samples are available
        }
        tmpi2 = ReadSetpointBuffer(tmpi1, data, data_len, start_idx);    
        // return number of samples transmitted which is returned from ReadSetpointBuffer
    }
    else
    {
        // no samples to return
        tmpi2 = 0;
    }
    return tmpi2; 
//#endif
}


/**
 * this function processes the user parameters data in u8arrBLEChRx 
 * @param start_idx: first data byte in u8arrBLEChRx
 */
    void ProcessUserParameterSettingsData(uint8_t* data_arr, int start_idx) 
{
    int tmpi;
    uint8_t tmpu8;
    //u8UserParamProcessed = 1;
    tmpi = start_idx;
    if ( ( data_arr[tmpi] & 0x01) == 0x01 )
    {
        if ( ( (data_arr[tmpi+5] > 0) && (data_arr[tmpi+5] <= 60) ) && 
             ( ( u8ApplicationState == APP_STATE_ON ) || 
                ( u8ApplicationState == APP_STATE_TIMER)
              )
           )
        {
            u8TimerMinutesToOff = data_arr[tmpi+5];
            if (u8ApplicationState != (uint8_t) APP_STATE_TIMER)
            {
                u8ApplicationState = (uint8_t) APP_STATE_TIMER;
            }
        }
        else if ( (data_arr[tmpi+5] == 0xFF) && (u8ApplicationState == APP_STATE_OFF) )
        {
            u8ApplicationState = APP_STATE_ON;
            ReadAwayScheduleMode();
        }

    }
    if ( ( data_arr[tmpi] & 0x02) == 0x02 )
    {
        //TODO: do not write to FRAM buffer if the setpoint change occurred from app
        if ( ( (data_arr[tmpi+3] > MIN_SCALE_TEMPERATURE_DEGF) && (data_arr[tmpi+3] <= MAX_SCALE_TEMPERATURE_DEGF) ) && (u8ApplicationState == APP_STATE_ON) )
        {
            u8NewCoolSetpoint = data_arr[tmpi+3];
        }
        else 
        {
            u8NewCoolSetpoint = MAX_SCALE_TEMPERATURE_DEGF;
        }
        if ( ( (data_arr[tmpi+4] > MIN_SCALE_TEMPERATURE_DEGF) && (data_arr[tmpi+4] <= MAX_SCALE_TEMPERATURE_DEGF) ) && (u8ApplicationState == APP_STATE_ON) )
        {
            u8NewHeatSetpoint = data_arr[tmpi+4];
        }
        else 
        {
            u8NewHeatSetpoint = MIN_SCALE_TEMPERATURE_DEGF;
        }
    }
    if ( ( data_arr[tmpi] & 0x04) == 0x04 )
    {
        if ( (data_arr[tmpi+2] > 0) && (data_arr[tmpi+2] <= 3 ) )  
        {
            u8Hysteresis = data_arr[tmpi+2];
        }
    }                
    if ( ( data_arr[tmpi] & 0x08) == 0x08 )
    {
        if ( ( data_arr[tmpi+1] >= MODE_HEATCOOL ) && (data_arr[tmpi+1] <= MODE_COOL_ONLY) )
        {
            u8ThermostatMode = data_arr[tmpi+1];
            WriteThermostatMode();
        }
    }                
    if ( (data_arr[tmpi] & 0x10) == 0x10)
    {
        if (data_arr[tmpi+6] == SCHEDULE_AWAY)
        {
            if ( !( ( data_arr[tmpi+7] >= MODE_HEATCOOL ) && (data_arr[tmpi+7] <= MODE_COOL_ONLY) ) )
            {
                data_arr[tmpi+7] = MODE_HEATCOOL;
            }
            if ( !( (data_arr[tmpi+8] > MIN_SCALE_TEMPERATURE_DEGF) && (data_arr[tmpi+8] <= MAX_SCALE_TEMPERATURE_DEGF) )  )
            {
                data_arr[tmpi+8] = COOL_SETPOINT_DEFAULT_DEGF;
            }
            if ( !( (data_arr[tmpi+9] > MIN_SCALE_TEMPERATURE_DEGF) && (data_arr[tmpi+9] <= MAX_SCALE_TEMPERATURE_DEGF) ) )
            {
                data_arr[tmpi+9] = HEAT_SETPOINT_DEFAULT_DEGF;
            }
//            if ( !( (data_arr[tmpi+8] > MIN_COOL_SETPOINT_DEGF) && (data_arr[tmpi+8] <= MAX_COOL_SETPOINT_DEGF) )  )
//            {
//                data_arr[tmpi+8] = COOL_SETPOINT_DEFAULT_DEGF;
//            }
//            if ( !( (data_arr[tmpi+9] > MIN_HEAT_SETPOINT_DEGF) && (data_arr[tmpi+9] <= MAX_HEAT_SETPOINT_DEGF) ) )
//            {
//                data_arr[tmpi+9] = HEAT_SETPOINT_DEFAULT_DEGF;
//            }
            WriteAwayScheduleMode(data_arr[tmpi+7], data_arr[tmpi+8], data_arr[tmpi+9]);
            u8AwayThermostatMode = data_arr[tmpi+7];
            u8AwayCoolSetpoint = data_arr[tmpi+8];
            u8AwayHeatSetpoint = data_arr[tmpi+9];
        }
        if (u8ApplicationState == APP_STATE_ON)
        {
            tmpu8 = data_arr[tmpi+6];
            if ( (tmpu8 == SCHEDULE_HOME ) || (tmpu8  == SCHEDULE_NONE) || (tmpu8  == SCHEDULE_AWAY) )
            {
                if (u8Schedule != tmpu8)
                {
                    u8GotoSchedule = tmpu8;
                }
            }
        } 
//        else // u8ApplicationState == APP_STATE_OFF
//        {
//            // do nothing
//        }
    }
    
    if ( (data_arr[tmpi] & 0x20) == 0x20)
    {
        tmpu8 = data_arr[tmpi+10];
        if ( (tmpu8 == 0x01) && (u8ApplicationState == APP_STATE_OFF) )
        {
            u8ApplicationState = APP_STATE_ON;
        }
        else if ( (tmpu8 == 0x02) && 
                  ( (u8ApplicationState == APP_STATE_ON) ||
                    (u8ApplicationState == APP_STATE_TIMER)
                   )
                )
        {
            u8ApplicationState = APP_STATE_OFF;
            u8TimerMinutesToOff = 0;
        }
    }    
    
    if ( (data_arr[tmpi] & 0x40) == 0x40)
    {
        if ( data_arr[tmpi + 11] == 0x01)
        {
            if ( u8Schedule == SCHEDULE_DREVENT )
            {
                OnExitScheduleDREvent();
            }
            else 
            {
                // if there is a pending DR event, opt-out of that event
                ClearDRSchedule();
            }
        }
    }
}

void ResetFRAM() 
{
    uint8_t tmpu8;
    uint16_t tmpu16;
    ClearRTCCInitializedMemory();
    tmpu8 = u8CoolSetpoint;
    u8CoolSetpoint = 0;
    WriteCoolSetpoint();
    u8CoolSetpoint = tmpu8;
    tmpu8 = u8HeatSetpoint;
    u8HeatSetpoint = 0;
    WriteHeatSetpoint();
    u8HeatSetpoint = tmpu8;
    FRAMClearInitialization();
    tmpu16 = u16PartialScreenUpdateCount;
    u16PartialScreenUpdateCount = 0x00;
    WritePartialScreenUpdateCount();
    u16PartialScreenUpdateCount = tmpu16;
    tmpu8 = u8Hysteresis;
    u8Hysteresis = 0;
    WriteHysteresis();
    u8Hysteresis = tmpu8;
    tmpu8 = u8ThermostatMode;
    u8ThermostatMode = 0;
    WriteThermostatMode();
    u8ThermostatMode = tmpu8;
    ScheduleInit();
    ClearBuffersInitialized();
}


void CheckRTCCInitialized(void) 
{
    //uint8_t data;
    struct tm setup_time;
    struct tm curr_time;
    double diff_time;
    time_t setup_t, curr_t;
    ReadRTCCInitializedMemory();
    if (rtccTimeInitialized == true)
    {
        // make sure the current time is greater than the setup time in FRAm
        ReadRTCCSetupTime(&setup_time);
        RTCC_TimeGet(&curr_time);
        setup_t = mktime(&setup_time);
        curr_t = mktime(&curr_time);
        diff_time = difftime(curr_t, setup_t);
        if (diff_time < 0)
        {
            // current time reported by rtcc is before setup time => the device was powered down 
            // for more than 3 to 4 days by which time the supercap stopped powering the rtcc module
            ClearRTCCInitializedMemory();
            // now all the data in memory needs to be erased since the time is no longer synced
            TemperatureBufferInit();
            RuntimeBufferInit();
            SetpointBufferInit();
            WriteBuffersInitialized();

        }
    }
}

/**
 * 
 * @param str
 * @param substr
 * @return 1 if substring is found in string, else returns 0
 */
uint8_t findSubstring(char *str, char *substr)
{
    int i = 0, j = 0;
    uint8_t u8ret;
    while ((*(str + j) != '\0')&&(*(substr + i) != '\0')) {
        if (*(substr + i) != *(str + j)) {
            j++;
            i = 0;
        }
        else {
            i++;
            j++;
        }
    }
    if (*(substr + i) == '\0')
        u8ret = 1;
    else
        u8ret = 0;
    
    return u8ret;
}


/**
 * Computes new heat and cool setpoints 
 * @param buttonTypePress WHICH COULD BE downButton / upButton
 * @param change_by
 * @return 1 => if there is temperature change else return 0
 */
int computeHeatCoolSetpoints(int buttonTypePress, uint8_t change_by)
{
    uint8_t tmpu8_csp;
    uint8_t tmpu8_hsp;
    uint8_t tmpu8;
    uint8_t tmpu8_change;
    int retval;
    if (change_by == 0)
    {
        return 0;
    }
    
    if ( (u8NewCoolSetpoint >= MAX_COOL_SETPOINT_DEGF) && (buttonTypePress == UP_BUTTON_PRESSED) )
    {
       return 0; 
    }
    
    if ( (u8NewHeatSetpoint <= MIN_HEAT_SETPOINT_DEGF) && (buttonTypePress == DOWN_BUTTON_PRESSED) )
    {
        return 0;
    }
    
    retval = 0;
    if (buttonTypePress == DOWN_BUTTON_PRESSED)
    {
        tmpu8 = u8NewHeatSetpoint - change_by;
        if (tmpu8 < MIN_SCALE_TEMPERATURE_DEGF)
        {
            tmpu8_change = u8NewHeatSetpoint - MIN_SCALE_TEMPERATURE_DEGF;
        }
        else
        {
            tmpu8_change = change_by;
        }
        tmpu8_hsp = u8NewHeatSetpoint - change_by;
        tmpu8_csp = u8NewCoolSetpoint - change_by;
        retval = 1;
//        if (tmpu8_hsp < MIN_SCALE_TEMPERATURE_DEGF)
//        {
//            tmpu8 = tmpu8_csp - tmpu8_hsp;
//            tmpu8_hsp = MIN_SCALE_TEMPERATURE_DEGF;
//            tmpu8_csp = tmpu8_hsp + tmpu8;
//        }
        u8NewHeatSetpoint = tmpu8_hsp;
        u8NewCoolSetpoint = tmpu8_csp;
        
//        if (u8ThermostatMode == MODE_HEATCOOL)
//        {
//            tmpu8_hsp = u8NewHeatSetpoint - change_by;
//            tmpu8_csp = u8NewCoolSetpoint - change_by;
//            if (tmpu8_hsp < MIN_SCALE_TEMPERATURE_DEGF)
//            {
//                tmpu8 = tmpu8_csp - tmpu8_hsp;
//                tmpu8_hsp = MIN_SCALE_TEMPERATURE_DEGF;
//                tmpu8_csp = tmpu8_hsp + tmpu8;
//            }
////            if (tmpu8_hsp < MIN_HEAT_SETPOINT_DEGF)
////            {
////                tmpu8_hsp = MIN_HEAT_SETPOINT_DEGF;
////            }
////            if (tmpu8_csp < MIN_COOL_SETPOINT_DEGF)
////            {
////                tmpu8_csp = MIN_COOL_SETPOINT_DEGF;
////            }
//            u8NewHeatSetpoint = tmpu8_hsp;
//            u8NewCoolSetpoint = tmpu8_csp;
//        }
//        else if (u8ThermostatMode == MODE_COOL_ONLY)
//        {
//            tmpu8_csp = u8NewCoolSetpoint - change_by;
//            if (tmpu8_csp < MIN_COOL_SETPOINT_DEGF)
//            {
//                tmpu8_csp = MIN_COOL_SETPOINT_DEGF;
//            }
//            u8NewCoolSetpoint = tmpu8_csp;
//        }
//        else if (u8ThermostatMode == MODE_HEAT_ONLY)
//        {
//            tmpu8_hsp = u8NewHeatSetpoint - change_by;
//            if (tmpu8_hsp < MIN_HEAT_SETPOINT_DEGF)
//            {
//                tmpu8_hsp = MIN_HEAT_SETPOINT_DEGF;
//            }
//            u8NewHeatSetpoint = tmpu8_hsp;
//        }
    }
    else if (buttonTypePress == UP_BUTTON_PRESSED)
    {
        tmpu8 = u8NewCoolSetpoint + change_by;
        if (tmpu8 > MAX_SCALE_TEMPERATURE_DEGF)
        {
            tmpu8_change = MAX_SCALE_TEMPERATURE_DEGF - u8NewCoolSetpoint;
        }
        else
        {
            tmpu8_change = change_by;
        }
        tmpu8_hsp = u8NewHeatSetpoint + change_by;
        tmpu8_csp = u8NewCoolSetpoint + change_by;
        retval = 1;
//        if (tmpu8_csp > MAX_SCALE_TEMPERATURE_DEGF)
//        {
//            tmpu8 = tmpu8_csp - tmpu8_hsp;
//            tmpu8_csp = MAX_SCALE_TEMPERATURE_DEGF;
//            tmpu8_hsp = tmpu8_hsp - tmpu8;
//        }
        u8NewHeatSetpoint = tmpu8_hsp;
        u8NewCoolSetpoint = tmpu8_csp;
        //        if (u8ThermostatMode == MODE_HEATCOOL)
//        {
//            tmpu8_hsp = u8NewHeatSetpoint + change_by;
//            tmpu8_csp = u8NewCoolSetpoint + change_by;
//            if (tmpu8_hsp > MAX_HEAT_SETPOINT_DEGF)
//            {
//                tmpu8_hsp = MAX_HEAT_SETPOINT_DEGF;
//            }
//            if (tmpu8_csp > MAX_COOL_SETPOINT_DEGF)
//            {
//                tmpu8_csp = MAX_COOL_SETPOINT_DEGF;
//            }
//            u8NewHeatSetpoint = tmpu8_hsp;
//            u8NewCoolSetpoint = tmpu8_csp;
//        }
//        else if (u8ThermostatMode == MODE_COOL_ONLY)
//        {
//            tmpu8_csp = u8NewCoolSetpoint + change_by;
//            if (tmpu8_csp > MAX_COOL_SETPOINT_DEGF)
//            {
//                tmpu8_csp = MAX_COOL_SETPOINT_DEGF;
//            }
//            u8NewCoolSetpoint = tmpu8_csp;
//        }
//        else if (u8ThermostatMode == MODE_HEAT_ONLY)
//        {
//            tmpu8_hsp = u8NewHeatSetpoint + change_by;
//            if (tmpu8_hsp > MAX_HEAT_SETPOINT_DEGF)
//            {
//                tmpu8_hsp = MAX_HEAT_SETPOINT_DEGF;
//            }
//            u8NewHeatSetpoint = tmpu8_hsp;
//        }
    }
    return retval;
}