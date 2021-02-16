 /*
 *      
 *      File:   main.c
 *      Author: Madhu Annapragada
 *      Company: Automation Research Group
 *               3401 Grays Ferry Ave, B197, STE305
 *               Philadelphia, PA 19146 
 *               302-897-7776
 *      Created on January 09, 2018
 * 
 */


#include <xc.h>
#include "main.h"
#include "system.h" //system level functions
#include "rtcc.h" //real time clock
#include "RN4871.h" //Bluetooth module
#include "FRAM.h" //Memory
#include "epaper.h"
#include "BME680.h"
#include "bme680_defs.h"
#include "hal.h" //all the hardware config functions + low level comms
#include "memory.h"

    
//*****************
//    Globals:
//*****************
char VERSION_STR[] = "1.4";
// Bluetooth ADV Parameters
char MODEL_NAME[] = "LYDIA";
char DeviceClass[] = {0}; //6 characters for serial number, 4 for name

//Misc Bluetooth parameters: 
char SerialNum[] = "00001"; // using 5 digit serial number only "00000001";
char MFG_UUID[] = "3e952a7a-18d4-11e9-ab14-d663bd873d93"; //generated on-line uuidgenerator.net
char HARDWARE_REVISION[] = "1.0.0";
char MFG_NAME[] = "EPRI";
char MFG_ID[5] = "FFFF"; //TODO : change this when product is registered and MFG id is obtained
char SOFTWARE_REV[] = "1.0.0";

bool bTimerButtonPressed = false;
bool bUpButtonPressed = false;
bool bDownButtonPressed = false;
bool bPowerButtonPressed = false;
uint8_t u8UpButtonPressCount = 0;
uint8_t u8DownButtonPressCount = 0;
uint8_t u8TimerButtonPressCount = 0;

bool PWR_24VAC_ON = false;

bool RTCCAlarmOn = false;

uint8_t u8CurrentState = 0;


//Generated online : 01/15/2019:12:55PM
//Service UUIDs:
char LITInfoServiceUUID[] = "968ec3f218ee11e9ab14d663bd873d93";

//Characteristics UUID for infoService:
char LITBETemperatureCharUUID[] = "968ec69a18ee11e9ab14d663bd873d93"; //Board Edge Temperature
char LITBETemperatureCharHandle[] = "0072"; //RN4871 default handles start at 0x72 and increment by 0x02
char LITBCTemperatureCharUUID[] = "ce01d1e619b411e9ab14d663bd873d93"; //Board Center Temperature
char LITBCTemperatureCharHandle[] = "0074"; //RN4871 default handles start at 0x72 and increment by 0x02
char LITACurrentSettingCharUUID[] = "ff21df2754f34790b6bf5b589358f656"; // current setting characteristic
char LITACurrentSettingCharHandle[] = "0072";
uint8_t LITACurrentSettingU8Length = 7;
char LITATimeSynchronizeCharUUID[] = "34d64eb8cfcb4424b7b7458ac60cd7ca";
char LITATimeSynchronizeCharHandle [] = "0074";
uint8_t LITATimeSynchronizeU8Length = 7;
char LITAUserParametersCharUUID [] = "49481a839ca64306845eeacd8528508d";
char LITAUserParametersCharHandle [] = "0076";
uint8_t LITAUserParametersU8Length = 12; // 11; for version 1.1 ; 12; // for ver 1.3 GATT_SPECS docucment
char LITASchedulesCharUUID[] = "db468f6304dd4ae6ba1d4861137a3d46";
char LITASchedulesCharHandle[] = "0078";
uint8_t LITASchedulesU8Length = 20;
char LITAAcknowledgementUUID[] = "a8d82b510db342a8af521bc8d7650152";
char LITAAcknowledgementHandle[] = "007A";
uint8_t LITAAcknowledgementLength = 1;
char LITATemperatureBufferUUID[] = "5b76db2d46324a9bbb5d88d77e805980";
char LITATemperatureBufferHandle[] = "007C";
uint8_t LITATemperatureBufferLength = 20;
char LITARuntimeBufferUUID[] = "12b874e7e64841b7a0d09b1f6a8bdcd0";
char LITARuntimeBufferHandle[] = "007E";
uint8_t LITARuntimeBufferLength = 20;
char LITASetpointBufferUUID[] = "d802af929f4f4c1981d26398bc7a04db";
char LITASetpointBufferHandle[] = "0080";
uint8_t LITASetpointBufferLength = 20;

#ifdef testble
uint8_t dbgTimeSyncBLE[7];
uint8_t dbgUserParamBLE[12];
uint8_t dbgSchedulesBLE[50*5];
uint8_t dbgSchedulesRecv[2048];
int     dbgSchedulesRecvIndex;
uint8_t dbgTemperatureBuffer[DBG_BUF_SIZE];
uint8_t dbgTemperatureBufIndex = 0;
uint8_t dbgRuntimeBuffer[DBG_BUF_SIZE];
uint8_t dbgRuntimeBufIndex = 0;
uint8_t dbgSetpointBuffer[DBG_BUF_SIZE];
uint8_t dbgSetpointBufIndex = 0;
#endif


uint8_t u8ApplicationState=0;
uint8_t u8CoolSetpoint=0;
uint8_t u8NewCoolSetpoint;
uint8_t u8SchNoneCoolSetpoint;
uint8_t u8HeatSetpoint=0;
uint8_t u8NewHeatSetpoint=0;
uint8_t u8SchNoneHeatSetpoint;
uint8_t u8Hysteresis=0;
uint16_t u16PartialScreenUpdateCount=0;
uint8_t u8ThermostatMode=0;
uint8_t u8PrevThermostatMode=0;
uint8_t u8arrTodaySchedule[] = {0};
uint8_t u8arrSchedule[] = {0};
uint8_t u8arrDrEvent[8] = {0};
uint8_t u8CurrentTemperature=0;
uint8_t u8PrevCurrentTemperature=0;
uint8_t u8CurrentThermostatAction=0;
uint8_t u8TimerMinutesToOff=0;
uint8_t display_digit_1=0xFF;
uint8_t display_digit_2=0xFF;
uint8_t u8ActionOnScreen = 0; // heat or cool is written to screen
uint8_t u8BatteryOnScreen = 0;
uint16_t u16PartialUpdateCount;
uint8_t u8TimerOnScreen=0;
uint8_t u8DummyOne=0;
uint8_t u8OffIconOnScreen=0;
uint8_t u8CwireOnScreen=0;
float fTempArr[TEMPERATURE_AVERAGE_MAX_ELEMENTS]={0};
float fTempSum;
float fTempAvg;
float fTempAvgPrev;
int u8TempArrIndex=0;
int u8TempArrCount;
uint16_t movAvg2Sum;
uint8_t movAvg2Arr[MOVAVG_2_MAX_ELEMENTS] = {0};
uint8_t movAvg2ArrIndex=0;
uint8_t movAvg2ArrCount=0;
float fKalman_xkp;
float fKalman_pkp;
float fKalman_gain;
uint8_t u8UseMovAverage;

uint8_t u8RelativeHumidity;
uint8_t u8arrBLEChRx[BLE_CHARACTERISTIC_SIZE] = {0}; // user to write to characteristic values in rn4871
uint8_t u8arrBLEChTx[BLE_CHARACTERISTIC_SIZE] = {0}; // user to write to characteristic values in rn4871
uint8_t u8arrTUartRx[TRANSPARENT_UART_BUF_SIZE] = {0};
uint8_t u8arrTUartTx[TRANSPARENT_UART_BUF_SIZE] = {0};

uint8_t u8IndoorAirQuality = 0;
uint8_t u8Schedule;
uint8_t u8PrevSchedule;
uint8_t u8PrevCoolSetpoint;
uint8_t u8PrevHeatSetpoint;
uint8_t u8GotoSchedule;
int32_t i32DurationMins;
uint8_t u8AwayThermostatMode;
uint8_t u8AwayHeatSetpoint;
uint8_t u8AwayCoolSetpoint;
uint8_t u8PlusMinusOnScreen = 0;
uint8_t u8PlusMinusDispTimeoutMins = 0;
uint8_t u8HeatIndOnScreen = 0;
uint8_t u8CoolIndOnScreen = 0;
bool bRunMainLoop = false;
bool bDRScheduleRead = false;
struct tm DRScheduleStartTime;
//uint8_t u8UserParamProcessed = 0;

int main(void)
{
    //*********************
    //      locals
    //*********************         
    uint16_t u16activityTimeout = 30; //timeout in seconds. System goes to sleep after this
    
    uint32_t temp_long = 0;  
    char tmpStr[30] = {0};
    char tmpStr2[30] = {0};
    bool IsBluetoothUp = false;
    
    uint16_t battVoltage = 0; //in mv
   
    uint8_t tmp_app_state;
    uint8_t tmpu8;
    //uint8_t tmpu8_dow, tmpu8_num;
    uint8_t tmpu8_count, tmpu8_idx;
    uint8_t tmpu8_hsp, tmpu8_csp;
    
    bool goto_sleep_immediate = false;
    
    static float fEdgeTemperature = 0;
    //static uint8_t u8BoardTemperature = 0;
    static float fPreviousEdgeTemperature = 0;
    //static uint8_t u8PreviousBoardTemperature = 0;

    //int8_t BME680_rslt = BME680_OK;
    struct tm time_recv;
    //struct tm * to_set_time;
    //time_t time_int_recv;
    int tmpi;
#ifdef checkfram
    int tmpi1, tmpi2;
#endif
    
#ifdef testble
    bool exit_loop = false;
#endif
    /***********************
    *   HARDWARE INIT:
    ************************/
    
    OSCILLATOR_Initialize();
    
    PIN_MANAGER_Initialize();     
    
    //TODO: Uncomment timers as needed - will increase power consumption
    //TMR1_Initialize(); //1ms timer 
    //TMR2_Initialize(); //100ms timer
    TMR3_Initialize(); //1000ms timer - used for activity timeout
    //TMR4_Initialize(); //10ms timer
    //TMR5_Initialize(); //100us timer 
    
    SPI1_Initialize(); //BME680, E-Paper, FRAM on this bus
    
    UART1_Initialize(); //Bluetooth  
    //TODO: Init UART as needed. Will increase power consumption
    //UART2_Initialize(); //LoRa Module
    //UART3_Initialize(); //Digi WiFi   
    
    RTCC_Initialize(); //Real time clock   // moved to after FRAM is turend on
    
    INTERRUPT_Initialize();

    ADC1_Initialize();
    
    
    //Reading in program memory location 0x2AF78 where the sqtp number is stored:
    temp_long = ReadSerialNumber();
    //TODO read nodeName from memory:
    //24bits are programmed by SQTP into the program address 0x02AF78
    //if there is no stored SQTP number, the program space will contain all 0xFF
    if (temp_long < 0x00FFFFFF){ 
        strcpy((char*)tmpStr, (char*)MODEL_NAME);
        strcat((char*)tmpStr,";");
        //converting serial number to  string: 6 characters:
        sprintf ( (char*) tmpStr2, "%.6lX", temp_long);
        strcpy((char*)SerialNum, (char*)tmpStr2);
        //combining node name and serial number: "NodeName;serialNumber"
        strcat((char*)tmpStr,(char*)tmpStr2);
        strcpy((char*)DeviceClass, (char*)tmpStr);
    }
    //enabling UART1 for BLE
    EnableUART1();

     //Power control: Enable power when needed
    TurnOn_Bluetooth_Power();
    TurnOn_FRAM_Power();
    TurnOn_BME680_Power();    
    TurnOn_Display_Power();
    TurnOn_LMT70_Power();
    
    TurnOff_LoRa_Power();
    TurnOff_WIFI_Power();
    
    
    //Bluetooth module init
    if (RN4871_Init()) //If this is not true, the BLE module is dead
    {
        IsBluetoothUp = true;
                
#ifdef transparentuart
        
        //RN4871_Reboot(2); //for the new GATT profiles to take effect
#else
        //Setting up BLE services and characteristics:
        RN4871_Clear_All_Services();
        
        //TODO: set up additional services (Max 3) + characteristics (Max 8 per service):        
        RN4871_SetUp_Private_Service(LITInfoServiceUUID); 
        // for initial testing and debug
        //RN4871_SetUp_Private_Characteristic(LITBETemperatureCharUUID,2,2); //read only, 2 bytes
        //RN4871_SetUp_Private_Characteristic(LITBCTemperatureCharUUID,2,2); //ready only, 2 bytes
        RN4871_SetUp_Private_Characteristic(LITACurrentSettingCharUUID, BLE_CH_READ , LITACurrentSettingU8Length );
        RN4871_SetUp_Private_Characteristic(LITATimeSynchronizeCharUUID, BLE_CH_WRITE, LITATimeSynchronizeU8Length );
        RN4871_SetUp_Private_Characteristic(LITAUserParametersCharUUID, BLE_CH_WRITE, LITAUserParametersU8Length );
        RN4871_SetUp_Private_Characteristic(LITASchedulesCharUUID, BLE_CH_WRITE, LITASchedulesU8Length );
        RN4871_SetUp_Private_Characteristic(LITAAcknowledgementUUID, BLE_CH_READ | BLE_CH_WRITE  , LITAAcknowledgementLength );
        RN4871_SetUp_Private_Characteristic(LITATemperatureBufferUUID, BLE_CH_READ , LITATemperatureBufferLength );
        RN4871_SetUp_Private_Characteristic(LITARuntimeBufferUUID, BLE_CH_READ , LITARuntimeBufferLength );
        RN4871_SetUp_Private_Characteristic(LITASetpointBufferUUID, BLE_CH_READ , LITASetpointBufferLength );
        RN4871_Reboot(1); //for the new GATT profiles to take effect
#endif        
     
        __delay_ms(25); // delay for chip reset
        //RN4871_Get_Handles_Characteristics(LITInfoServiceUUID);
        //uint8_t tmp_sts = RN4871_Get_Handles_Characteristics(LITInfoServiceUUID);
        //setting up the interrupt for BLE connection:
        BLE_Connected = false;
        BLEStatus_InterruptFlagclear();
        BLEStatus_InterruptEnable();
    }
    else
    {
        tmpu8 = 0; // fir a breakoiubt
        //TODO: display error on display?
    }

    
    if (P24vacGOOD_GetValue()) //Will be high if 24VAC power is present
    {
        PWR_24VAC_ON = true;
    }

    tmpu8 = 0;
    // discard first five readings on power up
    while (tmpu8 < 5)
    {
        fEdgeTemperature = ReadBoardTemperature(BoardEdge);
        //u8BoardTemperature = ReadBoardTemperature(BoardMiddle);
        __delay_ms(1000);
        tmpu8++;
    }
    //u8BoardTemperature = ReadBoardTemperature(BoardMiddle);
    fPreviousEdgeTemperature = fEdgeTemperature;
    //u8PreviousBoardTemperature = u8BoardTemperature;
    
    battVoltage = ReadBatteryVoltage(); //low battery below 2400
    
    EnableSPI1(); //for Display, BME and Fram
    
    //Initializing BME680:
    ConfigureBME680();

#ifdef resetfram
    ResetFRAM();
#endif
    
    tmpu8 = FRAMInitialized();
    if (tmpu8 == 0)
    {
        // intialize FRAM buffers, and other relevant locations for buffers and schedules
        ParametersInit();
        TemperatureBufferInit();
        RuntimeBufferInit();
        SetpointBufferInit();
        ScheduleInit();
        WriteBuffersInitialized();
        // now that initialization is all done write the FRAM initialization bytes
        FRAMWriteInitialization(); 
    } 
    CheckRTCCInitialized();
    Application_Initialize();
    Heat_off();
    Cool_off();
    Fan_off();
    ReadParametersFromFRAM();
    //Initializing Display:
    epaper_init();
    epaper_draw_screen(u8CoolSetpoint,u8HeatSetpoint);
    u8CurrentTemperature = Temperature_average(fEdgeTemperature);
    //u8CurrentTemperature = Temperature_average_with_kalman(fEdgeTemperature);
    u8PrevCurrentTemperature = u8CurrentTemperature;
    drawTemperature(u8CurrentTemperature);
    
#ifdef testble
    PopulateDebugBuffers(dbgTemperatureBuffer, dbgRuntimeBuffer, dbgSetpointBuffer, 200, TRANSPARENT_UART_BUF_SIZE-3); // -3 since those elements are for header
#endif

#ifdef checkfram
    
    tmpi1 = (int) GetItemCountTemperatureBuffer();
    tmpi2 = (int) GetCapacityTemperatureBuffer();
    tmpi1 = (int) GetItemCountRuntimeBuffer();
    tmpi2 = (int) GetCapacityRuntimeBuffer();
    tmpi1 = (int) GetItemCountSetpointBuffer();
    tmpi2 = (int) GetCapacitySetpointBuffer();
    ReadRTCCInitializedMemory(); // rtccTimeInitialized will be true if it has been initialized else it will be zero
#endif

//    __delay_ms(5000);
//    tmpu8 = u8EdgeTemperature+5;
//    clear_indicator(temperature_min_x, temperature_min_y, temperature_max_x, temperature_max_y);
//    drawTemperature(tmpu8);
//    __delay_ms(5000);
//    clear_indicator(temperature_min_x, temperature_min_y, temperature_max_x, temperature_max_y);
//    drawTemperature(u8EdgeTemperature);
//    __delay_ms(5000);
//    //epaper_draw_heatcool(0); // draws cool
//    //epaper_draw_heatcool(1); // draws heat
#ifdef testscreensymbols
    if (u8ActionOnScreen == 0)
    {
        epaper_draw_heatcool(DISPLAY_ACTION_HEAT);
        __delay_ms(7000);
        epaper_draw_heatcool(DISPLAY_ACTION_COOL);
        __delay_ms(3000);
        u8ActionOnScreen = 1;
    }
    
    u8BatteryOnScreen = 1;
    epaper_draw_battery(1);
    __delay_ms(7000);
//    if (u8BatteryOnScreen == 1) 
//    {
//        u8BatteryOnScreen = 0;
//        epaper_draw_battery(0);
//    }
    
    u8TimerOnScreen = 1;
    epaper_draw_timer(1);
    
//    u8OffIconOnScreen = 1;
//    epaper_draw_officon(1);

    
    __delay_ms(7000);
    if (u8TimerOnScreen == 1)
    {
        u8TimerOnScreen = 0;
        epaper_draw_timer(0);
    }
    
    if (u8BatteryOnScreen == 1) 
    {
        u8BatteryOnScreen = 0;
        epaper_draw_battery(0);
    }
    
    if (u8CwireOnScreen == 0)
    {
        u8CwireOnScreen = 1;
        epaper_draw_cwire(1);
    }
    
    if (u8PlusMinusOnScreen == (uint8_t)INCDEC_STATE_NONE)
    {
        epaper_draw_plusminus((uint8_t)INCDEC_STATE_INC);
        u8PlusMinusOnScreen = (uint8_t)INCDEC_STATE_INC;
        __delay_ms(7000);
        epaper_draw_plusminus((uint8_t)INCDEC_STATE_DEC);
        u8PlusMinusOnScreen = (uint8_t)INCDEC_STATE_DEC;
        __delay_ms(7000);
    }
    
    if (u8PlusMinusOnScreen != (uint8_t)INCDEC_STATE_NONE)
    {
        epaper_draw_plusminus((uint8_t)INCDEC_STATE_NONE);
        u8PlusMinusOnScreen = (uint8_t)INCDEC_STATE_NONE;
    }

   
    if (u8ActionOnScreen == 1)
    {
        epaper_draw_heatcool((uint8_t)DISPLAY_ACTION_NONE); // clears space
        u8ActionOnScreen = 0;
    }
    
    __delay_ms(5000);
//    if (u8OffIconOnScreen == 1)
//    {
//        epaper_draw_officon(0);
//        u8OffIconOnScreen = 0;
//    }
    
    if (u8CwireOnScreen == 1)
    {
        u8CwireOnScreen = 0;
        epaper_draw_cwire(0);
    }
#endif
    
    epaper_off();

#ifdef testepapertiming
    __delay_ms(60000);
    epaper_on();
    if (u8OffIconOnScreen == 0)
    {
        epaper_draw_officon(1);
        u8OffIconOnScreen = 1;
    }
    epaper_off();
    __delay_ms(30);
    epaper_on();
    if (u8OffIconOnScreen == 1)
    {
        epaper_draw_officon(0);
        u8OffIconOnScreen = 0;
    }
    epaper_off();
#endif    
    
    RTCC_AlarmEnable(3);
    
    //going to sleep now:
    ClrWdt();
    FRAMSleep(0);
    RN4871_SetLowPowerMode();
    
    // TEST FRAM
    // write to FRAM
    //FRAMWake(0);
    //u16PartialScreenUpdateCount = 0x4bd9;
    //WritePartialScreenUpdateCount();
    //FRAMSleep(0);
    // power off comment out the previous 4 lines
    // read from FRAM
    //FRAMWake(0);
    //ReadPartialScreenUpdateCount();
    //FRAMSleep(0);
    
    
    //DEBUG:
//    while (1)
//    {
//        u8EdgeTemperature = ReadBoardTemperature(BoardEdge);
//    }
    
//    while (1)
//    {
//        bme680_set_sensor_mode(&gas_sensor); //to force sensor to take readings
//        BME680_delay_ms(meas_period); //delay until readings are valid
//        /* temperature = data.temperature/100 f
//         * pressure = data.pressure/100 hPa
//         * humidity = data.humidity/1000 %rH
//        */
//        BME680_rslt = bme680_get_sensor_data(&data, &gas_sensor);
//        __delay_ms(1000);
//    }
    /*************************************************************************
                            APPLICATION LOOP
     ************************************************************************/
//    // start while loop by going to activestate immediately
//    u8CurrentState = STATE_ACTIVE;
//    RTCCAlarmOn = true;
    while(1)
    {
        /**************************************************
         *              SLEEP STATE
         *  Only moves out of this state when a phone is connected
        ***************************************************/
        while (u8CurrentState == STATE_SLEEP)
        {
            
            Sleep();
            //Wake up from sleep can happen with any of the 
            //interrupts : either button presses or BLE connection            
        }
        
        /**************************************************
         *              CONNECTED STATE
         *  In this state when phone is connected
         *  Goes back to sleep when phone is disconnected
        ***************************************************/
        while (u8CurrentState == STATE_CONNECT)
        {
            ClrWdt();
            RTCC_AlarmDisable();
   
            // initial code for debug
//            u8EdgeTemperature = ReadBoardTemperature(BoardEdge);
//            u8BoardTemperature = ReadBoardTemperature(BoardMiddle);     
//            
//            battVoltage = ReadBatteryVoltage(); //low battery below 2400
//           
//            //TODO: Read BME temperature
//            //updating the temperature characteristic only if different from before:
//            if (u8EdgeTemperature != u8PreviousEdgeTemperature)
//            {
//                //TODO : use BME temperature when available
//                RN4871_Set_Characteristic_Value_Word(LITBETemperatureCharHandle,u8EdgeTemperature);
//                u8PreviousEdgeTemperature = u8EdgeTemperature;
//                //updating display with current temperature
//                //epaper_on();
//                //drawTemperature(u8EdgeTemperature);
//                //epaper_off();
//            }
//            if (u8BoardTemperature != u8PreviousBoardTemperature)
//            {
//                RN4871_Set_Characteristic_Value_Word(LITBCTemperatureCharHandle,u8BoardTemperature);
//                u8PreviousBoardTemperature = u8BoardTemperature;
//            }

              // moved to later part of code
//            u8arrBLEChTx[0] = u8CurrentTemperature;
//            u8arrBLEChTx[1] = u8ThermostatMode;
//            u8arrBLEChTx[2] = u8RelativeHumidity;
//            u8arrBLEChTx[3] = u8IndoorAirQuality;
//            u8arrBLEChTx[4] = u8Schedule;
//            if (battVoltage < 2400)
//            {
//                u8arrBLEChTx[5] = 0x01;
//            }
//            else 
//            {
//                u8arrBLEChTx[5] = 0x00;
//            }
//            u8arrBLEChTx[6] = u8ApplicationState;
//            u8arrBLEChTx[7] = u8CoolSetpoint;
//            u8arrBLEChTx[8] = u8HeatSetpoint;
//            u8arrBLEChTx[9] = (uint8_t)(battVoltage & 0xFF);
//            u8arrBLEChTx[10] = (uint8_t) ( (battVoltage >> 8) & 0xFF );
            
            
#ifdef testble
            
#ifdef transparentuart
            __delay_ms(2000);
            // is stream ready (get %strem_open% from RN4871)
            tmpu8 = 0;
            tmpu8_idx = 0;
            while ( (tmpu8 < 8) && (tmpu8_idx == 0) )
            {
                // using tmpu8_idx since we did not want to define a new variable
                tmpu8_idx = RN4871_Is_Stream_Ready(2000);
                tmpu8 ++;
            }
            // send current settings; read time sync and user parameters
            u8arrTUartTx[1] = 0x01; // num of characteristics
            u8arrTUartTx[2] = TUART_HW2APP_CURRENT_SETTINGS; // first characteristic being sent
            memcpy(&u8arrTUartTx[3], u8arrBLEChTx, 7);
            u8arrTUartTx[0] = 0x0a;
            tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, 0x0a, 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
            if (tmpu8 == 1) 
            {
                // succ recvd time sync and user parameters
                if (u8arrTUartRx[2] == TUART_APP2HW_TIME_SYNC)
                {
#ifdef testtimesync
                    ReadRTCCInitializedMemory();
                    if (rtccTimeInitialized == false)
                    {
                        time_recv.tm_year = u8arrTUartRx[3];
                        time_recv.tm_mon = u8arrTUartRx[4];
                        time_recv.tm_mday = u8arrTUartRx[5];
                        time_recv.tm_hour = u8arrTUartRx[6];
                        time_recv.tm_min = u8arrTUartRx[7];
                        time_recv.tm_sec = u8arrTUartRx[8];
                        time_recv.tm_isdst = u8arrTUartRx[9];
                        time_recv.tm_wday = u8arrTUartRx[10];
                        RTCC_TimeSet(&time_recv);
                        WriteRTCCInitializedMemory();
                        WriteRTCSetupTime(&time_recv);
                    }
                    
#else
                    // in testing if rtcc was setup it will be cleared when testimesync has been removed
                    ClearRTCCInitializedMemory();
                    memcpy(dbgTimeSyncBLE, &u8arrTUartRx[3], LITATimeSynchronizeU8Length);
#endif                    
                } 
                if (u8arrTUartRx[11] == TUART_APP2HW_USER_SETTINGS)
                {
                    memcpy(dbgUserParamBLE, &u8arrTUartRx[11], LITAUserParametersU8Length);
                    // next line for testing; comment it after testing
                    ProcessUserParameterSettingsData(u8arrTUartRx, 12);
                }
            }
            // send ack from HW to APP, this triggers the App to send schedules
            u8arrTUartTx[1] = 0x01; // number of characteristics
            u8arrTUartTx[2] = TUART_HW2APP_STATUS;
            u8arrTUartTx[3] = 0x00; // dummy value
            u8arrTUartTx[0] = 0x04; // lenght of packet
            tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, 0x04, 6000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
            exit_loop = false;
            tmpu8_count = 0;
            if ( (tmpu8 == 1) && (u8arrTUartRx[2] == TUART_APP2HW_SCHEDULES))
            {
                // recvd schedules
                tmpu8_count = ProcessSchedules(u8arrTUartRx, 3);
                if (tmpu8_count != 0xFF) // recd end of schedules no need of ack
                {
                    // maximum of 2 schedule messages only.
                    // send ack from HW to APP, this triggers the App to send more schedules if available
                    u8arrTUartTx[1] = 0x01; // number of characteristics
                    u8arrTUartTx[2] = TUART_HW2APP_STATUS;
                    u8arrTUartTx[3] = tmpu8_count; // dummy value
                    u8arrTUartTx[0] = 0x04; // lenght of packet
                    tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, 0x04, 6000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                    if ( (tmpu8 == 1 ) && (u8arrTUartRx[2] == TUART_APP2HW_SCHEDULES) )
                    {
                        // recvd second schedules
                        tmpu8_count = ProcessSchedules(u8arrTUartRx, 3);
                        // tmpu8_count should be 0xFF; cannot have more than 2 schedule messages in the transparent uart scheme
                        
                    }
                }
            }
            
            // send temperature buffer and read ack
            do {
                tmpu8_count = 0;
                //TODO: check and uncomment 
                //tmpu8_count = LoadTemperatureBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                if (tmpu8_count > 0)
                {
                    u8arrTUartTx[1] = 0x01; // number of characteristics
                    u8arrTUartTx[2] = TUART_HW2APP_TEMPERATURE_BUF;
                    u8arrTUartTx[0] = (uint8_t) ( tmpu8_count + 0x03 ); // lenght of packet
                    tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                    if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                    {
                        // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                        tmpu8_count = u8arrTUartRx[3]; // recv count resp sent from App
                    }
                }
            } while (tmpu8_count > 0);
            
            // send runtime buffer and read ack
            do {
                tmpu8_count = 0;
                //TODO: check and uncomment 
                //tmpu8_count = LoadRuntimeBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                if (tmpu8_count > 0)
                {
                    u8arrTUartTx[1] = 0x01; // number of characteristics
                    u8arrTUartTx[2] = TUART_HW2APP_RUNTIME_BUF;
                    u8arrTUartTx[0] = (uint8_t) ( tmpu8_count + 0x03 ); // lenght of packet
                    tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                    if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                    {
                        // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                        tmpu8_count = u8arrTUartRx[3]; // recv count resp sent from App
                    }
                }
            } while (tmpu8_count > 0);
            
            // send setp butter and read ack
            do {
                tmpu8_count = 0;
                //TODO: check and uncomment 
                //tmpu8_count = LoadSetpointBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                if (tmpu8_count > 0)
                {
                    u8arrTUartTx[1] = 0x01; // number of characteristics
                    u8arrTUartTx[2] = TUART_HW2APP_SETPOINT_BUF;
                    u8arrTUartTx[0] = (uint8_t) ( tmpu8_count + 0x03 ); // lenght of packet
                    tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                    if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                    {
                        // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                        tmpu8_count = u8arrTUartRx[3]; // recv count resp sent from App
                    }
                }
            } while (tmpu8_count > 0);
                
#else
            //RN4871_Get_Handles_Characteristics(LITInfoServiceUUID);
            /*
            __delay_ms(1000);
            tmpu8 = RN4871_Goto_Mode(1,500);
            if (tmpu8 == 1) {
                tmpu8 = RN4871_Send_Data_Read_Resp("hello12345", 10, 500, u8arrBLEChRx, BLE_CHARACTERISTIC_SIZE);
                // IF tmpu8 is 1, succ write and read response
            }
            tmpu8 = RN4871_Goto_Mode(2, 500);
            */
            RN4871_Write_Characteristic_Value(LITACurrentSettingCharHandle, u8arrBLEChRx, LITACurrentSettingU8Length);
            __delay_ms(5000);
            tmpu8 = RN4871_Read_Characteristic_Value(LITATimeSynchronizeCharHandle, u8arrBLEChRx, LITATimeSynchronizeU8Length);
            if (tmpu8 == 1) {
                memcpy(dbgTimeSyncBLE, u8arrBLEChRx, LITATimeSynchronizeU8Length);
            }
            //__delay_ms(50);
            tmpu8 = RN4871_Read_Characteristic_Value(LITAUserParametersCharHandle, u8arrBLEChRx, LITAUserParametersU8Length);
            if (tmpu8 == 1) 
            {
                memcpy(dbgUserParamBLE, u8arrBLEChRx, LITAUserParametersU8Length);
            }            
            __delay_ms(250);
            exit_loop = false;
            dbgSchedulesRecvIndex = 0;
            while (!exit_loop)
            {
                tmpu8_count = 0;
                tmpu8 = RN4871_Read_Characteristic_Value(LITASchedulesCharHandle, u8arrBLEChRx, 20);
                if (tmpu8 == 1)
                {
                    tmpu8_idx = 0;
                    while (tmpu8_idx < LITASchedulesU8Length) 
                    {
                        memcpy(&dbgSchedulesRecv[dbgSchedulesRecvIndex], u8arrBLEChRx, 20);
                        dbgSchedulesRecvIndex += 20;
                        if (    (u8arrBLEChRx[tmpu8_idx] <= 0x64) || 
                                (u8arrBLEChRx[tmpu8_idx] == 0x0F) || 
                                (u8arrBLEChRx[tmpu8_idx] == 0xFF) || 
                                (u8arrBLEChRx[tmpu8_idx] == 0xFF) 
                            )
                        {
                            if (u8arrBLEChRx[tmpu8_idx] == 0x0F)
                            {
                                dbgSchedulesBLE[145] = u8arrBLEChRx[tmpu8_idx+1];
                                dbgSchedulesBLE[146] = u8arrBLEChRx[tmpu8_idx+2];
                                dbgSchedulesBLE[147] = u8arrBLEChRx[tmpu8_idx+3];
                                dbgSchedulesBLE[148] = u8arrBLEChRx[tmpu8_idx+4];
                                
                            }
                            else if (u8arrBLEChRx[tmpu8_idx] == 0xF0)
                            {
                                dbgSchedulesBLE[149] = u8arrBLEChRx[tmpu8_idx+1];
                                dbgSchedulesBLE[150] = u8arrBLEChRx[tmpu8_idx+2];
                                dbgSchedulesBLE[151] = u8arrBLEChRx[tmpu8_idx+3];
                                dbgSchedulesBLE[152] = u8arrBLEChRx[tmpu8_idx+4];
                            }
                            else if (u8arrBLEChRx[tmpu8_idx] == 0xFF)
                            {
                                exit_loop = true;
                            }
                            else
                            {
                                // valid schedule received
                                tmpu8_dow = (u8arrBLEChRx[tmpu8_idx] & 0xF0) >> 4;
                                tmpu8_num = u8arrBLEChRx[tmpu8_idx] & 0x0F;
								tmpi1 = 5 + 20*tmpu8_dow + tmpu8_num * 4;
								dbgSchedulesBLE[tmpi1] = u8arrBLEChRx[tmpu8_idx+1];
								dbgSchedulesBLE[tmpi1+1] = u8arrBLEChRx[tmpu8_idx+2];
								dbgSchedulesBLE[tmpi1+2] = u8arrBLEChRx[tmpu8_idx+3];
								dbgSchedulesBLE[tmpi1+3] = u8arrBLEChRx[tmpu8_idx+4];
                                tmpu8_count += 1;
                            }
                        }
                        tmpu8_idx += 5;
                    }
                    u8arrBLEChTx[0] = tmpu8_count;
                    RN4871_Write_Characteristic_Value(LITAAcknowledgementHandle, u8arrBLEChTx, 1);
                    __delay_ms(250);
                }
                else 
                {
                    exit_loop = true;
                }
            }
            
            __delay_ms(100);
            exit_loop = false;
			tmpi2 = 0; 
            while (!exit_loop)
            {
                // assuming that LITATemperatureBufferLength is 20
                // and size of u8arrChTx is also 20
                tmpi1 = (BLE_CHARACTERISTIC_SIZE - 6) / 4;
                if (tmpi2 < 200)
                {
				    memcpy(u8arrBLEChTx, &dbgTemperatureBuffer[tmpi2], BLE_CHARACTERISTIC_SIZE);
                    RN4871_Write_Characteristic_Value(LITATemperatureBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    __delay_ms(250);
                    tmpu8 = RN4871_Read_Characteristic_Value(LITAAcknowledgementHandle, u8arrBLEChRx, 1);
                    if (tmpu8 == 1)
                    {
                        // number of elements transferred was acknowledged
                        if (tmpi1 == (int)u8arrBLEChRx[0])
                        {
                            //DeleteEntriesTemperatureBuffer((int)u8arrBLEChRx[0]);
							memset(&dbgTemperatureBuffer[tmpi2], 0, BLE_CHARACTERISTIC_SIZE);
                        }
                    }
                    else 
                    {
                        exit_loop = true;
                    }
					tmpi2 += 20;
                }
                else 
                {
                    u8arrBLEChTx[6] = 0x00;
                    RN4871_Write_Characteristic_Value(LITATemperatureBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    exit_loop = true;
                }
            }
    
            exit_loop = false;
			tmpi2 = 0;
            while (!exit_loop)
            {
                // assuming that LITARuntimeBufferLength is 20 
                // and size of u8arrChRx is 20
                tmpi1 = (BLE_CHARACTERISTIC_SIZE - 6) / 4;
                if (tmpi2 < 200)
                {
				    memcpy(u8arrBLEChTx, &dbgRuntimeBuffer[tmpi2], BLE_CHARACTERISTIC_SIZE);
                    RN4871_Write_Characteristic_Value(LITARuntimeBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    tmpu8 = RN4871_Read_Characteristic_Value(LITAAcknowledgementHandle, u8arrBLEChRx, 1);
                    if (tmpu8 == 1)
                    {
                        // number of elements transferred was acknowledged
                        if (tmpi1 == (int)u8arrBLEChRx[0])
                        {
							memset(&dbgRuntimeBuffer[tmpi2], 0, BLE_CHARACTERISTIC_SIZE);
                            //DeleteEntriesRuntimeBuffer((int)u8arrBLEChRx[0]);
                        }
                    }
                    else 
                    {
                        exit_loop = true;
                    }
					tmpi2 += 20;
                }
                else 
                {
                    u8arrBLEChTx[6] = 0x00;
                    RN4871_Write_Characteristic_Value(LITARuntimeBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    exit_loop = true;
                }
            }

            exit_loop = false;
			tmpi2 = 0;
            while (!exit_loop)
            {
                // assuming that LITASetpointBufferLength is 20 
                // and size of u8arrChRx is 20
                tmpi1 = (BLE_CHARACTERISTIC_SIZE - 6) / 4;
                if (tmpi2 > 0)
                {
				    memcpy(u8arrBLEChTx, &dbgSetpointBuffer[tmpi2], BLE_CHARACTERISTIC_SIZE);
                    RN4871_Write_Characteristic_Value(LITASetpointBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    tmpu8 = RN4871_Read_Characteristic_Value(LITAAcknowledgementHandle, u8arrBLEChRx, 1);
                    if (tmpu8 == 1)
                    {
                        // number of elements transferred was acknowledged
                        if (tmpi1 == (int)u8arrBLEChRx[0])
                        {
							memset(&dbgSetpointBuffer[tmpi2], 0, BLE_CHARACTERISTIC_SIZE);
                            //DeleteEntriesSetpointBuffer((int)u8arrBLEChRx[0]);
                        }
                    }
                    else 
                    {
                        exit_loop = true;
                    }
					tmpi2 += 20;
                }
                else 
                {
                    u8arrBLEChTx[6] = 0x00;
                    RN4871_Write_Characteristic_Value(LITASetpointBufferHandle, u8arrBLEChTx, BLE_CHARACTERISTIC_SIZE);
                    exit_loop = true;
                }
            }
            
#endif
            
            
#else
            //__delay_ms(2000);
            // is stream ready (get %strem_open% from RN4871)
            //u8UserParamProcessed = 0;
            
            tmpu8 = 0;
            tmpu8_idx = 0;
            while ( (tmpu8 < 10) && (tmpu8_idx == 0) )
            {
                // using tmpu8_idx since we did not want to define a new variable
                tmpu8_idx = RN4871_Is_Stream_Ready(2000);
                tmpu8 ++;
            }
            if (tmpu8_idx == 1) // %stream_open% was received
            {
                // send ack from HW to APP, this triggers write time and user params
                u8arrTUartTx[1] = 0x01; // number of characteristics
                u8arrTUartTx[2] = TUART_HW2APP_PROTOCOL_START;
                u8arrTUartTx[3] = 0x00; // dummy value
                u8arrTUartTx[0] = 0x04; // lenght of packet
                tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 6000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                
                if (tmpu8 == 1) 
                {
                    // succ recvd time sync and user parameters
                    // sometimes the buff contains the %conn_param
                    if (u8arrTUartRx[0] == 0x25) // it is the % character
                    {
                        tmpu8_idx = 1;
                        while ( (tmpu8_idx < BTH_Rx_DataIndex) && 
                                (u8arrTUartRx[tmpu8_idx] != 0x25)
                               )
                        {
                            tmpu8_idx ++;
                        }
                        if (u8arrTUartRx[tmpu8_idx] == 0x25) 
                        {
                            tmpu8 = 1; // valid data received
                            tmpu8_idx ++;
                        }
                        else
                        {
                            tmpu8 = 0; // invalid data
                        }
                    }
                    else 
                    {
                        tmpu8_idx = 0;
                        tmpu8 = 1;
                    }
                        
                    if (tmpu8 == 1) 
                    {
                        if (u8arrTUartRx[tmpu8_idx + 2] == TUART_APP2HW_TIME_SYNC)
                        {
                            ReadRTCCInitializedMemory();
                            if (rtccTimeInitialized == false)
                            {
                                time_recv.tm_year = u8arrTUartRx[tmpu8_idx + 3];
                                time_recv.tm_mon = u8arrTUartRx[tmpu8_idx + 4];
                                time_recv.tm_mday = u8arrTUartRx[tmpu8_idx + 5];
                                time_recv.tm_hour = u8arrTUartRx[tmpu8_idx + 6];
                                time_recv.tm_min = u8arrTUartRx[tmpu8_idx + 7];
                                time_recv.tm_sec = u8arrTUartRx[tmpu8_idx + 8];
                                time_recv.tm_isdst = u8arrTUartRx[tmpu8_idx + 9];
                                time_recv.tm_wday = u8arrTUartRx[tmpu8_idx + 10];
                                RTCC_TimeSet(&time_recv);
                                WriteRTCCInitializedMemory();
                                WriteRTCSetupTime(&time_recv);
                                rtccTimeInitialized = true;
                            }
                            else 
                            {
                                // if current time difference is greater than 1 minute, than set the time up again
                                RTCC_TimeGet(&time_recv);
                                if ( (time_recv.tm_hour != u8arrTUartRx[tmpu8_idx + 6]) || (time_recv.tm_min != u8arrTUartRx[tmpu8_idx + 7]) )
                                {
                                    time_recv.tm_year = u8arrTUartRx[tmpu8_idx + 3];
                                    time_recv.tm_mon = u8arrTUartRx[tmpu8_idx + 4];
                                    time_recv.tm_mday = u8arrTUartRx[tmpu8_idx + 5];
                                    time_recv.tm_hour = u8arrTUartRx[tmpu8_idx + 6];
                                    time_recv.tm_min = u8arrTUartRx[tmpu8_idx + 7];
                                    time_recv.tm_sec = u8arrTUartRx[tmpu8_idx + 8];
                                    time_recv.tm_isdst = u8arrTUartRx[tmpu8_idx + 9];
                                    time_recv.tm_wday = u8arrTUartRx[tmpu8_idx + 10];
                                    RTCC_TimeSet(&time_recv);
                                }
                            }
                        } 
                        if (u8arrTUartRx[tmpu8_idx + 11] == TUART_APP2HW_USER_SETTINGS)
                        {
                            ProcessUserParameterSettingsData(u8arrTUartRx, tmpu8_idx + 12);
                        }

                        //// FOR DEBUG ONLY, REMOVE AFTER THAT
                        //if (u8UserParamProcessed == 0)
                        //{
                        //    u8UserParamProcessed = 0;
                        //}

                        // send current settings; read schedules
                        u8arrBLEChTx[0] = u8CurrentTemperature;
                        u8arrBLEChTx[1] = u8ThermostatMode;
                        u8arrBLEChTx[2] = u8RelativeHumidity;
                        u8arrBLEChTx[3] = u8IndoorAirQuality;
                        u8arrBLEChTx[4] = u8Schedule;
                        if (battVoltage < 2400)
                        {
                            u8arrBLEChTx[5] = 0x01;
                        }
                        else 
                        {
                            u8arrBLEChTx[5] = 0x00;
                        }
                        u8arrBLEChTx[6] = u8ApplicationState;
                        u8arrBLEChTx[7] = u8NewCoolSetpoint;
                        u8arrBLEChTx[8] = u8NewHeatSetpoint;
                        u8arrBLEChTx[9] = (uint8_t)(battVoltage & 0xFF);
                        u8arrBLEChTx[10] = (uint8_t) ( (battVoltage >> 8) & 0xFF );

                        u8arrTUartTx[1] = 0x01; // num of characteristics
                        u8arrTUartTx[2] = TUART_HW2APP_CURRENT_SETTINGS; // first characteristic being sent
                        memcpy(&u8arrTUartTx[3], u8arrBLEChTx, 11); //7);
                        u8arrTUartTx[0] = 0x0e; //0x0a;
                        tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                        tmpu8_count = 0; // require this to enter the wh8le loop
                        while ( (tmpu8 == 1) && (u8arrTUartRx[2] == TUART_APP2HW_SCHEDULES)  && (tmpu8_count != 0xFF) )
                        {
                            // recvd schedules
                            tmpu8_count = ProcessSchedules(u8arrTUartRx, 3);
                            // if tmpu8_count is 0xFF, have recd end of schedules no need of ack
                            if (tmpu8_count != 0xFF) 
                            {
                                // maximum of 2 schedule messages only.
                                // send ack from HW to APP, this triggers the App to send more schedules if available
                                u8arrTUartTx[1] = 0x01; // number of characteristics
                                u8arrTUartTx[2] = TUART_HW2APP_STATUS;
                                u8arrTUartTx[3] = tmpu8_count; // dummy value
                                u8arrTUartTx[0] = 0x04; // lenght of packet
                                tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 6000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                            }
                        }
    //                    if ( (tmpu8 == 1) && (u8arrTUartRx[2] == TUART_APP2HW_SCHEDULES))
    //                    {
    //                        // recvd schedules
    //                        tmpu8_count = ProcessSchedules(u8arrTUartRx, 3);
    //                        // if tmpu8_count is 0xFF, have recd end of schedules no need of ack
    //                        if (tmpu8_count != 0xFF) 
    //                            
    //                        {
    //                            // maximum of 2 schedule messages only.
    //                            // send ack from HW to APP, this triggers the App to send more schedules if available
    //                            u8arrTUartTx[1] = 0x01; // number of characteristics
    //                            u8arrTUartTx[2] = TUART_HW2APP_STATUS;
    //                            u8arrTUartTx[3] = tmpu8_count; // dummy value
    //                            u8arrTUartTx[0] = 0x04; // lenght of packet
    //                            tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 6000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
    //                            if ( (tmpu8 == 1 ) && (u8arrTUartRx[2] == TUART_APP2HW_SCHEDULES) )
    //                            {
    //                                // recvd second schedules
    //                                tmpu8_count = ProcessSchedules(u8arrTUartRx, 3);
    //                                // tmpu8_count should be 0xFF; cannot have more than 2 schedule messages in the transparent uart scheme
    //                            }
    //                        }
    //                    }

                        // send temperature buffer and read ack
                        do {
                            tmpu8_count = LoadTemperatureBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                            if (tmpu8_count > 0)
                            {
                                u8arrTUartTx[1] = 0x01; // number of characteristics
                                u8arrTUartTx[2] = TUART_HW2APP_TEMPERATURE_BUF;
                                u8arrTUartTx[0] = (uint8_t) ( tmpu8_count*4 + 6 + 0x03 ); // length of packet
                                tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                                if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                                {
                                    // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                                    if ( tmpu8_count == u8arrTUartRx[3]) // recv count resp sent from App
                                    {
                                        DeleteEntriesTemperatureBuffer((int)tmpu8_count);
                                    }
                                }
                            }
                        } while (tmpu8_count > 0);

                        // send runtime buffer and read ack
                        do {
                            tmpu8_count = LoadRuntimeBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                            if (tmpu8_count > 0)
                            {
                                u8arrTUartTx[1] = 0x01; // number of characteristics
                                u8arrTUartTx[2] = TUART_HW2APP_RUNTIME_BUF;
                                u8arrTUartTx[0] = (uint8_t) ( tmpu8_count*4 + 6 + 0x03 ); // length of packet
                                tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                                if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                                {
                                    // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                                    if ( tmpu8_count == u8arrTUartRx[3]) // recv count resp sent from App
                                    {
                                        DeleteEntriesRuntimeBuffer((int)tmpu8_count);
                                    }
                                }
                            }
                        } while (tmpu8_count > 0);

                        // send setpoint buffer and read ack
                        do {
                            tmpu8_count = LoadSetpointBufferForBLE(u8arrTUartTx, TRANSPARENT_UART_BUF_SIZE, 3);
                            if (tmpu8_count > 0)
                            {
                                u8arrTUartTx[1] = 0x01; // number of characteristics
                                u8arrTUartTx[2] = TUART_HW2APP_SETPOINT_BUF;
                                u8arrTUartTx[0] = (uint8_t) ( tmpu8_count*4 + 6 + 0x03 ); // length of packet
                                tmpu8 = RN4871_Send_Bytes_Read_Resp(u8arrTUartTx, u8arrTUartTx[0], 2000, u8arrTUartRx, TRANSPARENT_UART_BUF_SIZE);
                                if ( (tmpu8 == 1)  && (u8arrTUartRx[2] == TUART_APP2HW_STATUS) )
                                {
                                    // todo: check that the right message came back which is TUART_APP2HW_STATUS and the count value in byte 4
                                    if ( tmpu8_count == u8arrTUartRx[3]) // recv count resp sent from App
                                    {
                                        DeleteEntriesSetpointBuffer((int)tmpu8_count);
                                    }
                                }
                            }
                        } while (tmpu8_count > 0);
                    }
                }
            }
            
#endif   
            //RN4871_Reset_Rx_Buffer();
            if (!BLE_Connected)
            {                
                RN4871_Goto_Mode(2, 2000);
                RN4871_SetLowPowerMode();
                RN4871_Goto_Mode(1, 2000);
                u8CurrentState = STATE_SLEEP;                
                RTCC_AlarmEnable(3);
            }
            else 
            {
                // since everything is done, simply disconnect
                RN4871_Goto_Mode(2, 2000);
                RN4871_Disconnect();
                RN4871_SetLowPowerMode();
                RN4871_Goto_Mode(1, 2000);
                u8CurrentState = STATE_SLEEP;                
                RTCC_AlarmEnable(3);
            }
            
            //__delay_ms(1000); //loop at 1 second rate
            // update timer/ on-off icon based on comm
            if (u8ApplicationState == APP_STATE_ON) 
            {
                if (u8OffIconOnScreen == 1)
                {
                    epaper_on();
                    epaper_draw_officon(0);
                    epaper_off();
                    u8OffIconOnScreen = 0;
                }
            } else if (u8ApplicationState == APP_STATE_OFF)
            {
                if ( (u8TimerOnScreen == 1) || (u8OffIconOnScreen == 0) )
                {
                    epaper_on();
                    if (u8OffIconOnScreen == 0)
                    {
                        epaper_draw_officon(1);
                        u8OffIconOnScreen = 1;
                    }
                    if (u8TimerOnScreen == 1)
                    {
                        epaper_draw_timer(0);
                        u8TimerOnScreen = 0;
                    }
                    epaper_off();
                    if (u8NewHeatSetpoint != u8HeatSetpoint)
                    {
                        u8HeatSetpoint = u8NewHeatSetpoint;
                    }
                    if (u8NewCoolSetpoint != u8CoolSetpoint)
                    {
                        u8CoolSetpoint = u8NewCoolSetpoint;
                    }
                }
                
            } else if (u8ApplicationState == APP_STATE_TIMER)
            {
                if (u8TimerOnScreen == 0)
                {
                    epaper_on();
                    epaper_draw_timer(1);
                    epaper_off();
                    u8TimerOnScreen = 1;
                }
            }
            if ( (u8ApplicationState == APP_STATE_ON) || (u8ApplicationState == APP_STATE_TIMER) )
            {
                if ( (u8NewHeatSetpoint != u8HeatSetpoint) || (u8NewCoolSetpoint != u8CoolSetpoint) )
                {
                    WriteSetpointBuffer(); // writes the u8NewHeatSetpoint and u8NewCoolSetpoint
                }
                epaper_on();
                // if there is a schedule change; display it immediately
                if (u8GotoSchedule != 0xFF) 
                {
                    if (u8NewHeatSetpoint != u8HeatSetpoint) 
                    {
                        u8HeatSetpoint = u8NewHeatSetpoint;
                    }
                    if (u8NewCoolSetpoint != u8CoolSetpoint)
                    {
                        u8CoolSetpoint = u8NewCoolSetpoint;
                    }
                    ExecuteSchedule();
                } 
                else 
                {
                    // new set of parameters in schedule_away
                    if (u8Schedule == SCHEDULE_AWAY)
                    {
                        if (u8ThermostatMode != u8AwayThermostatMode)
                        {
                            u8ThermostatMode = u8AwayThermostatMode;
                        }
                        if (u8ThermostatMode == MODE_HEATCOOL)
                        {
                            u8NewCoolSetpoint = u8AwayCoolSetpoint;
                            u8NewHeatSetpoint = u8AwayHeatSetpoint;
                        }
                        else if (u8ThermostatMode == MODE_HEAT_ONLY)
                        {
                            u8NewHeatSetpoint = u8AwayHeatSetpoint;
                        }
                        else if (u8ThermostatMode == MODE_COOL_ONLY)
                        {
                            u8NewCoolSetpoint = u8AwayCoolSetpoint;
                        }
                    }
                }
                if (u8ThermostatMode == MODE_HEATCOOL) 
                {
                    if (u8HeatIndOnScreen == 0) 
                    {
                        draw_heat_sp_indicator(u8NewHeatSetpoint);
                        u8HeatSetpoint = u8NewHeatSetpoint;
                        u8HeatIndOnScreen == 1;
                    }
                    if (u8CoolIndOnScreen == 0)
                    {
                        draw_cool_sp_indicator(u8NewCoolSetpoint);
                        u8CoolSetpoint = u8NewCoolSetpoint;
                        u8CoolIndOnScreen == 1;
                    }
                }
                else if (u8ThermostatMode == MODE_HEAT_ONLY)
                {
                    if (u8HeatIndOnScreen == 0) 
                    {
                        draw_heat_sp_indicator(u8NewHeatSetpoint);
                        u8HeatSetpoint = u8NewHeatSetpoint;
                        u8HeatIndOnScreen == 1;
                    }
                    if (u8CoolIndOnScreen == 1)
                    {
                        clear_indicator(cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                        u8CoolIndOnScreen == 0;
                    }
                }
                else if (u8ThermostatMode == MODE_COOL_ONLY)
                {
                    if (u8HeatIndOnScreen == 1)
                    {
                        clear_indicator(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y);
                        u8HeatIndOnScreen = 0;
                    }
                    if (u8CoolIndOnScreen == 0)
                    {
                        draw_cool_sp_indicator(u8NewCoolSetpoint);
                        u8CoolSetpoint = u8NewCoolSetpoint;
                        u8CoolIndOnScreen == 1;
                    }
                }
                if ( (u8NewHeatSetpoint != u8HeatSetpoint) && (u8NewCoolSetpoint != u8CoolSetpoint) &&
                      (u8HeatIndOnScreen == 1) && (u8CoolIndOnScreen == 1)
                    )
                {
                    clear_both_indicators(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y, 
                            cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                    draw_heatcool_sp_indicators(u8NewHeatSetpoint, u8NewCoolSetpoint);
                    u8CoolSetpoint = u8NewCoolSetpoint;
                    u8HeatSetpoint = u8NewHeatSetpoint;
                }                
                else 
                {
                    if (u8NewHeatSetpoint != u8HeatSetpoint)
                    {
                        if (u8HeatIndOnScreen == 1)
                        {
                            //epaper_on();
                            clear_indicator(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y);
                            draw_heat_sp_indicator(u8NewHeatSetpoint);
                            //epaper_off();
                        }
                        u8HeatSetpoint = u8NewHeatSetpoint;
                    }
                    if (u8NewCoolSetpoint != u8CoolSetpoint)
                    {
                        if (u8CoolIndOnScreen == 1)
                        {
                            //epaper_on();
                            clear_indicator(cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                            draw_cool_sp_indicator(u8NewCoolSetpoint);
                            //epaper_off();
                        }
                        u8CoolSetpoint = u8NewCoolSetpoint;
                    }
                }
                epaper_off();
            }
        }
        
        /**************************************************
         *              ACTIVE STATE
         *  In this state when user has pressed the power button in sleep state
         *  should go back to sleep after a activity timeout
        ***************************************************/
        while (u8CurrentState == STATE_ACTIVE)
        {
            ClrWdt();
            RTCC_AlarmDisable();
            //Starting up the 1s timer to keep track of user activity
            TMR3_Start();
            goto_sleep_immediate = false;
            if (bPowerButtonPressed == true)
            {
                bPowerButtonPressed = false;
                bRunMainLoop = true;
                //ReadApplicationState();
                if ( (u8ApplicationState == (uint8_t) APP_STATE_ON) || (u8ApplicationState == (uint8_t) APP_STATE_TIMER) )
                {
                    u8ApplicationState = (uint8_t) APP_STATE_OFF;
                    if (u8OffIconOnScreen == 0)
                    {
                        epaper_on();
                        epaper_draw_officon(1);
                        if (u8TimerOnScreen == 1)
                        {
                            epaper_draw_timer(0);
                            u8TimerOnScreen = 0;
                        }
                        epaper_off();
                        u8OffIconOnScreen = 1;
                    }
                }
                else if (u8ApplicationState == (uint8_t) APP_STATE_OFF) 
                {
                    u8ApplicationState = (uint8_t) APP_STATE_ON;
                    if (u8OffIconOnScreen == 1)
                    {
                        epaper_on();
                        epaper_draw_officon(0);
                        epaper_off();
                        u8OffIconOnScreen = 0;
                    }
                    
                }
                goto_sleep_immediate = true;
            }
            if (bUpButtonPressed == true)
            {
                bUpButtonPressed = false;
//                if (u8PlusMinusOnScreen != INCDEC_STATE_INC)
//                {
//                    epaper_on();
//                    epaper_draw_plusminus(INCDEC_STATE_INC);
//                    u8PlusMinusOnScreen = INCDEC_STATE_INC;
//                    if (u8PlusMinusDispTimeoutMins <= 0)
//                    {
//                        u8PlusMinusDispTimeoutMins = (uint8_t)INCDEC_DISP_TIMEOUT_MINS;
//                    }
//                    epaper_off();
//                }
                tmpu8 = u8UpButtonPressCount;
                u8UpButtonPressCount = 0;
                tmpi = computeHeatCoolSetpoints(UP_BUTTON_PRESSED, tmpu8);
                if (tmpi == 1)
                {
                    bRunMainLoop = true;
                }
                else 
                {
                    bRunMainLoop = false;
                }
                goto_sleep_immediate = true;
            }
            if (bDownButtonPressed == true)
            {
                bDownButtonPressed = false;
//                if (u8PlusMinusOnScreen != INCDEC_STATE_DEC)
//                {
//                    epaper_on();
//                    epaper_draw_plusminus(INCDEC_STATE_DEC);
//                    u8PlusMinusOnScreen = INCDEC_STATE_DEC;
//                    if (u8PlusMinusDispTimeoutMins <= 0)
//                    {
//                        u8PlusMinusDispTimeoutMins = (uint8_t)INCDEC_DISP_TIMEOUT_MINS;
//                    }
//                    epaper_off();
//                }
                tmpu8 = u8DownButtonPressCount;
                u8DownButtonPressCount = 0;
                tmpi = computeHeatCoolSetpoints(DOWN_BUTTON_PRESSED, tmpu8);
                if (tmpi == 1)
                {
                    bRunMainLoop = true;
                }
                else 
                {
                    bRunMainLoop = false;
                }
                goto_sleep_immediate = true;
            }
            if (bTimerButtonPressed == true)
            {
                bRunMainLoop = false;
                if (u8ApplicationState != (uint8_t) APP_STATE_TIMER)
                {
                    u8ApplicationState = (uint8_t) APP_STATE_TIMER;
                    //u8TimerMinutesToOff = 15 * (u8TimerButtonPressCount % 5);
                    u8TimerMinutesToOff = 15;
                    u8TimerButtonPressCount = 0;
                    if (u8TimerOnScreen == 0) 
                    {
                        epaper_on();
                        epaper_draw_timer(1);
                        epaper_off();
                        u8TimerOnScreen = 1;
                    }
                    
                }
                else 
                {
                    if ( (u8TimerMinutesToOff >= 0 ) && (u8TimerMinutesToOff < 60) )
                    {
                        u8ApplicationState = (uint8_t) APP_STATE_ON;
                        u8TimerMinutesToOff = 0;
                        if ( (u8TimerMinutesToOff == 0) && (u8TimerOnScreen == 1) )
                        {
                            epaper_on();
                            epaper_draw_timer(0);
                            epaper_off();
                            u8TimerOnScreen = 0;
                        }
                    }
                }
                goto_sleep_immediate = true;
                bTimerButtonPressed = false;
            }
            if ( (RTCCAlarmOn == true)  || ( bRunMainLoop == true) )
            {
                if (RTCCAlarmOn == true)
                {
                    RTCCAlarmOn = false;
                }
                if (bRunMainLoop == false)
                {
                    tmp_app_state = u8ApplicationState;
                    //read temperature and update display
                    //Going to sleep after a period of no activity (user buttons or ble)
                    fEdgeTemperature = ReadBoardTemperature(BoardEdge);
                    // TODO: if using algorithms to compute based on edge/ center temperature it needs to be done here
                    // and set to the variable u8CurrenetTemperature
                    u8PrevCurrentTemperature = u8CurrentTemperature;
                    u8CurrentTemperature =  Temperature_average(fEdgeTemperature) ;
                    //u8CurrentTemperature =  Temperature_average_with_kalman(fEdgeTemperature) ;

                    if (u8PrevCurrentTemperature != u8CurrentTemperature)
                    {
                        WriteTemperatureBuffer();
                    }
                }
                
                epaper_on();
                if (u16PartialScreenUpdateCount >= SCREEN_REFRESH_AFTER_PARTIAL_UPDATES)
                {
                    // redraw the screen
                    //epaper_on();
                    if (u8NewCoolSetpoint != u8CoolSetpoint)
                    {
                        u8CoolSetpoint = u8NewCoolSetpoint;
                    }
                    if (u8NewHeatSetpoint != u8HeatSetpoint)
                    {
                        u8HeatSetpoint = u8NewHeatSetpoint;
                    }
                    epaper_draw_screen(u8CoolSetpoint,u8HeatSetpoint);
                    if (u8ActionOnScreen == 1)
                    {
                        if (u8CurrentThermostatAction == (uint8_t) ACTION_HEATING) 
                        {
                            epaper_draw_heatcool(DISPLAY_ACTION_HEAT);
                        }
                        else if (u8CurrentThermostatAction == (uint8_t) ACTION_COOLING)
                        {
                            epaper_draw_heatcool(DISPLAY_ACTION_COOL);
                        }
                    }
                    if (u8BatteryOnScreen == 1)
                    {
                        epaper_draw_battery(1);
                    }
                    drawTemperature(u8CurrentTemperature);
                    //epaper_off();
                }
                else 
                {
                    if ( (u8PrevCurrentTemperature != u8CurrentTemperature) && 
                            ( (u8ApplicationState == (uint8_t) APP_STATE_ON ) || ( u8ApplicationState == (uint8_t) APP_STATE_TIMER ) ) &&
                            ( bRunMainLoop == false )
                        )
                    {
                        //epaper_on();
                        clear_indicator(temperature_min_x, temperature_min_y, temperature_max_x, temperature_max_y);
                        drawTemperature(u8CurrentTemperature);
                        //epaper_off();
                    }

                    if ( (u8NewHeatSetpoint != u8HeatSetpoint) || (u8NewCoolSetpoint != u8CoolSetpoint) )
                    {
                        if ( (u8NewHeatSetpoint != u8HeatSetpoint) && (u8NewCoolSetpoint != u8CoolSetpoint) &&
                              (u8HeatIndOnScreen == 1) && (u8CoolIndOnScreen == 1)
                            )
                        {
                            clear_both_indicators(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y, 
                                    cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                        }                
                        else 
                        {
                            if ( (u8NewHeatSetpoint != u8HeatSetpoint) && (u8HeatIndOnScreen == 1) )
                            {
                                clear_indicator(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y);
                            }
                            if ( (u8NewCoolSetpoint != u8CoolSetpoint) && (u8CoolIndOnScreen == 1) )
                            {
                                clear_indicator(cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                            }
                        }
                        if (bRunMainLoop == true)
                        {
                            tmpu8_csp = u8NewCoolSetpoint;
                            tmpu8_hsp = u8NewHeatSetpoint;
                            if (u8UpButtonPressCount > u8DownButtonPressCount)
                            {
                                tmpu8 = u8UpButtonPressCount - u8DownButtonPressCount;
                                tmpi = computeHeatCoolSetpoints(UP_BUTTON_PRESSED, tmpu8); 
                                // ignore tmpi, since one change is already pending if this part of code is reached
                            }
                            else if ( u8DownButtonPressCount > u8UpButtonPressCount )
                            {
                                tmpu8 = u8DownButtonPressCount - u8UpButtonPressCount;
                                tmpi = computeHeatCoolSetpoints(DOWN_BUTTON_PRESSED, tmpu8); 
                                // ignore tmpi, since one change is already pending if this part of code is reached
                            }
                            else // both are equal
                            {
                            }
                            u8UpButtonPressCount = 0;
                            u8DownButtonPressCount = 0;
                            bUpButtonPressed = false;
                            bDownButtonPressed = false;
                            // has the setponts moved in a fashion that the resultant setpoints have not changed
                            // if yes, need to draw indicators again, since they have already been erased if code has reached here
                            if ( 
                                    ( (tmpu8_csp != u8NewCoolSetpoint) && (u8NewCoolSetpoint == u8CoolSetpoint) ) ||
                                    ( (tmpu8_hsp != u8NewHeatSetpoint) && (u8NewHeatSetpoint == u8HeatSetpoint) ) 
                                )
                            {
                                if ( (u8HeatIndOnScreen == 1) && (u8CoolIndOnScreen == 1) )
                                {
                                    draw_heatcool_sp_indicators(u8HeatSetpoint, u8CoolSetpoint);
                                }
                                else 
                                {
                                    if (u8HeatIndOnScreen == 1)
                                    {
                                        draw_heat_sp_indicator(u8HeatSetpoint);
                                    }
                                    if (u8CoolIndOnScreen == 1)
                                    {
                                        draw_cool_sp_indicator(u8CoolSetpoint);
                                    }
                                }
                            }
                        }
                        if ( (u8NewHeatSetpoint != u8HeatSetpoint) || (u8NewCoolSetpoint != u8CoolSetpoint) )
                        {
                            WriteSetpointBuffer();
                        }
                        if ( (u8NewHeatSetpoint != u8HeatSetpoint) && (u8NewCoolSetpoint != u8CoolSetpoint) &&
                              (u8HeatIndOnScreen == 1) && (u8CoolIndOnScreen == 1)
                            )
                        {
                            u8HeatSetpoint = u8NewHeatSetpoint;
                            u8CoolSetpoint = u8NewCoolSetpoint;
                            draw_heatcool_sp_indicators(u8HeatSetpoint, u8CoolSetpoint);
                        }
                        else
                        {
                            if (u8NewHeatSetpoint != u8HeatSetpoint)
                            {
                                u8HeatSetpoint = u8NewHeatSetpoint;
                                if (u8HeatIndOnScreen == 1) 
                                {
                                    draw_heat_sp_indicator(u8HeatSetpoint);
                                }
                            } 
                            if (u8NewCoolSetpoint != u8CoolSetpoint)
                            {
                                u8CoolSetpoint = u8NewCoolSetpoint;
                                if (u8CoolIndOnScreen == 1)
                                {
                                    draw_cool_sp_indicator(u8CoolSetpoint);
                                }
                            }
                        }
                    }
                }
                
                if (PWR_24VAC_ON == false)
                {
                    if (u8CwireOnScreen == 1)
                    {
                        epaper_draw_cwire(0);
                        u8CwireOnScreen = 0;
                    }
                    battVoltage = ReadBatteryVoltage(); //low battery below 2400
                    if (battVoltage < 2400) // i.e. less than 2.4 V
                    {
                        // user needs to replace battery, draw battery icon
                        if (u8BatteryOnScreen == 0)
                        {
                            //epaper_on();
                            epaper_draw_battery(1);
                            //epaper_off();
                            u8BatteryOnScreen = 1;
                        }
                    }
                    else 
                    {
                        if (u8BatteryOnScreen == 1)
                        {
                            //epaper_on();
                            epaper_draw_battery(0);
                            //epaper_off();
                            u8BatteryOnScreen = 0;
                        }
                    }
                }
                else // if (PWR_24VAC_ON == true)
                {
                    if (u8CwireOnScreen == 0)
                    {
                        epaper_draw_cwire(1);
                        u8CwireOnScreen = 1;
                    }
                } 
                
                
                if (u8PlusMinusDispTimeoutMins > 0)
                {
                    u8PlusMinusDispTimeoutMins--;
                    if (u8PlusMinusDispTimeoutMins == 0)
                    {
                        if (u8PlusMinusOnScreen != INCDEC_STATE_NONE)
                        {
                            //epaper_on();
                            epaper_draw_plusminus(INCDEC_STATE_NONE);
                            //epaper_off();
                            u8PlusMinusOnScreen = (uint8_t) INCDEC_STATE_NONE;
                        }
                    }
                }

                //epaper_on(); // for indicators only
                if (u8ThermostatMode == MODE_HEATCOOL)
                {
                    if (u8HeatIndOnScreen == 0)
                    {
                        draw_heat_sp_indicator(u8HeatSetpoint);
                        u8HeatIndOnScreen = 1;
                    }
                    if (u8CoolIndOnScreen == 0)
                    {
                        draw_cool_sp_indicator(u8CoolSetpoint);
                        u8CoolIndOnScreen = 1;
                    }
                }
                else if (u8ThermostatMode == MODE_HEAT_ONLY)
                {
                    if (u8HeatIndOnScreen == 0)
                    {
                        draw_heat_sp_indicator(u8HeatSetpoint);
                        u8HeatIndOnScreen = 1;
                    }
                    if (u8CoolIndOnScreen == 1)
                    {
                        clear_indicator(cool_sp_min_x, cool_sp_min_y, cool_sp_max_x, cool_sp_max_y);
                        u8CoolIndOnScreen = 0;
                    }
                }
                else if (u8ThermostatMode == MODE_COOL_ONLY)
                {
                    if (u8CoolIndOnScreen == 0)
                    {
                        draw_cool_sp_indicator(u8CoolSetpoint);
                        u8CoolIndOnScreen = 1;
                    }
                    if (u8HeatIndOnScreen == 1)
                    {
                        clear_indicator(heat_sp_min_x, heat_sp_min_y, heat_sp_max_x, heat_sp_max_y);
                        u8HeatIndOnScreen = 0;
                    }
                }
                //epaper_off(); // indicators is done.
                
                if ( (u8ApplicationState == (uint8_t) APP_STATE_ON ) || ( u8ApplicationState == (uint8_t) APP_STATE_TIMER ) )
                {
                    if (u8OffIconOnScreen == 1)
                    {
                        //epaper_on();
                        epaper_draw_officon(0);
                        //epaper_off();
                        u8OffIconOnScreen = 0;
                    }

                    ExecuteSchedule();
                    if (u8CurrentThermostatAction == (uint8_t) ACTION_HEATING) 
                    {
                        if (  (u8CurrentTemperature >= (u8HeatSetpoint + u8Hysteresis)) ||
                               (u8ThermostatMode == MODE_COOL_ONLY) ) // this condition could be triggered by changing the mode in ble comms
                        {
                            u8CurrentThermostatAction = (uint8_t) ACTION_NONE;
                            // turn off heating relay
                            Heat_off();
                            Fan_off();
                            WriteRuntimeBuffer();
                            if (u8ActionOnScreen == 1)
                            {
                                //epaper_on();
                                epaper_draw_heatcool(DISPLAY_ACTION_NONE);
                                //epaper_off();
                                u8ActionOnScreen = 0;
                            }
                        }
                    }
                    else if (u8CurrentThermostatAction == (uint8_t) ACTION_COOLING )
                    {
                        if ( (u8CurrentTemperature <= (u8CoolSetpoint - u8Hysteresis)) || 
                              (u8ThermostatMode == MODE_HEAT_ONLY) ) // this would happen if there is a mode change
                        {
                            u8CurrentThermostatAction = (uint8_t)ACTION_NONE;
                            // turn off cooling relay
                            Cool_off();
                            Fan_off();
                            WriteRuntimeBuffer();
                            if (u8ActionOnScreen == 1)
                            {
                                //epaper_on();
                                epaper_draw_heatcool(DISPLAY_ACTION_NONE);
                                //epaper_off();
                                u8ActionOnScreen = 0;
                            }
                        }
                    }
                    else if (u8CurrentThermostatAction == (uint8_t) ACTION_NONE)
                    {
                        // hysterisis only above heat setpoint
                        //if ( (u8CurrentTemperature < (u8HeatSetpoint - u8Hysteresis) ) && 
                        if ( (u8CurrentTemperature < ( u8HeatSetpoint ) ) && 
                                ( (u8ThermostatMode == MODE_HEATCOOL ) || (u8ThermostatMode == MODE_HEAT_ONLY) )
                            )
                            
                        {
                            u8CurrentThermostatAction = (uint8_t) ACTION_HEATING;
                            // TURN ON RELAY TO HEAT
                            Heat_on();
                            Fan_on();
                            WriteRuntimeBuffer();
                            if (u8ActionOnScreen == 0)
                            {
                                //epaper_on();
                                epaper_draw_heatcool(DISPLAY_ACTION_HEAT);
                                //epaper_off();
                                u8ActionOnScreen = 1;
                            }
                            
                        } 
                        // hysteresis only below cool set point
                        //else if ( (u8CurrentTemperature > (u8CoolSetpoint + u8Hysteresis) ) &&
                        else if ( (u8CurrentTemperature > (u8CoolSetpoint) ) &&
                                    ( (u8ThermostatMode == MODE_HEATCOOL) || (u8ThermostatMode == MODE_COOL_ONLY) ) 
                                )
                        {
                            u8CurrentThermostatAction = (uint8_t) ACTION_COOLING;
                            // TURN ON RELAY TO cool
                            Cool_on();
                            Fan_on();
                            WriteRuntimeBuffer();
                            if (u8ActionOnScreen == 0)
                            {
                                //epaper_on();
                                epaper_draw_heatcool(DISPLAY_ACTION_COOL);
                                //epaper_off();
                                u8ActionOnScreen = 1;
                            }
                        }
                    }
                    if (u8ApplicationState == (uint8_t) APP_STATE_TIMER)
                    {
                        // decrement timer and turn off
                        if (u8TimerMinutesToOff > 0)
                        {
                            if (u8TimerOnScreen != 1) 
                            {
                                //epaper_on();
                                epaper_draw_timer(1);
                                //epaper_off();
                                u8TimerOnScreen = 1;
                            }
                            u8TimerMinutesToOff --;
                            if (u8TimerMinutesToOff == 0)
                            {
                                u8ApplicationState = (uint8_t)APP_STATE_OFF;
                                if (u8TimerOnScreen == 1) 
                                {
                                    //epaper_on();
                                    epaper_draw_timer(0);
                                    //epaper_off();
                                    u8TimerOnScreen = 0;
                                }
//                                if (u8Schedule == SCHEDULE_HOME)
//                                {
//                                    //WriteScheduleMode(0,0,0);
//                                }
//                                else if (u8Schedule == SCHEDULE_DREVENT)
//                                {
//                                    //WriteScheduleMode(0xFF, 0xFF, 0xFF);
//                                }
//                                else if (u8Schedule == SCHEDULE_NONE)
//                                {
//                                    //WriteScheduleMode(0x0F, 0x0F, 0x0F);
//                                }
//                                else
//                                {
//                                    //WriteScheduleMode(SCHEDULE_AWAY, u8CoolSetpoint, u8HeatSetpoint);
//                                }
                                
                            }
                        }
                    }

                }
                else if (u8ApplicationState == (uint8_t) APP_STATE_OFF)
                {
                    if (u8OffIconOnScreen == 0)
                    {
                        //epaper_on();
                        epaper_draw_officon(1);
                        //epaper_off();
                        u8OffIconOnScreen = 1;
                    }
                    if (u8ActionOnScreen == 1)
                    {
                        //epaper_on();
                        if (u8OffIconOnScreen == 0)
                        {
                            epaper_draw_heatcool(DISPLAY_ACTION_NONE);
                        }
                        //epaper_off();
                        u8ActionOnScreen = 0;
                        Heat_off();
                        Cool_off();
                        Fan_off();
                        u8CurrentThermostatAction = (uint8_t) ACTION_NONE;
                    }
                    if (u8TimerOnScreen == 1) 
                    {
                        //epaper_on();
                        epaper_draw_timer(0);
                        //epaper_off();
                        u8TimerOnScreen = 0;
                    }
                    if (u8PlusMinusOnScreen != INCDEC_STATE_NONE)
                    {
                        //epaper_on();
                        epaper_draw_plusminus(INCDEC_STATE_NONE);
                        //epaper_off();
                        u8PlusMinusOnScreen = (uint8_t) INCDEC_STATE_NONE;
                    }

                    
                    // nothing else to do here
                }
                else 
                {
                    // do nothing
                    // code should never reach here
                }
                epaper_off();
                
                if (tmp_app_state != u8ApplicationState)
                {
                    //WriteApplicationState();
                    // state change occurred
                    // TODO: state change actions to be done here
                }
                goto_sleep_immediate = true;
                if (bRunMainLoop == true)
                {
                    bRunMainLoop = false;
                }
                
            }
            //If a phone is connected, go to connected state
            if (BLE_Connected)
            {
                u8CurrentState = STATE_CONNECT;                
            }
            else if ( (bTimerButtonPressed == true) || (bUpButtonPressed == true) || (bDownButtonPressed == true) || (bPowerButtonPressed == true) )
            {
                // cannot go to screen right away
                __delay_ms(1000);
                u8CurrentState = STATE_ACTIVE;
            }
            else if ( ( goto_sleep_immediate == true) || (u16Timer3Tick1s >= u16activityTimeout) )
            {
                RTCC_AlarmEnable(3);
                u8CurrentState = STATE_SLEEP;
            }
            
        }
         
        
    } //END APPLICATION LOOP
    
    return 1;
}


