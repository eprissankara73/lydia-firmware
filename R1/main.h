
#ifndef MAIN_H
#define	MAIN_H

#define _XTAL_FREQ  8000000UL //system clock = 8MHz internal clk - PLL disabled
#define FCY 16000000 //system instruction clock = system clock/2

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <libpic30.h>

//******************
// important defines
//******************
//#define testble 1 // for testing ble comms: comment out to disable test: test will copy data to separate buffers but will not execute any values
#define transparentuart  1
//#define checkfram 1 // for checking data stored in FRAM
//#define resetfram 1 // clears all FRAM data
//#define testscreensymbols 1 // for displaying symbols on screen
//#define testtimesync 1 // to be used in conjunction with testble set to 1 and trnasparnet uart set to 1; otherwise this has no effect on code
//#define testepapertiming 1 
// enable next line  "debugboardtemp" if you wants to store the board temperature in FRAM instead of humidity
//#define debugboardtemp 1
//#define testinghwunit 1

// if the following define in uncommented the thick dial will be used.
// else the original thin dial will be used
#define thickdial 1

/******************************************************************************
 *      GLOBALS
******************************************************************************/
extern uint8_t u8CurrentState; //Main.c state machine

//Overlay related:
extern bool bTimerButtonPressed;
extern bool bUpButtonPressed;
extern bool bDownButtonPressed;
extern bool bPowerButtonPressed;
#define MAX_BUTTON_PRESS_COUNT   5
extern uint8_t u8UpButtonPressCount ;
extern uint8_t u8DownButtonPressCount ;
extern uint8_t u8TimerButtonPressCount;

extern bool PWR_24VAC_ON;
extern bool RTCCAlarmOn;

//Bluetooth related:
extern char MODEL_NAME[19];
extern char DeviceClass[26];
extern char SerialNum[9];
extern char HARDWARE_REVISION[6];
extern char MFG_NAME[4];
extern char MFG_ID[5];
extern char SOFTWARE_REV[6];



//Service and characteristic UUIDS used for BLE:

extern char LITInfoServiceUUID[]; 
extern char LITTemperatureCharUUID[];

// global vars for application  
extern uint8_t u8ApplicationState; // on, off, timer, default is ON
#define APP_STATE_ON    0x01
#define APP_STATE_OFF   0x02
#define APP_STATE_TIMER 0x03 // NOT USED

extern uint8_t u8CoolSetpoint;
extern uint8_t u8NewCoolSetpoint;
extern uint8_t u8SchNoneCoolSetpoint;
#ifdef testinghwunit
#define COOL_SETPOINT_DEFAULT_DEGF   76 //76
#define HEAT_SETPOINT_DEFAULT_DEGF   66
#else
#define COOL_SETPOINT_DEFAULT_DEGF   78 //76
#define HEAT_SETPOINT_DEFAULT_DEGF   65
#endif
#define MIN_COOL_SETPOINT_DEGF      75 // 68 //75
#define MAX_COOL_SETPOINT_DEGF      90
extern uint8_t u8HeatSetpoint;
extern uint8_t u8NewHeatSetpoint;
extern uint8_t u8SchNoneHeatSetpoint;
#define MIN_HEAT_SETPOINT_DEGF      50
#define MAX_HEAT_SETPOINT_DEGF      74 //70 //74
#define MIN_SCALE_TEMPERATURE_DEGF  50
#define MAX_SCALE_TEMPERATURE_DEGF  90
extern uint8_t u8Hysteresis;
#define SCREEN_REFRESH_AFTER_PARTIAL_UPDATES  2000 // TODO: check out a good value for this
extern uint16_t u16PartialScreenUpdateCount;
#define MODE_COOL_ONLY 0x03
#define MODE_HEAT_ONLY 0x02
#define MODE_HEATCOOL  0x01
extern uint8_t u8ThermostatMode;
extern uint8_t u8PrevThermostatMode;
extern uint8_t u8arrTodaySchedule[20]; // format hour, minute, cool setpoint, heat setpoint
extern uint8_t u8arrSchedule[140]; // format hour, minute, cool setpoint, heat setpoint

extern uint8_t u8arrDrEvent[8]; // format month, dayofmonth, hour, minute, duration hours, duration mins, cool setpoint, heat setpoint
extern uint8_t u8CurrentTemperature;
extern uint8_t u8PrevCurrentTemperature;
extern uint8_t u8CurrentThermostatAction;
#define ACTION_HEATING      0x01
#define ACTION_COOLING      0x02
#define ACTION_NONE         0x03
extern uint8_t u8TimerMinutesToOff;
#define MAX_TIMER_OFF_MINUTES  60
extern uint8_t display_digit_1;
extern uint8_t display_digit_2;
extern uint8_t u8ActionOnScreen; // heat or cool is written to screen
extern uint8_t u8BatteryOnScreen;
extern uint8_t u8TimerOnScreen;
extern uint8_t u8DummyOne;
extern uint8_t u8OffIconOnScreen;
// for moving average
#define TEMPERATURE_AVERAGE_MAX_ELEMENTS  13 //7 //13
extern float fTempArr[TEMPERATURE_AVERAGE_MAX_ELEMENTS];
extern float fTempSum;
extern float fTempAvg;
extern float fTempAvgPrev;
extern int u8TempArrIndex;
extern int u8TempArrCount;
#define MOVAVG_2_MAX_ELEMENTS  3
extern uint16_t movAvg2Sum;
extern uint8_t movAvg2Arr[MOVAVG_2_MAX_ELEMENTS];
extern uint8_t movAvg2ArrIndex;
extern uint8_t movAvg2ArrCount;
#define KALMAN_R_FACTOR 0.2
extern float fKalman_xkp;
extern float fKalman_pkp;
extern float fKalman_gain;
extern uint8_t u8UseMovAverage;

extern uint8_t u8RelativeHumidity;
#define BLE_CHARACTERISTIC_SIZE     20
extern uint8_t u8arrBLEChRx[BLE_CHARACTERISTIC_SIZE];
extern uint8_t u8arrBLEChTx[BLE_CHARACTERISTIC_SIZE];
extern uint8_t u8IndoorAirQuality;
#define TRANSPARENT_UART_BUF_SIZE   128
extern uint8_t u8arrTUartRx[TRANSPARENT_UART_BUF_SIZE];
extern uint8_t u8arrTUartTx[TRANSPARENT_UART_BUF_SIZE];

#define SCHEDULE_HOME       0x01
#define SCHEDULE_AWAY       0x02
#define SCHEDULE_DREVENT    0x03
#define SCHEDULE_NONE       0x04
extern uint8_t u8Schedule;
extern uint8_t u8PrevSchedule;
extern uint8_t u8PrevCoolSetpoint;
extern uint8_t u8PrevHeatSetpoint;
extern uint8_t u8GotoSchedule;
extern int32_t i32DurationMins;

extern uint8_t u8AwayThermostatMode;
extern uint8_t u8AwayHeatSetpoint;
extern uint8_t u8AwayCoolSetpoint;

extern uint8_t u8CoolSPChanged;
extern uint8_t u8HeatSPChanged;
extern uint8_t u8UserParamProcessed;
extern bool bDRScheduleRead;
extern struct tm DRScheduleStartTime;


//state machine related
#define STATE_SLEEP     0
#define STATE_CONNECT   1
#define STATE_ACTIVE    2

//Temp sensor locations:
#define BoardEdge       0x0A
#define BoardMiddle     0x0B

#define BLE_CH_READ             0x02
#define BLE_CH_WRITE_NO_RSP     0x04
#define BLE_CH_WRITE            0x08
#define BLE_CH_NOTIFY           0x10
#define BLE_CH_INDICATE         0x20

#define TUART_HW2APP_CURRENT_SETTINGS   0x01
#define TUART_HW2APP_STATUS             0x03
#define TUART_HW2APP_TEMPERATURE_BUF    0x05
#define TUART_HW2APP_RUNTIME_BUF        0x07
#define TUART_HW2APP_SETPOINT_BUF       0x09
#define TUART_HW2APP_PROTOCOL_START     0x0B

#define TUART_APP2HW_TIME_SYNC          0x02
#define TUART_APP2HW_USER_SETTINGS      0x04
#define TUART_APP2HW_SCHEDULES          0x06
#define TUART_APP2HW_STATUS             0x08

#define INCDEC_STATE_NONE               0x00
#define INCDEC_STATE_INC                0x01 // draw plus sign
#define INCDEC_STATE_DEC                0x02 // draw minus sign
#define INCDEC_DISP_TIMEOUT_MINS        0x03  // delete plus/minus sign after this many minuetes

extern uint8_t u8PlusMinusDispTimeoutMins;
extern uint8_t u8PlusMinusOnScreen;
extern uint8_t u8HeatIndOnScreen;
extern uint8_t u8CoolIndOnScreen;

#define DOWN_BUTTON_PRESSED 0x01
#define UP_BUTTON_PRESSED   0x02

#ifdef testble
#define DBG_BUF_SIZE 200
extern uint8_t dbgTimeSyncBLE[7];
extern uint8_t dbgUserParamBLE[12];
extern uint8_t dbgSchedulesBLE[50*5];
extern uint8_t dbgSchedulesRecv[2048];
extern int     dbgSchedulesRecvIndex;
extern uint8_t dbgTemperatureBuffer[DBG_BUF_SIZE];
extern uint8_t dbgTemperatureBufIndex;
extern uint8_t dbgRuntimeBuffer[DBG_BUF_SIZE];
extern uint8_t dbgRuntimeBufIndex;
extern uint8_t dbgSetpointBuffer[DBG_BUF_SIZE];
extern uint8_t dbgSetpointBufIndex;
#endif




#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */


#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* MAIN_H */

