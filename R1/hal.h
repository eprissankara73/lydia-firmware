
#ifndef HAL_H
#define	HAL_H

#include <xc.h>
#include "main.h"



/*******************************************************************************
 *   UART SPEEDS
*******************************************************************************/
//UART Baud with BRGH = 1
#define BRGH_115200 (((FCY)/(4*115200))-1)
#define BRGH_57600 (((FCY)/(4*57600))-1)
//UART baud with BRGH = 0
#define BRGL_115200 (((FCY)/(16*115200))-1)

/*******************************************************************************
 *   PIN LEVEL MACROS
*******************************************************************************/ 
#define P24vacGOOD_GetValue()         _RB3  //High when 24VAC power is present

//Module power control:
#define TurnOn_Display_Power()         do {_LATE0=1;_LATE5=1;_LATB2=1;} while(0) //E0 is reset,E5 is CS,  B2 is power  
#define TurnOff_Display_Power()        do {_LATE0=0;_LATE5=0;_LATB2 = 0;} while(0)
#define TurnOn_LoRa_Power()            do {_LATF5=1;_LATE3=1; _LATB9 = 1;} while(0) //F5 is Tx, E3 is reset, B9 is power  
#define TurnOff_LoRa_Power()           do { _LATB9 = 0;_LATF5=0;_LATE3=0;} while(0)
#define TurnOn_BME680_Power()          do {_LATE6=1; _LATC12 = 1;} while(0) //E6 is CS, C12 is power 
#define TurnOff_BME680_Power()         do {_LATE6=0; _LATC12 = 0;} while(0)
#define TurnOn_LMT70_Power()           do { AD1CON1bits.ADON=1;_LATC15 = 1;} while(0) //enabling ADC module and power line
#define TurnOff_LMT70_Power()          do { AD1CON1bits.ADON=0;_LATC15 = 0;} while(0)
#define TurnOn_WIFI_Power()            do { _LATF3 = 1; _LATE1 = 1;_LATF7 = 1;} while(0) //F3 is Wifi Tx, E1 is reset, F7 is power line
#define TurnOff_WIFI_Power()           do {_LATF7 = 0; _LATF3 = 0; _LATE1 = 0;} while(0)
#define TurnOn_FRAM_Power()            do { _LATE4 = 1; _LATG2 = 1;} while(0) //E4 is CS, G2 is power line
#define TurnOff_FRAM_Power()           do { _LATG2 = 0;_LATE4 = 0;} while(0) //E4 is CS, G2 is power line
#define TurnOn_Bluetooth_Power()       do { _LATG6 = 1; _LATG3 = 1;} while(0) //G6 is TX line, G3 is power line
#define TurnOff_Bluetooth_Power()      do { _LATG6 = 0; _LATG3 = 0;} while(0) 

//UART module control
#define DisableUART1()            (U1MODEbits.UARTEN = 0)
#define EnableUART1()             (U1MODEbits.UARTEN = 1)
#define DisableUART2()            (U2MODEbits.UARTEN = 0)
#define EnableUART2()             (U2MODEbits.UARTEN = 1)
#define DisableUART3()            (U3MODEbits.UARTEN = 0)
#define EnableUART3()             (U3MODEbits.UARTEN = 1)




//SPI Module control
#define DisableSPI1()             (SPI1CON1Lbits.SPIEN = 0)
#define EnableSPI1()              (SPI1CON1Lbits.SPIEN = 1)

//ADC module control:
#define DisableADC()              (AD1CON1bits.ADON = 0)
#define EnableADC()               (AD1CON1bits.ADON = 1)

//BME680 
#define CS_BME680_SetHigh()          _LATE6 = 1
#define CS_BME680_SetLow()           _LATE6 = 0

//Display
#define DISP_Reset_SetHigh()        _LATE0 = 1
#define DISP_Reset_SetLow()         _LATE0 = 0
#define CS_DISP_SetHigh()           _LATE5 = 1
#define CS_DISP_SetLow()           _LATE5 = 0
#define DC_DISP_SetHigh()          _LATE7 = 1 //Pin#11 of Display interface
#define DC_DISP_SetLow()           _LATE7 = 0
#define BSY_DISP_GetValue()        _RF0 //Pin#9 of display interface

//Memory
//defined in FRAM.h

// BLUETOOTH
//defined in RN4871.h

//LoRa
#define LoRa_Reset_SetHigh()          _LATE3 = 1
#define LoRa_Reset_SetLow()           _LATE3 = 0


//WiFi
#define WIFI_Reset_SetHigh()          _LATE1 = 1
#define WIFI_Reset_SetLow()           _LATE1 = 0


//Relays
#define HEAT_SET_SetHigh()          _LATB15 = 1     
#define HEAT_SET_SetLow()           _LATB15 = 0
#define HEAT_RESET_SetHigh()        _LATD3 = 1
#define HEAT_RESET_SetLow()         _LATD3 = 0
#define COOL_SET_SetHigh()          _LATD4 = 1
#define COOL_SET_SetLow()           _LATD4 = 0
#define COOL_RESET_SetHigh()        _LATD6 = 1
#define COOL_RESET_SetLow()         _LATD6 = 0
#define FAN_SET_SetHigh()           _LATD7 = 1
#define FAN_SET_SetLow()            _LATD7 = 0        
#define FAN_RESET_SetHigh()         _LATD8 = 1
#define FAN_RESET_SetLow()          _LATD8 = 0

/*******************************************************************************
 *   Interrupt Macros
*******************************************************************************/
//Timer button is on D0 - INT0
#define TimerButton_InterruptFlagClear()    (IFS0bits.INT0IF = 0)
#define TimerButton_InterruptDisable()      (IEC0bits.INT0IE = 0)
#define TimerButton_InterruptEnable()       (IEC0bits.INT0IE = 1)
#define TimerButton_NegativeEdgeSet()          (INTCON2bits.INT0EP = 1)
#define TimerButton_PositiveEdgeSet()          (INTCON2bits.INT0EP = 0)


//Down button is on D11 - INT1
#define DownButton_InterruptFlagClear()     (IFS1bits.INT1IF = 0)
#define DownButton_InterruptDisable()       (IEC1bits.INT1IE = 0)
#define DownButton_InterruptEnable()        (IEC1bits.INT1IE = 1)
#define DownButton_NegativeEdgeSet()        (INTCON2bits.INT1EP = 1)
#define DownButton_PositiveEdgeSet()        (INTCON2bits.INT1EP = 0)

//Up button is on F4 - INT2
#define UpButton_InterruptFlagClear()       (IFS1bits.INT2IF = 0)
#define UpButton_InterruptDisable()         (IEC1bits.INT2IE = 0)
#define UpButton_InterruptEnable()          (IEC1bits.INT2IE = 1)
#define UpButton_NegativeEdgeSet()          (INTCON2bits.INT2EP = 1)
#define UpButton_PositiveEdgeSet()          (INTCON2bits.INT2EP = 0)

//Power button is on D10 - INT3
#define PowerButton_InterruptFlagClear()   (IFS3bits.INT3IF = 0)
#define PowerButton_InterruptDisable()     (IEC3bits.INT3IE = 0)
#define PowerButton_InterruptEnable()      (IEC3bits.INT3IE = 1)
#define PowerButton_NegativeEdgeSet()      (INTCON2bits.INT3EP = 1)
#define PowerButton_PositiveEdgeSet()      (INTCON2bits.INT3EP = 0)

//Bluetooth connected status - INT4
#define BLEStatus_InterruptFlagclear()    (IFS3bits.INT4IF = 0)
#define BLEStatus_InterruptDisable()      (IEC3bits.INT4IE = 0)
#define BLEStatus_InterruptEnable()       (IEC3bits.INT4IE = 1)
#define BLEStatus_NegativeEdgeSet()       (INTCON2bits.INT4EP = 1)
#define BLEStatus_PositiveEdgeSet()       (INTCON2bits.INT4EP = 0)  

#define P24vacGOOD_GetValue()                  _RB3


typedef enum 
{
    /* ----- Traps ----- */
    TRAPS_OSC_FAIL = 0, /** Oscillator Fail Trap vector */
    TRAPS_STACK_ERR = 1, /** Stack Error Trap Vector */
    TRAPS_ADDRESS_ERR = 2, /** Address Error Trap Vector */
    TRAPS_MATH_ERR = 3, /** Math Error Trap Vector */
} TRAPS_ERROR_CODE;

//Globals:
extern uint16_t u16Timer1Tick1ms;
extern uint16_t Timer2Tick100ms;
extern uint16_t u16Timer3Tick1s;
extern uint16_t u16Timer4Tick10ms;
extern uint32_t u32Timer5Tick100us;

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */
void __attribute__((naked, noreturn, weak)) TRAPS_halt_on_error(uint16_t code);

void OSCILLATOR_Initialize(void);
void PIN_MANAGER_Initialize(void);
void INTERRUPT_Initialize (void);

void TMR1_Initialize (void);
void TMR1_Start( void );
void TMR1_Stop( void );
void TMR1_Reset(void);

void TMR2_Initialize (void);
void TMR2_Start( void );
void TMR2_Stop( void );
void TMR2_Reset(void);

void TMR3_Initialize (void);
void TMR3_Start( void );
void TMR3_Stop( void );
void TMR3_Reset(void);

void TMR4_Initialize (void);
void TMR4_Start( void );
void TMR4_Stop(void);
void TMR4_Reset(void);

void TMR5_Initialize (void);
void TMR5_Start( void );
void TMR5_Stop( void );
void TMR5_Reset(void);

void SPI1_Initialize (void);
void SPI1_Write8bit(uint8_t data);
uint8_t SPI1_Exchange8bit(uint8_t data);
uint16_t SPI1_Exchange16bit(uint16_t data);

void UART1_Initialize(void);
void UART1_EnableFlowControl(void);
void UART1_SendString(const char *msg);
void UART1_SendByte(uint8_t msg);
void UART1_SendBytes(const uint8_t *byte, uint8_t startIndex, uint8_t cmdLen);
void UART1_FlushRxFIFO(void);
bool UART1_IsDataAvailable(void);

void UART2_Initialize(void);
void UART2_SendString(char *msg);
void UART2_SendBytes(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen);
void UART2_SendByte(uint8_t byte);

void UART3_Initialize(void);
void UART3_SendByte(uint8_t msg);
void UART3_SendString(char *msg);
void UART3_SendBytes(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen);



void ADC1_Initialize (void);
uint16_t ADC1_ReadChannel(uint8_t ch);
//uint8_t ReadBoardTemperature(uint8_t location);



void CRYPTO_Initialize(void);
uint8_t CRYPTO_Get8BitRandomNumber(void);

uint32_t ReadSerialNumber(void);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* HAL_H */

