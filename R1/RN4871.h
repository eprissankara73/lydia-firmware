#ifndef RN4871_H
#define	RN4871_H

#include <xc.h> // include processor files - each processor file is guarded.  

/*
 * ===========================
 *  Bluetooth RN4871 Related
 * ===========================
 */
#define RN4871_Wake_SetHigh()            _LATB14 = 1
#define RN4871_Wake_SetLow()             _LATB14 = 0
#define RN4871_Reset_SetLow()            _LATE2 = 0
#define RN4871_Reset_SetHigh()           _LATE2 = 1
#define RN4871_RTS_SetHigh()             _LATD2 = 1
#define RN4871_RTS_SetLow()              _LATD2 = 0
#define RN4871_Status1                   _RB5

extern char BTH_ADV_PWR[];

#define BTH_BUFFER_SIZE 1024
extern volatile uint16_t BTH_Rx_DataIndex;
extern uint8_t BTH_RxData[BTH_BUFFER_SIZE];
extern volatile uint8_t BTH_DataAvailable;
extern bool BLE_Connected;

#define BTH_MAX_RETRIES 6
#define BTH_SLEEP_BETWEEN_COMMANDS_MS   1000

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

uint8_t RN4871_Init(void);
uint8_t RN4871_Clear_All_Services(void);
uint8_t RN4871_SetUp_Private_Service(const char *serviceUUID);
uint8_t RN4871_SetUp_Private_Characteristic(const char *characteristicUUID,uint8_t rwi, uint8_t charcLen);
uint8_t RN4871_Set_Characteristic_Value_Word(const char *handle, uint16_t data );
uint16_t RN4871_Read_Characteristic_Value_Word(const char *handle);
uint8_t RN4871_UpdateAdvPacket(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen);
void RN4871_SetLowPowerMode();
void RN4871_Reset_Rx_Buffer(void);
void RN4871_Send_Command (const char *msg);
uint8_t RN4871_Check_For_AOK(uint16_t timeout);
uint8_t RN4871_Check_For_END(uint16_t timeout);
uint8_t RN4871_Wait_For_Response (uint16_t timeout);
uint8_t RN4871_Check_Response(const char *msg);
uint8_t  RN4871_Reboot(int);
void RN4871_Reset(void);
void RN4871_Disconnect(void);
uint8_t RN4871_Write_Characteristic_Value(const char *handle, uint8_t* data, int len);
uint8_t RN4871_Read_Characteristic_Value(const char *handle, uint8_t* data, int len);
uint8_t RN4871_Send_String_Read_Resp(char* msg, uint16_t resp_char_count, uint16_t timeout, uint8_t* data, int len);
uint8_t RN4871_Send_Bytes_Read_Resp(uint8_t* msg, uint16_t msg_len, uint16_t timeout, uint8_t* data, int data_len);
uint8_t RN4871_Is_Stream_Ready(int timeout);
void RN4871_Send_Bytes (const uint8_t *msg, uint16_t msg_len);
uint8_t RN4871_Wait_For_TUart_Response (uint16_t timeout);
uint8_t RN4871_Goto_Mode(uint8_t mode, uint16_t timeout);
uint8_t RN4871_Verify(void);
#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* RN4871_H */

