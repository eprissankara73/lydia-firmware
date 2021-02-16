 /*
 *      
 *      File:   main.c
 *      Author: Madhu Annapragada
 *      Company: Automation Research Group
 *               3401 Grays Ferry Ave, B197, STE305
 *               Philadelphia, PA 19146 
 *               302-897-7776
 *      Created on February 11, 2016
 * 
 */

#include <xc.h>
#include "Main.h"
#include "HAL.h"
#include "RN4871.h"
#include "system.h"


char BTH_ADV_PWR[] = "3"; //0 to 5 , 0 = highest advertisement power
char BTH_CON_PWR[] = "3"; //0 to 5 , 0 = highest connection power

//GLOBALS:
volatile uint16_t BTH_Rx_DataIndex = 0;
uint8_t BTH_RxData[BTH_BUFFER_SIZE] = {0};
volatile uint8_t BTH_DataAvailable = 0;
bool BLE_Connected = false;

/*
 * RN4871 Initialize
 * 
 * Power is set to 3 (mid range) 
 * Default services are device info and transparent UART only - Line 82
 * Beacons and adv are enabled - Line 88
 * Min interval = 1s, max = 2s, latency = 80, timeout = 2s - Line 78
 */
uint8_t RN4871_Init(void)
{
    char tmpStr[40] = {0};
    
    //Waking the module up
    RN4871_RTS_SetHigh();
    RN4871_Wake_SetLow();
    __delay_ms(5);
    
    RN4871_Reset();
    
    //Going into command mode:    
    RN4871_Send_Command("$$$"); //this should return a cmd within 10ms
    __delay_ms(25);
    
    //RN4871_Verify();
    
    //Disabling command prompt:
    RN4871_Send_Command("SR,4000\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    /*Setting connection parameters:
     * RN4871 user guide page # 24
     * 
     * MIN INTERVAL - SET IN 1.25ms increments
     * Android expects 16.25ms minimum communication interval
     * iOS expects this value >= 16
     * Setting min interval to 1000ms = 800(0x0320)[old value->]16.25 => = 13 in 1.25ms increments = 0x0D
     * 
     * MAX INTERVAL - SET IN 1.25ms increments
     * max interval >=20 for iOS -> setting it to 2000ms = 1600 (0x0540)[old value] 600ms = 480 (1E0) in 1.25ms units
     * 
     * Latency  < (Timeout*10) / (Interval * 1.25 -1) <= 80 (0x0050) [old value] 34 - setting to 30 (0x1E)
     *       
     * Timeout : maximum time allowed before link is considered lost:
     *           The app would normally be getting data every 5 seconds
     *           Setting the timeout to 2seconds = 2000ms = 200 in 10ms units = 0x00C8[old value] 0x7D0
     * iOs requires (interval+16)*(latency+1) < Timeout * 8/3
     *              466 * 11 < 5333 - OK
     * 
    */
    
//    // av: max interval is 400 ms, timeout is 4 seconds, 
//    //     4000/(240*1.25 -1 ) = 13.3779; using value of 11
//    RN4871_Send_Command("ST,000D,00F0,000B,0190\r"); // trial 1 => stream open takes a long time close to 20 secs; but the rest of the protocol executes in 10 secs
    // av: max interval is 400 ms, timeout is 4 seconds, 
    //     4000/(300*1.25 -1 ) = 10.665; using value of 10
    RN4871_Send_Command("ST,000D,012C,000A,0190\r"); // trial 2 => discover services is fast, but then it takes a long time for stream open to happen; but the rest of the protocol executes very quickly
    //RN4871_Send_Command("ST,000D,01E0,000A,07D0\r");
    //These settings make the adv interval longer..
    //RN4871_Send_Command("ST,0320,0540,0050,00C8\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting default services to device info, Transparent UART:
    RN4871_Send_Command("SS,C0\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //enabling  beacon with connectable adv:
    RN4871_Send_Command("SC,2\r"); //
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
 
    //Setting up the model name with the serial number appended
    strcpy((char*)tmpStr,"SN,");
    strcat((char*)tmpStr,(char*)MODEL_NAME);
    strcat((char*)tmpStr,(char*)SerialNum);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting up Manufacturer Name:
    strcpy((char*)tmpStr,"SDN,");
    strcat((char*)tmpStr,(char*)MFG_NAME);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting up Hardware Revision:
    strcpy((char*)tmpStr,"SDH,");
    strcat((char*)tmpStr,(char*)HARDWARE_REVISION);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }    
    
    //setting up Software Revision:
    strcpy((char*)tmpStr,"SDR,");
    strcat((char*)tmpStr,(char*)SOFTWARE_REV);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //setting up Serial Number
    strcpy((char*)tmpStr,"SDS,");
    strcat((char*)tmpStr,(char*)SerialNum);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }    
    
    //Setting advertisements and connection to power specified
    strcpy((char*)tmpStr,"SGA,");
    strcat((char*)tmpStr,(char*)BTH_ADV_PWR);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }     
    strcpy((char*)tmpStr,"SGC,");
    strcat((char*)tmpStr,(char*)BTH_CON_PWR);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting up configurable IO to make P12 as Status1
    RN4871_Send_Command("SW,0A,07\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //enabling UART_Rx_IND
    RN4871_Send_Command("SW,0C,04\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
//    // read version
//    RN4871_Send_Command("V\r");
//    if (!RN4871_Check_For_AOK(400))
//    {
//        //return 0;
//    }
    
    //RN4871_Reboot(1); /* to make the changes take effect */
    RN4871_Reboot(2); // stay in data mode, do not do anything else

    return 1;
    
}

uint8_t RN4871_Clear_All_Services(void)
{
     //Clearing up all settings and services:
    RN4871_Send_Command("PZ\r");
    if (!RN4871_Check_For_AOK(500))
    {
        return 0;
    }
    return 1;
}

/**
 * Set up Private Service
 * 
 * @param serviceUUID
 * @return 
 */
uint8_t RN4871_SetUp_Private_Service(const char *serviceUUID)
{
    char tmpStr[40] = {0};
    strcpy((char*)tmpStr,"PS,");
    strcat((char*)tmpStr,(char*)serviceUUID);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    if (!RN4871_Check_For_AOK(3000))
    {
        return 0;
    }
    return 1;
}

/**
 * Set up Private Characteristic
 * 
 * @param characteristicUUID - 128 bit private UUID
 * @param rwi: property - read = 2, write_no_response = 4,write = 8, notify = 16, indicate = 32
 * @param charcLen - number of bytes of data
 * @return 
 */
uint8_t RN4871_SetUp_Private_Characteristic(const char *characteristicUUID, uint8_t rwi, uint8_t charcLen)
{
    char tmpStr[50] = {0};
    char tmpStr2[10] = {0};
    
    strcpy((char*)tmpStr,"PC,");
    strcat((char*)tmpStr,(char*)characteristicUUID);
    strcat((char*)tmpStr,",");
    sprintf ( (char*) tmpStr2, "%02X,%02X\r", rwi,charcLen); 
    strcat((char*)tmpStr, tmpStr2);     
    RN4871_Send_Command(tmpStr);
    
    if (!RN4871_Check_For_AOK(3000))
    {
        return 0;
    }    
    return 1;
}

/**
 * Set Characteristic Value
 * 
 * Uses the command SHW to update the local characteristic
 * 
 * @param handle to the private characteristic
 * @param data uint16_t
 * @param len - number of hex bytes to be sent - this should correspond to the 
 *        length allocated to the data field when setting up the characteristic
 * @return 1 if successful, 0 on failure
 */
uint8_t RN4871_Set_Characteristic_Value_Word(const char *handle, uint16_t data)
{
    char tmpStr[40] = {0};
    char tmpStr2[10] = {0};
    
    strcpy((char*)tmpStr,"SHW,");
    strcat((char*)tmpStr,(char*)handle);
    strcat((char*)tmpStr,",");
    //TODO: note that we always send out 2 bytes of data - have to fix this
    sprintf ( (char*) tmpStr2, "%04X\r", data);
    strcat((char*)tmpStr, tmpStr2);     
    RN4871_Send_Command(tmpStr);
    
    if (!RN4871_Check_For_AOK(300))
    {
        return 0;
    }    
    return 1;
}

uint16_t RN4871_Read_Characteristic_Value_Word(const char *handle)
{
    char tmpStr[20] = {0};
    uint16_t retval = 0;
    
    strcpy((char*)tmpStr,"SHR,");
    strcat((char*)tmpStr,(char*)handle);
    strcat((char*)tmpStr,"\r");
    RN4871_Send_Command(tmpStr);
    
    //waiting for response for 1second:
    if (!RN4871_Wait_For_Response(1000))
    {
        return 0; //exit if no response
    }
    //TODO: change from expecting 4 numbers to make this function more generic.
    //converting string to uint16_t
    //we will always get 4 numbers in a string:
    memset (tmpStr,0,20);
    strncpy ((char*)tmpStr, (const char*)BTH_RxData,4);
    retval = atol(tmpStr);
    return retval;
    
}

/**
 * Update Advertisement packet of the bluetooth module
 * 
 * @param byte - pointer to byte array to be sent as the adv packet
 * @param startIndex - start position in the byte array
 * @param cmdLen - total length of the command array
 * @return 
 */
uint8_t RN4871_UpdateAdvPacket(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen)
{
    
    uint8_t i = 0;

    char tmpStr2[10] = {0};


    RN4871_Send_Command("IA,Z\r"); //clear advertising buffer
//        if (!BT_CheckForAok(200))
//        return false;//waiting for ok    
    if (!RN4871_Check_For_AOK(200))
    {
        return 2;
    } 
 
    //now sending the message as a custom advertisement
    //RN4871_Reset_Rx_Buffer();
    RN4871_Send_Command("IA,FF,0000");
    for (i=startIndex; i< startIndex+cmdLen; i++)
    { 
       sprintf ( (char*) tmpStr2, "%X", byte[i]); 
       RN4871_Send_Command(tmpStr2);
       
    }
    RN4871_Send_Command("\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 2;
    } 
    RN4871_Send_Command("A\r"); //start advertising
    if (!RN4871_Check_For_AOK(200))
    {
        return false;//waiting for ok
    }
 
     //sending the same message out as a beacon
//    strcpy((char*)tmpStr,"IB,FF,");
//    strcat((char*)tmpStr,(char*)tmpStr2);
//    strcat((char*)tmpStr,"\r");
//    BT_SendCommand(tmpStr);
//    if (!BT_CheckForAok(200))
//    {
//        return 2;
//    } 
//    ResetBth_RxBuffer();
//    if (!BT_CheckForAok(200))
//    {
//        return false;//waiting for ok
//    }

    return 1;
    
}

void RN4871_SetLowPowerMode()
{
   
   RN4871_Send_Command("SO,1\r");
   RN4871_Wake_SetHigh();
//   RN4871_Reboot();
//   RN4871_Wake_SetLow();
//    //Going into command mode:    
//    RN4871_Send_Command("$$$"); //this should return a cmd within 10ms
//    __delay_ms(25);
//    RN4871_Send_Command("SO,1\r");    
}

/**
 * Send the disconnect command 
 */
void RN4871_Disconnect()
{
    RN4871_Send_Command("K,1\r");
    // wait for command to execute
    RN4871_Check_For_AOK(200);
    // added based on info from microchip FAE
    RN4871_Reboot(2);
}

/**
 * use this to go from command mode to data mode
 * @param mode : 1 =>  data mode, 2 => command mode
 * @param timeout
 */
uint8_t RN4871_Goto_Mode(uint8_t mode, uint16_t timeout)
{
    if (mode == 1) {
        RN4871_Send_Command("---");
        if (!RN4871_Check_For_END(timeout))
        {
            return 0;
        }    
        return 1;
    } else {
        __delay_ms(100);
        RN4871_Send_Command("$$$");
        if (!RN4871_Check_For_AOK(timeout))
        {
            return 0;
        }    
        return 1;
        
    }
    
}


/* 
 * Reset Rx Buffer
 * 
 * zeroing out the receive buffer 
 * and resetting the index to the buffer
 */
void RN4871_Reset_Rx_Buffer(void)
{
    memset (BTH_RxData, 0, BTH_BUFFER_SIZE);
    BTH_DataAvailable = 0;
    BTH_Rx_DataIndex = 0;
}


/*
 * RN4871 Check For END\r\n
 * 
 * Checks for the reception of the string "END\r\n"
 * from RN4871 for a given timeout value in ms
 * 
 * returns 1 if successful and a 0 if there is no response
 * or the response does not match AOK (could have been ERR)
 */
uint8_t RN4871_Check_For_END(uint16_t timeout)
{
    if (!RN4871_Wait_For_Response(timeout))
    {
        return 0; //exit if no response
    }
    
    if (RN4871_Check_Response("END\r\n") != 1)
    {
        return 0;
    }
    
    return 1;
}


/*
 * RN4871 Check For AOK
 * 
 * Checks for the reception of the string "AOK\r\n"
 * from RN4871 for a given timeout value in ms
 * 
 * returns 1 if successful and a 0 if there is no response
 * or the response does not match AOK (could have been ERR)
 */
uint8_t RN4871_Check_For_AOK(uint16_t timeout)
{
    if (!RN4871_Wait_For_Response(timeout))
    {
        return 0; //exit if no response
    }
    
    if (RN4871_Check_Response("AOK\r\n") != 1)
    {
        return 0;
    }
    
    return 1;
}

/*
 * RN4871 Wait For Response
 * 
 * Parameters : timeout in ms
 * 
 * Waits the specified timeout while
 * checking the receive buffer for 
 * characters from RN4871
 * 
 * Returns 1 if successful, 0 on failure
 */
uint8_t RN4871_Wait_For_Response (uint16_t timeout)
{
    uint16_t timeTick = 0;
    
    if (BTH_DataAvailable == 1)
    {
        return 1;
    }
    while (BTH_DataAvailable == 0)
    {
        __delay_ms(1);
        timeTick ++;
        if (timeTick > timeout)
        {
            return 0;
        }
    }
    
    return 1;
}


/*
 * RN4871 Wait For Response in data mode / transparent uart mode
 * 
 * Parameters : timeout in ms
 * 
 * Waits the specified timeout while
 * checking the receive buffer for 
 * characters from RN4871
 * 
 * Returns 1 if successful, 0 on failure
 */
uint8_t RN4871_Wait_For_TUart_Response (uint16_t timeout)
{
    uint16_t timeTick = 0;
    bool done = false;
    
    while ( !done )
    {
        __delay_ms(100);
        timeTick += 100;
        if (timeTick > timeout)
        {
            return 0;
        }
        if  ( (BTH_Rx_DataIndex > 0) && (BTH_Rx_DataIndex >= BTH_RxData[0] ) )
        {
            done = true;
        }
    }
    
    return 1;
}


/*
 * RN4871 Check Response
 * 
 * Parameters: pointer to a string
 * 
 * Compares the receive buffer to the string
 * The entire length of the receive buffer
 * is compared byte to byte with the string
 * 
 * returns 1 if the bytes are the same 
 * and a 0 if the bytes are different
 */
uint8_t RN4871_Check_Response(const char *msg)
{
    uint16_t i = 0;
    
    for (i=0; i< BTH_Rx_DataIndex; i++)
    {
        if (msg[i] != BTH_RxData[i])
        {
            return 0;
        }
    }
    
    return 1;
}

uint8_t RN4871_Send_String_Read_Resp(char* msg, uint16_t resp_char_count, uint16_t timeout, uint8_t* data, int len)
{
    uint8_t tmpu8;
    RN4871_Send_Command(msg);
    if (!RN4871_Wait_For_Response(timeout))
    {
        return 0; //exit if no response
    }
    tmpu8 = 0;
    while ( (tmpu8 < 5) && (BTH_Rx_DataIndex < resp_char_count ) ) {
        tmpu8++;
        __delay_ms(timeout);
    }
//    while ( (tmpu8 < 5) && (UART1_IsDataAvailable() ) ) {
//        tmpu8++;
//        __delay_ms(timeout);
//    }

    if (BTH_Rx_DataIndex >= resp_char_count)
    {
        memcpy(data, BTH_RxData, BTH_Rx_DataIndex);
        return 1;
    }
    else 
    {
        return 0;
    }

}

uint8_t RN4871_Send_Bytes_Read_Resp(uint8_t* msg, uint16_t msg_len, uint16_t timeout, uint8_t* data, int data_len)
{
    RN4871_Send_Bytes(msg, msg_len);
    if (!RN4871_Wait_For_TUart_Response(5*timeout))
    {
        return 0; //exit if no response
    }
    if (BTH_Rx_DataIndex <= data_len) 
    {
        memcpy(data, BTH_RxData, BTH_Rx_DataIndex);
    }
    else 
    {
        memcpy(data, BTH_RxData, data_len);
    }
    return 1;
}


uint8_t  RN4871_Reboot(int sequence_num)
{
    RN4871_Send_Command("R,1\r");
    
    if (RN4871_Wait_For_Response(2000) == 0)
    {
        return 0;
    }
    __delay_ms(1500); /* can take this long for the command prompt */
    
    if (sequence_num == 2) 
    {
        return 1; // stay in data mode
    }
    
    //Going into command mode:    
    RN4871_Send_Command("$$$"); //this should return a cmd withing 10ms
    __delay_ms(25);
        
    //Disabling command prompt:
    RN4871_Send_Command("SR,4000\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //clearing all previous advertisement payloads:
    RN4871_Send_Command("IA,Z");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //starting Advertisements with local name
    RN4871_Send_Command("A\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    return 1;
}

void RN4871_Reset(void)
{
     //re-setting the module
    RN4871_Reset_SetLow();
    __delay_ms(1);
    RN4871_Reset_SetHigh();
    __delay_ms(1000);
}

/*
 * RN4871 Send Command
 * 
 * Resets the Rx buffer and sends a string out
 * via the UART interface
 */
void RN4871_Send_Command (const char *msg)
{
    RN4871_Reset_Rx_Buffer();
    UART1_SendString(msg);
}

/*
 * RN4871 Send Bytes
 * 
 * Resets the Rx buffer and sends a string out
 * via the UART interface
 */
void RN4871_Send_Bytes (const uint8_t *msg, uint16_t msg_len)
{
    RN4871_Reset_Rx_Buffer();
    UART1_SendBytes(msg, 0, msg_len);
}


/**
 * Set Characteristic Value
 * 
 * Uses the command SHW to update the local characteristic
 * 
 * @param handle to the private characteristic
 * @param data uint8_t *
 * @param len - number of hex bytes to be sent - this should correspond to the 
 *        length allocated to the data field when setting up the characteristic
 * @return 1 if successful, 0 on failure
 */
uint8_t RN4871_Write_Characteristic_Value(const char *handle, uint8_t* data, int len)
{
    int ii;
    char tmpStr[100] = {0};
    char tmpStr2[10] = {0};
    
    strcpy((char*)tmpStr,"SHW,");
    strcat((char*)tmpStr,(char*)handle);
    strcat((char*)tmpStr,",");
    for (ii = 0; ii < len; ii++)
    {
        if (ii != len-1)
        {
            sprintf ( (char*) tmpStr2, "%02X", data[ii]);
        }
        else
        {
            // last byte of data, append "\r"
            sprintf ( (char*) tmpStr2, "%02X\r", data[ii]);
        }
        strcat((char*)tmpStr, tmpStr2);     
    }
    RN4871_Send_Command(tmpStr);
    
    if (!RN4871_Check_For_AOK(300))
    {
        return 0;
    }    
    return 1;
}


/**
 * Reads the characteristic value and populates the data array
 *      the array must be pre-allocated before this call
 *      The len variable provides the length of data that will be read into the buffer
 * @param handle
 * @param data
 * @param len
 * @return 
 *      1 on success, 0 on failure
 */
uint8_t RN4871_Read_Characteristic_Value(const char *handle, uint8_t* data, int len)
{
    
    // len is the number of bytes
    // rn4871 returns hex characters, so its length will be 2*len
    char tmpStr[20] = {0};
    int tmpi, tmpj;
    uint8_t num_tries = 0;
    bool got_data = false;

    if ( (2*len + 2) > BTH_BUFFER_SIZE ) // 2 hex characters for each byte and the last 2 is for \r\n
    {
        // insufficient memory
        return 0;
    }

    strcpy((char*)tmpStr,"SHR,");
    strcat((char*)tmpStr,(char*)handle);
    strcat((char*)tmpStr,"\r");
    while ( (num_tries < BTH_MAX_RETRIES) && (!got_data) )
    {
        RN4871_Send_Command(tmpStr);

        //waiting for response for 1second:
        if (!RN4871_Wait_For_Response(1000))
        {
            //num_tries++;
            //return 0; //exit if no response
        }
        // got a N/A response; try again
        if (strncmp(BTH_RxData,"N/A", 3) == 0 )  
        {
            num_tries ++;
            __delay_ms(BTH_SLEEP_BETWEEN_COMMANDS_MS);
        }
        else if (strncmp(BTH_RxData, "ERR", 3) == 0) 
        {
            // error return immediately
            num_tries = BTH_MAX_RETRIES; // allows the function to exit the while loop; got_data is still false
        }
        else if ( (BTH_RxData[0] == 0x00) && (BTH_RxData[1] == 0x00) && (BTH_RxData[2] == 0x00))
        {
            num_tries ++;
            __delay_ms(BTH_SLEEP_BETWEEN_COMMANDS_MS);
        }
        else 
        {
            if (BTH_Rx_DataIndex <= (2*len))
            {
                // not sufficient data; return
                num_tries = BTH_MAX_RETRIES;
            }
            else 
            {
                tmpi = 0;
                tmpj = 0;
                while (tmpi < 2*len)
                {
                    data[tmpj] = (uint8_t) ( ( ( ( BTH_RxData[tmpi] %32 ) + 9 ) % 25 ) * 16 + ( ( BTH_RxData[tmpi+1] %32 ) + 9 ) % 25 );  // convert ascii 2 characters to value
                    tmpi += 2;
                    tmpj++;
                }
                got_data = true;
            }
        }
    }
    if (got_data)
    {
        return 1;
    }
    else 
    {
        return 0;
    }
    
}

uint8_t RN4871_Get_Handles_Characteristics(const char *serviceUUID)
{
    char tmpStr[50] = {0};
    int try_count = 0;
    int tmpi_sts = 0;
    
    //strcpy((char*)tmpStr,"LS,");
    //strcat((char*)tmpStr,(char*)serviceUUID);
    //strcat((char*)tmpStr,"\r");
    sprintf(tmpStr, "LS,%s\r", serviceUUID);
    //sprintf(tmpStr, "LS\r", serviceUUID);
    RN4871_Send_Command(tmpStr);
    tmpi_sts = RN4871_Check_For_END(5000); 
    while ( (tmpi_sts != 1) && (try_count < 5) )
    {
        __delay_ms(1000); // is this neceasary
        tmpi_sts = RN4871_Check_For_END(5000); 
        try_count ++;
    }    
    if (tmpi_sts == 1)
    {
        return 1;
    }
    else 
    {
        return 0;
    }
}

uint8_t RN4871_Is_Stream_Ready(int timeout) 
{
    uint8_t tmpu8;
    uint16_t timeTick;
    timeTick = 0;
    while (BTH_Rx_DataIndex == 0)
    {
        __delay_ms(250);
        timeTick += 250;
        if (timeTick > timeout)
        {
            return 0;
        }
    }
    tmpu8 = 0;
    while ( timeTick < timeout ) 
    {
        tmpu8 = findSubstring((char*)BTH_RxData, "STREAM_OPEN");
        if ( (BTH_Rx_DataIndex > 0) && ( tmpu8 == 1 ) )
        {
            return 1;
        }
        else
        {
            __delay_ms(250);
            timeTick += 250;
        }
    }
    return 0;
}

/*
 * RN4871 Verify
 * 
 * verify that the values set are true
 * only for testing, do not use in production
 */
uint8_t RN4871_Verify(void)
{
    //char tmpStr[40] = {0};
    
    
    //Disabling command prompt:
//    RN4871_Send_Command("GR\r");
//    if (!RN4871_Check_For_AOK(5000))
//    {
//        return 0;
//    }
    
    /*Read connection parameters:
     * 
    */
    
//    RN4871_Send_Command("GT\r"); 
//    if (!RN4871_Check_For_AOK(200))
//    {
//        return 0;
//    }
    
    //Setting default services to device info, Transparent UART:
//    RN4871_Send_Command("GS\r");
//    if (!RN4871_Check_For_AOK(5000))
//    {
//        return 0;
//    }
    
    //enabling  beacon with connectable adv:
    RN4871_Send_Command("GC\r\n"); //
    if (!RN4871_Check_For_AOK(5000))
    {
        return 0;
    }
    
 
    //Setting up the model name with the serial number appended
    RN4871_Send_Command("GN\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting up Manufacturer Name:
    RN4871_Send_Command("GDN\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    RN4871_Send_Command("GDH\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }    
    
    //setting up Software Revision:
    RN4871_Send_Command("GDR\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //setting up Serial Number
    RN4871_Send_Command("GDS\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }    
    
    //Setting advertisements and connection to power specified
    RN4871_Send_Command("GGA\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }     
	
    RN4871_Send_Command("GGC\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //Setting up configurable IO to make P12 as Status1
    RN4871_Send_Command("GW\r");
    if (!RN4871_Check_For_AOK(200))
    {
        return 0;
    }
    
    //enabling UART_Rx_IND
    //RN4871_Send_Command("SW,0C,04\r");
    //if (!RN4871_Check_For_AOK(200))
    //{
    //    return 0;
    //}
    
//    // read version
//    RN4871_Send_Command("V\r");
//    if (!RN4871_Check_For_AOK(400))
//    {
//        //return 0;
//    }
    
    //RN4871_Reboot(1); /* to make the changes take effect */
    return 1;
    
}
