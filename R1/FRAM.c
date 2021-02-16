/*
===========================================================================================
SOURCE FILE FRAM.c
PURPOSE Functions for Fujistsu Serial FRAM chips  **24BIT ADRESS SPACE**
AUTHOR Madhu Annapragada, Automation Research Group
DATE CREATED January, 20, 2011
REVISION: A                                  
   CHANGES :NONE from Rev A
                                                
                                                
:REQUIREMENTS: 
         1) SPI2 should be set up as Master with Low to high transitions
         2) All functions here are passed the bank select line for cases where
            there is more than one memory chip. Chip select lines are named
            CS_MEM1, CS_MEM2 etc.

===========================================================================================*/

#include <xc.h>
#include "Main.h"
#include "HAL.h"
#include "FRAM.h"

/**
 * Sets the chip select line high or low 
 * bank selects the memory chip to be accessed
 * 
 * @param HighOrLow 0 pulls cs low, 1 sets it high
 * @param bank memory bank. starting from 0
 */
void FRAMSetChipSelect(uint8_t HighOrLow, uint8_t bank)
{
    switch (bank)
    {
        case 0:
            if (HighOrLow == 0)
            {
                CS_MEM1_SetLow();
            }
            else
            {
                CS_MEM1_SetHigh();
            }
            break;
        case 1: //Not Used in LIT
            if (HighOrLow == 0)
            {
               // CS_MEM2_SetLow();
            }
            else
            {
               // CS_MEM2_SetHigh();
            }
            break;
        default:
            if (HighOrLow == 0)
            {
                CS_MEM1_SetLow();
            }
            else
            {
                CS_MEM1_SetHigh();
            }
            break;
    }
}

/**
 * USES SPI 1 bus - change as needed
 * @param data
 * @return 
 */
uint8_t  FramDataExchangeByte(uint8_t data)
{
    uint8_t retval = 0;
    retval = SPI1_Exchange8bit(data);
    return retval;
}

/**
 *          READ STATUS REGISTER
 * @param bank
 * @return  byte with the value of the status register
 */
uint8_t FRAMReadStatusRegister(uint8_t bank)
{
    uint8_t status = 0;
    
    FRAMSetChipSelect(0,bank);    
    FramDataExchangeByte(RDSR);
    status = FramDataExchangeByte(0);    
    FRAMSetChipSelect(1,bank);
    
    return status;
}

void FRAMClearBlockProtect(uint8_t bank)
{
    FRAMSetChipSelect(0,bank);    
    FramDataExchangeByte(WRSR);
    FramDataExchangeByte(0);    
    FRAMSetChipSelect(1,bank);
}

uint32_t FRAMReadID(uint8_t bank)
{
   uint8_t datalsb, data1, data2, datamsb;
   uint32_t data = 0;

   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(RDID); //first the read op code   
   datalsb = FramDataExchangeByte(0); //eight dummy bytes to generate the clock required to read in data
   data1 = FramDataExchangeByte(0);
   data2 = FramDataExchangeByte(0);
   datamsb = FramDataExchangeByte(0);
   FRAMSetChipSelect(1,bank);
   data = ((uint32_t)datamsb <<24) | ((uint32_t)data2 <<16) | ((uint32_t)data1 <<8) | datalsb;
   return data;
    
}
void FRAMWriteEnable(uint8_t bank)
{
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(WREN); //enable write mode
   FRAMSetChipSelect(1,bank);
    
}

/**
 * Puts device into sleep
 * Wake up is automatic on any high to low transition of the CS line
 * @param bank
 */
void FRAMSleep(uint8_t bank)
{
    FRAMSetChipSelect(0,bank);
    FramDataExchangeByte(SLEEP);
    FRAMSetChipSelect(1,bank);
}

/**
 *  Wake up the FRAM before doing any
 * read / writes 
 */
void FRAMWake(uint8_t bank)
{
    FRAMSetChipSelect(0,bank);
    __delay_us(500);
    FRAMSetChipSelect(1,bank);
}

/*=========================================
      Function: FRAM Write Byte 
- writes a byte of data to the specified 
address
(24bit address - )
==========================================*/
void FRAMWriteByte(uint8_t data, uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0, status;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   //enabling the write enable latch:
   FRAMWriteEnable(bank); 
   
   //checking on the state of the WEL bit in the status register:
   status = FRAMReadStatusRegister(bank);
   while ((status & 2) != 2)
   {
       __delay_us(10);
       status = FRAMReadStatusRegister(bank);
       //checking to see if block protection is on and disabling it if it is:
       if (((status & 4) == 4) || ((status & 8) == 8))
       {
           FRAMClearBlockProtect(bank);
           FRAMWriteEnable(bank);
       }
   }
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(WRITE); //first the write op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   FramDataExchangeByte(data); //eight bytes of data
   FRAMSetChipSelect(1,bank);
   
   return;
}

/*=========================================
      Function: FRAM Write Word
- writes 2 bytes of data to the specified 
address
(24bit address)
==========================================*/
void FRAMWriteWord(uint16_t data, uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0, status, temp;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   //enabling the write enable latch:
    FRAMWriteEnable(bank);
   
   //checking on the state of the WEL bit in the status register:
   status = FRAMReadStatusRegister(bank);
   while ((status & 2) != 2)
   {
       //checking to see if block protection is on and disabling it if it is:
       if (((status & 4) == 4) || ((status & 8) == 8))
       {
           FRAMClearBlockProtect(bank);
           FRAMWriteEnable(bank);
       }
       __delay_us(10);
       status = FRAMReadStatusRegister(bank);
       //checking to see if block protection is on and disabling it if it is:
       if (((status & 4) == 4) || ((status & 8) == 8))
       {
           FRAMClearBlockProtect(bank);
           FRAMWriteEnable(bank);
       }
   }
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(WRITE); //first the write op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   temp = (uint8_t)(data); //lsb first   
   FramDataExchangeByte(temp); //eight bytes of data
   temp = (uint8_t)(data>>8); //msb  
   FramDataExchangeByte(temp); //eight bytes of data
   FRAMSetChipSelect(1,bank);
   
   return;
}



/*=========================================
      Function: FRAM Write Long
- writes 4 bytes of data to the specified 
address
(24bit address)
==========================================*/
void FRAMWriteLong(uint32_t data, uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0,status, temp;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   //enabling the write enable latch:
   FRAMWriteEnable(bank);
   
   //checking on the state of the WEL bit in the status register:
   status = FRAMReadStatusRegister(bank);
   while ((status & 2) != 2)
   {
       __delay_us(10);
       status = FRAMReadStatusRegister(bank);
       if (((status & 4) == 4) || ((status & 8) == 8))
       {
           FRAMClearBlockProtect(bank);
           FRAMWriteEnable(bank);
       }
   }
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(WRITE); //first the write op code
   FramDataExchangeByte(byte2); //next the two address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   temp = (uint8_t)(data); //lsb first   
   FramDataExchangeByte(temp); //eight bytes of data
   temp = (uint8_t)(data>>8);   
   FramDataExchangeByte(temp); //eight bytes of data
   temp = (uint8_t)(data>>16);   
   FramDataExchangeByte(temp); //eight bytes of data
   temp = (uint8_t)(data>>24);   
   FramDataExchangeByte(temp); //eight bytes of data
   FRAMSetChipSelect(1,bank);
   return;
}

/*=========================================
      Function: FRAM Write Long Long
- writes 8 bytes of data to the specified 
address
(24bit address)
==========================================*/
void FRAMWriteLongLong(uint64_t data, uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0,status, temp;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   //enabling the write enable latch:
   FRAMWriteEnable(bank);
   
   //checking on the state of the WEL bit in the status register:
   status = FRAMReadStatusRegister(bank);
   while ((status & 2) != 2)
   {
       __delay_us(10);
       status = FRAMReadStatusRegister(bank);
       if (((status & 4) == 4) || ((status & 8) == 8))
       {
           FRAMClearBlockProtect(bank);
           FRAMWriteEnable(bank);
       }
   }
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(WRITE); //first the write op code
   FramDataExchangeByte(byte2); //next the two address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   temp = (uint8_t)(data); //lsb first   
   FramDataExchangeByte(temp); //byte 0
   temp = (uint8_t)(data>>8);   
   FramDataExchangeByte(temp); //byte 1
   temp = (uint8_t)(data>>16);   
   FramDataExchangeByte(temp); //byte 2
   temp = (uint8_t)(data>>24);   
   FramDataExchangeByte(temp); //byte 3
   temp = (uint8_t)(data>>32);   
   FramDataExchangeByte(temp); //byte 4
   temp = (uint8_t)(data>>40);   
   FramDataExchangeByte(temp); //byte 5
   temp = (uint8_t)(data>>48);   
   FramDataExchangeByte(temp); //byte 6
   temp = (uint8_t)(data>>56);   
   FramDataExchangeByte(temp); //byte 7
   FRAMSetChipSelect(1,bank);
   return;
}

/*=========================================
      Function: FRAM read
- returns a byte of data from the specified 
address
(24bit address)
==========================================*/
uint8_t FRAMReadByte(uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0, data;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(READ); //first the read op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   data = FramDataExchangeByte(0); //eight dummy bytes to generate the clock required to read in data
   FRAMSetChipSelect(1,bank);
   return data;
}

/*=========================================
      Function: FRAM read word
- returns two bytes of data from the specified 
address
(24bit address)
==========================================*/
uint16_t FRAMReadWord(uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0,datalsb, datamsb;
   uint16_t data = 0;
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(READ); //first the read op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   datalsb = FramDataExchangeByte(0); //eight dummy bytes to generate the clock required to read in data
   datamsb = FramDataExchangeByte(0);
   FRAMSetChipSelect(1,bank);
   data = (uint16_t)datamsb <<8 | datalsb;
   return data;
}

/*=========================================
      Function: FRAM read long
- returns four bytes of data from the specified 
address
(24bit address)
==========================================*/
uint32_t FRAMReadLong(uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0, datalsb, data1, data2, datamsb;
   uint32_t data = 0;
   
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(READ); //first the read op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   datalsb = FramDataExchangeByte(0); //eight dummy bytes to generate the clock required to read in data
   data1 = FramDataExchangeByte(0);
   data2 = FramDataExchangeByte(0);
   datamsb = FramDataExchangeByte(0);
   FRAMSetChipSelect(1,bank);
   data = ((uint32_t)datamsb <<24) | ((uint32_t)data2 <<16) | ((uint32_t)data1 <<8) | datalsb;
   return data;
}

/*=========================================
      Function: FRAM read long long
- returns eight bytes of data from the specified 
address
(24bit address)
==========================================*/
uint64_t FRAMReadLongLong(uint32_t address, uint8_t bank) 
{
   uint8_t byte2,byte1,byte0, datalsb, data1, data2, data3, data4, data5, data6, datamsb;
   uint64_t data = 0;
   
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(READ); //first the read op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   datalsb = FramDataExchangeByte(0); //eight dummy bytes to generate the clock required to read in data
   data1 = FramDataExchangeByte(0);
   data2 = FramDataExchangeByte(0);
   data3 = FramDataExchangeByte(0);
   data4 = FramDataExchangeByte(0);
   data5 = FramDataExchangeByte(0);
   data6 = FramDataExchangeByte(0);
   datamsb = FramDataExchangeByte(0);
   FRAMSetChipSelect(1,bank);
   data = ((uint64_t)datamsb <<56) | ((uint64_t)data6 <<48) | ((uint64_t)data5 <<40) |
           ((uint64_t)data4<<32) | ((uint64_t)data3 <<24) | ((uint64_t)data2 <<16) | 
           ((uint64_t)data1 <<8) | datalsb;
   return data;
}

/*=========================================
      Function: FRAM read "length" bytes into uint8_t data array passed as the last parameter
 no error checking, calling funciton needs to ensure uint8_t array is correct
- returns eight bytes of data from the specified 
address
(24bit address)
==========================================*/
void FRAMReadByteArray(uint32_t address, uint8_t bank, uint8_t * data_rd, int bytes_to_read) 
{
   uint8_t byte2,byte1,byte0; //, datalsb, data1, data2, data3, data4, data5, data6, datamsb;
   int ii;
   
   byte2 = (uint8_t)(address >>  16);
   byte1 = (uint8_t)(address >>  8);
   byte0 = (uint8_t)(address);
   
   FRAMSetChipSelect(0,bank);
   FramDataExchangeByte(READ); //first the read op code
   FramDataExchangeByte(byte2); //next the three address bytes - MSB first
   FramDataExchangeByte(byte1);
   FramDataExchangeByte(byte0);
   for (ii=0; ii<bytes_to_read; ii++) 
   {
       data_rd[ii] = FramDataExchangeByte(0);
   }
   FRAMSetChipSelect(1,bank);
}
