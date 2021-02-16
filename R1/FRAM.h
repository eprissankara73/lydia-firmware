#ifndef FRAM_H
#define	FRAM_H

#include <xc.h> // include processor files - each processor file is guarded.  

//PIN DEFINITIONS:
#define CS_MEM1_SetHigh()          _LATE4 = 1
#define CS_MEM1_SetLow()           _LATE4 = 0

//OP CODES for RAMTRON FRAM
#define WREN  0b00000110   //Write enable
#define WRDI  0b00000100 //write disable
#define RDSR  0b00000101 //Read status register
#define WRSR  0b00000001 //write status register
#define READ  0b00000011 //Read memory data
#define FSTRD 0b00001011 //Fast memory data read
#define WRITE 0b00000010 //write memory data
#define RDID  0b10011111 //Read device ID
#define SLEEP 0b10111001 //Sleep mode
        
#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

void FRAMSetChipSelect(uint8_t HighOrLow, uint8_t bank);
uint8_t  FramDataExchangeByte(uint8_t data);
uint8_t FRAMReadStatusRegister(uint8_t bank);
void FRAMClearBlockProtect(uint8_t bank);
uint32_t FRAMReadID(uint8_t bank);
void FRAMWriteEnable(uint8_t bank);
void FRAMSleep(uint8_t bank);
void FRAMWake(uint8_t bank);
void FRAMWriteByte(uint8_t data, uint32_t address, uint8_t bank);
void FRAMWriteWord(uint16_t data, uint32_t address, uint8_t bank);
void FRAMWriteLong(uint32_t data, uint32_t address, uint8_t bank);
void FRAMWriteLongLong(uint64_t data, uint32_t address, uint8_t bank);
uint8_t FRAMReadByte(uint32_t address, uint8_t bank);
uint16_t FRAMReadWord(uint32_t address, uint8_t bank);
uint32_t FRAMReadLong(uint32_t address, uint8_t bank);
uint64_t FRAMReadLongLong(uint32_t address, uint8_t bank);
void FRAMReadByteArray(uint32_t address, uint8_t bank, uint8_t * data_rd, int bytes_to_read);

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* FRAM_H */

