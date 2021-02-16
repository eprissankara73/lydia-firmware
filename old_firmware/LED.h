/* 
 * File: LED.h
 * Author: Evan Giarta
 * Comments: 2018-05-03
 * Revision history: Initial testing for Microchip LCDExplorer XLP
 */

#ifndef LED_H
#define LED_H

extern void InitLED();
extern void SetLED(unsigned char);
extern void ToggleLED(void);
extern void LimitLED(unsigned char);

extern unsigned char CountLED;

#define LED_OUT_TRIS 	TRISFbits.TRISF6
#define LED_OUT_LAT     LATFbits.LATF6
#define LED_OUT         PORTFbits.RF6

#define LED_SET_OUTPUT  0
#define LED_ON          1
#define LED_OFF         0

#endif //eof