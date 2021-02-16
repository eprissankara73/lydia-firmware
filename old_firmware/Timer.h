/* 
 * File: Timer.h
 * Author: Evan Giarta
 * Comments: 2018-05-03
 * Revision history: Initial testing for Microchip LCDExplorer XLP
 */

#ifndef TIMER_H
#define TIMER_H

extern void InitTimer();

#define TIMER1_ON   T1CONbits.TON
#define SOCS_32KHZ_EN  OSCCONbits.SOSCEN

/* Definitions *****************************************************/
#define TMR1_PERIOD 0x0020 // to adjust frequency, need to determine formula
#define TIMER_16BIT_MODE 0x0000
#define TIMER_INTERRUPT_PRIORITY 0x0001
#define TIMER_INTERRUPT_PRIORITY_4 0x0004

/*
 * T1CON: TIMER1 CONTROL REGISTER
 */
/* TCS */
#define TIMER_SOURCE_INTERNAL 0x0000
#define TIMER_SOURCE_EXTERNAL 0x0002
/* TSYNC (When TCS = 1) */
#define ENABLE_SINC_EXTERNAL_INPUT 0x0004
#define DISABLE_SINC_EXTERNAL_INPUT 0x0000
/* TCKPS<1:0> */
#define TIMER_PRESCALER_1 0x0000
#define TIMER_PRESCALER_8 0x0010
#define TIMER_PRESCALER_64 0x0020
#define TIMER_PRESCALER_256 0x0030
/* TGATE (When TCS = 0) */
#define GATED_TIME_DISABLED 0x0000
#define GATED_TIME_ENABLED 0x0040
/* TSIDL */
#define STOP_TIMER_IN_IDLE_MODE 0x2000
/* TON */
#define TIMER_ON    0x8000
#define TIMER_OFF   0x0000

#endif //eof