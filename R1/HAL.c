// FSEC
#pragma config BWRP = OFF               // Boot Segment Write Protect (Boot segment may be written)
#pragma config BSS = OFF                // Boot segment Protect (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write Protect (Writes to program memory are allowed)
#pragma config GSS = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config CWRP = OFF               // Configuration Segment Program Write Protection bit (Configuration Segment may be written)
#pragma config CSS = DIS                // Configuration Segment Code Protection Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = DISABLE        // Alternate Interrupt Vector Table Disable bit (Disable AIVT)

// FBSLIM
//#pragma config BSLIM = 0xFFFFFF         // Boot Segment Code Flash Page Address Limit bits (Enter Hexadecimal value)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with divide-by-n (FRCDIV))
#pragma config PLLMODE = PLL4X          // Frequency Multiplier Select Bits (No PLL used; PLLEN bit is not available)
#pragma config IESO = ON                // Internal External Switchover (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMOD = NONE           // Primary Oscillator Select (Primary Oscillator disabled)
#pragma config OSCIOFCN = ON            // OSCO Pin Configuration (OSCO/CLKO/RC15 functions as port I/O (RC15))
#pragma config SOSCSEL = ON             // SOSC Power Selection Configuration bits (SOSC is used in crystal (SOSCI/SOSCO) mode)
#pragma config PLLSS = PLL_PRI          // PLL Secondary Selection Configuration bit (PLL is fed by the Primary oscillator)
#pragma config IOL1WAY = OFF             // IOLOCK One-Way Set Enable (The IOLOCK bit can be set and cleared using the unlock sequence)
#pragma config FCKSM = CSECMD           // Clock Switching and Monitor Selection (Clock switching enabled, Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPS = PS1024           // Watchdog Timer Postscaler->1:1024
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard Watchdog Timer enabled (Windowed-mode is disabled))
#pragma config WDTWIN = PS25_0          // Watchdog Window Select bits (Watch Dog Timer Window Width is 25 percent)
#pragma config WDTCMX = WDTCLK          // WDT Clock Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)

// FPOR
#pragma config BOREN = OFF               // Brown-out Reset Enable bits (Brown-out Reset Enable)
#pragma config LPCFG = ON               // Low power regulator control (Enabled)

// FICD
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)
#pragma config BTSWP = OFF              // BOOTSWP Instruction Enable bit (BOOTSWP instruction is disabled)

// FDS
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68,719,476,736 (25.7 days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select bit (DSWDT uses Low Power RC Oscillator (LPRC))
#pragma config DSBOREN = OFF             // Deep Sleep Zero-Power BOR Enable bit (Deep Sleep BOR enabled in Deep Sleep)
#pragma config DSWDTEN = ON            // Deep Sleep Watchdog Timer Enable bit (DSWDT disabled)
#pragma config DSSWEN = ON              // Deep Sleep Software Control Select Bit (Deep Sleep disabled)

// FDEVOPT1
#pragma config ALTCMPI = DISABLE        // Alternate Comparator Input Enable bit (C1INC, C2INC, and C3INC are on their standard pin locations)
#pragma config TMPRPIN = OFF            // Tamper Pin Enable bit (TMPRN pin function is disabled)
#pragma config TMPRWIPE = OFF           // RAM Based Entryption Key Wipe Enable bit (Cryptographic Engine Key RAM is not erased onTMPR pin events)
#pragma config ALTVREF = ALTVREFDIS     // Alternate VREF location Enable (VREF is on a default pin (VREF+ on RA10 and VREF- on RA9))

// FBOOT
#pragma config BTMODE = SINGLE    // Boot Mode Select bits->Device is in Single Boot (legacy) mode

#include <xc.h>
#include <p24FJ256GB406.h>
#include "hal.h"
#include "RN4871.h"
#include "FRAM.h"

#define ERROR_HANDLER __attribute__((interrupt,no_auto_psv))
#define ERROR_HANDLER_NORETURN ERROR_HANDLER __attribute__((noreturn))
#define FAILSAFE_STACK_GUARDSIZE 8

/**
 * a private place to store the error code if we run into a severe error
 */
static uint16_t TRAPS_error_code = -1;

//Globals INIT:
uint16_t u16Timer1Tick1ms = 0;
uint16_t Timer2Tick100ms = 0;
uint16_t u16Timer3Tick1s = 0;
uint16_t u16Timer4Tick10ms = 0;
uint32_t u32Timer5Tick100us = 0;



/*******************************************************************************
*                   UART1 [Bluetooth] INTERRUPT
*******************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void)
{
     BTH_RxData[BTH_Rx_DataIndex] = U1RXREG;
    
    if (BTH_RxData[BTH_Rx_DataIndex] == 10) //we have a line feed:
    {
        BTH_DataAvailable = 1;
    }
    
    BTH_Rx_DataIndex++;
    IFS0bits.U1RXIF = 0; //clearing U4 - bluetooth interrupt

}
void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1ErrInterrupt ( void )
{
    if ((U1STAbits.OERR == 1)) //just in case we get an overruun error - we do want
    {                          //to clear it up and keep going..
        U1STAbits.OERR = 0;
    }

    IFS4bits.U1ERIF = false;
}

/*******************************************************************************
                        UART2 LoRa INTERRUPT
*******************************************************************************/

void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void)
{
   //TODO: code for LoRa reception
    IFS1bits.U2RXIF = 0; //clearing UART2  interrupt    
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2ErrInterrupt ( void )
{
    if ((U2STAbits.OERR == 1)) //just in case we get an overruun error - wewant
    {                          //to clear it up and keep going..
        U2STAbits.OERR = 0;
    }

    IFS4bits.U2ERIF = false;
}

/*******************************************************************************
                    UART3 DiGi WiFi INTERRUPT
*******************************************************************************/

void __attribute__((interrupt, no_auto_psv)) _U3RXInterrupt(void)
{
    //TODO: code for WiFi reception
    IFS5bits.U3RXIF = 0; //clearing  UART3 interrupt
}

/*******************************************************************************
                    INT0: Timer button  INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT0Interrupt(void)
{
    if ( ( (u8ApplicationState == APP_STATE_ON) || (u8ApplicationState == APP_STATE_TIMER) ) &&
           ( bTimerButtonPressed == false ) 
        )
    {
        bTimerButtonPressed = true;
        u8TimerButtonPressCount++;
        u8CurrentState = STATE_ACTIVE;
    }
    TimerButton_InterruptFlagClear();
}
/*******************************************************************************
                    INT1: Down button  INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT1Interrupt(void)
{
   //bDownButtonPressed = true; 
   bPowerButtonPressed = true; 
   DownButton_InterruptFlagClear();
   u8CurrentState = STATE_ACTIVE;

}
/*******************************************************************************
                    INT2: Up button  INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT2Interrupt(void)
{
    if ( (u8ApplicationState == APP_STATE_ON) || (u8ApplicationState == APP_STATE_TIMER) )
    {
        bUpButtonPressed = true;
        if (u8UpButtonPressCount < MAX_BUTTON_PRESS_COUNT)
        {
            u8UpButtonPressCount ++;
        }
        u8CurrentState = STATE_ACTIVE;
    }
    UpButton_InterruptFlagClear();
}
/*******************************************************************************
                    INT3: Power button  INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT3Interrupt(void)
{
    if ( (u8ApplicationState == APP_STATE_ON) || (u8ApplicationState == APP_STATE_TIMER) )
    {
        //bPowerButtonPressed = true;
        bDownButtonPressed = true;
        if (u8DownButtonPressCount < MAX_BUTTON_PRESS_COUNT)
        {
            u8DownButtonPressCount ++;
        }
        u8CurrentState = STATE_ACTIVE;
    }
    PowerButton_InterruptFlagClear();
}

/*******************************************************************************
                    INT4: BLuetooth connected interrupt
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _INT4Interrupt(void)
{
    if (RN4871_Status1 == 0) //Bluetooth connection established
    {
        BLE_Connected = true;
        BLEStatus_PositiveEdgeSet(); //set to trigger when phone is disconnected. 
         //Waking up BLE UART function:
        RN4871_Wake_SetLow();        
        u8CurrentState = STATE_CONNECT;
    }
    else
    {
        BLE_Connected = false;
        BLEStatus_NegativeEdgeSet(); //goes low when phone is connected
    }
    
    BLEStatus_InterruptFlagclear();
}


/*******************************************************************************
                    RTCC Alarm interrupt
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _ISR _RTCCInterrupt( void )
{
    if (RTCSTATLbits.ALMEVT == 1) //i.e this is an alarm event
    {
        RTCCAlarmOn = true;
        RTCSTATLbits.ALMEVT = 0;
    }    
    IFS3bits.RTCIF = false;
    u8CurrentState = STATE_ACTIVE;
}

/*******************************************************************************
                      1 ms TIMER1 INTERRUPT 
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T1Interrupt (  )
{
    /* Check if the Timer Interrupt/Status is set */
    
    u16Timer1Tick1ms++;
    IFS0bits.T1IF = false;
}

/*******************************************************************************
                      100 ms TIMER2 INTERRUPT 
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  )
{
    Timer2Tick100ms++;
    IFS0bits.T2IF = false;
}

/*******************************************************************************
                      1000 ms TIMER3 INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T3Interrupt (  )
{
    u16Timer3Tick1s++;
    IFS0bits.T3IF = false;

}

/*******************************************************************************
                      10 ms TIMER4 INTERRUPT
******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T4Interrupt (  )
{
    u16Timer4Tick10ms++;
    IFS1bits.T4IF = false;
}
/*******************************************************************************
                      100 us TIMER5 INTERRUPT
*******************************************************************************/
void __attribute__ ( ( interrupt, no_auto_psv ) ) _T5Interrupt (  )
{
    u32Timer5Tick100us++;
    IFS1bits.T5IF = false;
}

/*******************************************************************************
                    EXCEPTIONS :
*******************************************************************************/
void __attribute__((naked, noreturn, weak)) TRAPS_halt_on_error(uint16_t code)
{
    TRAPS_error_code = code;
#ifdef __DEBUG    
    __builtin_software_breakpoint();
    /* If we are in debug mode, cause a software breakpoint in the debugger */
#endif
    asm ("RESET");
    while(1);
    
}

/**
 * Sets the stack pointer to a backup area of memory, in case we run into
 * a stack error (in which case we can't really trust the stack pointer)
 */
inline static void use_failsafe_stack(void)
{
    static uint8_t failsafe_stack[32];
    asm volatile (
        "   mov    %[pstack], W15\n"
        :
        : [pstack]"r"(failsafe_stack)
    );
/* Controls where the stack pointer limit is, relative to the end of the
 * failsafe stack
 */    
    SPLIM = (uint16_t)(((uint8_t *)failsafe_stack) + sizeof(failsafe_stack) 
            - FAILSAFE_STACK_GUARDSIZE);
}
/** Oscillator Fail Trap vector**/
void ERROR_HANDLER_NORETURN _OscillatorFail(void)
{
    INTCON1bits.OSCFAIL = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_OSC_FAIL);
}
/** Stack Error Trap Vector**/
void ERROR_HANDLER_NORETURN _StackError(void)
{
    /* We use a failsafe stack: the presence of a stack-pointer error
     * means that we cannot trust the stack to operate correctly unless
     * we set the stack pointer to a safe place.
     */
    use_failsafe_stack(); 
    INTCON1bits.STKERR = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_STACK_ERR);
}
/** Address Error Trap Vector**/
void ERROR_HANDLER_NORETURN _AddressError(void)
{
    INTCON1bits.ADDRERR = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_ADDRESS_ERR);
}
/** Math Error Trap Vector**/
void ERROR_HANDLER_NORETURN _MathError(void)
{
    INTCON1bits.MATHERR = 0;  //Clear the trap flag
    TRAPS_halt_on_error(TRAPS_MATH_ERR);
}

void OSCILLATOR_Initialize(void)
{
    // CF no clock failure; NOSC FRC; SOSCEN enabled; POSCEN disabled;
    // CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONL((uint8_t) (0x0002 & 0x00FF));
    // CPDIV 1:1; PLLEN disabled; RCDIV FRC/1; DOZE 1:8; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3000;
    // STOR disabled; STORPOL Interrupt when STOR is 1; STSIDL disabled; 
    // STLPOL Interrupt when STLOCK is 1; STLOCK disabled; STSRC SOSC; 
    // STEN enabled; TUN Center frequency; 
    OSCTUN = 0x8000;
    // ROEN disabled; ROSEL FOSC; ROSIDL disabled; ROSWEN disabled; ROOUT disabled; ROSLP disabled; 
    REFOCONL = 0x0000;
    // RODIV 0; 
    REFOCONH = 0x0000;
    // ROTRIM 0; 
    REFOTRIML = 0x0000;
}

void PIN_MANAGER_Initialize(void)
{
    /****************************************************************************
     *   Output Latch SFR(s)
     ***************************************************************************/
    //**** OUTPUTS LATCHES NOW CONTROLLED BY POWER ENABLE MACROS IN HAL.H
    LATB = 0x0000;//0x5000; //WiFi_DTR, Bluetooth_Wake  HIGH
    LATC = 0x0000;
    LATD = 0x0000; 
    LATE = 0x0000;//0x007F; //CS and Reset lines high at startup
    LATF = 0x0000;//0x0028; //WiFiTx,LoRaTx high 
    LATG = 0x0000;//0x0040; //Bluetooth_TX  high
    
    
    TRISB = 0x2C6B;  //24VAC_good, Bluetooth_stat1, MISO, T1ADC, T2ADC, VDCIN are inputs
    TRISC = 0x0000; // All outputs
    TRISD = 0x0C23; //INT0, Bluetooth_CTS, Bluetooth_Rx, INT3, INT1 are  inputs
    TRISE = 0x0000; // All outputs
    TRISF = 0x0011; // BSY_Epaper, INT2, are inputs
    TRISG = 0x0380; // LoRa_Rx, WiFi_Rx, WiFi_CTS are inputs
    
    /****************************************************************************
     * Analog/Digital Configuration SFR
     ***************************************************************************/
    ANSA = 0x0000;
    ANSB = 0x2C00;
    ANSC = 0x0000;
    ANSD = 0x0000;
    ANSE = 0x0000;
    ANSF = 0x0000;
    ANSG = 0x0000;
    ANSH = 0x0000;

    /****************************************************************************
     * Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    IOCPDA = 0x0000;
    IOCPDB = 0x0000; //0x008;//24VACgood pull down
    IOCPDC = 0x0000;
    IOCPDD = 0x0000; //0x01D8;//heat_reset, cool_set, cool_reset, fan_set, fan_reset pull down
    IOCPDE = 0x0000; //0x0020;
    IOCPDF = 0x0000; //0x0080;
    IOCPDG = 0x0000; //0x000C;
    IOCPDH = 0x0000;
    IOCPDJ = 0x0000;
    IOCPUA = 0x0000;
    IOCPUB = 0x0000;
    IOCPUC = 0x0000;
    IOCPUD = 0x0000;
    IOCPUE = 0x0000;//0x0051; //rst_epaper, CS_mem, CS_BME pulled up
    IOCPUF = 0x0000;
    IOCPUG = 0x0000;
    IOCPUH = 0x0000;
    IOCPUJ = 0x0000;

    /****************************************************************************
     * Open Drain SFR(s)
     ***************************************************************************/
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;
    ODCH = 0x0000;
    ODCJ = 0x0000;
    
    /****************************************************************************
     * Setting the PPS
     ***************************************************************************/
    __builtin_write_OSCCONL(OSCCON & 0xbf); // unlock PPS
    
    RPINR0bits.INT1R = 0x000C;   //RD11->EXT_INT:INT1;
    RPOR8bits.RP16R = 0x0013;   //RF3->UART3:U3TX;
    RPOR8bits.RP17R = 0x0005;   //RF5->UART2:U2TX;
    RPINR1bits.INT3R = 0x0003;   //RD10->EXT_INT:INT3;
    RPINR1bits.INT2R = 0x000A;   //RF4->EXT_INT:INT2;
    RPOR11bits.RP23R = 0x0004;   //RD2->UART1:U1RTS;
    RPOR4bits.RP8R = 0x0008;   //RB8->SPI1:SCK1OUT;
    RPOR10bits.RP21R = 0x0003;   //RG6->UART1:U1TX;
    RPINR19bits.U2RXR = 0x001A;   //RG7->UART2:U2RX;
    RPINR18bits.U1CTSR = 0x0018;   //RD1->UART1:U1CTS;
    RPOR3bits.RP7R = 0x0007;   //RB7->SPI1:SDO1;
    RPINR18bits.U1RXR = 0x0014;   //RD5->UART1:U1RX;
    RPINR2bits.INT4R = 0x0012;   //RB5->EXT_INT:INT4;
    RPINR21bits.U3CTSR = 0x001B;   //RG9->UART3:U3CTS;
    RPINR17bits.U3RXR = 0x0013;   //RG8->UART3:U3RX;
    RPINR20bits.SDI1R = 0x0006;   //RB6->SPI1:SDI1;

    __builtin_write_OSCCONL(OSCCON | 0x40); // lock   PPS
    
    
    //Enabling low power regulator during sleep : saves about 5uA
    RCONbits.RETEN = 1;
    
    //Disabling all peripherals by default. Will be enabling needed ones manually
    PMD1 = 0xFFFF;
    PMD2 = 0xFFFF;
    PMD3 = 0xFFFF;
    PMD4 = 0xFFFF;
    PMD5 = 0xFFFF;
    PMD6 = 0xFFFF;
    PMD7 = 0xFFFF;
    PMD8 = 0xFFFF;
    
    //enabling just UART1, ADC and SPI1:
    PMD1bits.U1MD = 0; //UART1
    PMD1bits.ADC1MD = 0; //ADC
    PMD1bits.SPI1MD = 0; //SPI1
    //TODO: enable peripherals needed here:
    //PMD1bits.U2MD = 0; //UART2 : LoRA
    //PMD3bits.U3MD = 0; //UART3 : WiFi
    
}

void INTERRUPT_Initialize (void)
{
    
    INTCON1bits.NSTDIS = 1; //disabling nested interrupts
    
    /***************************************************************************     
     * interrupt priority level : 1 is the lowest, 7 is the highest
    ***************************************************************************/ 

    IPC16bits.U1ERIP = 1; //    UERI: U1E - UART1 (BLE) Error
    IPC3bits.U1TXIP = 1;//    UTXI: U1TX - UART1 Transmitter
    IPC2bits.U1RXIP = 1; //    URXI: U1RX - UART1 Receiver
    IPC16bits.U2ERIP = 1; //    UERI: U2E - UART2(LoRa) Error
    IPC7bits.U2TXIP = 1; //    UTXI: U2TX - UART2 Transmitter
    IPC7bits.U2RXIP = 1; //    URXI: U2RX - UART2 Receiver
    IPC20bits.U3ERIP = 1; //    UERI: U3E - UART3(WiFi) Error
    IPC20bits.U3TXIP = 1; //    UTXI: U3TX - UART3 Transmitter
    IPC20bits.U3RXIP = 1; //    URXI: U3RX - UART3 Receiver

    IPC0bits.INT0IP = 1;//    INT0I: INT0 - Timer button
    IPC5bits.INT1IP = 1; //    INT1I: INT1 -Down button
    IPC13bits.INT3IP = 1; //    INT3I: INT3 - Power button   
    IPC7bits.INT2IP = 1; //    INT2I: INT2 - Up button
    IPC13bits.INT4IP = 1; //BLE Status 1 signal
    
    IPC7bits.T5IP = 1; //    TI: T5 - Timer5
    IPC6bits.T4IP = 1;//    TI: T4 - Timer4
    IPC1bits.T2IP = 1;//    TI: T2 - Timer2

    
    /****************************************************************************
     * Clearing and Enabling UART interrupts
     ***************************************************************************/
    IFS0bits.U1RXIF = 0; //clearing up the U1Rx interrupt for Bluetooth
    IEC0bits.U1RXIE = 1;
    
    //TODO: Enable LoRa and WiFi interrupts when needed
    IFS1bits.U2RXIF = 0;  //Clear LoRa Receive interrupt flag
    //IEC1bits.U2RXIE = 1;  //Enable LoRa Receive interrupt
    
    IFS5bits.U3RXIF = 0; //clearing flag - WiFi
    //IEC5bits.U3RXIE = 1; //enabling receive interrupt for WiFi
    
    /****************************************************************************
     * Clearing and Enabling button interrupts
    ***************************************************************************/
    UpButton_NegativeEdgeSet();
    UpButton_InterruptFlagClear();    
    UpButton_InterruptEnable();
    
    DownButton_NegativeEdgeSet();
    DownButton_InterruptFlagClear();    
    DownButton_InterruptEnable();
    
    TimerButton_NegativeEdgeSet();
    TimerButton_InterruptFlagClear();    
    TimerButton_InterruptEnable();
    
    PowerButton_NegativeEdgeSet();
    PowerButton_InterruptFlagClear();
    PowerButton_InterruptEnable();
    
    //BLE STATUS INTERRUPT:
    //BLE connected interrupt is enabled after RN4871 init
    BLEStatus_NegativeEdgeSet(); //goes to zero when connected
    //BLEStatus_InterruptFlagclear();
    //BLEStatus_InterruptEnable();
}

/**
 * 1ms timer
 */
void TMR1_Initialize (void) //TMR1: 1ms tick
{
    //TMR1 0; 
    TMR1 = 0x0000;
    //Period = 0.001 s; Frequency = 16000000 Hz; PR1 16000; 
    PR1 = 0x3E80; //interrupt every 1ms: (16000/16E6) = 1ms
    //TCKPS 1:1; TON enabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TSYNC disabled; TGATE disabled; 
    T1CON = 0x0000;
    IFS0bits.T1IF = false;
    IEC0bits.T1IE = true;
}

void TMR1_Start( void )
{
    u16Timer1Tick1ms = 0; //resetting the tick counter    
    IEC0bits.T1IE = true;//Enable the interrupt   
    T1CONbits.TON = 1; //starting up the timer
}

void TMR1_Stop( void )
{
    T1CONbits.TON = false; //Stopping timer
    //Disabling the interrupt*/
    IEC0bits.T1IE = false;    
    u16Timer1Tick1ms = 0;
}
void TMR1_Reset(void)
{
    TMR1 = 0x0000;
    u16Timer1Tick1ms = 0;
}

void TMR2_Initialize (void) //TMR2: 100ms tick
{
    //TMR2 0; 
    TMR2 = 0x0000;
    //Period = 0.1 s; Frequency = 16000000 Hz; PR2 25000; 
    PR2 = 0x61A8; //interrupt every 100ms: (25000/16E6)*64 = 0.1s 
    //TCKPS 1:64; T32 16 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T2CON = 0x0020;
    IFS0bits.T2IF = false;
    IEC0bits.T2IE = true;
}

void TMR2_Start( void )
{
    /*Enable the interrupt*/
    IEC0bits.T2IE = true;
    Timer2Tick100ms = 0;
    /* Start the Timer */
    T2CONbits.TON = 1;
}
void TMR2_Stop( void )
{
    /* Stop the Timer */
    Timer2Tick100ms = 0;
    T2CONbits.TON = false;
    
    /*Disable the interrupt*/
    IEC0bits.T2IE = false;
}

void TMR2_Reset(void)
{
    TMR2 = 0x0000;
    Timer2Tick100ms = 0;
}
void TMR3_Initialize (void) //TMR3 : 1s tick
{
    TMR3 = 0x0000;
    //Period = 1 s; Frequency = 16000000 Hz; PR4 2000; 
    PR3 = 0xF424;
    //TCKPS 1:256; T32 16 Bit; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T3CON = 0x0030;
    IFS1bits.T4IF = false;
    IEC1bits.T4IE = false;

}
void TMR3_Start( void )
{
    /*Enable the interrupt*/
    IFS0bits.T3IF = false;
    IEC0bits.T3IE = true;
    u16Timer3Tick1s = 0;
    /* Start the Timer */
    T3CONbits.TON = 1;
}
void TMR3_Stop( void )
{
    /* Stop the Timer */
    T3CONbits.TON = false;
    
    /*Disable the interrupt*/
    IEC0bits.T3IE = false;
}
void TMR3_Reset(void)
{
    TMR3 = 0x0000;
    u16Timer3Tick1s = 0;
}

void TMR4_Initialize (void) //TMR4 : 10ms tick
{
    //TMR4 0; 
    TMR4 = 0x0000;
    //Period = 0.01 s; Frequency = 16000000 Hz; PR4 20000; 
    PR4 = 0x4E20;
    //TCKPS 1:8; T32 16 Bit; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T4CON = 0x0010;
    
    IFS0bits.T3IF = false;
    IEC0bits.T3IE = false;

}
void TMR4_Start( void )
{
    /*Enable the interrupt*/
    IEC1bits.T4IE = true;
    u16Timer4Tick10ms = 0;
    /* Start the Timer */
    T4CONbits.TON = 1;
}

void TMR4_Stop( void )
{
    /* Stop the Timer */
    T4CONbits.TON = false;
    
    /*Disable the interrupt*/
    IEC1bits.T4IE = false;
}
void TMR4_Reset(void)
{
    TMR4 = 0x0000;
    u16Timer4Tick10ms = 0;
}

void TMR5_Initialize (void) //TMR5 : 100us tick
{
    //TMR5 0; 
    TMR5 = 0x0000;
    //Period = 0.0001 s; Frequency = 16000000 Hz; PR5 200; 
    PR5 = 0x00C8;
    //TCKPS 1:8; TON disabled; TSIDL disabled; TCS FOSC/2; TECS SOSC; TGATE disabled; 
    T5CON = 0x0010;
    
    IFS1bits.T5IF = false;
    IEC1bits.T5IE = false;
}

void TMR5_Reset(void)
{
    TMR5 = 0x0000;
    u32Timer5Tick100us = 0;
}
void TMR5_Start( void )
{
    /*Enabling interrupt*/
    IFS1bits.T5IF = false;
    IEC1bits.T5IE = true;
    u32Timer5Tick100us = 0;
    /* Start the Timer */
    T5CONbits.TON = 1;
}
void TMR5_Stop( void )
{
    /* Stop the Timer */
    T5CONbits.TON = false;
    IEC1bits.T5IE = false;

}


void SPI1_Initialize (void)
{
    // AUDEN disabled; FRMEN disabled; AUDMOD I2S; FRMSYPW One clock wide; 
    //AUDMONO stereo; FRMCNT 0; MSSEN disabled; FRMPOL disabled; 
    //IGNROV disabled; SPISGNEXT not sign-extended; FRMSYNC disabled; URDTEN disabled; IGNTUR disabled; 
    SPI1CON1H = 0x0000;
    // WLENGTH 0; 
    SPI1CON2L = 0x0000; //data length set by mode
    // SPIROV disabled; FRMERR disabled; 
    SPI1STATL = 0x0000;
    // SPI1BRGL 1; 
    SPI1BRGL = 0x0003; //2MHZ 0x0003, 1MHz 0x0007, 500KHz = 0x000F
    // SPITBFEN disabled; SPITUREN disabled; FRMERREN disabled; SRMTEN disabled; 
    // SPIRBEN disabled; BUSYEN disabled; SPITBEN disabled; SPIROVEN disabled; SPIRBFEN disabled; 
    SPI1IMSKL = 0x0000;
    // RXMSK 0; TXWIEN disabled; TXMSK 0; RXWIEN disabled; 
    SPI1IMSKH = 0x0000;
    // SPI1URDTL 0; 
    SPI1URDTL = 0x0000;
    // SPI1URDTH 0; 
    SPI1URDTH = 0x0000;
    // SPIEN enabled; DISSDO disabled; MCLKEN FOSC/2; CKP Idle:Low, Active:High; 
    // SSEN disabled; MSTEN Master; MODE16 disabled; SMP Middle; DISSCK disabled; 
    // SPIFE Frame Sync pulse precedes; CKE Idle to Active; MODE32 disabled; 
    // SPISIDL disabled; ENHBUF enabled; DISSDI disabled; 
    //SPI1CON1L = 0x8060; //sampling at mid Mode 3. Previous end sampling: 0x8060; 
    SPI1CON1Lbits.MSTEN = 1; //Master mode enabled
    SPI1CON1Lbits.SMP = 0; //1=data sampled at the end 0 = in the middle of data output time
    //CPOL = 0, CPHA = 0 : SPI Mode 0
    SPI1CON1Lbits.CKP = 0; //1=clock idles high, 0 = clock idles low -> same as CPOL
    SPI1CON1Lbits.CKE = 1; //1 = transmit happens on active to idle, 0 = tx on idle to active, -> inverse of CHPA
    SPI1CON1Lbits.ENHBUF = 0;
    
    
    SPI1CON1Hbits.IGNROV = 1; //ignore rx overflow errors
    //SPI1CON1Lbits.SPIEN = 1; //will be enabling the peripheral manually when needed
}
void SPI1_Write8bit(uint8_t data)
{
    uint8_t rxData = 0;
    while (SPI1STATLbits.SPITBF == true) {} //wait if transmit buffer is full
    SPI1BUFL = data;
    while (SPI1STATLbits.SPIBUSY == true) {} //wait until data is transmitted
    rxData = SPI1BUFL; //to clear out the rx buffer just in case
}

uint8_t SPI1_Exchange8bit(uint8_t data){
   uint8_t rxData; 
   rxData = SPI1BUFL; //to clear out the rx buffer just in case
   SPI1STATLbits.SPIROV = 0; //clearing the overrun bit
   while (SPI1STATLbits.SPITBF == true) {} //wait if transmit buffer is full
   SPI1BUFL = data;   
   while (SPI1STATLbits.SPIBUSY == true){} //wait while transactions are not complete
   while (SPI1STATLbits.SPIRBF != true) {} //wait while rx buffer is not full
   Nop(); //1 clk cycle delay for trailing bit
   rxData = SPI1BUFL;    
   return (rxData);   
}

uint16_t SPI1_Exchange16bit(uint16_t data){
   uint16_t receiveDataL;
   uint16_t receiveDataH;
   while (SPI1STATLbits.SPITBF == true) {} //wait if transmit buffer is full
   SPI1BUFL = data>>8; //high byte
   while (SPI1STATLbits.SPIBUSY == true){} //wait while transactions are not complete
   while (SPI1STATLbits.SPIRBF != true) {} //wait while rx buffer is not full
   Nop(); //1 clk cycle delay for trailing bit
   receiveDataH = SPI1BUFL;    
   while (SPI1STATLbits.SPITBF == true) {} //wait if transmit buffer is full
   SPI1BUFL = data; //low byte
   while (SPI1STATLbits.SPIBUSY == true){} //wait while transactions are not complete
   while (SPI1STATLbits.SPIRBF != true) {} //wait while rx buffer is not full
   Nop(); //1 clk cycle delay for trailing bit
   receiveDataL = SPI1BUFL; 
   return (receiveDataH<<8)|receiveDataL;   
}

/*******************************************************************************
                     UART1 used for Bluetooth Comms
*******************************************************************************/
void UART1_Initialize(void)
{
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; 
    // USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; 
    // BRGH enabled; URXINV disabled; UEN TX_RX; 
   U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U1STA = 0x0000;
   // BaudRate = 115200; Frequency = 4000000 Hz; U1BRG 8; 
   U1BRG = BRGH_115200;
   // ADMADDR 0; ADMMASK 0; 
   U1ADMD = 0x0000;
   // T0PD 1 ETU; PTRCL T0; TXRPT Retransmits the error byte once; CONV Direct; SCEN disabled; 
   U1SCCON = 0x0000;
   // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; PARIE disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
   U1SCINT = 0x0000;
   // GTC 0; 
   U1GTC = 0x0000;
   // WTCL 0; 
   U1WTCL = 0x0000;
   // WTCH 0; 
   U1WTCH = 0x0000;
   
   U1MODEbits.UARTEN = 1;  // enabling UART ON bit
   U1STAbits.UTXEN = 1;
}

void UART1_EnableFlowControl(void)
{
    U1MODE = (U1MODE & 0xf7ff) | 0x0200;    //Enable RTC/CTS flow control
}

/**
 * Pushes a null terminated C string to TXREG
 * @param msg - pointer to the string that is null terminated
 */
void UART1_SendString(const char *msg)
{
    uint8_t msgLen=0;
    uint8_t i=0;
    msgLen = strlen(msg);
    for (i=0; i<= msgLen-1; i++){
        while (U1STAbits.UTXBF){} //wait until at least one character 
        U1TXREG = msg[i];         //can be written to the transmit buffer
    }
    //now we wait for transmit shift register to get empty
    while (!U1STAbits.TRMT){}
}



/**
 * Pushes a single byte of data in TXREG
 * @param msg - byte
 */
void UART1_SendByte(uint8_t msg)
{
    while (U1STAbits.UTXBF){} //wait until at least one character 
    U1TXREG = msg;
    while (!U1STAbits.TRMT){}
}

void UART1_SendBytes(const uint8_t *byte, uint8_t startIndex, uint8_t cmdLen)
{
    uint8_t i = 0;
    for (i=startIndex; i< startIndex+cmdLen; i++)
    {
       while (U1STAbits.UTXBF){} //wait until at least one character 
        U1TXREG = byte[i];         //can be written to the transmit buffer 
    }
    //now we wait for transmit shift register to get empty
    while (!U1STAbits.TRMT){}
}

/**
 * Keeps reading the RXREG until there are no more bytes left
 */
void UART1_FlushRxFIFO(void)
{
    uint8_t tmp;
    while (U1STAbits.URXDA) //while there is data present
    {
        tmp = U1RXREG; //read and discard
    }  
}

/*******************************************************************************
                     UART2 - used for LoRa comms:
*******************************************************************************/
void UART2_Initialize(void)
{
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; 
    // USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; 
    // BRGH enabled; URXINV disabled; UEN TX_RX; 
   U2MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U2STA = 0x0000;
   // BaudRate 
   U2BRG = BRGH_57600;
   // ADMADDR 0; ADMMASK 0; 
   U2ADMD = 0x0000;
   // T0PD 1 ETU; PTRCL T0; TXRPT Retransmits the error byte once; CONV Direct; SCEN disabled; 
   U2SCCON = 0x0000;
   // TXRPTIF disabled; TXRPTIE disabled; WTCIF disabled; WTCIE disabled; PARIE disabled; GTCIF disabled; GTCIE disabled; RXRPTIE disabled; RXRPTIF disabled; 
   U2SCINT = 0x0000;
   // GTC 0; 
   U2GTC = 0x0000;
   // WTCL 0; 
   U2WTCL = 0x0000;
   // WTCH 0; 
   U2WTCH = 0x0000;

   U2MODEbits.UARTEN = 1;  // enabling UART ON bit
   U2STAbits.UTXEN = 1;
    
}

/**
 * Pushes a null terminated C string to TXREG
 * @param msg - pointer to the string that is null terminated
 */
void UART2_SendString(char *msg)
{
    uint8_t msgLen=0;
    uint8_t i=0;
    msgLen = strlen(msg);
    for (i=0; i<= msgLen-1; i++){
        while (U2STAbits.UTXBF){} //wait until at least one character 
        U2TXREG = msg[i];         //can be written to the transmit buffer
    }
    //now we wait for transmit shift register to get empty
    while (!U2STAbits.TRMT){}

}

void UART2_SendBytes(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen)
{
    uint8_t i = 0;
    for (i=startIndex; i< startIndex+cmdLen; i++)
    {
       while (U2STAbits.UTXBF){} //wait until at least one character 
        U2TXREG = byte[i];         //can be written to the transmit buffer 
    }
    //now we wait for transmit shift register to get empty
    while (!U2STAbits.TRMT){}
}

void UART2_SendByte(uint8_t byte)
{
    uint16_t i = 0;
    while (U2STAbits.UTXBF){} //wait until at least one character 
    U2TXREG = byte;        //can be written to the transmit buffer 
    //now we wait for transmit shift register to get empty
    while (!U2STAbits.TRMT && i < 5000)
    {
        __delay_us(1);
        i++;
    }
}


/*******************************************************************************
                UART3 used for Digi WiFI Module
*******************************************************************************/
void UART3_Initialize(void)
{  
    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; 
    // USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; 
    // BRGH enabled; URXINV disabled; UEN TX_RX; 
   U3MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U3STA = 0x0000;
   // BaudRate = 57600
   U3BRG = BRGH_57600;
   // ADMADDR 0; ADMMASK 0; 
   U3ADMD = 0x0000;

    U3MODEbits.UARTEN = 1;  // enabling UART ON bit
    U3STAbits.UTXEN = 1; //enabling transmit
}

/**
 * Pushes a single byte of data in TXREG
 * @param msg - byte
 */
void UART3_SendByte(uint8_t msg)
{
    while (U3STAbits.UTXBF){} //wait until at least one character 
    U3TXREG = msg;
    while (!U3STAbits.TRMT){}
}

/**
 * Pushes a null terminated C string to TXREG
 * @param msg - pointer to the string that is null terminated
 */
void UART3_SendString(char *msg)
{
    uint8_t msgLen=0;
    uint8_t i=0;
    msgLen = strlen(msg);
    for (i=0; i<= msgLen-1; i++){
        while (U3STAbits.UTXBF){} //wait until at least one character 
        U3TXREG = msg[i];         //can be written to the transmit buffer
    }
    //now we wait for transmit shift register to get empty
    while (!U3STAbits.TRMT) {}
}

void UART3_SendBytes(uint8_t *byte, uint8_t startIndex, uint8_t cmdLen)
{
    uint8_t i = 0;
    for (i=startIndex; i< startIndex+cmdLen; i++)
    {
       while (U3STAbits.UTXBF){} //wait until at least one character 
        U3TXREG = byte[i];       //can be written to the transmit buffer 
    }
    //now we wait for transmit shift register to get empty
    while (!U5STAbits.TRMT) {}
}

void ADC1_Initialize (void)
{
    
   AD1CON1 = 0x8400; //ADON, Mode = 12bit, 
   //AD1CON2bits.PVCFG = 0x00; //VREF+ is VDD,  VREF- is VSS
   AD1CON2 = 0x0000;
   AD1CON3 = 0x0C04; //12 // TAD = 10
   //AD1CON5bits.BGREQ = 1; //Band gap is enabled
   
   AD1CHS = 0x0000;// CH0SA AN0; CH0SB AN0; CH0NB AVSS; CH0NA AVSS;
   AD1CHITH = 0x0000;
   AD1CSSH = 0x0000;
   AD1CSSL = 0x0000;
   AD1CTMENH = 0x0000;
   ANCFG = 0x0000;
   
   AD1CON1bits.ADON=1; //enabling ADC module
}

uint16_t ADC1_ReadChannel(uint8_t ch)
{
   uint16_t result16 = 0;
   AD1CHS =  ch;
   AD1CON1bits.SAMP = 1; //initiating sampling
   __delay_us(100);
   AD1CON1bits.SAMP = 0; //end sampling, begin conversion
   Nop();
   while (AD1CON1bits.DONE != 1){} //wait for done bit to go high
   Nop();
   Nop();
   result16 = ADC1BUF0;
   return result16;
}



//Cryptographic engine - currently used for random number generation
void CRYPTO_Initialize(void)
{
    CRYCONLbits.CRYON = 1;
}

uint8_t CRYPTO_Get8BitRandomNumber(void)
{
    uint8_t rNumber = 0;
    
    CRYCONLbits.OPMOD = 0b1010;
    CRYCONLbits.CRYGO = 1;
    asm("NOP");
    while (CRYCONLbits.CRYGO == 1)
    {
        asm("NOP");
    }
    rNumber = (uint8_t)(CRYTXTA);
    return rNumber;
}
uint32_t ReadSerialNumber(void)
{
    //SQTP number is stored at 0x02AF78 by the IPE programmer
    uint16_t word0 = 0;
    uint16_t word1 = 0;
    
    TBLPAG = 0x0002; 
    word0 = __builtin_tblrdl(0x02AF78);
    word1 = __builtin_tblrdh(0x02AF78);
    return ( (uint32_t)word0<<16 | word1);
}