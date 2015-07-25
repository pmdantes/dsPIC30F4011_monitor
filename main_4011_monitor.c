/*
 * File:   main.c
 * Author: PaoloDantes
 *
 * Created on April 17, 2015, 2:23 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include "config.h"
#include "p30f4011.h"


#define PIC_ID          4
#define SEND_DATA       (PIC_ID-1)
#define UPDATE_POSITION (PIC_ID-3)
#define PIC_HAPTIC      (PIC_ID+3)
#define SLAVE_INDATA    0//(PIC_ID-4)
#define MASTER_INDATA   (PIC_ID-1)

// IO definitions
#define LEDRED  LATEbits.LATE3 // RED
#define LEDYLW  LATEbits.LATE4 // YELLOW
#define LEDGRN  LATEbits.LATE5 // GREEN
#define TRISRED  TRISEbits.TRISE3 // YELLOW
#define TRISYLW  TRISEbits.TRISE4 // YELLOW
#define TRISGRN  TRISEbits.TRISE5 // GREEN
#define UART_TX_LEN 36

// Display commands for LMB162ABC
#define CLEAR_DISPLAY       0x01 // Writes "20h" (ASCII for space
#define RETURN_HOME         0X02
#define ENTRY_MODE_SET      0x06 // No screen shifting, AC=AC+1
#define DISPLAY_ON          0x0F // Display on, cursor on, cursor blinking on
#define CURSOR_SHIFT        0x14 // Shift cursor to right side
#define FUNCTION_SET        0x28 // 4-bit interface, 2-line display, 5x8 dots font
#define SET_CGRAM_ADD       0x40 // OR this with AC0-AC5 CGRAM address
#define SET_DDRAM_ADD       0x80 // OR this with AC0-AC6 DDRAM address

// IO for 16x2 LCD
#define RS      LATEbits.LATE0 // RED WIRE
#define RW      LATEbits.LATE1 // BLUE WIRE
#define EN      LATEbits.LATE2 // GREEN WIRE
#define DATA    LATB

// CAN Operation Modes
#define CONFIG_MODE     4
#define LISTEN_ALL      7
#define LISTEN_ONLY     3
#define LOOPBACK        2
#define DISABLE         1
#define NORMAL          0

// CAN bit timing
#define FOSC        20000000 // 7.37MHz
#define FCY         FOSC/4
#define BITRATE     1000000  // 100kbps
#define NTQ         16      // Amount of Tq per bit time
#define BRP_VAL     (((4*FCY)/(2*NTQ*BITRATE))-1) // refer to pg. 693 of Family Reference

// UART baud rate
#define UART_BAUD   115000
#define UART_BRG    (FCY/(16*UART_BAUD))-1 // refer to pg. 506 of family reference

// Define motorState state values
#define INITIALIZE          100
#define READ_INPUT_OUTPUT   101
#define CALCULATE_PID       102
#define UPDATE_PID          103
#define IDLE                104
#define SEND_HOME           105
#define KP_PROGRAM          106
#define KD_PROGRAM          107
#define KI_PROGRAM          108
#define HAPTIC              109

// IDLE state machine
#define HOME               10
#define NEW_KP_CONST       11
#define NEW_KD_CONST       12
#define NEW_KI_CONST       13

// Define PID controls
#define PID_KP  123.45
#define PID_KD  123.45
#define PID_KI  123.45
#define PID_TI  0
#define PID_TD  0
#define PID_TS  10
#define PID_N   10

// Define PWM constants
#define PWM_FREQUENCY       16000
#define PWM_PRESCALER       0       // 0=1:1 1=1:4 2=1:16 3=1:64
#define PWM_COUNTS_PERIOD   (FCY/PWM_FREQUENCY)-1   // 16kHz for FRC

// pid_t type

typedef struct {
    float Kp, Kd, Ki, T;
    unsigned short N; // derivative filter parameter (3-20)
    float i, ilast; // integral --> NOT USED IN THIS VERSION. FOR FUTURE IMPLEMENTATION
    float y, ylast; // output
    float d, dlast; // derivative term
    float u; // u=uc-y
    float e, elast; // error
} pid_t;


//// CAN1 MODULE
unsigned int masterData[4] = {0, 0, 0, 0};
unsigned int slaveData[4] = {0, 0, 0, 0};
unsigned int OutData0[4] = {0, 0, 0, 0};
unsigned int motorState = INITIALIZE;
unsigned char txInProgress, sendMsg = 1;
unsigned int ADCValue0, ADCValue1 = 0;
char txData[UART_TX_LEN] = {'\0'};
char rxData[UART_TX_LEN] = {'\0'};

void InitCan(void) {
    // Initializing CAN Module Control Register
    C1CTRLbits.REQOP = CONFIG_MODE; // 4 = Configuration mode
    C1CTRLbits.CANCAP = 1; // Enable CAN capture
    C1CTRLbits.CSIDL = 0; // 0 = Continue CAN module op in idle mode
    C1CTRLbits.CANCKS = 0; // 1: Fcan=Fcy 0: Fcan=4Fcy
    C1CFG1bits.SJW = 0; // Synchronized jump width is 1xTq
    C1CFG1bits.BRP = 8; // Baud rate prescaler = 20 (CAN baud rate of 100kHz
    C1CFG2bits.SEG2PHTS = 1; // 1=Freely Programmable 0=Maximum of SEG1PH or 3Tq's whichever is greater
    C1CFG2bits.PRSEG = 1; // Propagation Segment = 2Tq
    C1CFG2bits.SEG1PH = 6; // Phase Buffer Segment 1 = 7Tq
    C1CFG2bits.SEG2PH = 5; // Phase Buffer Segment 2 = 6Tq
    C1CFG2bits.SAM = 1; // 1=Bus line sampled 3 times 0=Bus line sampled once

    // Initializing CAN interrupt
    C1INTF = 0; // Reset all CAN interrupts
    IFS1bits.C1IF = 0; // Reset Interrupt flag status register
    C1INTE = 0x00FF; // Enable all CAN interrupt sources
    IEC1bits.C1IE = 1; // Enable CAN1 Interrupt

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Receive Buffer Register
    C1RX0CONbits.DBEN = 0; // Buffer 0 does not overflow in buffer 1
    C1RX0CONbits.FILHIT = 0; // Buffer 0 = Acceptance filter 0
    C1RX1CONbits.FILHIT = 2; // Buffer 1 = Acceptance filter 2

    // Initializing Acceptance Mask Register
    C1RXM0SID = 0x0001; // SID = 11111111111
    C1RXM1SID = 0x1FFD; // SID = 11111111111

    // Initializing Message Acceptance filter
    C1RXF0SID = 0x0AA8; // SID = 01010101010
    C1RXF2SID = 0x1554; // SID = 10101010101
    C1RXF1SID = 0x0000; // SID = 00000000000
    C1RXF3SID = 0x0000; // SID = 00000000000
    C1RXF4SID = 0x0000; // SID = 00000000000
    C1RXF5SID = 0x0000; // SID = 00000000000

    /*---------------------------------------------------------
     *  CONFIGURE RECEIVER REGISTERS, FILTERS AND MASKS
     *---------------------------------------------------------*/

    // Configure CAN Module Transmit Buffer Register
    C1TX0CONbits.TXPRI = 1; // 1 = High message priority
    C1TX1CONbits.TXPRI = 2; // 2 = High intermediate message priority

    // Initializing Transmit SID
    C1TX0SID = 0X50A8; // SID = 01010101010
    C1TX1SID = 0XA854; // SID = 10101010101
    C1TX0DLCbits.DLC = 8; // Data length is 8bytes
    C1TX1DLCbits.DLC = 8; // Data length is 8bytes

}

void InitQEI(void) {
    ADPCFG |= 0x003B; // RB3, RB4, RB5 configured to digital pin
    QEICONbits.QEIM = 0; // Disable QEI module
    QEICONbits.CNTERR = 0; // Clear any count errors
    QEICONbits.QEISIDL = 0; // Continue operation during sleep
    QEICONbits.SWPAB = 0; // QEA and QEB not swapped
    QEICONbits.PCDOUT = 0; // Normal I/O pin operation
    QEICONbits.POSRES = 1; // Index pulse resets position counter
    QEICONbits.TQCS = 0; // Internal clock source (Fcy) = 2Mhz
    DFLTCONbits.CEID = 1; // Count error interrupts disabled
    DFLTCONbits.QEOUT = 1; // Digital filters output enabled for QEn pins
    DFLTCONbits.QECK = 2; // 1:4 clock divide for digital filter for QEn
    // FILTER_DIV = (MIPS*FILTERED_PULSE)/3
    //  ==> 5MHz*5usec/3 = 3.33 --> 2
    //    DFLTCONbits.IMV1 = 1;       // Required state of phase B input signal for match on index pulse
    POSCNT = 12000; // Reset position counter
    QEICONbits.QEIM = 7; // X4 mode with position counter reset by
    // 6 - index pulse
    // 7 - match (MAXCNT)
    return;
}

void InitPwm(void) {
    PTCONbits.PTEN = 0;         // Disable PWM timerbase
    PTCONbits.PTCKPS = PWM_PRESCALER; // prescaler
    PTCONbits.PTOPS = 0;        // 1:1 postscaler
    PTCONbits.PTMOD = 0;        // free running mode
    PWMCON1bits.PMOD1 = 1;      // PWM in independent mode
    PWMCON1bits.PMOD2 = 1;      // PWM in independent mode
    PWMCON1bits.PEN1L = 1;      // PWM1L (PIN 24) output controlled by PWM
    PWMCON1bits.PEN2L = 1;      // PWM2L (PIN 26) output controlled by PWM
    PTMR = 0;                   // PWM counter value start at 0
    PTPER = (unsigned int) PWM_COUNTS_PERIOD; // Set PWM period
    return;
}

void InitUart(){
    U1MODEbits.UARTEN = 0;      // UART is disabled
    U1MODEbits.USIDL = 0;       // Continue operation in Idle Mode
    U1MODEbits.ALTIO = 1;       // UART communicates using U1ATX and U1ARX (pins 11&12)
    U1MODEbits.WAKE = 1;        // Enable wake-up on Start bit detec durign sleep mode
    U1MODEbits.PDSEL = 0;       // 8-bit data, no parity
    U1MODEbits.STSEL = 0;       // 1 stop bit

    U1STAbits.UTXISEL = 0;      // Interrupt when TX buffer has one character empty
    U1STAbits.UTXBRK = 0;       // U1TX pin operates normally
    U1STAbits.URXISEL = 0;      // Interrupt when word moves from REG to RX buffer
    U1STAbits.ADDEN = 0;        // Address detect mode disabled

//    U1BRG = 11;                 // p.507 of family reference
//                                // 9600 baud rate for FOSC = 7.37MHz
//    U1BRG =  UART_BRG;           // p.507 of family reference
//                                // 115000 baud rate for FOSC = 20MHz
    U1BRG =  7;           // p.507 of family reference
                                // 38400 baud rate for FOSC = 20MHz

    IFS0bits.U1TXIF = 0;        // Clear U1TX interrupt
    IFS0bits.U1RXIF = 0;        // Clear U1RX interrupt
    IPC2bits.U1TXIP = 5;        // U1TX interrupt 5 priority
    IPC2bits.U1RXIP = 5;        // U1RX interrupt 5 priority
    IEC0bits.U1TXIE = 1;        // Enable U1TX interrupt
    IEC0bits.U1RXIE = 1;        // Enable U1RX interrupt

    U1MODEbits.LPBACK = 0;      // Enable loopback mode
    U1MODEbits.UARTEN = 1;      // UART is enabled
    U1STAbits.UTXEN = 1;        // U1TX pin enabled

}

void InitAdc(void) {
    ADPCFG &= ~0x0004; // Sets  PB2 as analog pin (clear bits)
    ADCON1bits.ADON = 0; // A/D converter module off
    ADCON1bits.ADSIDL = 0; // Continue module operation in Idle mode
    ADCON1bits.SIMSAM = 0; // Samples multiple channels individually in sequence
    ADCON1bits.FORM = 0; // Integer (DOUT = 0000 00dd dddd dddd)
    ADCON1bits.SSRC = 7; // Internal counter ends sampling and starts conversion (auto convert)
    ADCON1bits.ASAM = 1; // Sampling in a channel begins when the conversion finishes (Auto-sets SAMP bit)
    //    ADCON1bits.SAMP = 1;        // At least one A/D sample/hold amplifier is sampling

    //    ADCON2bits.VCFG = 3; // Set external Vref+ and Vref- pins as A/D VrefH and VrefL
    ADCON2bits.VCFG = 7; // Set AVss and AVdd as A/D VrefH and VrefL
    ADCON2bits. CSCNA = 0;
    ADCON2bits.BUFM = 0; // Buffer configured as one 16-word buffer ADCBUF(15...0)
    ADCON2bits.CHPS = 0; // Convert CH0
    ADCON2bits.SMPI = 0; // Interrupts at the completion of conversion for each sample/convert sequence
    //    ADCON2bits.ALTS = 0;        // Always use MUX A input multiplexer settings

    ADCON3bits.ADRC = 0; // Clock derived from system clock
    ADCON3bits.ADCS = 31; // A/D Conversion clock select = 32*Tcy
    ADCON3bits.SAMC = 31; // 31*Tad Auto-Sample time


    ADCHSbits.CH0SA = 2; // Channel 0 positive input is AN2
    ADCHSbits.CH0NA = 0; // Channel 0 negative input is Vref-
    ADCHSbits.CH123SA = 1; // Channel positive input is AN3
    ADCHSbits.CH123NA = 0; // Channel negative input is Vref-


    // ADC Interrupt Initialization
    IFS0bits.ADIF = 0; // Clear timer 1 interrupt flag
    IEC0bits.ADIE = 1; // Enable timer 1 interrupts
    IPC2bits.ADIP = 5; // ADC interrupt priority 5 (default 4)
    ADCON1bits.ADON = 1; // A/D converter module on
    return;
}

void InitInt(void) {

    INTCON1bits.NSTDIS = 1; // Interrupt nesting is disabled
    IEC0bits.INT0IE = 0; // Disable external interrupt 0
    IEC1bits.INT1IE = 0; // Disable external interrupt 1
    IEC1bits.INT2IE = 0; // Disable external interrupt 2
    INTCON2bits.ALTIVT = 0; // Default interrupt vector table
    INTCON2bits.INT0EP = 1; // Detect on 1 = negative edge 0 = positive for interrupt 0
    INTCON2bits.INT1EP = 1; // Detect on 1 = negative edge 0 = positive for interrupt 1
    INTCON2bits.INT2EP = 1; // Detect on 1 = negative edge 0 = positive for interrupt 2
    IPC0bits.INT0IP = 5; // External interrupt 0 is priority 5
    IPC4bits.INT1IP = 5; // External interrupt 1 is priority 5
    IPC5bits.INT2IP = 5; // External interrupt 2 is priority 5
    IFS0bits.INT0IF = 0; // Clear external interrupt 0 flag
    IFS1bits.INT1IF = 0; // Clear external interrupt 1 flag
    IFS1bits.INT2IF = 0; // Clear external interrupt 2 flag
    IEC0bits.INT0IE = 1; // Enable external interrupt 0
    IEC1bits.INT1IE = 1; // Enable external interrupt 1
    IEC1bits.INT2IE = 1; // Enable external interrupt 2
    return;
}

void msDelay(unsigned int mseconds) //For counting time in ms
//-----------------------------------------------------------------
{
    int i;
    int j;
    for (i = mseconds; i; i--) {
         for (j = 714; j; j--) {
            // 5667 for XT_PLL8 -> Fcy = 34MHz
            // 714 for 20MHz oscillator -> Fcy=4.3MHz
            // 265 for FRC
        }
    }
    return;
}

void InitPid(pid_t *p, float kp, float kd, float ki, float T, unsigned short N, float il, float yl, float dl, float el) {
    p->Kp = kp;
    p->Kd = kd;
    p->Ki = ki;
    p->T = T;
    p->N = N;
    p->ilast = il;
    p->ylast = yl;
    p->dlast = dl;
    p->elast = el;
}

void UpdatePid(pid_t *mypid) {
    // Update PD variables
    mypid->ylast = mypid->y;
    mypid->dlast = mypid->d;
    mypid->elast = mypid->e;
}

unsigned int * CalcPid(pid_t *mypid, float degPOT, float degMTR) {
    float pidOutDutyCycle;
    static unsigned int pwmOUT[2] = {0, 0};

    mypid->y = degMTR;
    mypid->e = degPOT - degMTR;

    mypid->d = (mypid->T * mypid->dlast) / mypid->N + (mypid->Kd * mypid->T)*(mypid->y - mypid->ylast) / (mypid->T + (mypid->T / mypid->N));
    mypid->u = mypid->Kp * mypid->e; //+mypid->d;

    pidOutDutyCycle = (float) ((mypid->u / degPOT)*2 * PWM_COUNTS_PERIOD);


    if (pidOutDutyCycle >= 2 * PWM_COUNTS_PERIOD) {
        pwmOUT[0] = (unsigned int) (2 * PWM_COUNTS_PERIOD);
        pwmOUT[1] = 0;
    } else if (pidOutDutyCycle <= -(2 * PWM_COUNTS_PERIOD)) {
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int) (2 * PWM_COUNTS_PERIOD);
    } else if (pidOutDutyCycle < 0) {
        pwmOUT[0] = 0;
        pwmOUT[1] = (unsigned int) ((-1)*2 * pidOutDutyCycle);
    } else {
        pwmOUT[0] = (unsigned int) (2 * pidOutDutyCycle);
        pwmOUT[1] = 0;
    }
    return pwmOUT;
}


int main() {
    InitCan();      // Initialize CAN module
    InitUart();     // Initialize UART module
    InitPwm();      // Initialize PWM module
    InitAdc();      // Initialize ADC module
    InitInt();      // Initialize External Interrupt

    // Misc. variables
    unsigned char i, j = 0;
    // Misc. variables
    float degPOT = 0.0;
    float degMTR = 0.0;
    unsigned int pwmOUT[4]={0, 0, 0, 0};
    pid_t mypid;
    float newKP = 0.0;
    float newKD = 0.0;
    float newKI = 0.0;
    unsigned char charArray[8] = {'\0'};




    while (1) {

        for (i = 0; i < UART_TX_LEN; i++) {
            txData[i] = '\0';
        }
        switch (motorState) {
            case INITIALIZE:

                // Initialization to offset POSCNT to three turns (12000 counts)
                POSCNT = 12000; // This prevents under and overflow of the POSCNT register

                // Enable PWM Module
                PTCONbits.PTEN = 1;

                // Initialize PID
                InitPid(&mypid, PID_KP, PID_KD, PID_KI, PID_TS, PID_N, 0, 0, 0, 0);

                // Enable CAN module
                C1CTRLbits.REQOP = NORMAL;
                while (C1CTRLbits.OPMODE != NORMAL);

                motorState = SEND_HOME;
                break;

            case SEND_HOME:
                switch(sendMsg){
                    case 49:
                        // Sends home command
                        C1TX0B4 = SEND_HOME;
                        C1TX0CONbits.TXREQ = 1;
                        while (C1TX0CONbits.TXREQ != 0);

                        sprintf(txData, "Homing...\r\n");
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }

                        sendMsg = 0;
                        break;

                    case 50:
                        // Sends normal operation command
                        C1TX0B4 = 6;
                        C1TX0CONbits.TXREQ = 1;
                        while (C1TX0CONbits.TXREQ != 0);
                        
                        sprintf(txData, "Normal Operation\r\n");
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }

                        sendMsg = 0;
                        break;
                    case 51:
                        // Sends new KP constants

                        newKP = (float) PID_KP;
                        sprintf(charArray, "%f\0", newKP);
                        j = 0;
                        for (i = 0; i < 3; i++) {
                            OutData0[i] = charArray[j];
                            OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
                            j += 2;
                        }
                        C1TX0B1 = OutData0[0];
                        C1TX0B2 = OutData0[1];
                        C1TX0B3 = OutData0[2];
                        C1TX0B4 = KP_PROGRAM;
                        C1TX0CONbits.TXREQ = 1;
                        while (C1TX0CONbits.TXREQ != 0);

                        sprintf(txData, "New KP of %f\r\n", newKP);
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }

                        sendMsg = 0;
                        break;
                    case 52:
                        // Sends new KP constants

                        newKD = (float) PID_KD;
                        sprintf(charArray, "%f\0", newKD);
                        j = 0;
                        for (i = 0; i < 3; i++) {
                            OutData0[i] = charArray[j];
                            OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
                            j += 2;
                        }
                        C1TX0B1 = OutData0[0];
                        C1TX0B2 = OutData0[1];
                        C1TX0B3 = OutData0[2];
                        C1TX0B4 = KD_PROGRAM;
                        C1TX0CONbits.TXREQ = 1;
                        while (C1TX0CONbits.TXREQ != 0);

                        sprintf(txData, "New KD of %f\r\n", newKD);
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }

                        sendMsg = 0;
                        break;
                    case 53:
                        // Sends new KP constants

                        newKI = (float) PID_KI;
                        sprintf(charArray, "%f\0", newKI);
                        j = 0;
                        for (i = 0; i < 3; i++) {
                            OutData0[i] = charArray[j];
                            OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
                            j += 2;
                        }
                        C1TX0B1 = OutData0[0];
                        C1TX0B2 = OutData0[1];
                        C1TX0B3 = OutData0[2];
                        C1TX0B4 = KI_PROGRAM;
                        C1TX0CONbits.TXREQ = 1;
                        while (C1TX0CONbits.TXREQ != 0);

                        sprintf(txData, "New KI of %f\r\n", newKI);
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }

                        sendMsg = 0;
                        break;

                    case 80:
                        // Sends position values
                        txInProgress = 0;
                        sprintf(txData, "%5u %5u %5u %5u %5u %5u\0", masterData[0], masterData[1], masterData[2], slaveData[0], slaveData[1], slaveData[2]);
                        for (i = 0; i < UART_TX_LEN; i++) {
                            U1TXREG = txData[i];
                            while (!(U1STAbits.TRMT));
                        }
                        txInProgress = 1;
                        sendMsg = 0;
                        break;

                }


                break;
        }

//        sprintf(txData, "%u %u %u %u %u %u %u\r\n", masterData[0], masterData[1], masterData[2], slaveData[0], slaveData[1], slaveData[2], masterData[3]);
//        sprintf(txData, "%u %u %u %u %u %u\r\n", masterData[0], masterData[1], masterData[2], slaveData[0], slaveData[1], slaveData[2]);
//        sprintf(txData, "%u %u %u %u %u %u\0", masterData[0], masterData[1], masterData[2], slaveData[0], slaveData[1], slaveData[2]);
//        for (i = 0; i < UART_TX_LEN; i++) {
//            U1TXREG = txData[i];
//            while (!(U1STAbits.TRMT));
//        }
        msDelay(PID_TS);
    } //while
} // main

void __attribute__((interrupt, no_auto_psv)) _C1Interrupt(void) {
    IFS1bits.C1IF = 0; // Clear interrupt flag
    if (C1INTFbits.TX0IF) {
        C1INTFbits.TX0IF = 0;
    } // TX0IF

    if (C1INTFbits.RX0IF) {
        C1INTFbits.RX0IF = 0;
        C1RX0CONbits.RXFUL = 0;
        masterData[3] = C1RX0B4;
//        if (txInProgress) {
            if (C1RX0B4 == 1) {
                masterData[0] = C1RX0B1;
            } else if (C1RX0B4 == 2) {
                masterData[1] = C1RX0B1;
            } else if (C1RX0B4 == 3) {
                masterData[2] = C1RX0B1;
            } else if (C1RX0B4 == 4) {
                slaveData[0] = C1RX0B1;
            } else if (C1RX0B4 == 5) {
                slaveData[1] = C1RX0B1;
            } else {
                slaveData[2] = C1RX0B1;
            }
//        }
    } // RX0IF
} // C1Interrupt

void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void) {
    IFS0bits.U1TXIF = 0; // Clear U1TX interrupt
}

void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void) {
    IFS0bits.U1RXIF = 0; // Clear U1RX interrupt
    if (sendMsg != 80) {
        sendMsg = U1RXREG;
    }
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
    IFS0bits.ADIF = 0;
    ADCValue0 = ADCBUF0;
}

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {
    IFS0bits.INT0IF = 0;
//    // Sends home command
//    C1TX0B4 = SEND_HOME;
//    C1TX0CONbits.TXREQ = 1;
//    while (C1TX0CONbits.TXREQ != 0);

//    unsigned char i, j = 0;
//    float newKP = 0.0;
//    unsigned char charArray[8]={'\0'};
//    // Sends new PID constants
//    newKP = (float) PID_KD;
//    sprintf(charArray, "%f\0", newKP);
//    j = 0;
//    for (i = 0; i < 3; i++) {
//        OutData0[i] = charArray[j];
//        OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
//        j += 2;
//    }
//
//    C1TX0B1 = OutData0[0];
//    C1TX0B2 = OutData0[1];
//    C1TX0B3 = OutData0[2];
//    C1TX0B4 = KD_PROGRAM;
//    C1TX0CONbits.TXREQ = 1;
//    while (C1TX0CONbits.TXREQ != 0);

}

void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void) {
    IFS1bits.INT1IF = 0;
    // Sends normal operation command
//    C1TX0B4 = 6;
//    C1TX0CONbits.TXREQ = 1;
//    while (C1TX0CONbits.TXREQ != 0);

//    unsigned char i, j = 0;
//    float newKP = 0.0;
//    unsigned char charArray[8]={'\0'};
//    // Sends new PID constants
//    newKP = (float) PID_KP;
//    sprintf(charArray, "%f\0", newKP);
//    j = 0;
//    for (i = 0; i < 3; i++) {
//        OutData0[i] = charArray[j];
//        OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
//        j += 2;
//    }
//
//    C1TX0B1 = OutData0[0];
//    C1TX0B2 = OutData0[1];
//    C1TX0B3 = OutData0[2];
//    C1TX0B4 = KP_PROGRAM;
//    C1TX0CONbits.TXREQ = 1;
//    while (C1TX0CONbits.TXREQ != 0);

/********************** code for decoding ASCII to float ***********************
*                j=0;
*                for (i = 0; i < 3; i++) {
*                    rxData[j] = (InData0[i] >> 8);
*                    rxData[j + 1] = (InData0[i]);
*                    j += 2;
*                }
*                degMTR = atof(rxData);
*******************************************************************************/
/********************** code for encoding float to ASCII ***********************
*    unsigned char i, j = 0;
*    float newKP = 0.0;
*    unsigned char charArray[8]={'\0'};
*    // Sends new PID constants
*    newKP = (float) PID_KP;
*    sprintf(charArray, "%f\0", newKP);
*    j = 0;
*    for (i = 0; i < 3; i++) {
*        OutData0[i] = charArray[j];
*        OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
*        j += 2;
*    }
*******************************************************************************/
}

void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void) {
    IFS1bits.INT2IF = 0;
//    unsigned char i, j = 0;
//    float newKP = 0.0;
//    unsigned char charArray[8]={'\0'};
//    // Sends new PID constants
//    newKP = (float) PID_KI;
//    sprintf(charArray, "%f\0", newKP);
//    j = 0;
//    for (i = 0; i < 3; i++) {
//        OutData0[i] = charArray[j];
//        OutData0[i] = (OutData0[i] << 8) + charArray[j + 1];
//        j += 2;
//    }
//
//    C1TX0B1 = OutData0[0];
//    C1TX0B2 = OutData0[1];
//    C1TX0B3 = OutData0[2];
//    C1TX0B4 = KI_PROGRAM;
//    C1TX0CONbits.TXREQ = 1;
//    while (C1TX0CONbits.TXREQ != 0);
}