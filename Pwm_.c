#pragma config USERID = 0xFFFF          // Enter Hexadecimal value (Enter Hexadecimal value)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USID Selection (Controlled by the USB Module)
#pragma config FVBUSONIO = ON           // USB VBUS ON Selection (Controlled by USB Module)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider (2x Divider)
#pragma config UPLLEN = OFF             // USB PLL Enable (Disabled and Bypassed)
#pragma config FPLLODIV = DIV_2         // System PLL Output Clock Divider (PLL Divide by 2)

// DEVCFG1
#pragma config FNOSC = FRCPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include<cp0defs.h>
#include<sys/attribs.h>
#include<stdint.h>
/* 
 * File:   main.c
 * Author: 918088932647
 *
 * Created on 12 August, 2022, 4:22 PM
 */
#include <stdio.h>
#include <stdlib.h>
void TIMER2(void);
void OUTPUT(void);
int main(int argc, char** argv){
    TRISBCLR=((1<<13)|(1<<14)|(1<<15)|(1<<3)|(1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<5));  /*OUTPUT IC PINS*/
    TRISASET=(1<<0);
    TRISASET=(1<<1);
    TRISBSET=(1<<7);
    LATBCLR=((1<<13)|(1<<14)|(1<<15));
    TIMER2();
    OUTPUT();
    while(1){
    }
return(EXIT_SUCCESS);
 }
//void __ISR( _TIMER_2_VECTOR,IPL5SOFT) TIMER_2ISR(void)
//{
//   
//    IFS0CLR=_IFS0_T2IF_MASK;
//    }
//void __ISR(_OUTPUT_COMPARE_1_VECTOR,IPL4SOFT) OUTPUT_COMPARE(void){
//    
//    IFS0CLR=_IFS0_OC1IF_MASK;
//}

void TIMER2(void){
    INTCONSET=_INTCON_MVEC_MASK;
    TMR2=0;
    PR2=50000;//for 1ms with 1khz freq //(PR2*PRESCALER/40000000)(used to get required delay of different ms) highest PR1 VALUE 65535
    T2CONSET=_T2CON_ON_MASK;
    T2CONCLR=_T2CON_TCKPS_MASK;
    T2CONSET=(3<<_T2CON_TCKPS_POSITION);//(TO SET PRESCALER VALUE )
    T2CONCLR=_T2CON_TCS_MASK;
    T2CONCLR=_T2CON_TGATE_MASK;
    IFS0CLR=_IFS0_T2IF_MASK;
    IEC0CLR=_IEC0_T2IE_MASK; // TO DISABLE INTERRUPTS 
    IPC2CLR=_IPC2_T2IP_MASK;
    IPC2SET=(5<<_IPC2_T2IP_POSITION);
    IPC2CLR=_IPC2_T2IS_MASK;
    IPC2SET=(0<<_IPC2_T2IS_POSITION);
//    __builtin_enable_interrupts();
    }
void OUTPUT(void){
    RPB15R=5;
    int x=0;
    int y=0;
    x++;
    if(x==x){
    OC1RS=25000;//FOR 50%DUTY CYCLE
    }
    if(x==x+2){
        OC1RS=12500;
    }
    INTCONSET=_INTCON_MVEC_MASK;
    OC1CONSET=_OC1CON_ON_MASK;
    OC1CONCLR=_OC1CON_OC32_MASK;
    OC1CONCLR=_OC1CON_OCTSEL_MASK;
    OC1CONCLR=_OC1CON_OCM_MASK;
    OC1CONSET=(6<<_OC1CON_OCM_POSITION);
    IFS0CLR=_IFS0_OC1IF_MASK;
    IEC0CLR=_IEC0_OC1IE_MASK;// TO DISABLE INTERUPTS
    IPC1CLR=_IPC1_OC1IP_MASK;
    IPC1SET=(4<<_IPC1_OC1IP_POSITION);
    IPC1CLR=_IPC1_OC1IS_MASK;
    IPC1SET=(0<<_IPC1_OC1IS_POSITION);
//    __builtin_enable_interrupts();
}
