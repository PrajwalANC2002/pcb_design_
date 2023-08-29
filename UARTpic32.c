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
#define CORE_TICK_RATE 20000000
/* 
 * File:   main.c
 * Author: 918088932647
 *
 * Created on 12 August, 2022, 4:22 PM
 */
#include <stdio.h>
#include <stdlib.h>
#include "LCD_Initialization.h"
#define PB_CLK (40000000)
#define MAX_COMMAND_LENGTH 25
#define U2_BAUDRATE (9600)
#define UX_BRG ((PB_CLK/(16*U2_BAUDRATE))-1)
uint8_t run_stop=0, start_stop_event=0;
uint8_t BLE_msg_rcvd=0;
uint8_t BLE_msg_string[MAX_COMMAND_LENGTH]="";
uint8_t RX_char=0;
void UART(void);
uint8_t recieve_string[MAX_COMMAND_LENGTH]="";

int main(int argc, char** argv){
    TRISBCLR=((1<<13)|(1<<14)|(1<<15)|(1<<3)|(1<<8)|(1<<9)|(1<<10)|(1<<11)|(1<<5));  /*OUTPUT IC PINS*/
    TRISASET=(1<<0);
    TRISASET=(1<<1);
    TRISBSET=(1<<7);
    ANSELBCLR=_ANSELB_ANSB0_MASK|_ANSELB_ANSB1_MASK;
    UART();
    LCD_Initialize();
    LCD_ClearDisplay();
    LCD_Display(" FreeRTOS with     PIC Micro");
    delay(1000);
    while(1){
        if(BLE_msg_rcvd)
        {
            LATBINV(1<<15);
            if(strncmp(BLE_msg_string, "PIC+START",9)==0)//User calibration
            {
                run_stop=0Xff;
                start_stop_event=1;
                BLE_transmit("PIC+START:DONE\r\n");
            }
            else if(strncmp(BLE_msg_string, "PIC+STOP",8)==0)//User calibration
            {
                run_stop=0;
                start_stop_event=1;
                BLE_transmit("PIC+STOP:DONE\r\n");
            }                
            BLE_msg_rcvd=0;
        } 
        if(start_stop_event)
        {
             LCD_ClearDisplay();
            if(run_stop)
            {
                T2CONSET=_T2CON_TON_MASK; // Timer2 ON
                T3CONSET=_T3CON_TON_MASK; // Timer3 ON
                T4CONSET=_T4CON_TON_MASK; // Timer4 ON
                LCD_Display("Timers Started");
                BLE_transmit("Timers Started\r\n");
            }
            else
            {
                T2CONCLR=_T2CON_TON_MASK; // Timer2 OFF
                T3CONCLR=_T3CON_TON_MASK; // Timer3 OFF
                T4CONCLR=_T4CON_TON_MASK; // Timer4 OFF  
                LCD_Display("Timers Stopped");
                BLE_transmit("Timers Stopped\r\n");
            }
            start_stop_event=0;
        }
       delay(100);
    }
    return (EXIT_SUCCESS);
    }
void __ISR(_UART_2_VECTOR,IPL2SOFT) BLUT(void){
        uint8_t BLE_char,i;
    static uint8_t BLE_msg_len=0;
    if(IFS1 &(1<<_IFS1_U2TXIF_POSITION)) // TXIF FLAG IS  SET
    {
        IFS1CLR=_IFS1_U2TXIF_MASK;
    }
    if(IFS1 &(1<<_IFS1_U2RXIF_POSITION)) // RXIF FLAG IS  SET
    {
        /* Clear the interrupt flag */
        /* ISR-specific processing */
        BLE_char=U2RXREG;
        if(BLE_char!=0)			
		{
			if((BLE_char != '\n') && BLE_msg_len <= 25)				//until charcter count reaches 98 (GSM Queue size is limited to 100) And end of line is reached, take the character. Discard characters after 98.
			{
				recieve_string[BLE_msg_len] = BLE_char;
				BLE_msg_len++;
			}
			else
			{
				recieve_string[BLE_msg_len-1] = '\0';
                for(i=0;i<BLE_msg_len;i++)
                BLE_msg_string[i]=recieve_string[i];
                BLE_msg_rcvd=1;
				BLE_msg_len = 0;
			}
		}        
        IFS1CLR=_IFS1_U2RXIF_MASK;
    }
    if(IFS1 &(1<<_IFS1_U2EIF_POSITION)) // EIF FLAG IS  SET
    {        
        IFS1CLR=_IFS1_U2EIF_MASK;
    }
}
void UART(void){
    U2RXR=2;
    RPB0R=2;
    U2BRG=UX_BRG;
    INTCONSET=_INTCON_MVEC_MASK;
    U2MODESET=_U2MODE_ON_MASK;
    U2MODECLR=_U2MODE_IREN_MASK;
    U2MODECLR=_U2MODE_UEN_MASK;
    U2MODESET=(0<<_U2MODE_UEN_POSITION);
    U2MODECLR=_U2MODE_LPBACK_MASK;
    U2MODECLR=_U2MODE_ABAUD_MASK;
    U2MODECLR=_U2MODE_PDSEL_MASK;
    U2MODESET=(0<<_U2MODE_PDSEL_POSITION);
    U2MODECLR=_U2MODE_STSEL_MASK;
    U2MODECLR=_U2MODE_BRGH_MASK;
    
    U2STASET=_U2STA_URXEN_MASK;
    U2STASET=_U2STA_UTXEN_MASK;
    U2STACLR=_U2STA_URXISEL_MASK;
    U2STASET=(0<<_U2STA_URXISEL_POSITION);
    
    IFS1CLR=_IFS1_U2RXIF_MASK;
    IEC1SET=_IEC1_U2RXIE_MASK;
    IFS1CLR=_IFS1_U2TXIF_MASK;
    IEC1CLR=_IEC1_U2TXIE_MASK; //IF SET REPEAT THE LOOP FOR BLURTOOTH TRANSMISSION IN _ISR 
    IPC9CLR=_IPC9_U2IP_MASK;
    IPC9SET=(2<<_IPC9_U2IP_POSITION);
    IPC9CLR=_IPC9_U2IS_MASK;
    IPC9SET=(1<<_IPC9_U2IS_POSITION);
    __builtin_enable_interrupts();
    }
void BLE_transmit(uint8_t *tras)
{
	uint8_t i=0;
	for(i=0;tras[i]!='\0';i++)
	{
		U2TXREG=tras[i];
		while(!(U2STA &(1<<_U2STA_TRMT_POSITION)));
	}
}
