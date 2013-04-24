/* 
 * File:   newmain.c
 * Author: blackbliss
 *
 * Created on 1 aprile 2013, 12.26
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
/*
 * 
 */

#pragma config FOSC = INTOSC// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
//#pragma config CP = OFF        // Data Code Protection bit (Data memory code protection is disabled)
//#pragma config BOREN = 0//SBODEN   // Brown Out Reset Selection bits (BOR controlled by SBOREN bit of the PCON register)

//new
//#pragma config BOREN = 2    // 10 = BOR enabled during operation and disabled in Sleep
//#pragma config BORV = 0     // 0 = Brown-out Reset Voltage (VBOR) set to 2.5 V nominal
//#pragma config PWRTE = ON   // 1 = PWRT disabled


//#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
//#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
////#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
//#pragma config PLLEN = OFF

// CONFIG2
//#pragma config VCAPEN=0;

#define _XTAL_FREQ 31000
#define WAIT_US( x ) _delay( x * ( _XTAL_FREQ / 4000000))
#define WAIT_MS( x ) _delay( x * ( _XTAL_FREQ / 4000))

int main(int argc, char** argv) {
    unsigned char i = 0;
    ANSELCbits.ANSC3=0;
    TRISCbits.TRISC3=0;
    PORTCbits.RC3=0;

    OSCCONbits.SCS=2;
    OSCCONbits.IRCF=1;

    while(1) {
        WAIT_MS(1000);
        PORTCbits.RC3=0;
        WAIT_MS(1000);
        PORTCbits.RC3=1;
    }
    return (EXIT_SUCCESS);
}

