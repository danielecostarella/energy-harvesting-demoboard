/* 
 * File:   main.c
 * Author: blackbliss
 *
 * Created on 12 febbraio 2013, 16.56
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "main.h"
#include "rfm70.h"


/* CONFIGURAZIONE MICROCONTROLLORE */
// CONFIG1
#pragma config FOSC = INTOSC// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
//#pragma config CP = OFF        // Data Code Protection bit (Data memory code protection is disabled)
//#pragma config BOREN = 0//SBODEN   // Brown Out Reset Selection bits (BOR controlled by SBOREN bit of the PCON register)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
//#pragma config BORV = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = 1FOURTH    // Flash Program Memory Self Write Enable bits (0000h to 07FFh write protected, 0800h to 1FFFh may be modified by EECON control)

#define _XTAL_FREQ 16000000L
#define WAIT_us( x ) _delay( x * ( _XTAL_FREQ / 4000000))
#define WAIT_ms( x ) _delay( x * ( _XTAL_FREQ / 4000))

/*
 * 
 */

void spiInit(void);

unsigned char rfm70id[4];

int main(int argc, char** argv) {

    long int i;
    int read = 0;
    unsigned char test;
    init();

    //printf(rfm70IsPresent);
    //test();


    printf("1\r\n");
    test = SPI_read(1);
    //test = rfm70Read(1);
    //rfm70Bank(1);
    printf("CONFREG = %X\r\n", test);
    printf("2\r\n");
    rfm70ReadBuffer(0x08,rfm70id, 4);
    printf("3\r\n");
    WAIT_ms(500);

    for (i=0; i<4; i++) {
        printf("ID[%d] = %X\r\n", i+1, rfm70id[i]);
    }
    //__delay_ms(100);
    TRISCbits.TRISC0=0;


    PORTCbits.RC0=1;
    printf(" test ");
    WAIT_us(5000);
    printf(" test again ");

    putch('Z');

    while(1); //break
    while(1) {
        PORTCbits.RC0=1;
        read = adcRead(2);
        for(i=0;i<50000;i++);
        PORTCbits.RC0=0;
        printf("VALUE: %d\n\r", read);
        for(i=0;i<50000;i++);
    }
    putch('Z');



    return (EXIT_SUCCESS);
}



void init(void) {
    //OSCCONbits.IRCF=0x0D;  //SET CLOCK to 4MHz
    OSCCONbits.IRCF=0x0F;  //SET CLOCK to 16MHz
    OSCCONbits.SCS=0x02;   //1X set to internal clock

    TRISAbits.TRISA2=1; // RA2 as input
    ANSELAbits.ANSA2=1;   // set pin RA2 to analog input

    //SPI
    TRISBbits.TRISB4=1;     // SDI pin
    TRISCbits.TRISC7=0;     // SDO pin
    TRISBbits.TRISB6=0;     // SCK pin (clock)
    TRISCbits.TRISC3=0;     // CSN pin (chip select)
    PORTCbits.RC3=1;        // set CSN pin

    PORTBbits.RB6=0;        // clear clock pin

    adcInit();
    serialInit();
    //spiInit();
}


void spiInit(void) {
    SSP1CON1bits.SSPEN=1;       // 1= Enables SPI
    SSP1CON1bits.CKP=0; //1 = Idle State for clock is a low level 
    SSP1CON1bits.SSPM=2;    // SPI Master Mode clock=Fosc/4

    SSP1STATbits.SMP=0;     // input data sampled at middle of data output time
    SSP1STATbits.CKE=1;     // Trasmit occurs on transition from idle to active clock state //in origine 0
    WAIT_ms(50);
}

void serialInit(void) {
    TRISBbits.TRISB5=1;     // RB5 as UART RX pin input
    TRISBbits.TRISB7=0;     // RB7 as UART TX pin input

    TXSTA=0x00;
    RCSTA=0x00;

    BAUDCON=0x00;
    BAUDCONbits.SCKP=0; // 1 = Transmit inverted data to the TX/CK pin


    TXSTAbits.BRGH=0;
    TXSTAbits.SYNC=0;

    SPBRGH=0x00;        // Baudrate
    SPBRGL=25;

    RCSTAbits.SPEN=1;
    TXSTAbits.TXEN=1;
    
}


// ADC Init
void adcInit(void) {
    // Configure ADC module
    // Select ADC conversion clock
    ADCON1bits.ADCS=0;
    // Select result format
    ADCON1bits.ADFM=1;
    // Set VDD and VSS reference
    ADCON1bits.ADPREF=0; //VDD REF
    // Turn on ADC module
    ADCON0bits.ADON=0;  // ADC is disabled and consumes no operating current
}


// Read by the selected channel
// input parameter: channel to read
// output parameter: result
int adcRead(unsigned char channel) {
    int i;
    ADCON0bits.CHS = channel;
    ADCON0bits.ADON=1;
    for(i=0;i<1000;i++);
    ADCON0bits.GO=1;
    //for(i=0;i<1000;i++);
    while (!ADIF) {
        // do nothing
    }
    ADIF=0;
    ADCON0bits.ADON=0;
    //printf("H: %d\r\n",ADRESH);
    //printf("L: %d\r\n",ADRESL);
    return (ADRESH << 8) | ADRESL;
}




//You must write putch() else printf will complain
void putch (char c)
{
    while(!TXIF);
    TXREG = c;
}

unsigned char getch() {
	/* retrieve one byte */
	while(!RCIF)	/* set when register is not empty */
		continue;
	return RCREG;
}

unsigned char getche(void) {
	unsigned char c;
	putch(c = getch());
	return c;
}