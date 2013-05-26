/*
 * File:   main.h
 * Author: blackbliss
 *
 * Created on 22 febbraio 2013, 20.56
 */

#ifndef MAIN_H
#define	MAIN_H

#define _XTAL_FREQ 8000000L
#define WAIT_US( x ) _delay( x * ( _XTAL_FREQ / 4000000))
#define WAIT_MS( x ) _delay( x * ( _XTAL_FREQ / 4000))

#define CE      PORTCbits.RC2
#define CSN     PORTDbits.RD2
#define SCK     PORTCbits.RC3
#define MISO    PORTCbits.RC4
#define MOSI    PORTCbits.RC5
#define IRQ     PORTDbits.RD1

//#define LED     PORTDbits.RD0

#define SKIP_MEASURE    PORTDbits.RD5

#define PGOOD           PORTBbits.RB0


/* MCU Configuration */
// da incollare

//void init(void);
//void spiInit(void);
//void serialInit(void);
//void adcInit(void);
//int adcRead(unsigned char channel);
//void putch (char c);
//unsigned char getch();
//unsigned char getche(void);

#endif	/* MAIN_H */
