/*****************************************************************************
 * File:        main.h
 * Author:      Daniele Costarella <daniele.costarella@gmail.com>
 *
 * Processor:   PIC16
 *              tested with 16LF707, 16F707
 * Compiler:    Microchip XC8 v1.12 or higher
 *
 * Released on: June, 2013
 *
 *****************************************************************************
 * File description:
 *
 * Energy Harvesting Demoboard firmware
 * Header file for main.c
 *
 * For new versions of this code please visit:
 * https://github.com/danielecostarella/energy-harvesting-demoboard
 *
 * Detailed information can also be found on my master thesis [IT]
 *
 *****************************************************************************/

#ifndef MAIN_H
#define	MAIN_H

#define _XTAL_FREQ 8000000L
#define WAIT_US( x ) _delay( x * ( _XTAL_FREQ / 4000000))
#define WAIT_MS( x ) _delay( x * ( _XTAL_FREQ / 4000))

/*********************************** RFM70 ***********************************/
#define CE      PORTCbits.RC2
#define CSN     PORTDbits.RD2
#define SCK     PORTCbits.RC3
#define MISO    PORTCbits.RC4
#define MOSI    PORTCbits.RC5
#define IRQ     PORTDbits.RD1

#define SKIP_MEASURE    PORTDbits.RD5
#define PGOOD           PORTBbits.RB0
#define SCAP_MOS        PORTAbits.RA2   // 0: enabled; 1: disabled

#endif	/* MAIN_H */
