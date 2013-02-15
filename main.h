/* 
 * File:   main.h
 * Author: blackbliss
 *
 * Created on 12 febbraio 2013, 16.56
 */

#ifndef MAIN_H
#define	MAIN_H


void init(void);
void serialInit(void);
void adcInit(void);
int adcRead(unsigned char);


void putch(unsigned char byte);
unsigned char getch();
unsigned char getche(void);

#endif	/* MAIN_H */

