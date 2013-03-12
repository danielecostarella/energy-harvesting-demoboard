/*
 * File:   main.c
 * Author: blackbliss
 *
 * Created on 26 febbraio 2013, 11.11
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

//new
//#pragma config BOREN = 2    // 10 = BOR enabled during operation and disabled in Sleep
//#pragma config BORV = 0     // 0 = Brown-out Reset Voltage (VBOR) set to 2.5 V nominal
//#pragma config PWRTE = ON   // 1 = PWRT disabled


//#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
//#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
//#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)
#pragma config PLLEN = ON

// CONFIG2
#pragma config VCAPEN=0;
//#pragma config BORV = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
//#pragma config WRT = 1FOURTH    // Flash Program Memory Self Write Enable bits (0000h to 07FFh write protected, 0800h to 1FFFh may be modified by EECON control)


#define VMIN            120         // 120 => Vstorage = 10V - 144 => Vstorage = 12V
#define SLEEP_COUNT     2           // 2 => 30s; 20 => 5min etc

//#define _XTAL_FREQ 16000000L
//#define WAIT_US( x ) _delay( x * ( _XTAL_FREQ / 4000000))
//#define WAIT_MS( x ) _delay( x * ( _XTAL_FREQ / 4000))
/*
 *
 */

void power_on_delay(void);
void spiInit(void);
void uartInit(void);
void putch (char c);
unsigned char getche(void);

void Send_Packet(uint8_t type,uint8_t* pbuf,uint8_t len);
void Receive_Packet(void);
uint8_t adcRead(uint8_t ch, uint8_t range);
void adcInit(void);
uint8_t adcReadTemp(void);
uint8_t adcReadVcap(void);

extern void RFM70_Initialize(void);
extern void SwitchToTxMode(void);
extern void SwitchToRxMode(void);
//void rfm70Task(void);

//uint8_t tx_buf[17]={0x00,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,
          //          0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x79};
//uint8_t tx_buf[MAX_PACKET_LEN];
uint8_t tx_buf[4]={0x00, 0x00, 0x00, 0x00};
uint8_t rx_buf[MAX_PACKET_LEN];

extern const uint8_t RX0_Address[];
extern const unsigned long Bank1_Reg0_13[];
uint8_t tmr1Counter = 0;
uint8_t tmr1Target = 1;
static bit flag = 0;

int16_t timelapse_count = 0;

void interrupt ISR(void)
   {
   if(TMR1IF) // interrupt su overflow timer1: è passato un secondo
      {
      tmr1Counter++;
      //TMR1H=0x00;
      //TMR1L=0x00;

      //TMR1H = 0xF0; //ritardo breve
      //TMR1H = 0x60; // 10s @ 32KHz questo è quello giusto
      TMR1H = 0x10; // 15s
      TMR1L = 0x00;

      if(tmr1Counter == tmr1Target){
        printf("Eccomi nell'ISR!\r\n");
        tmr1Counter = 0;
        flag = 1;
        //timelapse_count++;
      }
      
      TMR1IF=0; // azzero il flag di interrupt
      }
   }


int main(int argc, char** argv) {
    long int i=0;
    int16_t value;
    uint8_t buffer[5];
    uint8_t temp_tx_buf[32];
    //int16_t timelapse_count = 0;
    uint8_t num_sleep;

    //power_on_delay();

    OSCCONbits.IRCF=2; //3 for 16MHz

    ANSELDbits.ANSD0=0;     // digital led
    //ANSELCbits.ANSC0=0;     // digital
    ANSELCbits.ANSC6=0;     // digital
    ANSELCbits.ANSC7=0;     // digital
    //TRISCbits.TRISC0=0;     //LED
    TRISDbits.TRISD0=0;         //LED
    LED=0;
    TRISCbits.TRISC6=0;     //TX

    ANSELCbits.ANSC2=0;     // CE as digital
    ANSELDbits.ANSD2=0;     // CSN as digital
    ANSELCbits.ANSC5=0;     // MOSI as digital
    ANSELDbits.ANSD1=0;     // IRQ as digital

    //adc
    TRISEbits.TRISE2=1;     // ADC in
    ANSELEbits.ANSE2=1;     // ADC pin as analog

    //RFM70 pins
    TRISCbits.TRISC2=0;     // CE
    TRISDbits.TRISD2=0;     // CSN
    TRISCbits.TRISC3=0;     // SCK
    TRISCbits.TRISC5=0;     // MOSI
    TRISCbits.TRISC4=1;     // MISO
    TRISDbits.TRISD1=1;     // IRQ
    PORTCbits.RC2=0;
    PORTDbits.RD2=1; //aggiunto csn
    PORTCbits.RC3=0; //aggiunto sck
    PORTCbits.RC5=0; //aggiunto mosi

    //WDT
    //OPTION_REGbits.PSA=1;
    //OPTION_REGbits.PS=0x07;

    // MCP9700A
    TRISEbits.TRISE1=0;     // MCP9700A power
    ANSELEbits.ANSE1=0;     // pin RE1 as digital
    TRISEbits.TRISE2=1;     // MCP9700A Vout
    ANSELEbits.ANSE2=1;     // set RE2 pin as analog

    TRISEbits.TRISE0=1;     // RE0 as input
    ANSELEbits.ANSE0=0;     // set RE0 pin as analog (Vstorage)

    // Vstorage monitor
    TRISEbits.TRISE0=1;     // for Vstorage measuring
    ANSELEbits.ANSE0=1;     // pin RE0 as analog
    TRISDbits.TRISD3=0;     // enable Vstorage measure
    PORTDbits.RD3=0;        // set to low



    GIE=1; // gestione globale interrupt attiva
    PEIE=1; // interrupt di periferica abilitati
    TMR1IE=1; // interrupt su overflow timer1 abilitato


        // LED blinking
        //LED=1;
        //WAIT_MS(100);
        //LED=0;
        //WAIT_MS(100);
    
    //TMR1H=0x00;
    //TMR1L=0x00;

    TMR1H = 0x10;
    TMR1L = 0x00;
    
    T1CONbits.TMR1CS=2; //0 for instruction clock
    T1CONbits.T1CKPS=3; //Prescaler
    T1GCONbits.TMR1GE=1;
    T1CONbits.nT1SYNC=1;
    T1CONbits.T1OSCEN=1;    //era 0 for disabled
    T1CONbits.TMR1ON=1;


    spiInit();
    uartInit();
    adcInit();
   
    //SPI_Write_Reg(WRITE_REG | CONFIG, 0x08);
    //printf("CONFIG NEW: %X\r\n", SPI_Read_Reg(CONFIG));
    rfm70setPowerdownMode(0); //?
    //printf("CONFIG INIZIALE: %X\r\n", SPI_Read_Reg(CONFIG));

    // rimettere sleep
    //SLEEP(); //<-- per farlo consumare poco
    SLEEP();
    SLEEP();
    // rimettere sleep


    //PORTEbits.RE1=1; //mcp9700 power up
    //adcRead(7,0); // read dummy data (?)
    //SLEEP();
    //while(1) {
        //WAIT_MS(500);
        //value = adcReadTemp();
        ////value = adcRead(7,0); //(4*adcRead(7,0)-500);
        //printf("TEMP value: %X => %d\r\n", value, value);
        //value = adcReadVcap();
        ////value = adcRead(5,1); //era 1
        //printf("Vstorage value: %d\r\n", value);
        //printf("\r\n");
    //}

    
    //printf("ADC value: %d\r\n", adcRead(7));
    
    value = adcReadVcap();
    while (value < VMIN) {
        SLEEP();
        value = adcReadVcap();
        printf("Vstorage too low: %d\r\n", value);
    }
    printf("Vstorage OK: %d\r\n", value);
    RFM70_Initialize();

    rfm70setPowerdownMode(0);
    //while(1) {
    
    //    WAIT_MS(1000);
    //}

    //SLEEP();
    SLEEP();

    //WAIT_MS(50);
    //tx_buf[0] = '\0';	// initialize tx buffer
    //power_on_delay();
    //rfm70setPowerdownMode(0);
    //CSN=0;

    //printf("Sto andando in SLEEP!");
    //WAIT_MS(2000);
    //SLEEP();




    //while(1);
    //Send_Packet(W_TX_PAYLOAD_NOACK_CMD,tx_buf,2);	// transmit
    //CE=0;
    //printf("Packet sent!\r\n");
    //while(1);
    //PORTCbits.RC0=1;

    /*
    printf("=== RFM70 REGISTERS ===\n\r");
    printf("CONFIG: %X\r\n", SPI_Read_Reg(CONFIG));
    printf("ENAA: %X\r\n", SPI_Read_Reg(0x01));
    printf("EN_RX_ADDR: %X\r\n", SPI_Read_Reg(0x02));
    printf("SETUP_AW: %X\r\n", SPI_Read_Reg(0x03));
    printf("SETUP_RETR: %X\r\n", SPI_Read_Reg(0x04));
    printf("RFCH: %X\r\n", SPI_Read_Reg(0x05));
    printf("RFSETUP: %X\r\n", SPI_Read_Reg(0x06));
    printf("STATUS: %X\r\n", SPI_Read_Reg(0x07));
    printf("OBSERVE_TX: %X\r\n", SPI_Read_Reg(0x08));
    printf("CD: %X\r\n", SPI_Read_Reg(0x09));
    printf("FIFO_STATUS: %X\r\n", SPI_Read_Reg(0x17));
    printf("DYNPD = %X\r\n",SPI_Read_Reg(0x1D));
    printf("FEATURE = %X\n\r\n\r", SPI_Read_Reg(0x1D));

    SPI_Read_Buf(0x10,buffer,5);
    printf("TX_ADDR = ");
    for (i=0;i<5;i++) {
        printf("%X-", buffer[i]);
    }
    */
    //LED=1;
    //WAIT_MS(50);
    //LED=0;
    timelapse_count = 0;
    while(1) {
        rfm70setPowerdownMode(1);
        //if(flag == 1){
        //    flag = 0;
        //    SLEEP();
        //}
        //PORTCbits.RC0=0;
        //WAIT_MS(100);
        for (i=0;i<2;i++) {
            temp_tx_buf[i]=tx_buf[i];
        }
        //tx_buf[0] = adcRead(5,1);

        //value = adcReadTemp();
        tx_buf[0] = adcReadTemp();
        tx_buf[1] = adcReadVcap();
        tx_buf[2] = (uint8_t)((timelapse_count) & 0xFF);
        tx_buf[3] = (uint8_t)(((timelapse_count) >> 8) & 0xFF);
        //tx_buf[2] = (uint8_t)((timelapse_count/SLEEP_COUNT) & 0xFF);
        //tx_buf[3] = (uint8_t)(((timelapse_count/SLEEP_COUNT) >> 8) & 0xFF);
        
        //PORTCbits.RC0=1;
        //Send_Packet(W_ACK_PAYLOAD_CMD,temp_tx_buf,17);	// transmit
        if (tx_buf[1]> VMIN) {
            //rfm70setPowerdownMode(1);
            Send_Packet(W_TX_PAYLOAD_NOACK_CMD,tx_buf,4);	// transmit
        }
        else
            printf("VStorage: %d  => not OK\r\n", tx_buf[1]);
        //tx_buf[0] = '\0';	// clear tx_buf
        //tx_buf[0]++;

        //tx_buf[1]++;
        //tx_buf[2]++;
        //tx_buf[3]++;
        //SwitchToRxMode();  	// switch to Rx mode - TX VERSION
        CE=0;
        rfm70setPowerdownMode(0);
        T1CONbits.TMR1ON=0;
        //TMR1H = 0xF0; //ritardo  breve
        //TMR1H = 0x60;
        TMR1H = 0x10;
        TMR1L = 0x00;
        T1CONbits.TMR1ON=1;
        //WAIT_MS(100);
        
        num_sleep = 0;
        while(num_sleep++ < SLEEP_COUNT) {
            SLEEP();
            printf("Sleep: %d\r\n", num_sleep);
        }
        timelapse_count++;
    }
    return (EXIT_SUCCESS);
}




uint8_t adcReadTemp(void) {
    uint8_t value;
    ADCON0bits.CHS=7;
    FVRCONbits.ADFVR=1;
    ADCON0bits.ADON=1;
    PORTEbits.RE1=1;     // mcp9700a power up
    WAIT_MS(5);
    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    value = ADRES;
    ADCON0bits.ADON=0;
    PORTEbits.RE1=0;     // turn off mcp9700a
    return value;
}

uint8_t adcReadVcap(void) {
    uint8_t value;
    ADCON0bits.CHS=5;
    FVRCONbits.ADFVR=2;
    ADCON0bits.ADON=1;
    PORTDbits.RD3=1;        // Select Vstorage measurement
    WAIT_MS(1);
    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    value = ADRES;
    ADCON0bits.ADON=0;
    PORTDbits.RD3=0;            // Turn off switch for Vstorage measure
    return value;
}

// range 0: Vref_int = 1.024
// range 1: Vref_int = 2.048
// ch = 7 => mpc9700a read
// ch = 5 => Vstorage read
uint8_t adcRead(uint8_t ch, uint8_t range) {
    uint8_t value;
    ADCON0bits.CHS=ch;          // read from channel 'ch'
    //ADCON0bits.ADON=1;
    
    if (range == 0) {              // read from MCP9700A
        //FVRCONbits.FVREN=0;     //FVR disabled
        FVRCONbits.ADFVR=1;     // 1.024 as reference
        ADCON0bits.ADON=1;
        //WAIT_MS(1);
        //FVRCONbits.FVREN=1;
        PORTEbits.RE1=1;     // mcp9700a power up
        WAIT_MS(20);             // mcp9700 Turn-on Time
    }
    else
        PORTDbits.RD3=1;        // Select Vstorage measurement
        FVRCONbits.ADFVR=2;  // read from Vstorage
        ADCON0bits.ADON=1;
        WAIT_MS(1);         // serve??????
    
    while(!FVRCONbits.FVRRDY);  // 1 = Fixed Voltage Reference output is ready for use
    // Note: FVRRDY is always ?1? on PIC16F707 devices.

    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    //while(!PIR1bits.ADIF);
    value = ADRES;
    ADCON0bits.ADON=0;       //ADC is disabled and consumes no operating current
    //PORTEbits.RE1=0;            // Turn off mcp9700a
    PORTDbits.RD3=0;            // Turn off switch for Vstorage measure
    return value;
}


void uartInit(void) {
    TRISCbits.TRISC7=1;             // RC7 as UART RX pin input
    TRISCbits.TRISC6=0;             // RC6 as UART TX pin input

    TXSTA=0x02;             // High speed
    RCSTA=0x00;

    //BAUDCON=0x00;
    //BAUDCONbits.SCKP=0; // 1 = Transmit inverted data to the TX/CK pin


    TXSTAbits.BRGH=0;
    TXSTAbits.SYNC=0;

    //SPBRGH=0x00;        // Baudrate
    SPBRG=12; // 25 for 16MHz

    RCSTAbits.SPEN=1;
    TXSTAbits.TXEN=1;

}

void spiInit(void) {
    SSPCONbits.SSPEN=1;       // 1= Enables SPI
    SSPCONbits.CKP=0; //0 = Idle State for clock is a low level
    SSPCONbits.SSPM=2;    // SPI Master Mode clock=Fosc/64

    SSPSTATbits.SMP=0;   //0  // input data sampled at middle of data output time
    SSPSTATbits.CKE=1;   //1  // Trasmit occurs on transition from idle to active
                            //clock state //in origine 0
    WAIT_MS(50);
}


void adcInit(void) {
    //ADCON0bits.CHS=0x07;
    //ADCON1bits.ADCS=0x00;
    ADCON1bits.ADCS=1;      // convert @ fosc/8
    //ADCON1bits.ADREF=0;
    ADCON1bits.ADREF=3;  // 11 = VREF is connected to internal Fixed Voltage Reference
    FVRCONbits.FVREN=1;     // 1 = Fixed Voltage Reference is enabled
}

//You must write putch() else printf will complain
void putch (char c)
{
    while(!TXIF);
    TXREG = c;
}

unsigned char getch() {
	// retrieve one byte
	while(!RCIF)	// set when register is not empty
		continue;
	return RCREG;
}

unsigned char getche(void) {
	unsigned char c;
	putch(c = getch());
	return c;
}

/*********************************************************
Function:      power_on_delay()

Description:

*********************************************************/
void power_on_delay(void)
{
	WAIT_MS(1000);
}

/**************************************************
Function: Send_Packet
Description:
	fill FIFO to send a packet
Parameter:
	type: WR_TX_PLOAD or  W_TX_PAYLOAD_NOACK_CMD
	pbuf: a buffer pointer
	len: packet length
Return:
	None
**************************************************/
void Send_Packet(uint8_t type,uint8_t* pbuf,uint8_t len)
{
	uint8_t fifo_sta,i;

	SwitchToTxMode();  //switch to tx mode

	fifo_sta=SPI_Read_Reg(FIFO_STATUS);// read register FIFO_STATUS's value
	if((fifo_sta&FIFO_STATUS_TX_FULL)==0)//if not full, send data
                                                //(writebuff)
	{
            
            printf("Sending: ");	// print sent message to terminal
            for (i=0;i<len;i++) {
                printf("%X",pbuf[i]);
            }
            printf("\n\r");

                PORTCbits.RC2=0;
                //WAIT_MS(4);
		SPI_Write_Buf(type, pbuf, len); // Writes data to buffer
                PORTCbits.RC2=1;
                WAIT_US(10);
                PORTCbits.RC2=0;
                WAIT_MS(1);

                
                //LED=1 ;
                

                while(!IRQ) {
                    //printf("Interrupt! \n\r");
                    //printf("STATUS: %X\r\n", SPI_Read_Reg(0x07));
                    //LED=1 ; //per risparmiare
                    if (rfm70TxDataSentInterrupt()) {
                        printf("Data sent: OK \r\n");
                        SwitchToRxMode();
                        //CE=0;
                    }
                    if (rfm70TxDataSentErrorInterrupt()) {
                        printf("No data sent: ERROR \r\n ");
                        SwitchToRxMode();
                        //CE=0;
                    }
                    
                }
                

		//pbuf[len-1] = '\0';// remove checksum before sending to terminal
		
		printf("\n\r\n\r");

		//WAIT_MS(100);
		//LED=0; //per risparmiare
		//WAIT_MS(50);
	}
}

/**************************************************
Function: Receive_Packet
Description:
	read FIFO to read a packet
Parameter:
	None
Return:
	None
**************************************************/
void Receive_Packet(void)
{
	uint8_t len,rx_count,j,sta,fifo_sta,chksum;
	uint8_t rx_buf[MAX_PACKET_LEN];

	rx_count = 0;
	sta=SPI_Read_Reg(STATUS);	// read register STATUS's value

	if((STATUS_RX_DR&sta) == 0x40)// if receive data ready (RX_DR) interrupt
	{
		do
		{
			len=SPI_Read_Reg(R_RX_PL_WID_CMD);	// read len

			if(len<=MAX_PACKET_LEN)
			{
				SPI_Read_Buf(RD_RX_PLOAD,rx_buf,len);// read
                                           //receive payload from RX_FIFO buffer
				rx_count += len;// keep tally of # of bytes
                                                //received
			}
			else
			{
				SPI_Write_Reg(FLUSH_RX,0);//flush Rx
			}

			fifo_sta=SPI_Read_Reg(FIFO_STATUS);// read register
                                                           //FIFO_STATUS's value

		}while((fifo_sta&FIFO_STATUS_RX_EMPTY)==0); //while not empty

		rx_buf[rx_count] = '\0';// mark end of buffer with null
                                        //character
		chksum = 0;
		for(j=0;j<(rx_count-1);j++)		// calculate checksum
		{
			chksum += rx_buf[j];
		}

		if(chksum==rx_buf[rx_count-1])
		{
			LED=1; //metterlo verde?
			WAIT_MS(100);
			LED=0;

			//Send_Packet(W_TX_PAYLOAD_NOACK_CMD,rx_buf,17);
                        // not used...

 	//=========USE THE FOLLOWING FOR RX VERSION=======
			SwitchToRxMode();//switch to RX mode// RX VERSION

			rx_buf[len-1] = '\0';// terminate string with NULL
                                        //character (and remove the checksum)

			printf("Receiving: ");
			printf(rx_buf);
			printf("\n\r\n\r");
	//===============================================

		}
	}
	SPI_Write_Reg(WRITE_REG|STATUS,sta);	// clear RX_DR or TX_DS or
                                                   //MAX_RT interrupt flag
}
