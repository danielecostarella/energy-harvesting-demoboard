/*****************************************************************************
 * File:        main.c
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
 *
 * For new versions of this code please visit:
 * https://github.com/blackbliss/energy-harvesting-demoboard
 *
 * Detailed information can also be found on my master thesis [IT]
 *
 *****************************************************************************/

/********************************** HEADERS **********************************/
#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "main.h"
#include "rfm70.h"


/**************************** MCU Configuration ******************************/
// set CONFIG1 bits
#pragma config FOSC = INTOSC// Oscillator Selection bits (INTOSCIO oscillator: 
                            // I/O function on RA6/OSC2/CLKOUT pin, I/O function
                            // on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF   // Watchdog Timer Enable bit (WDT disabled and can
                            // be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF  // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON   // RE3/MCLR pin function select bit (RE3/MCLR pin
                            // function is MCLR)
#pragma config CP = OFF     // Code Protection bit (Program memory code
                            // protection is disabled)
#pragma config BOREN = 0    // Brown Out Reset Selection bits (BOR controlled by
                            // SBOREN bit of the PCON register)
#pragma config PLLEN = ON

// set CONFIG2 bits
//#pragma config VCAPEN=0;  // only 16F707 (not used for 16LF707)

/******************************** VARIABLES **********************************/

/*****************************************************************************/
// Vmeas = (1000K/95K)*ADC_Count * 2048/256 = (8/95)*ADC_Count
// Fixed Voltage Reference: 2048 => 8mV/step

#define VMIN            8*(95/8)            // 120 => Vstorage = 10V
                                            // 144 => Vstorage = 12V
#define VCAP_MIN        (2.2*(68/248))/0.004// VCAP = 2.2 partitore: 68/248;
                                            // VRES = 1024/256 = 4mV
#define VCHARGE_START   11*(95/8)           // 11 Volts

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
void sleep(void);

extern void RFM70_Initialize(void);
extern void SwitchToTxMode(void);
extern void SwitchToRxMode(void);
uint8_t adcReadVsupercap(void);
//void rfm70Task(void);

//uint8_t tx_buf[17]={0x00,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,
          //          0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x79};
//uint8_t tx_buf[MAX_PACKET_LEN];
uint8_t tx_buf[10]={0x00, 0x00, 0x00, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0xAA, 0x00};
uint8_t rx_buf[MAX_PACKET_LEN];

extern const uint8_t RX0_Address[];
extern const unsigned long Bank1_Reg0_13[];
uint8_t tmr1Counter = 0;
uint8_t tmr1Target = 1;
static bit flag = 0;
static bit scap_on = 0; //scap_on = 1 => power supply not present
uint8_t jumper_stat=0;
uint8_t sleep_counter;
uint8_t sleep_count;

uint8_t value = 0;
//unsigned int value;
int16_t timelapse_count = 0;
unsigned int charge_time;   // int16_t
unsigned int charge_pulse; //= 10; //era 50 100; // pulse length in ms
float charge_pulse_float = 10; //charge_pulse = (int)charge_pulse_float

unsigned int dac_value = 26; //era 25

uint8_t values[2] = {0x00, 0x00};
unsigned int energy=0; // energy
void boardInit(void);

void interrupt ISR(void)
   {
   if(TMR1IF) // interrupt on TIMER1 overflow
      {
      tmr1Counter++;

      TMR1H = 0x10; // 15s
      TMR1L = 0x00;

      if(tmr1Counter == tmr1Target){
        ///printf("This is the ISR!\r\n");
        /*********************************************************************
         * jumper_stat byte
         *  _______ _____ _____ _____ _____ _____ _________ _________ ________
         * |supply |     |     |     |     |     |  sleep  | sleep   |  skip  |
         * |status |PGOOD|  x  |  x  |  x  |  x  | counter | counter |  meas  |
         * |       |     |     |     |     |     |    1    |    0    |        |
         * |_______|_____|_____|_____|_____|_____|_________|_________|________|
         *
         *********************************************************************/
        jumper_stat = (jumper_stat & 0x80) | (PORTD>>5 & 0x07) | PGOOD * 64 ; 
                                            // read current jumper status
                                            // [jumper_stat.7 = supply status]
        sleep_counter = PORTD>>6 & 0x03;    // read sleep count bits

        switch (sleep_counter) {
            case 0:
                sleep_count = 1;            // delay = 15s
                break;
            case 1:
                sleep_count = 20;           // delay = 5min
                break;
            case 2:
                sleep_count = 40;           // delay = 10min
                break;
            case 3:
                sleep_count = 120;          // delay = 30min
                break;
        }

        tmr1Counter = 0;
        flag = 1; // not used?!
      }

      TMR1IF=0;                             // clean the Interrupt Flag
      }
   }


int main(int argc, char** argv) {
    long int i=0;

    uint8_t num_sleep;

    //power_on_delay();

    boardInit();
    spiInit();
    uartInit();
    adcInit();
    

    rfm70setPowerdownMode(0); //?
    sleep_counter = PORTD>>6 & 0x03;
    ///printf("Sleep counter: %X\r\n", sleep_counter );

    SLEEP();

    while (adcReadVcap() < VMIN && !SKIP_MEASURE && adcReadVsupercap() < VCAP_MIN ) {
        SLEEP();
        ///printf("Vstorage too low: %d\r\n", value);
    }
    ///printf("Vstorage OK or measure skipped: %d\r\n", value);
    RFM70_Initialize();

    rfm70setPowerdownMode(0);

    /*
    value = adcReadVcap();
    while (value < VCHARGE_START && !SKIP_MEASURE) {
        SLEEP();
        value = adcReadVcap();
        printf("Vstorage too low (to charge SUPERCAP: %d\r\n", value);
    }

    value = adcReadVcap();
    */
    /*
    while(value > VCHARGE_STOP) {
        PORTBbits.RB4=0;
        T1CONbits.TMR1ON=0;
        charge_time = 0 - (charge_pulse<<2);    //50ms*4
        TMR1H = (uint8_t)(((charge_time) >> 8) & 0xFF);
        TMR1L = (uint8_t)((charge_time) & 0xFF);
        T1CONbits.TMR1ON=1;
        SLEEP();    // SLEEP 50MS

        PORTBbits.RB4=1;
        charge_time = 0x1000 - charge_time;
        T1CONbits.TMR1ON=0;
        TMR1H = (uint8_t)(((charge_time) >> 8) & 0xFF);
        TMR1L = (uint8_t)((charge_time) & 0xFF);
        T1CONbits.TMR1ON=1;
        SLEEP();    // SLEEP 15s - 50ms
        value = adcReadVcap();
        //WAIT_MS(100);
        } 
        */
    SLEEP();

    /******************************** TESTS **********************************
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
    **************************************************************************/


    timelapse_count = 0;
    while(1) {
        rfm70setPowerdownMode(1);

        //value = adcReadTemp();
        //tx_buf[0] = adcReadTemp(); posticipata
        tx_buf[1] = adcReadVcap();
        tx_buf[2] = (uint8_t)((timelapse_count) & 0xFF);
        tx_buf[3] = (uint8_t)(((timelapse_count) >> 8) & 0xFF);
        tx_buf[4] = jumper_stat;
        //energy += (values[0] - values[1])*(values[0]+values[1]);
        //temp = values[0]*values[0]-values[1]*values[1];
        tx_buf[5] = (uint8_t)(energy & 0xFF);
        tx_buf[6] = (uint8_t)((energy >> 8) & 0xFF);
        tx_buf[7] = adcReadVsupercap();
        tx_buf[8] = (int)charge_pulse_float;
        tx_buf[9] = ((((int)charge_pulse_float) >> 8) & 0xFF);

        // test if Vin is rising
        //PORTCbits.RC0=1;
        //Send_Packet(W_ACK_PAYLOAD_CMD,temp_tx_buf,17);	// transmit
        if (tx_buf[1] > VMIN || SKIP_MEASURE) {
            //rfm70setPowerdownMode(1);
            tx_buf[0] = adcReadTemp();
            //if ((jumper_stat & 0x80) == 0) {
            //    // c'è stata una transizione 0->1 (problema nella salita con intervallo impostato a 15s)
            //    SLEEP();
            //    jumper_stat |= 0x40;
            //}
            jumper_stat |= 0x80;                // power supply is OK
            tx_buf[4] = jumper_stat;
            Send_Packet(W_TX_PAYLOAD_NOACK_CMD,tx_buf,10);	// transmit
            energy = 0;
        }
        else {              // Vin < VMIN
            if ((tx_buf[7] > VCAP_MIN) && (tx_buf[1] <= value) && !PGOOD) { // vmin sale o scende?
                SCAP_MOS =0;                // MOS enabled
                tx_buf[0] = adcReadTemp();
                tx_buf[7] = adcReadVsupercap();
                jumper_stat &= 0x7F;            // power supply is not present
                tx_buf[4] = jumper_stat;
                Send_Packet(W_TX_PAYLOAD_NOACK_CMD,tx_buf,10);	// transmit 
                SCAP_MOS =1;                // MOS OFF
                charge_pulse_float = 10;        // re-init charge pulse length
                energy = 0;
            }
            else if ((tx_buf[7] > VCAP_MIN) && (tx_buf[1] <= value) && PGOOD) {
                jumper_stat &= 0x7F;            // power supply is not present
                tx_buf[4] = jumper_stat;
                Send_Packet(W_TX_PAYLOAD_NOACK_CMD,tx_buf,10);	// transmit
                charge_pulse_float = 10;        // re-init charge pulse length
                energy = 0;
            }

            else {
                printf("VStorage: %d  => not OK\r\n", tx_buf[1]);
            }

        }

        CE=0;
        rfm70setPowerdownMode(0);

        T1CONbits.TMR1ON=0;
        TMR1H = 0x10;
        TMR1L = 0x00;
        T1CONbits.TMR1ON=1;

        value = adcReadVcap();
        ///printf("Sleep counter = %d\r\n", sleep_counter);

        num_sleep = 0;
        while(num_sleep++ < sleep_count) {
            ///printf("Sleep: %d on %d\r\n", num_sleep, sleep_count);
            sleep();
            energy += (values[0] - values[1])*(values[0]+values[1])/sleep_count; //scalato
            ///printf("Sleep result: %d\r\n", energy_after_pulse);
        }
        //vsupercap = adcReadVsupercap();
        timelapse_count++;
    }
    return (EXIT_SUCCESS);
}


void sleep(void) {
    uint8_t new_value;
    value = adcReadVcap();
    charge_pulse = (int)(charge_pulse_float);
    //value = 135;
    //charge_pulse = f(value);
    if (value > VCHARGE_START && charge_pulse > 0 && !SKIP_MEASURE ) {

        //PORTBbits.RB4=0;
        DACCON1 = 0; //0; //21;                      // Preprogrammed
        DACCON0 = 0xA0;                         // DAC Enabled
        T1CONbits.TMR1ON=0;
        charge_time = 0 - (charge_pulse<<2);    //50ms*4 TMR1 è incrementato di 4096 count/s => 4.096 count/ms
        TMR1H = (uint8_t)(((charge_time) >> 8) & 0xFF);
        TMR1L = (uint8_t)((charge_time) & 0xFF);
        T1CONbits.TMR1ON=1;
        SLEEP();

        //PORTBbits.RB4=1;
        //DACCON1 = 31;
        DACCON0 = 0x00;         // DAC disabled
        SCAP_MOS=1;        // and OUTPUT = High
        new_value = adcReadVcap();
        charge_time = 0x1000 - charge_time;
        T1CONbits.TMR1ON=0;
        TMR1H = (uint8_t)(((charge_time) >> 8) & 0xFF);
        TMR1L = (uint8_t)((charge_time) & 0xFF);
        T1CONbits.TMR1ON=1;
        SLEEP();    // SLEEP 15s - 50ms

        if (new_value > value)
            new_value = value; // errore di lettura
        
        //new_value = 121;
        values[0] = value;
        values[1] = new_value;

        if (new_value > VMIN) {
            if (charge_pulse_float < 10000)
                charge_pulse_float *= 1.05;     // +5% ms
        }
        else {
            if (charge_pulse_float > 0)
                charge_pulse_float /= 1.05;     // -5% ms
        }
        /*
        if (new_value > VMIN) {
            if (charge_pulse < 15000)
                charge_pulse++;    // + 1ms
            }
        else {
            if (charge_pulse > 0)
                charge_pulse--;   // or -1ms
        }
         */
    }
    else {
        T1CONbits.TMR1ON=0;
        TMR1H = 0x10;
        TMR1L = 0x00;
        T1CONbits.TMR1ON=1;
        SLEEP();
        values[0] = 0;
        values[1] = 0;
    }
}

/*
void makePacket(scap_on) {
    unsigned int temp=0;
    if (scap_on) {
        PORTAbits.RA2=0;        // MOS enabled
        tx_buf[0] = adcReadTemp();
        PORTAbits.RA2=1;        // MOS off
    }
    else
        tx_buf[0] = adcReadTemp();
    //tx_buf[0] = adcReadTemp();
    tx_buf[1] = adcReadVcap();
    tx_buf[2] = (uint8_t)((timelapse_count) & 0xFF);
    tx_buf[3] = (uint8_t)(((timelapse_count) >> 8) & 0xFF);
    tx_buf[4] = jumper_stat;
    temp += (values[0] - values[1])*(values[0]+values[1]);
    //temp = values[0]*values[0]-values[1]*values[1];
    tx_buf[5] = (uint8_t)(temp & 0xFF);
    tx_buf[6] = (uint8_t)((temp >> 8) & 0xFF);
    tx_buf[7] = adcReadVsupercap();
    tx_buf[8] = (int)charge_pulse_float;
    tx_buf[9] = ((((int)charge_pulse_float) >> 8) & 0xFF);
}
*/

uint8_t adcReadTemp(void) {
    uint8_t value;
    FVRCONbits.FVREN=1;     // 1 = Fixed Voltage Reference is enabled
    ADCON0bits.CHS=7;
    FVRCONbits.ADFVR=1;
    while(!FVRCONbits.FVRRDY);      // wait for a stable FVR
    ADCON0bits.ADON=1;
    PORTDbits.RD0=1;            // mcp9700a power up
    WAIT_MS(1);
    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    value = ADRES;
    ADCON0bits.ADON=0;
    PORTDbits.RD0=0;            // turn off mcp9700a
    FVRCONbits.FVREN=0;     // 0 = Fixed Voltage Reference is disabled
    return value;
}

uint8_t adcReadVcap(void) {
    uint8_t value;
    FVRCONbits.FVREN=1;     // 1 = Fixed Voltage Reference is enabled
    ADCON0bits.CHS=5;
    FVRCONbits.ADFVR=2;
    ADCON0bits.ADON=1;
    PORTDbits.RD3=1;            // Select Vstorage measurement
    WAIT_MS(1);
    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    value = ADRES;
    ADCON0bits.ADON=0;
    PORTDbits.RD3=0;            // Turn off switch for Vstorage measure
    FVRCONbits.FVREN=0;     // 0 = Fixed Voltage Reference is disabled
    return value;
}

uint8_t adcReadVsupercap(void) {
    uint8_t value;
    FVRCONbits.FVREN=1;     // 1 = Fixed Voltage Reference is enabled
    ADCON0bits.CHS=6;
    FVRCONbits.ADFVR=1;
    while(!FVRCONbits.FVRRDY);      // wait for a stable FVR
    ADCON0bits.ADON=1;
    PORTDbits.RD4=1;        // Select V supercap measurement pin
    WAIT_MS(1);
    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    value = ADRES;
    ADCON0bits.ADON=0;
    PORTDbits.RD4=0;            // Turn off switch for V supercap measure
    FVRCONbits.FVREN=0;     // 0 = Fixed Voltage Reference is disabled
    return value;
}


// not used
// range 0: Vref_int = 1.024
// range 1: Vref_int = 2.048
// ch = 7 => mpc9700a read
// ch = 5 => Vstorage read
uint8_t adcRead(uint8_t ch, uint8_t range) {
    uint8_t value;
    ADCON0bits.CHS=ch;          // read from channel 'ch'
    //ADCON0bits.ADON=1;

    if (range == 0) {           // read from MCP9700A
        //FVRCONbits.FVREN=0;   //FVR disabled
        FVRCONbits.ADFVR=1;     // 1.024 as reference
        ADCON0bits.ADON=1;
        //WAIT_MS(1);
        //FVRCONbits.FVREN=1;
        PORTEbits.RE1=1;        // mcp9700a power up
        WAIT_MS(20);            // mcp9700 Turn-on Time
    }
    else
        PORTDbits.RD3=1;        // Select Vstorage measurement
        FVRCONbits.ADFVR=2;     // read from Vstorage
        ADCON0bits.ADON=1;
        WAIT_MS(1);             // serve??????

    while(!FVRCONbits.FVRRDY);  // 1 = Fixed Voltage Reference output is ready for use
    // Note: FVRRDY is always ?1? on PIC16F707 devices.

    ADCON0bits.GO_nDONE=1;
    while(ADCON0bits.GO_nDONE);
    //while(!PIR1bits.ADIF);
    value = ADRES;
    ADCON0bits.ADON=0;      //ADC is disabled and consumes no operating current
    //PORTEbits.RE1=0;      // Turn off mcp9700a
    PORTDbits.RD3=0;        // Turn off switch for Vstorage measure
    return value;
}


void uartInit(void) {
    TRISCbits.TRISC7=1;     // RC7 = UART RX pin input
    TRISCbits.TRISC6=0;     // RC6 = UART TX pin input

    TXSTA=0x02;             // High speed
    RCSTA=0x00;

    //BAUDCON=0x00;
    //BAUDCONbits.SCKP=0; // 1 = Transmit inverted data to the TX/CK pin


    TXSTAbits.BRGH=0;
    TXSTAbits.SYNC=0;

    //SPBRGH=0x00;          // Baudrate
    SPBRG=12;               // 25 for 16MHz

    RCSTAbits.SPEN=1;
    TXSTAbits.TXEN=1;

}

void spiInit(void) {
    SSPCONbits.SSPEN=1;     // 1= Enables SPI
    SSPCONbits.CKP=0;       //0 = Idle State for clock is a low level
    SSPCONbits.SSPM=2;      // SPI Master Mode clock=Fosc/64

    SSPSTATbits.SMP=0;      // input data sampled at middle of data output time
    SSPSTATbits.CKE=1;      // Trasmit occurs on transition from idle to active
                            // clock state
    WAIT_MS(50);
}


void adcInit(void) {
    //ADCON0bits.CHS=0x07;
    //ADCON1bits.ADCS=0x00;
    ADCON1bits.ADCS=1;      // convert @ fosc/8
    //ADCON1bits.ADREF=0;
    ADCON1bits.ADREF=3;     // 11 = VREF is connected to internal Fixed Voltage Reference
    //FVRCONbits.FVREN=1;     // 1 = Fixed Voltage Reference is enabled
}


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
		SPI_Write_Buf(type, pbuf, len); // Writes data to buffer
                PORTCbits.RC2=1;
                WAIT_US(10);
                PORTCbits.RC2=0;
                WAIT_MS(1);

                while(!IRQ) {
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
		printf("\n\r\n\r");
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
			//LED=1; //metterlo verde?
			WAIT_MS(100);
			//LED=0;

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

void boardInit(void) {
        OSCCONbits.IRCF=2; //3 for 16MHz and 2 for 8MHz and 0 for 2MHz

    //ANSELDbits.ANSD0=0;   // digital led
    //ANSELCbits.ANSC0=0;   // digital
    ANSELCbits.ANSC6=0;     // digital
    ANSELCbits.ANSC7=0;     // digital
    TRISCbits.TRISC6=0;     //TX

    ANSELCbits.ANSC2=0;     // CE as digital
    ANSELDbits.ANSD2=0;     // CSN as digital
    ANSELCbits.ANSC5=0;     // MOSI as digital
    ANSELDbits.ANSD1=0;     // IRQ as digital

    //adc init
    TRISEbits.TRISE2=1;     // ADC in
    ANSELEbits.ANSE2=1;     // ADC pin as analog

    // supercap meas
    TRISDbits.TRISD4=0;
    ANSELDbits.ANSD4=0;
    PORTDbits.RD4=0;        // era 1

    // dac init
    TRISAbits.TRISA2=0;     // RA2 = DAC voltage output
    ANSELAbits.ANSA2=0;
    //PORTAbits.RA2=1;
    SCAP_MOS=1;


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
    TRISDbits.TRISD0=0;     // MCP9700A power
    ANSELDbits.ANSD0=0;     // pin RD0 as digital
    TRISEbits.TRISE2=1;     // MCP9700A Vout
    ANSELEbits.ANSE2=1;     // set RE2 pin as analog

    TRISEbits.TRISE0=1;     // RE0 as input
    ANSELEbits.ANSE0=0;     // set RE0 pin as analog (Vstorage)

    PORTDbits.RD0=0;        // turn off MCP9700A

    // Vstorage monitor
    TRISEbits.TRISE0=1;     // for Vstorage measuring
    ANSELEbits.ANSE0=1;     // pin RE0 as analog
    TRISDbits.TRISD3=0;     // enable Vstorage measure
    PORTDbits.RD3=0;        // set to low

    // Skip measure
    TRISDbits.TRISD5=1;     // RD5 as input
    ANSELDbits.ANSD5=0;     // RD5 as digital
    // Number of SLEEP (Set delay)
    TRISDbits.TRISD6=1;
    TRISDbits.TRISD7=1;
    ANSELDbits.ANSD6=0;
    ANSELDbits.ANSD7=0;


    GIE=1; // gestione globale interrupt attiva
    PEIE=1; // interrupt di periferica abilitati
    TMR1IE=1; // interrupt su overflow timer1 abilitato

    TMR1H = 0x10;
    TMR1L = 0x00;

    T1CONbits.TMR1CS=2; //0 for instruction clock
    T1CONbits.T1CKPS=3; //Prescaler
    T1GCONbits.TMR1GE=1;
    T1CONbits.nT1SYNC=1;
    T1CONbits.T1OSCEN=1;    //era 0 for disabled
    T1CONbits.TMR1ON=1;

    // pgood
    TRISBbits.TRISB0=1;
    ANSELBbits.ANSB0=0;


    // not used pins
    TRISAbits.TRISA0=0; //per PIC16LF707
    TRISAbits.TRISA1=0;
    //TRISAbits.TRISA2=0;
    TRISAbits.TRISA3=0;
    TRISAbits.TRISA4=0;
    TRISAbits.TRISA5=0;
    TRISAbits.TRISA6=0;
    TRISAbits.TRISA7=0;
    ANSELAbits.ANSA0=0; //per PIC16LF707
    ANSELAbits.ANSA1=0;
    //ANSELAbits.ANSA2=0;
    ANSELAbits.ANSA3=0;
    ANSELAbits.ANSA4=0;
    ANSELAbits.ANSA5=0;
    ANSELAbits.ANSA6=0;
    ANSELAbits.ANSA7=0;
    PORTAbits.RA0=0;    //per PIC16LF707
    PORTAbits.RA1=0;
    //PORTAbits.RA2=0;
    PORTAbits.RA3=0;
    PORTAbits.RA4=0;
    PORTAbits.RA5=0;
    PORTAbits.RA6=0;
    PORTAbits.RA7=0;

    //TRISBbits.TRISB0=0;
    TRISBbits.TRISB1=0;
    TRISBbits.TRISB2=0;
    TRISBbits.TRISB3=0;
    TRISBbits.TRISB4=0;
    TRISBbits.TRISB5=0;
    //ANSELBbits.ANSB0=0;
    ANSELBbits.ANSB1=0;
    ANSELBbits.ANSB2=0;
    ANSELBbits.ANSB3=0;
    ANSELBbits.ANSB4=0;
    ANSELBbits.ANSB5=0;
    //PORTBbits.RB0=0;
    PORTBbits.RB1=0;
    PORTBbits.RB2=0;
    PORTBbits.RB3=0;
    PORTBbits.RB4=1;
    PORTBbits.RB5=0;

    //TRISDbits.TRISD4=0;
    //ANSELDbits.ANSD4=0;
    //PORTDbits.RD4=0;

    // Vsupercap measure
    TRISEbits.TRISE1=1;
    ANSELEbits.ANSE1=1;
    PORTEbits.RE1=0;
}