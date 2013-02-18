/* 
 * @File:   rfm70.h
 * @Author: blackbliss
 *
 * Created on 15 febbraio 2013, 17.53
 */

#ifndef RFM70_H
#define	RFM70_H

#include <xc.h>

/** RFM70 SPI commands */

/// Read  command and status registers
#define RFM70_R_REGISTER            0x00

/// Write command and status registers
#define RFM70_W_REGISTER            0x20

/// Read a received payload
#define RFM70_R_RX_PAYLOAD          0x61

/// Write a payload to be sent
#define RFM70_W_TX_PAYLOAD          0xA0

/// Empty the transmit queue
#define RFM70_FLUSH_TX              0xE1

/// Empty the receive queue
#define RFM70_FLUSH_RX              0xE2

/// Packets are repeatedly retransmitted as long as CE is high reusing the last payload
#define RFM70_REUSE_TX_PL           0xE3

/// Toggle register bank or toggle extended functions
#define RFM70_ACTIVATE              0x50

/// Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO
#define RFM70_R_RX_PL_WID           0x60

/// Write Payload to be transmitted together with ACK packet
#define RFM70_W_ACK_PAYLOAD         0xA8

/// Disables AUTOACK on this specific packet
#define RFM70_W_TX_PAYLOAD_NOACK    0xB0

/// No Operation
#define RFM70_NOP                   0xFF


#define _XTAL_FREQ 16000000L
#define RFM70_WAIT_US( x ) _delay( x * ( _XTAL_FREQ / 4000000))
#define RFM70_WAIT_MS( x ) _delay( x * ( _XTAL_FREQ / 4000))

//#define RFM70_CE PORTCbits.RC3


// Register MAP
#define RFM70_STATUS    0x07
void rfm70Write(unsigned char reg, unsigned char value);
unsigned char rfm70Read(unsigned char reg);
void rfm70ReadBuffer(unsigned char reg, unsigned char *pBuf, unsigned char length);
void rfm70Bank(unsigned char bank);
unsigned char rfm70IsPresent(void);
void test(void);
unsigned char rfm70_SPI_RW(unsigned char);
unsigned char SPI_read(unsigned char reg);

#endif	/* RFM70_H */

