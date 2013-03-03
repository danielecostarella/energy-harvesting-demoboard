/*
 * File:   rfm70.h
 * Author: blackbliss
 *
 * Created on 22 febbraio 2013, 20.55
 */

#ifndef RFM70_H
#define	RFM70_H

#include <stdint.h>
#include <xc.h>

#define MAX_PACKET_LEN  32// max value is 32

//****************************************************************************//
// RFM70(nRF24L01) commands
//****************************************************************************//
#define READ_REG        	0x00  // Define read command to register
#define WRITE_REG       	0x20  // Define write command to register
#define RD_RX_PLOAD     	0x61  // Define RX payload register address
#define WR_TX_PLOAD     	0xA0  // Define TX payload register address
#define FLUSH_TX        	0xE1  // Define flush TX register command
#define FLUSH_RX        	0xE2  // Define flush RX register command
#define REUSE_TX_PL     	0xE3  // Define reuse TX payload register command
#define W_TX_PAYLOAD_NOACK_CMD  0xB0
#define W_ACK_PAYLOAD_CMD	0xA8
#define ACTIVATE_CMD		0x50
#define R_RX_PL_WID_CMD		0x60
#define NOP_NOP            	0xFF  // Define No Operation, might be used to read status register

//****************************************************************************//
// RFM70(nRF24L01) registers (addresses)
//****************************************************************************//
#define CONFIG          0x00  // 'Config' register address
#define EN_AA           0x01  // 'Enable Auto Acknowledgment' register address
#define EN_RXADDR       0x02  // 'Enabled RX addresses' register address
#define SETUP_AW        0x03  // 'Setup address width' register address
#define SETUP_RETR      0x04  // 'Setup Auto. Retrans' register address
#define RF_CH           0x05  // 'RF channel' register address
#define RF_SETUP        0x06  // 'RF setup' register address
#define STATUS          0x07  // 'Status' register address
#define OBSERVE_TX      0x08  // 'Observe TX' register address
#define CD              0x09  // 'Carrier Detect' register address
#define RX_ADDR_P0      0x0A  // 'RX address pipe0' register address
#define RX_ADDR_P1      0x0B  // 'RX address pipe1' register address
#define RX_ADDR_P2      0x0C  // 'RX address pipe2' register address
#define RX_ADDR_P3      0x0D  // 'RX address pipe3' register address
#define RX_ADDR_P4      0x0E  // 'RX address pipe4' register address
#define RX_ADDR_P5      0x0F  // 'RX address pipe5' register address
#define TX_ADDR         0x10  // 'TX address' register address
#define RX_PW_P0        0x11  // 'RX payload width, pipe0' register address
#define RX_PW_P1        0x12  // 'RX payload width, pipe1' register address
#define RX_PW_P2        0x13  // 'RX payload width, pipe2' register address
#define RX_PW_P3        0x14  // 'RX payload width, pipe3' register address
#define RX_PW_P4        0x15  // 'RX payload width, pipe4' register address
#define RX_PW_P5        0x16  // 'RX payload width, pipe5' register address
#define FIFO_STATUS     0x17  // 'FIFO Status Register' register address
#define DYNPD           0x1C
#define FEATURE         0x1D

//interrupt status
#define STATUS_RX_DR 	0x40
#define STATUS_TX_DS 	0x20
#define STATUS_MAX_RT 	0x10

#define STATUS_TX_FULL 	0x01

//FIFO_STATUS
#define FIFO_STATUS_TX_REUSE 	0x40
#define FIFO_STATUS_TX_FULL 	0x20
#define FIFO_STATUS_TX_EMPTY 	0x10

#define FIFO_STATUS_RX_FULL 	0x02
#define FIFO_STATUS_RX_EMPTY 	0x01


uint8_t SPI_Read_Reg(uint8_t reg);
void SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);

void SPI_Write_Reg(uint8_t reg, uint8_t value);
//void SPI_Write_Reg_Bank0(const uint8_t reg, const uint8_t value);
//void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes);
void SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t length);
void SPI_Write_Buf_bank(uint8_t reg, uint8_t *pBuf, uint8_t length);

void SwitchToTxMode(void);
void SwitchToRxMode(void);

void SPI_Bank1_Read_Reg(uint8_t reg, uint8_t *pBuf);
void SPI_Bank1_Write_Reg(uint8_t reg, uint8_t *pBuf);
void SwitchCFG(char _cfg);

void RFM70_Initialize(void);

uint8_t rfm70_data_sent(void); //deprecated
uint8_t rfm70Interrupt(void);
uint8_t rfm70TxDataSentErrorInterrupt(void);
uint8_t rfm70TxDataSentInterrupt(void);
void rfm70setPowerdownMode(uint8_t value);

#endif	/* RFM70_H */