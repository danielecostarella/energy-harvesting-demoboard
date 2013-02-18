#include "rfm70.h"


/*
unsigned char rfm70_SPI_RW( unsigned char value ){
   unsigned char i;
   for( i =0 ; i < 8; i++ ){
      RFM70_WAIT_US( 1 );
      RFM70_MOSI( value & 0x80  );      
      value = (value << 1);    // shift next bit into MSB..
      
      RFM70_WAIT_US( 1 );
      RFM70_SCK( 1 );          
      value |= RFM70_MISO;     // capture current MISO bit
      
      RFM70_WAIT_US( 1 );
      RFM70_SCK( 0 );
      RFM70_WAIT_US( 1 );
  }
  return value;
} */

/*
unsigned char rfm70_rw(unsigned char value) {
    unsigned char i;
    for (i=0;i<8;i++) {
        RFM70_WAIT_US(1);
        ...
        value = (value << 1);
    }
}
 * */

void test(void) {
    SSPBUF = RFM70_R_REGISTER;
    while(!SSPSTATbits.BF);
    TXREG = SSPBUF;
    //return SSPBUF;
}

void rfm70Write(unsigned char reg, unsigned char value) {
    reg |= RFM70_W_REGISTER;

    PORTCbits.RC3=0;
    SSPBUF=reg;
    while(!SSPSTATbits.BF);
    //while(!PIR1bits.SSP1IF);
    SSPBUF=value;
    while(!SSPSTATbits.BF);
    PORTCbits.RC3=1;
    //while(!PIR1bits.SSP1IF);
}

unsigned char rfm70Read(unsigned char reg) {
    unsigned char value=0;

    if(reg<RFM70_W_REGISTER) {
        reg |= RFM70_R_REGISTER;
    }
    PORTCbits.RC3=0;
    //RFM70_CE = 0;
    SSPBUF=reg;  // Select register to read
    //while(!SSPSTATbits.BF);
    while(!PIR1bits.SSP1IF);;
    SSPBUF=0; // Send dummy byte to start reading
    //while(!SSPSTATbits.BF);
    while(!PIR1bits.SSP1IF);
    value = SSPBUF;
    //RFM70_CE = 1;
    PORTCbits.RC3=1;

    return value;
}

void rfm70ReadBuffer(unsigned char reg, unsigned char *pBuf, unsigned char length) {
    unsigned char i = 0;

    if(reg<RFM70_W_REGISTER) {
        reg |= RFM70_R_REGISTER;
    }

    PORTCbits.RC3=0;
    SSPBUF=reg;             // Select register to read
    //while(!SSPSTATbits.BF);
    while(!PIR1bits.SSP1IF);

    for (i=0; i<length; i++) {
        SSPBUF=0;
        TXREG = 'E';
        RFM70_WAIT_MS(4);
        //while(!SSPSTATbits.BF);
        while(!PIR1bits.SSP1IF);
        TXREG = 'F';
        *pBuf++=SSPBUF;
        //pBuf[i]=SSPBUF;
        //while(!SSPSTATbits.BF);
        TXREG = 'G';
        while(!PIR1bits.SSP1IF);
        
    }
    PORTCbits.RC3=1;
}

void rfm70Bank(unsigned char bank) {
    unsigned char st=0;
    
    st = rfm70Read(0x07);
    st = st & 0x80;
    //TXREG = 0x30+st;
    if ((st==0x80) && (bank == 0)){
        PORTCbits.RC3=0;
        SSPBUF = 0x53;
        TXREG = 'C';
        while(!PIR1bits.SSP1IF);
        PORTCbits.RC3=1;
        //while(!SSPSTATbits.BF);
        TXREG = 'L';
    }
    if ((st==0x00) && (bank == 1)){
        PORTCbits.RC3=0;
        SSPBUF = 0x53;
        //while(!SSPSTATbits.BF);
        while(!PIR1bits.SSP1IF);
        PORTCbits.RC3=1;
    }
}

unsigned char rfm70IsPresent(void) {
    TXREG = 'U';
    RFM70_WAIT_MS(4);
    unsigned char st1, st2;
    st1 = rfm70Read(RFM70_STATUS);
    TXREG = st1;
    RFM70_WAIT_MS(4);
    rfm70Write(RFM70_ACTIVATE, 0x53);
    st2 = rfm70Read(RFM70_STATUS);
    rfm70Write(RFM70_ACTIVATE, 0x53);
    return (st1^st2)==0x80;

}


unsigned char rfm70_SPI_RW(unsigned char byte) {
    unsigned char i;
    unsigned char bytes[8];
    //byte=255;

    for (i=0;i<8;i++) {
        PORTCbits.RC7=(byte >> 7);      // MOSI PIN
        RFM70_WAIT_US(1);
        byte = (byte << 1);
        bytes[i]=byte;
        RFM70_WAIT_US(10);
        PORTBbits.RB6=1;            // SCK to 1
        RFM70_WAIT_US(1);
        byte |= PORTBbits.RB4;      // MISO PIN
        //byte |= 1;
        RFM70_WAIT_US(1);
        PORTBbits.RB6=0;            // SCK to 0
        RFM70_WAIT_US(1);
    }
    PORTCbits.RC7=0;
    RFM70_WAIT_US(1);
    TXREG='d';
    RFM70_WAIT_MS(50);
    for (i=0;i<8;i++) {
        TXREG=bytes[i];
        RFM70_WAIT_MS(50);

    }
    RFM70_WAIT_MS(50);
    TXREG='e';
    RFM70_WAIT_MS(50);

    return(byte);                   // return 'read' byte

}

unsigned char SPI_read(unsigned char reg) {
    unsigned char value;

    PORTCbits.RC3=0;            // clear CSN
    rfm70_SPI_RW(reg);
    value = rfm70_SPI_RW(0);
    PORTCbits.RC3=1;

    return value;
}
