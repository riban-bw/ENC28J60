// Microchip ENC28J60 Ethernet Interface Driver
// Derived from work by Guido Socher
// Adapted by Brian Walton
// Copyright: LGPL V3
//
// Based on the enc28j60.c file from the AVRlib library by Pascal Stang.
// For AVRlib See http://www.procyonengineering.com/
// Used with explicit permission of Pascal Stang.
//
// 2010-05-20 <jc@wippler.nl>
// 2014-05-28 <brian@riban.co.uk>

#if ARDUINO >= 100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif
#include "enc28j60.h"

// ENC28J60 Control Registers (encoded with register address (bits 0-4), bank number (bits 5-6) and MAC/MII indicator (bit 7)
static const byte REGISTER_MASK     = 0x1F; //bits 0-4
static const byte BANK_MASK         = 0x60; //bits 5-6
static const byte MACMII_MASK       = 0x80; //bit 7 - used by SPIReadRegister() because MAC and MII registers are read on second byte!!!???
static const byte BANK0_MASK        = 0x00; //!< Control register bank 0 mask
static const byte BANK1_MASK        = 0x20; //!< Control register bank 1 mask
static const byte BANK2_MASK        = 0x40; //!< Control register bank 2 mask
static const byte BANK3_MASK        = 0x60; //!< Control register bank 3 mask

// Common registers to all banks
static const uint16_t EIE            = 0x1B; //!< Ethernet interrupt enable register
static const uint16_t EIR            = 0x1C; //!< Ethernet interrupt request (flag) register
static const uint16_t ESTAT          = 0x1D; //!< Ethernet status register
static const uint16_t ECON2          = 0x1E; //!< Ethernet control register 2
static const uint16_t ECON1          = 0x1F; //!< Ethernet control register 1
// Bank 0 registers
static const uint16_t ERDPT         = (0x00|BANK0_MASK); //!< Read pointer
static const uint16_t EWRPT         = (0x02|BANK0_MASK); //!< Write pointer
static const uint16_t ETXST         = (0x04|BANK0_MASK); //!< Tx start pointer
static const uint16_t ETXND         = (0x06|BANK0_MASK); //!< Tx end pointer
static const uint16_t ERXST         = (0x08|BANK0_MASK); //!< Rx start pointer
static const uint16_t ERXND         = (0x0A|BANK0_MASK); //!< Rx end pointer
static const uint16_t ERXRDPT       = (0x0C|BANK0_MASK); //!< Rx read pointer
static const uint16_t ERXWRPT       = (0x0E|BANK0_MASK); //!< Pointer to next NIC RAM address to write recieved packet data
static const uint16_t EDMAST        = (0x10|BANK0_MASK); //!< DMA start
static const uint16_t EDMAND        = (0x12|BANK0_MASK); //!< DMA end
static const uint16_t EDMADST       = (0x14|BANK0_MASK); //!< DMA destination
static const uint16_t EDMACS        = (0x16|BANK0_MASK); //!< DMA checksum
// Bank 1 registers
static const uint16_t EHT0           = (0x00|BANK1_MASK); //!< Hash table byte 0
static const uint16_t EHT1           = (0x01|BANK1_MASK); //!< Hash table byte 1
static const uint16_t EHT2           = (0x02|BANK1_MASK); //!< Hash table byte 2
static const uint16_t EHT3           = (0x03|BANK1_MASK); //!< Hash table byte 3
static const uint16_t EHT4           = (0x04|BANK1_MASK); //!< Hash table byte 4
static const uint16_t EHT5           = (0x05|BANK1_MASK); //!< Hash table byte 5
static const uint16_t EHT6           = (0x06|BANK1_MASK); //!< Hash table byte 6
static const uint16_t EHT7           = (0x07|BANK1_MASK); //!< Hash table byte 7
static const uint16_t EPMM0          = (0x08|BANK1_MASK); //!< Pattern match mask byte 0
static const uint16_t EPMM1          = (0x09|BANK1_MASK); //!< Pattern match mask byte 1
static const uint16_t EPMM2          = (0x0A|BANK1_MASK); //!< Pattern match mask byte 2
static const uint16_t EPMM3          = (0x0B|BANK1_MASK); //!< Pattern match mask byte 3
static const uint16_t EPMM4          = (0x0C|BANK1_MASK); //!< Pattern match mask byte 4
static const uint16_t EPMM5          = (0x0D|BANK1_MASK); //!< Pattern match mask byte 6
static const uint16_t EPMM6          = (0x0E|BANK1_MASK); //!< Pattern match mask byte 6
static const uint16_t EPMM7          = (0x0F|BANK1_MASK); //!< Pattern match mask byte 7
static const uint16_t EPMCS          = (0x10|BANK1_MASK); //!< Pattern match mask byte 8
static const uint16_t EPMO           = (0x14|BANK1_MASK); //!< Pattern match offset
static const uint16_t EWOLIE         = (0x16|BANK1_MASK); //!< Ethernet wake-up on LAN interrupt enable register
static const uint16_t EWOLIR         = (0x17|BANK1_MASK); //!< Wake-up on LAN interrupt request (flag) register
static const uint16_t ERXFCON        = (0x18|BANK1_MASK); //!< Receive filter control register
static const uint16_t EPKTCNT        = (0x19|BANK1_MASK); //!< Ethernet packet count - quantity of unprocessed recieved packets
// Bank 2 registers
static const uint16_t MACON1         = (0x00|BANK2_MASK|MACMII_MASK); //!< MAC control register 1
static const uint16_t MACON2         = (0x01|BANK2_MASK|MACMII_MASK); //!< MAC control register 2
static const uint16_t MACON3         = (0x02|BANK2_MASK|MACMII_MASK); //!< MAC control register 3
static const uint16_t MACON4         = (0x03|BANK2_MASK|MACMII_MASK); //!< MAC control register 4
static const uint16_t MABBIPG        = (0x04|BANK2_MASK|MACMII_MASK); //!< MAC back-to-back inter-packet gap register
static const uint16_t MAIPG          = (0x06|BANK2_MASK|MACMII_MASK); //!< Non-back-to-back inter-packet gap
static const uint16_t MACLCON1       = (0x08|BANK2_MASK|MACMII_MASK); //!< Retransmission maximum
static const uint16_t MACLCON2       = (0x09|BANK2_MASK|MACMII_MASK); //!< Collision window
static const uint16_t MAMXFL         = (0x0A|BANK2_MASK|MACMII_MASK); //!< Maximum frame length
static const uint16_t MAPHSUP        = (0x0D|BANK2_MASK|MACMII_MASK); //!< MAC-PHY support register
static const uint16_t MICON          = (0x11|BANK2_MASK|MACMII_MASK); //!< MII control register
static const uint16_t MICMD          = (0x12|BANK2_MASK|MACMII_MASK); //!< MII command register
static const uint16_t MIREGADR       = (0x14|BANK2_MASK|MACMII_MASK); //!< MII register address
static const uint16_t MIWR           = (0x16|BANK2_MASK|MACMII_MASK); //!< MII write data
static const uint16_t MIRD           = (0x18|BANK2_MASK|MACMII_MASK); //!< MII read data
// Bank 3 registers
static const uint16_t MAADR1         = (0x00|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 1
static const uint16_t MAADR0         = (0x01|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 0
static const uint16_t MAADR3         = (0x02|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 3
static const uint16_t MAADR2         = (0x03|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 2
static const uint16_t MAADR5         = (0x04|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 5
static const uint16_t MAADR4         = (0x05|BANK3_MASK|MACMII_MASK); //!< Magic Packet(TM) destination address byte 4
static const uint16_t EBSTSD         = (0x06|BANK3_MASK); //!< Built in self test fill seed
static const uint16_t EBSTCON        = (0x07|BANK3_MASK); //!< Built in self test control register
static const uint16_t EBSTCS         = (0x08|BANK3_MASK); //!< Built in self test checksum
static const uint16_t MISTAT         = (0x0A|BANK3_MASK|MACMII_MASK); //!< MII status register
static const uint16_t EREVID         = (0x12|BANK3_MASK); //!< Ethernet revision
static const uint16_t ECOCON         = (0x15|BANK3_MASK); //!< Reset type (bits 0-2 = 100 if power-on reset)
static const uint16_t EFLOCON        = (0x17|BANK3_MASK); //!< Ethernet flow control register
static const uint16_t EPAUS          = (0x18|BANK3_MASK); //!< Pause timer value

// ENC28J60 ERXFCON Register Bit Definitions
static const uint16_t ERXFCON_UCEN   = 0x80; //!< Unicast filter enable bit
static const uint16_t ERXFCON_ANDOR  = 0x40; //!< AND / OR filter select bit
static const uint16_t ERXFCON_CRCEN  = 0x20; //!< Post-Filter CRC check enable bit
static const uint16_t ERXFCON_PMEN   = 0x10; //!< Pattern match enable bit
static const uint16_t ERXFCON_MPEN   = 0x08; //!< Magic Packet filter enable bit
static const uint16_t ERXFCON_HTEN   = 0x04; //!< Hash table filter enable bit
static const uint16_t ERXFCON_MCEN   = 0x02; //!< Multicast match enable bit
static const uint16_t ERXFCON_BCEN   = 0x01; //!< Broadcast match enable bit
// ENC28J60 EIE Register Bit Definitions
static const uint16_t EIE_INTIE      = 0x80;
static const uint16_t EIE_PKTIE      = 0x40;
static const uint16_t EIE_DMAIE      = 0x20;
static const uint16_t EIE_LINKIE     = 0x10;
static const uint16_t EIE_TXIE       = 0x08;
static const uint16_t EIE_WOLIE      = 0x04;
static const uint16_t EIE_TXERIE     = 0x02;
static const uint16_t EIE_RXERIE     = 0x01;
// ENC28J60 EIR Register Bit Definitions
static const uint16_t EIR_PKTIF      = 0x40;
static const uint16_t EIR_DMAIF      = 0x20;
static const uint16_t EIR_LINKIF     = 0x10;
static const uint16_t EIR_TXIF       = 0x08;
static const uint16_t EIR_WOLIF      = 0x04;
static const uint16_t EIR_TXERIF     = 0x02;
static const uint16_t EIR_RXERIF     = 0x01;
// ENC28J60 ESTAT Register Bit Definitions
static const uint16_t ESTAT_INT      = 0x80;
static const uint16_t ESTAT_LATECOL  = 0x10;
static const uint16_t ESTAT_RXBUSY   = 0x04;
static const uint16_t ESTAT_TXABRT   = 0x02;
static const uint16_t ESTAT_CLKRDY   = 0x01;
// ENC28J60 ECON2 Register Bit Definitions
static const uint16_t ECON2_AUTOINC  = 0x80; //!< Automatic buffer pointer increment enable bit
static const uint16_t ECON2_PKTDEC   = 0x40; //!< Packet Decrement bit
static const uint16_t ECON2_PWRSV    = 0x20; //!< Power save enable bit
static const uint16_t ECON2_VRPS     = 0x08; //!< Voltage regulator power save enable bit
// ENC28J60 ECON1 Register Bit Definitions
static const byte ECON1_TXRST    = 0x80; //!< Transmit logic reset bit
static const byte ECON1_RXRST    = 0x40; //!< Recieve logic reset bit
static const byte ECON1_DMAST    = 0x20; //!< DMA start and busy status bit
static const byte ECON1_CSUMEN   = 0x10; //!< DMA checksum enable bit (erata B7.17 - do not use during packet reception)
static const byte ECON1_TXRTS    = 0x08; //!< Transmit request to send bit
static const byte ECON1_RXEN     = 0x04; //!< Recieve enable bit
static const byte ECON1_BSEL1    = 0x02; //!< Bank select bit 1
static const byte ECON1_BSEL0    = 0x01; //!< Bank select bit 0
// ENC28J60 MACON1 Register Bit Definitions
static const uint16_t MACON1_LOOPBK  = 0x10;
static const uint16_t MACON1_TXPAUS  = 0x08;
static const uint16_t MACON1_RXPAUS  = 0x04;
static const uint16_t MACON1_PASSALL = 0x02;
static const uint16_t MACON1_MARXEN  = 0x01;
// ENC28J60 MACON2 Register Bit Definitions
static const uint16_t MACON2_MARST   = 0x80;
static const uint16_t MACON2_RNDRST  = 0x40;
static const uint16_t MACON2_MARXRST = 0x08;
static const uint16_t MACON2_RFUNRST = 0x04;
static const uint16_t MACON2_MATXRST = 0x02;
static const uint16_t MACON2_TFUNRST = 0x01;
// ENC28J60 MACON3 Register Bit Definitions
static const uint16_t MACON3_PADCFG2 = 0x80;
static const uint16_t MACON3_PADCFG1 = 0x40;
static const uint16_t MACON3_PADCFG0 = 0x20;
static const uint16_t MACON3_TXCRCEN = 0x10;
static const uint16_t MACON3_PHDRLEN = 0x08;
static const uint16_t MACON3_HFRMLEN = 0x04;
static const uint16_t MACON3_FRMLNEN = 0x02;
static const uint16_t MACON3_FULDPX  = 0x01;
// ENC28J60 MICMD Register Bit Definitions
static const uint16_t MICMD_MIISCAN  = 0x02;
static const uint16_t MICMD_MIIRD    = 0x01;
// ENC28J60 MISTAT Register Bit Definitions
static const uint16_t MISTAT_NVALID  = 0x04;
static const uint16_t MISTAT_SCAN    = 0x02;
static const uint16_t MISTAT_BUSY    = 0x01;

// ENC28J60 EBSTCON Register Bit Definitions
static const uint16_t EBSTCON_PSV2   = 0x80;
static const uint16_t EBSTCON_PSV1   = 0x40;
static const uint16_t EBSTCON_PSV0   = 0x20;
static const uint16_t EBSTCON_PSEL   = 0x10;
static const uint16_t EBSTCON_TMSEL1 = 0x08;
static const uint16_t EBSTCON_TMSEL0 = 0x04;
static const uint16_t EBSTCON_TME    = 0x02;
static const uint16_t EBSTCON_BISTST = 0x01;

// PHY registers
static const byte PHCON1         = 0x00; //!< PHY control register
static const byte PHSTAT1        = 0x01; //!< Physical layer status register 1
static const byte PHHID1         = 0x02; //!< PHY identifier (bits 3-18) = 0x0083
static const byte PHHID2         = 0x03; //!< PHY identifier(bits 19-24) = 0x05, PHY P/N = 0x00, PHY revision = 0x00
static const byte PHCON2         = 0x10; //!< PHY control register 2
static const byte PHSTAT2        = 0x11; //!< Physical layer status register 2
static const byte PHIE           = 0x12; //!< PHY interrupt enable register
static const byte PHIR           = 0x13; //!< PHY interrupt request (flag) register
static const byte PHLCON         = 0x14; //!< PHY LED control register

// ENC28J60 PHY PHCON1 Register Bit Definitions
static const uint16_t PHCON1_PRST    = 0x8000;
static const uint16_t PHCON1_PLOOPBK = 0x4000;
static const uint16_t PHCON1_PPWRSV  = 0x0800;
static const uint16_t PHCON1_PDPXMD  = 0x0100;
// ENC28J60 PHY PHSTAT1 Register Bit Definitions
static const uint16_t PHSTAT1_PFDPX  = 0x1000;
static const uint16_t PHSTAT1_PHDPX  = 0x0800;
static const uint16_t PHSTAT1_LLSTAT = 0x0004;
static const uint16_t PHSTAT1_JBSTAT = 0x0002;
// ENC28J60 PHT PHSTAT2 Register Bit Definitions
static const uint16_t PHSTAT2_TXSTAT    = 0x2000; //!< PHY transmit status bit
static const uint16_t PHSTAT2_RXSTAT    = 0x1000; //!< PHY revieve status bit
static const uint16_t PHSTAT2_COLSTAT   = 0x0800; //!< PHY collision status bit
static const uint16_t PHSTAT2_LSTAT     = 0x0400; //!< PHY link status status bit
static const uint16_t PHSTAT2_DPXSTAT   = 0x0200; //!< PHY duplex status bit
static const uint16_t PHSTAT2_PLRITY    = 0x0020; //!< PHY polarity status bit
// ENC28J60 PHY PHCON2 Register Bit Definitions
static const uint16_t PHCON2_FRCLINK = 0x4000;
static const uint16_t PHCON2_TXDIS   = 0x2000;
static const uint16_t PHCON2_JABBER  = 0x0400;
static const uint16_t PHCON2_HDLDIS  = 0x0100;

// ENC28J60 Packet Control Byte Bit Definitions
static const byte PKTCTRL_PHUGEEN   = 0x08;
static const byte PKTCTRL_PPADEN    = 0x04;
static const byte PKTCTRL_PCRCEN    = 0x02;
static const byte PKTCTRL_POVERRIDE = 0x01;

// ENC28J60 SPI instruction set
static const byte SPI_OPCODE_MASK   = 0xe0; //!< Bitwise mask of SPI opcode (3 most significant bits of first instruction byte)
static const byte SPI_ARGUMENT_MASK = 0x1F; //!< Bitwise mask of SPI argument (5 least significant bits of first instruction byte)
static const byte ENC28J60_SPI_RCR  = 0x00; //!< Read control register
static const byte ENC28J60_SPI_RBM  = 0x3A; //!< Read buffer memory
static const byte ENC28J60_SPI_WCR  = 0x40; //!< Write control register
static const byte ENC28J60_SPI_WBM  = 0x7A; //!< Write buffer memory
static const byte ENC28J60_SPI_BFS  = 0x80; //!< Bit field set
static const byte ENC28J60_SPI_BFC  = 0xA0; //!< Bit field clear
static const byte ENC28J60_SPI_SC   = 0xFF; //!< System command (soft reset)

//Used to configure NIC 8K RAM
static const uint16_t RX_BUFFER_START   = 0x0000; //!< Start of recieve circular buffer (erata B7.5)
static const uint16_t TX_BUFFER_START   = 0x1A0C; //!< Start of transmit buffer which gives space for 1 packet
static const uint16_t TX_BUFFER_END     = 0x1FFF; //!< End of transmit buffer which is end of available memory

static const uint16_t MAX_FRAMELEN      = 1518; //!< Maximum frame length

void ENC28J60::SPIInit()
{
    Serial.println("Initialise SPI");
    pinMode(SS, OUTPUT);
    digitalWrite(SS, HIGH);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);

    digitalWrite(MOSI, HIGH);
    digitalWrite(MOSI, LOW);
    digitalWrite(SCK, LOW);

    SPCR = bit(SPE) | bit(MSTR); // Enable SPI as master, disable interupt, shift msb first, clock idle when low, sample on rising edge, fastest speed fosc/4
    bitSet(SPSR, SPI2X); //Enable SPI double speed (fosc/2)
}

byte ENC28J60::SPITransfer(byte data)
{
    //From ATmega release notes sample code
    SPDR = data; //populate SPI data register to start transfer
    while(!(SPSR & (1 << SPIF))) //wait for transfer to complete
        ;
    return SPDR;
}

void ENC28J60::SPISetBank(byte nAddress)
{
    if(((nAddress & BANK_MASK) != m_nBank) && ((nAddress & SPI_ARGUMENT_MASK) < EIE)) //only switch bank if not already selected and not a common register
    {
        EnableChip();
        SPIClearBits(ECON1, ECON1_BSEL1 | ECON1_BSEL0); //Clear bank selection (select bank 0)
        m_nBank = nAddress & BANK_MASK; //Decode bank from address (bits 5-6)
        SPISetBits(ECON1, m_nBank >> 5); //Set bank (BSEL0/1 are bits 0/1)
        DisableChip();
//        Serial.print("Select bank ");
//        Serial.print("[");
//        Serial.print(m_nBank >> 5);
//        Serial.print("]");
    }
}

byte ENC28J60::SPIReadRegister(byte nRegister)
{
    SPISetBank(nRegister);
    EnableChip();
    SPITransfer(ENC28J60_SPI_RCR | (nRegister & SPI_ARGUMENT_MASK));
    byte nResult = SPITransfer();
    if(nRegister & MACMII_MASK)
        nResult = SPITransfer(); //If MAC or MII register, use second byte (after dummy first byte)
    DisableChip();
//    Serial.print("Read value 0x");
//    Serial.print(nResult, HEX);
//    Serial.print(" from register 0x");
//    Serial.println(nRegister & SPI_ARGUMENT_MASK, HEX);
    return nResult;
}

void ENC28J60::SPIReadBuf(byte* pData, uint16_t nLen)
{
    EnableChip();
    SPITransfer(ENC28J60_SPI_RBM); //initiate read buffer
    for(uint16_t i = 0; i < nLen; ++i)
        pData[i] = SPITransfer(); //get each byte from SPI interface
    DisableChip();
}

void ENC28J60::SPIWriteReg(byte nRegister, byte nData)
{
    SPISetBank(nRegister);
    EnableChip();
    SPITransfer(ENC28J60_SPI_WCR | (nRegister & SPI_ARGUMENT_MASK));
    SPITransfer(nData);
    DisableChip();
//    Serial.print("Write value 0x");
//    Serial.print(nData, HEX);
//    Serial.print(" to register 0x");
//    Serial.println(nRegister & SPI_ARGUMENT_MASK, HEX);
}

void ENC28J60::SPIWriteBuf(byte* pData, uint16_t nLen)
{
    EnableChip();
    SPITransfer(ENC28J60_SPI_WBM); //initiate write buffer
    for(uint16_t i = 0; i < nLen; ++i)
        SPITransfer(pData[i]); //send each byte to SPI interface
    DisableChip();
}

void ENC28J60::SPISetBits(byte nRegister, byte nBits)
{
    SPISetBank(nRegister);
    EnableChip();
    SPITransfer(ENC28J60_SPI_BFS | (nRegister & SPI_ARGUMENT_MASK));
    SPITransfer(nBits);
    DisableChip();
}

void ENC28J60::SPIClearBits(byte nRegister, byte nBits)
{
    SPISetBank(nRegister);
    EnableChip();
    SPITransfer(ENC28J60_SPI_BFC | (nRegister & SPI_ARGUMENT_MASK));
    SPITransfer(nBits);
    DisableChip();
}

void ENC28J60::SPIReset()
{
    PowerUp(); //erata B7.19 - reset does not work when in power save mode
    EnableChip();
    SPITransfer(ENC28J60_SPI_SC);
    DisableChip();
    delay(1); //erata B7.2
}

void ENC28J60::EnableChip()
{
    cli();
    digitalWrite(m_nSelectPin, LOW);
}

void ENC28J60::DisableChip()
{
    digitalWrite(m_nSelectPin, HIGH);
    sei();
}

uint16_t ENC28J60::ReadRegWord(byte nAddress)
{
    return SPIReadRegister(nAddress) + (SPIReadRegister(nAddress+1) << 8);
}

void ENC28J60::WriteRegWord(byte nAddress, uint16_t nData)
{
    SPIWriteReg(nAddress, (byte)(nData & 0x00FF));
    SPIWriteReg(nAddress + 1, (byte)(nData >> 8));
}

uint16_t ENC28J60::ReadPhyWord(byte nAddress)
{
    //1. Write the address of the PHY register to read from into the MIREGADR register.
    SPIWriteReg(MIREGADR, nAddress);
    //2. Set the MICMD.MIIRD bit. The read operation begins and the MISTAT.BUSY bit is set.
    SPISetBits(MICMD, MICMD_MIIRD);
    //3. Wait 10.24μs. Poll the MISTAT.BUSY bit to be certain that the operation is complete.
    while(SPIReadRegister(MISTAT) & MISTAT_BUSY)
        ;
    //4. Clear the MICMD.MIIRD bit.
    SPIClearBits(MICMD, MICMD_MIIRD);
    //5. Read the desired data from the MIRDL and MIRDH registers.
    return ReadRegWord(MIRD);
}

void ENC28J60::WritePhyWord(byte nAddress, uint16_t nData)
{
    //1. Write the address of the PHY register to into the MIREGADR register.
    SPIWriteReg(MIREGADR, nAddress);
    //2. Write the lower 8 bits of data to write into the MIWRL register.
    //3. Write the upper 8 bits of data to write into the MIWRH register. Writing to this register automatically begins the MII transaction, so it must be written to after MIWRL.
    WriteRegWord(MIWR, nData);
    //   The MISTAT.BUSY bit becomes set. The PHY register will be written after the MII operation completes, which takes 10.24μs.
    //   When the write operation has completed, the BUSY bit will clear itself. The host controller should not start any MIISCAN or MIIRD operations while busy.
    while(SPIReadRegister(MISTAT) & MISTAT_BUSY)
        ;
}

byte ENC28J60::Initialize(const byte* pMac, byte nChipSelectPin)
{
    m_nBank = 0xFF; //Set bank to invalid value to ensure initial bank selection
    m_bBroadcastEnabled = true;
    if(bitRead(SPCR, SPE) == 0)
        SPIInit();
    m_nSelectPin = nChipSelectPin;
    pinMode(m_nSelectPin, OUTPUT);
    DisableChip();

    SPIReset();

    m_rxHeader.nNextPacket = RX_BUFFER_START;
    m_rxHeader.nSize = 0;
    m_rxHeader.nStatus = 0;
    m_nRxPacketPtr = RX_BUFFER_START;
    WriteRegWord(ERXST, RX_BUFFER_START);
    WriteRegWord(ERXRDPT, RX_BUFFER_START);
    WriteRegWord(ERXND, TX_BUFFER_START - 1);
    WriteRegWord(ETXST, TX_BUFFER_START);
    WriteRegWord(ETXND, TX_BUFFER_END);
    WriteRegWord(EPMM0, 0x303f);
    WriteRegWord(EPMCS, 0xf7f9);
    SPIWriteReg(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
    SPIWriteReg(MACON2, 0x00);
    SPISetBits(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
    WriteRegWord(MAIPG, 0x0C12);
    SPIWriteReg(MABBIPG, 0x12);
    WriteRegWord(MAMXFL, MAX_FRAMELEN);
    SPIWriteReg(MAADR5, pMac[0]);
    SPIWriteReg(MAADR4, pMac[1]);
    SPIWriteReg(MAADR3, pMac[2]);
    SPIWriteReg(MAADR2, pMac[3]);
    SPIWriteReg(MAADR1, pMac[4]);
    SPIWriteReg(MAADR0, pMac[5]);
    WritePhyWord(PHCON2, PHCON2_HDLDIS);
    SPISetBits(EIE, EIE_INTIE | EIE_PKTIE);
    SPISetBits(ECON1, ECON1_RXEN);
    EnableBroadcast(); //!@todo Docs say this is enabled by default - check if this is true

    byte nRevision = SPIReadRegister(EREVID);
    // microchip forgot to step the number on the silcon when they
    // released the revision B7. 6 is now rev B7. We still have
    // to see what they do when they release B8. At the moment
    // there is no B8 out yet
    if(nRevision > 5) ++nRevision;
    return nRevision;
}

bool ENC28J60::IsLinkUp()
{
    return(ReadPhyWord(PHSTAT2) & PHSTAT2_LSTAT);
}

void ENC28J60::PacketSend(byte* pBuffer, uint16_t nLen)
{
    while(SPIReadRegister(ECON1) & ECON1_TXRTS)
        ; //Wait for transmission to complete
    if(SPIReadRegister(EIR) & EIR_TXERIF) //Tx aborted
    {
        //Reset Tx, i.e. abort transmission
        SPISetBits(ECON1, ECON1_TXRST);
        SPIClearBits(ECON1, ECON1_TXRST);
    }
    WriteRegWord(EWRPT, TX_BUFFER_START); //Reset to start of Tx buffer
    WriteRegWord(ETXND, TX_BUFFER_START + nLen); //Set length of packet
    //Write to Tx buffer
    SPIWriteReg(ENC28J60_SPI_WBM, 0); //Reset control word to use default send configuration
    SPIWriteBuf(pBuffer, nLen); //Write data to Tx buffer
    SPISetBits(ECON1, ECON1_TXRTS); //Start transmission
}

uint16_t ENC28J60::PacketReceive(byte* pBuffer, uint16_t nSize)
{
    uint16_t nLen = 0;
    if(RxBegin() > 0)
        nLen = RxGetData(pBuffer, nSize);
    RxEnd();
    return nLen;
}

void ENC28J60::PowerDown()
{
    //1. Turn off packet reception by clearing ECON1.RXEN.
    SPIClearBits(ECON1, ECON1_RXEN);
    //2. Wait for any in-progress packets to finish being received by polling ESTAT.RXBUSY. This bit should be clear before proceeding.
    while(SPIReadRegister(ESTAT) & ESTAT_RXBUSY)
        ;
    //3. Wait for any current transmissions to end by confirming ECON1.TXRTS is clear.
    while(SPIReadRegister(ECON1) & ECON1_TXRTS)
        ;
    //4. Set ECON2.VRPS (if not already set).
    //5. Enter Sleep by setting ECON2.PWRSV. AllMAC, MII and PHY registers become inaccessible as a result. Setting PWRSV also clears ESTAT.CLKRDY automatically.
    SPISetBits(ECON2, ECON2_VRPS | ECON2_PWRSV);
}

void ENC28J60::PowerUp()
{
    //1. Wake-up by clearing ECON2.PWRSV.
    SPIClearBits(ECON2, ECON2_PWRSV);
    //2. Wait at least 300us for the PHY to stabilize. To accomplish the delay, the host controller may poll ESTAT.CLKRDY and wait for it to become set.
    while(!SPIReadRegister(ESTAT) & ESTAT_CLKRDY)
        ;
    //3. Restore receive capability by setting ECON1.RXEN.
    SPISetBits(ECON1, ECON1_RXEN);
}

void ENC28J60::EnableUnicast()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_UCEN);
}

void ENC28J60::DisableUnicast()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_UCEN);
}

void ENC28J60::EnableBroadcast(bool bTemporary)
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_BCEN);
    if(!bTemporary)
        m_bBroadcastEnabled = true;
}

void ENC28J60::DisableBroadcast(bool bTemporary)
{
    if(!bTemporary)
        m_bBroadcastEnabled = false;
    if(!m_bBroadcastEnabled)
        SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_BCEN);
}

void ENC28J60::EnableMulticast()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_MCEN);
}

void ENC28J60::DisableMulticast()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_MCEN);
}

void ENC28J60::EnablePatternMatch(uint16_t nOffset, uint16_t nChecksum, uint64_t nMask)
{
    //Write offset
    WriteRegWord(EPMO, nOffset & 0xFFFE); //erata B7 Carification 1 - offset must be even
    //Write mask
    WriteRegWord(EPMM0, nMask & 0xFF);
    WriteRegWord(EPMM2, (nMask >> 16) & 0xFF);
    WriteRegWord(EPMM4, (nMask >> 32) & 0xFF);
    WriteRegWord(EPMM6, (nMask >> 48) & 0xFF);
    //Write checksum
    WriteRegWord(EPMCS, nChecksum);
    //Enable pattern matcihng
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_PMEN);
}

void ENC28J60::DisablePatternMatch()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_PMEN);
}

void ENC28J60::EnableHashFilter(uint64_t nHashFlags)
{
    WriteRegWord(EHT0, nHashFlags & 0xFFFF);
    WriteRegWord(EHT2, (nHashFlags >> 16) & 0xFFFF);
    WriteRegWord(EHT4, (nHashFlags >> 32) & 0xFFFF);
    WriteRegWord(EHT6, (nHashFlags >> 48) & 0xFFFF);
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_HTEN);
}

void ENC28J60::DisableHashFilter()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_HTEN);
}

void ENC28J60::EnableMagicPacket(byte* pAddress)
{
    SPIWriteReg(MAADR0, pAddress[0]);
    SPIWriteReg(MAADR1, pAddress[1]);
    SPIWriteReg(MAADR2, pAddress[2]);
    SPIWriteReg(MAADR3, pAddress[3]);
    SPIWriteReg(MAADR4, pAddress[4]);
    SPIWriteReg(MAADR5, pAddress[5]);
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) | ERXFCON_MPEN);
}

void ENC28J60::DisableMagicPacket()
{
    SPIWriteReg(ERXFCON, SPIReadRegister(ERXFCON) & ~ERXFCON_MPEN);
}

bool ENC28J60::BIST(byte nTest)
{
    /*Four tests:
        ENC29J60_BIST_RDFM      = Random data fill
        ENC29J60_BIST_RDFM_RACE = Random data fill with race
        ENC29J60_BIST_AFM       = Address fill
        ENC29J60_BIST_PSFM      = Pattern shift fill
    */

    //Initialise and reset interface to ensure we are in a known state
    SPIInit();
    SPIReset();

    //1. Program the EDMAST register pair to 0000h
    WriteRegWord(EDMAST, 0);
    //2. Program EDMAND and ERXND register pairs to 1FFFh
    WriteRegWord(EDMAND, 0x1FFFu);
    WriteRegWord(ERXND, 0x1FFFu);
    //3. Configure the DMA for checksum generation by setting CSUMEN in ECON1
    WriteRegWord(ECON1, ECON1_CSUMEN);
    //4. Write the seed/initial shift value byte to the EBSTSD register (this is not necessary if Address Fill mode is used)
    if(ENC28J60_BIST_AFM != nTest)
        SPIWriteReg(EBSTSD, 0b10101010 | millis()); //random data
    //5. Enable Test mode, select the desired test, select the desired port configuration for the test
    WriteRegWord(EDMAST, nTest);
    //6. Start the BIST by setting EBSTCON.BISTST
    SPISetBits(EBSTCON, EBSTCON_BISTST);
    //7. If Random Data Fill with Race mode is not used, start the DMA checksum by setting DMAST in ECON1. The DMA controller will read thememory at the same rate the BIST controller willwrite to it, so the DMA can be started any timeafter the BIST is started
    if(ENC28J60_BIST_RDFM_RACE != nTest)
        SPISetBits(ECON1, ECON1_DMAST);
    //8. Wait for the DMA to complete by polling the DMAST bit
    while(SPIReadRegister(ECON1) & ECON1_DMAST)
        ;
    //9.Compare the EDMACS registers with the EBSTCS registers
    bool bResult = (SPIReadRegister(EDMACS) == SPIReadRegister(EBSTCS));

    SPIReset();
    return bResult;
}

int16_t ENC28J60::RxBegin()
{
    if(m_rxHeader.nNextPacket != m_nRxPacketPtr)
        RxEnd(); //End previous packet transaction just in case user did not
    m_nRxPacketPtr = m_rxHeader.nNextPacket;
    WriteRegWord(ERDPT, m_nRxPacketPtr); //Advance to next packet

    if(0 == SPIReadRegister(EPKTCNT))
        return 0; //There are no packets to process
    SPIReadBuf((byte*)&m_rxHeader, sizeof(m_rxHeader)); //Get NIC packet header (ENC28J60 data - not packet data)
    m_rxHeader.nSize -= 4; //Reduce size to ignore CRC at end of recieve buffer which is checked by NIC
    if(!(m_rxHeader.nStatus & ENC28J60_RX_OK)) //!@todo check whether all errors are detected by OK flag, e.g. length out of range
        return -1; //Indicate error
    return m_rxHeader.nSize;
}

uint16_t ENC28J60::RxGetData(byte* pBuffer, uint16_t nLen, uint16_t nOffset)
{
    if(nOffset >= m_rxHeader.nSize)
        return 0; //Requested start beyond end of packet
    if(TX_BUFFER_START - m_nRxPacketPtr <= nOffset) //Offset wraps receive circular buffer
        WriteRegWord(ERDPT, RX_BUFFER_START + nOffset - (TX_BUFFER_START - m_nRxPacketPtr));
    else
        WriteRegWord(ERDPT, m_nRxPacketPtr + nOffset);
    m_nRxOffset = nOffset;
    return RxGetData(pBuffer, nLen);
}

uint16_t ENC28J60::RxGetData(byte* pBuffer, uint16_t nLen)
{
    uint16_t nQuant = min(nLen, m_rxHeader.nSize - m_nRxOffset); //Limit data length to available buffer
    SPIReadBuf(pBuffer, nQuant); //Get data from NIC
    m_nRxOffset += nQuant;
    return nQuant;
}

uint16_t ENC28J60::RxGetStatus()
{
    return m_rxHeader.nStatus;
}

uint16_t ENC28J60::GetRxPacketSize()
{
    return m_rxHeader.nSize;
}

void ENC28J60::RxEnd()
{
    //erata B7.14 - ensure points to odd address
    if(ERXST == m_rxHeader.nNextPacket)
        WriteRegWord(ERXRDPT, ERXND);
    else
        WriteRegWord(ERXRDPT, m_rxHeader.nNextPacket - 1);
    SPISetBits(ECON2, ECON2_PKTDEC); //Decrement the packet count
}

byte ENC28J60::TxGetStatus()
{
    if(SPIReadRegister(ECON1) & ECON1_TXRTS)
        return ENC28J60_TX_IN_PROGRESS;
    if(SPIReadRegister(EIR) & EIR_TXERIF)
        return ENC28J60_TX_FAILED;
    return ENC28J60_TX_SUCCESS;
}

void ENC28J60::TxClearError()
{
    SPIClearBits(ESTAT, ESTAT_LATECOL | ESTAT_TXABRT);
}

void ENC28J60::TxBegin()
{
    while(SPIReadRegister(ECON1) & ECON1_TXRTS) //!@todo Consider erata B7.12 which says this may block indefinitely
        ; //Wait for previous transmission (if any) to complete
    if(SPIReadRegister(EIR) & EIR_TXERIF) //Tx aborted
    {
        //Reset Tx, i.e. abort transmission
        SPISetBits(ECON1, ECON1_TXRST); //Transmit reset only
        SPIClearBits(ECON1, ECON1_TXRST); //Return to normal operation
    }
    WriteRegWord(EWRPT, TX_BUFFER_START); //Reset start of Tx buffer
    SPIWriteReg(ENC28J60_SPI_WBM, 0); //Reset control word to use default send configuration
    m_nTxLen = 0;
}

void ENC28J60::TxAppend(byte* pData, uint16_t nLen)
{
    //Write to Tx buffer
    SPIWriteBuf(pData, nLen); //Write data to Tx buffer
    m_nTxLen += nLen;
}

void ENC28J60::TxEnd()
{
    WriteRegWord(ETXND, TX_BUFFER_START + m_nTxLen); //Set length of packet
    SPISetBits(ECON1, ECON1_TXRTS); //Start transmission
}

void ENC28J60::SetLedMode(uint16_t nMode)
{
    Serial.print("Setting LED mode ");
    Serial.println(nMode);
    WritePhyWord(PHLCON, nMode);
}

