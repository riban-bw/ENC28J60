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
static const byte REGISTER_MASK     = 0x1F; //!< Mask for encoding register bits 0-4
static const byte BANK_MASK         = 0x60; //!< Mask for encoding bank bits 5-6
static const byte MACMII_MASK       = 0x80; //!< Mask for encoding MAC or MII registers bit 7 - used by SPIReadRegister() because MAC and MII registers are read on second byte!!!???
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

// ENC28J60 EIE Register Bit Definitions
static const uint16_t EIE_INTIE      = 0x80; //!< Global INT interrupt enable bit
static const uint16_t EIE_PKTIE      = 0x40; //!< Receive packet pending interrupt enable bit
static const uint16_t EIE_DMAIE      = 0x20; //!< DMA interrupt enable bit
static const uint16_t EIE_LINKIE     = 0x10; //!< Link status change interrupt enable bit
static const uint16_t EIE_TXIE       = 0x08; //!< Transmit enable bit
static const uint16_t EIE_TXERIE     = 0x02; //!< Transmit error interrupt enable bit
static const uint16_t EIE_RXERIE     = 0x01; //!< Receive error interrupt enable bit
// ENC28J60 EIR Register Bit Definitions
static const uint16_t EIR_PKTIF      = 0x40; //!< Receive packet pending interrupt flag bit
static const uint16_t EIR_DMAIF      = 0x20; //!< DMA interrupt flag bit
static const uint16_t EIR_LINKIF     = 0x10; //!< Link change interrupt flag bit
static const uint16_t EIR_TXIF       = 0x08; //!< Transmit interrupt flag bit
static const uint16_t EIR_TXERIF     = 0x02; //!< Transmit error interrupt Flag bit
static const uint16_t EIR_RXERIF     = 0x01; //!< Receive error interrupt flag bit
// ENC28J60 ESTAT Register Bit Definitions
static const uint16_t ESTAT_INT      = 0x80; //!< INT interrupt flag bit
static const uint16_t ESTAT_BUFFER   = 0x40; //!< INT interrupt flag bit
static const uint16_t ESTAT_LATECOL  = 0x10; //!< Ethernet buffer error status bit
static const uint16_t ESTAT_RXBUSY   = 0x04; //!< Receive busy bit
static const uint16_t ESTAT_TXABRT   = 0x02; //!< Transmit abort error bit
static const uint16_t ESTAT_CLKRDY   = 0x01; //!< Clock ready bit (resets to ‘0’ on Power-on Reset but is unaffected on all other Resets)
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
// ENC28J60 ERXFCON Register Bit Definitions
static const uint16_t ERXFCON_UCEN   = 0x80; //!< Unicast filter enable bit
static const uint16_t ERXFCON_ANDOR  = 0x40; //!< AND / OR filter select bit
static const uint16_t ERXFCON_CRCEN  = 0x20; //!< Post-Filter CRC check enable bit
static const uint16_t ERXFCON_PMEN   = 0x10; //!< Pattern match enable bit
static const uint16_t ERXFCON_MPEN   = 0x08; //!< Magic Packet filter enable bit
static const uint16_t ERXFCON_HTEN   = 0x04; //!< Hash table filter enable bit
static const uint16_t ERXFCON_MCEN   = 0x02; //!< Multicast match enable bit
static const uint16_t ERXFCON_BCEN   = 0x01; //!< Broadcast match enable bit
// ENC28J60 MACON1 Register Bit Definitions
static const uint16_t MACON1_TXPAUS  = 0x08; //!< Pause control frame transmission enable bit
static const uint16_t MACON1_RXPAUS  = 0x04; //!< Pause control frame reception enable bit
static const uint16_t MACON1_PASSALL = 0x02; //!< Pass all received frames enable bit
static const uint16_t MACON1_MARXEN  = 0x01; //!< MAC receive enable bit
// ENC28J60 MACON2 Register removed from release note B7 so ommited here
// ENC28J60 MACON3 Register Bit Definitions
static const uint16_t MACON3_PADCFG2 = 0x80; //!< Automatic pad and CRC configuration bit 2
static const uint16_t MACON3_PADCFG1 = 0x40; //!< Automatic pad and CRC configuration bit 1
static const uint16_t MACON3_PADCFG0 = 0x20; //!< Automatic pad and CRC configuration bit 0
static const uint16_t MACON3_TXCRCEN = 0x10; //!< Transmit CRC enable bit
static const uint16_t MACON3_PHDRLEN = 0x08; //!< Proprietary header enable bit
static const uint16_t MACON3_HFRMLEN = 0x04; //!< Huge frame enable bit
static const uint16_t MACON3_FRMLNEN = 0x02; //!< Frame length checking enable bit
static const uint16_t MACON3_FULDPX  = 0x01; //!< MAC Full-Duplex enable bit
// ENC28J60 MACON4 Register Bit Definitions
static const uint16_t MACON4_DEFER   = 0x40;   //!< Defer transmission enable bit (applies to half duplex only)
static const uint16_t MACON4_BPEN    = 0x20;   //!< No backoff during backpressure enable bit (applies to half duplex only)
static const uint16_t MACON4_NOBKOFF = 0x10;   //!< No backoff enable bit (applies to half duplex only)
// ENC28J60 MICMD Register Bit Definitions
static const uint16_t MICMD_MIISCAN  = 0x02; //!< MII scan enable bit
static const uint16_t MICMD_MIIRD    = 0x01; //!< MII read enable bit
// ENC28J60 EBSTCON Register Bit Definitions
static const uint16_t EBSTCON_PSV2   = 0x80; //!< Pattern shift value bit 2
static const uint16_t EBSTCON_PSV1   = 0x40; //!< Pattern shift value bit 1
static const uint16_t EBSTCON_PSV0   = 0x20; //!< Pattern shift value bit 0
static const uint16_t EBSTCON_PSEL   = 0x10; //!< Port select bit
static const uint16_t EBSTCON_TMSEL1 = 0x08; //!< Test mode select bit 1
static const uint16_t EBSTCON_TMSEL0 = 0x04; //!< Test mode select bit 0
static const uint16_t EBSTCON_TME    = 0x02; //!< Test mode enable bit
static const uint16_t EBSTCON_BISTST = 0x01; //!< Built-in self-test start/busy bit
// ENC28J60 MISTAT Register Bit Definitions
static const uint16_t MISTAT_NVALID     = 0x04; //!< MII management read data not valid bit
static const uint16_t MISTAT_SCAN       = 0x02; //!< MII management scan operation bit
static const uint16_t MISTAT_BUSY       = 0x01; //!< MII management busy bit
// ENC28J60 ECOCON Register Bit Definitions
static const uint16_t ECOCON_COCON2     = 0x04; //!< Clock output configuration bit 2
static const uint16_t ECOCON_COCON1     = 0x02; //!< Clock output configuration bit 1
static const uint16_t ECOCON_COCON0     = 0x01; //!< Clock output configuration bit 0
// ENC28J60 EFLOCON Register Bit Definitions
static const uint16_t EFLOCON_FULDPXS   = 0x04; //!< Read-only MAC full-duplex shadow bit
static const uint16_t EFLOCON_FCEN1     = 0x02; //!< Flow control enable bit 1
static const uint16_t EFLOCON_FCEN0     = 0x01; //!< Flow control enable bit 0
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
static const uint16_t PHCON1_PRST    = 0x8000; //!< PHY software reset bit
static const uint16_t PHCON1_PLOOPBK = 0x4000; //!< PHY loopback bit
static const uint16_t PHCON1_PPWRSV  = 0x0800; //!< PHY power-down bit
static const uint16_t PHCON1_PDPXMD  = 0x0100; //!< PHY duplex mode bit
// ENC28J60 PHY PHSTAT1 Register Bit Definitions
static const uint16_t PHSTAT1_PFDPX  = 0x1000; //!< PHY full-duplex capable bit
static const uint16_t PHSTAT1_PHDPX  = 0x0800; //!< PHY half-duplex capable bit
static const uint16_t PHSTAT1_LLSTAT = 0x0004; //!< PHY latching link status bit
static const uint16_t PHSTAT1_JBSTAT = 0x0002; //!< PHY latching jabber status bit
// ENC28J60 PHT PHSTAT2 Register Bit Definitions
static const uint16_t PHSTAT2_TXSTAT    = 0x2000; //!< PHY transmit status bit
static const uint16_t PHSTAT2_RXSTAT    = 0x1000; //!< PHY revieve status bit
static const uint16_t PHSTAT2_COLSTAT   = 0x0800; //!< PHY collision status bit
static const uint16_t PHSTAT2_LSTAT     = 0x0400; //!< PHY link status status bit
static const uint16_t PHSTAT2_DPXSTAT   = 0x0200; //!< PHY duplex status bit
static const uint16_t PHSTAT2_PLRITY    = 0x0020; //!< PHY polarity status bit
// ENC28J60 PHY PHCON2 Register Bit Definitions
static const uint16_t PHCON2_FRCLINK = 0x4000; //!< PHY Force linkup bit
static const uint16_t PHCON2_TXDIS   = 0x2000; //!< Twisted-pair transmitter disable bit
static const uint16_t PHCON2_JABBER  = 0x0400; //!< Jabber correction disable bit
static const uint16_t PHCON2_HDLDIS  = 0x0100; //!< PHY half-duplex loopback disable bit

// ENC28J60 Packet Control Byte Bit Definitions
static const byte PKTCTRL_PHUGEEN   = 0x08; //!< Per packet huge frame enable bit
static const byte PKTCTRL_PPADEN    = 0x04; //!< Per packet padding enable bit
static const byte PKTCTRL_PCRCEN    = 0x02; //!< Per packet CRC enable bit
static const byte PKTCTRL_POVERRIDE = 0x01; //!< Per packet override bit

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

// NIC 8K RAM allocation
static const uint16_t MAX_FRAMELEN      = 1514; //!< Maximum frame length minus 4 byte crc appended by hardware
static const uint16_t TX_BUFFER_END     = 0x1FFF; //!< End of transmit buffer which is end of available memory
static const uint16_t TX_BUFFER_PPCB    = TX_BUFFER_END - MAX_FRAMELEN - 7; //!< Address of 'per packet control byte', immediately before start of tx Buffer
static const uint16_t TX_BUFFER_START   = TX_BUFFER_PPCB + 1; //!< Start of transmit buffer which gives space for 1 packet (1514 frame bytes, 7 Tx status bytes)
static const uint16_t RX_BUFFER_START   = 0x0000; //!< Start of recieve circular buffer (erata B7.5 work-around is to place Rx buffer at 0x0000)
static const uint16_t RX_BUFFER_END     = TX_BUFFER_PPCB - 1; //!< End of recieve circular buffer
static const uint16_t RX_HEADER_SIZE    = 6; //!< Quantity of bytes in the ENC28J60 recieve header, not part of the received packet

//***SPI bus interface functions***

void ENC28J60::SPIInit()
{
    //Configure SPI interface pins
    pinMode(SS, OUTPUT); //SS pin must be output for SPI to work as master, even if another pin is used for CS
    digitalWrite(SS, HIGH);
    pinMode(MOSI, OUTPUT);
    pinMode(SCK, OUTPUT);
    pinMode(MISO, INPUT);

//    digitalWrite(MOSI, HIGH);
//    digitalWrite(MOSI, LOW);
//    digitalWrite(SCK, LOW);

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

//***NIC low-level control and interface functions***

uint16_t ENC28J60::ReadRegWord(byte nAddress)
{
    return SPIReadRegister(nAddress) + (SPIReadRegister(nAddress + 1) << 8);
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

//***NIC high-level control and interface functions***

byte ENC28J60::Initialize(const byte* pMac, byte nChipSelectPin)
{
    m_nBank = 0xFF; //Set bank to invalid value to ensure initial bank selection
    m_bBroadcastEnabled = true;
    m_bFlowControl = true;
    m_bRxPause = false;
    m_nSelectPin = nChipSelectPin;
    pinMode(m_nSelectPin, OUTPUT);
    DisableChip();
    if(bitRead(SPCR, SPE) == 0)
        SPIInit(); //SPI not enabled so intialise it

    Reset();

    DisableReception(); //Avoid incoming traffic effecting configuration
    //Configure internal buffers
    m_rxHeader.nNextPacket = RX_BUFFER_START;
    m_rxHeader.nSize = 0;
    m_rxHeader.nStatus = 0;
    m_nRxPacketPtr = RX_BUFFER_START;
    WriteRegWord(ERXST, RX_BUFFER_START);
    WriteRegWord(ERXRDPT, RX_BUFFER_START);
    WriteRegWord(ERXND, RX_BUFFER_END);
    WriteRegWord(ETXST, TX_BUFFER_PPCB);
    WriteRegWord(ETXND, TX_BUFFER_END);
    //Configure MAC
    SPIWriteReg(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
    SPIWriteReg(MACON2, 0x00);
    SPIWriteReg(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
    WriteRegWord(MAIPG, 0x0c12);
    WriteRegWord(MAMXFL, MAX_FRAMELEN + 4); //Set maximum frame length, including 4 bytes for Ethernet CRC
    SPIWriteReg(MAADR5, pMac[0]); //Set MAC address
    SPIWriteReg(MAADR4, pMac[1]);
    SPIWriteReg(MAADR3, pMac[2]);
    SPIWriteReg(MAADR2, pMac[3]);
    SPIWriteReg(MAADR1, pMac[4]);
    SPIWriteReg(MAADR0, pMac[5]);
    WritePhyWord(PHCON2, PHCON2_HDLDIS); //Disable loopback when in half duplex
    SPIWriteReg(EIE, 0); //Disable all interupts

    SetHalfDuplex(); //ENC28J60 does not auto-negotiate and presents as half duplex so safest mode is to use half duplex by default
    EnableReception();

    return SPIReadRegister(EREVID);
}

void ENC28J60::Reset()
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

void ENC28J60::PowerUp()
{
    //1. Wake-up by clearing ECON2.PWRSV.
    SPIClearBits(ECON2, ECON2_PWRSV);
    //2. Wait at least 300us for the PHY to stabilize. To accomplish the delay, the host controller may poll ESTAT.CLKRDY and wait for it to become set.
    while(!SPIReadRegister(ESTAT) & ESTAT_CLKRDY)
        ;
    //3. Restore receive capability by setting ECON1.RXEN.
    EnableReception();
}

void ENC28J60::PowerDown()
{
    //1. Turn off packet reception by clearing ECON1.RXEN.
    DisableReception();
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

void ENC28J60::EnableReception()
{
    SPISetBits(ECON1, ECON1_RXEN | ECON1_CSUMEN);
}

bool ENC28J60::DisableReception()
{
    bool bReturn = SPIReadRegister(ECON1) & ECON1_RXEN;
    while(ReadPhyWord(PHSTAT2) & PHSTAT2_RXSTAT)
        ; //Wait until not receiving data
    SPIClearBits(ECON1, ECON1_RXEN);
    return bReturn;
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

void ENC28J60::SetFullDuplex()
{
    //!@todo Test full / half duplex
    bool bRx = DisableReception();
    //Wait for Tx to complete
    while(SPIReadRegister(ECON1) & ECON1_TXRTS)
        ;
    uint16_t nMacon3 = SPIReadRegister(MACON3);
    uint16_t nPhcon1 = SPIReadRegister(PHCON1);
    SPIWriteReg(MACON3, nMacon3 | MACON3_FULDPX);
    SPIWriteReg(PHCON1, nPhcon1 | PHCON1_PDPXMD);
    SPIWriteReg(MABBIPG, 0x15);
    if(bRx)
        EnableReception();
}

void ENC28J60::SetHalfDuplex()
{
    bool bRx = DisableReception();
    //Wait for Tx to complete
    while(SPIReadRegister(ECON1) & ECON1_TXRTS)
        ;
    uint16_t nMacon3 = SPIReadRegister(MACON3);
    uint16_t nPhcon1 = SPIReadRegister(PHCON1);
    SPIWriteReg(MACON3, nMacon3 & ~MACON3_FULDPX);
    SPIWriteReg(PHCON1, nPhcon1 & ~PHCON1_PDPXMD);
    SPIWriteReg(MABBIPG, 0x12);
    if(bRx)
        EnableReception();
}

void ENC28J60::EnableFlowControl()
{
    m_bFlowControl = true;
    SPIWriteReg(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
}

void ENC28J60::DisableFlowControl()
{
    m_bFlowControl = false;
    SPIWriteReg(MACON1, MACON1_MARXEN);
}

bool ENC28J60::IsLinkUp()
{
    return(ReadPhyWord(PHSTAT2) & PHSTAT2_LSTAT);
}

void ENC28J60::SetLedMode(uint16_t nMode)
{
    WritePhyWord(PHLCON, nMode);
}

//***Packet reception functions***

int16_t ENC28J60::RxBegin()
{
    if(m_rxHeader.nSize)
        RxEnd(); //End previous packet transaction when user did not

    if(0 == SPIReadRegister(EPKTCNT))
        return 0; //There are no packets to process
    //!@todo Consider whether flow control should be in RxBegin, RxEnd or elsewhere
    if(m_bFlowControl) //!@todo flow control causing problems
    {
        if(RxGetFreeSpace() < MAX_FRAMELEN && !m_bRxPause)
        {
            //Too many packets and not yet asserted flow control pause so do it now
            SPIWriteReg(EFLOCON, 0x02);
            m_bRxPause = true;
        }
        else if(m_bRxPause && RxGetFreeSpace() >= MAX_FRAMELEN)
        {
            //Flow control asserted and fallen below packet threshold so unassert flow control pause
            SPIWriteReg(EFLOCON, 0x03);
            m_bRxPause = false;
        }
    }
    WriteRegWord(ERDPT, m_nRxPacketPtr); //Reset read pointer to start of packet
    SPIReadBuf((byte*)&m_rxHeader, sizeof(m_rxHeader)); //Get NIC packet header (ENC28J60 data - not packet data)
    m_rxHeader.nSize -= 4; //Reduce size to ignore CRC at end of recieve buffer which is checked by NIC
    m_nRxPacketPtr += sizeof(m_rxHeader); //Advance pointer to start of Ethernet packet (beyond internal recieve header)
    if(!(m_rxHeader.nStatus & ENC28J60_RX_OK)) //!@todo check whether all errors are detected by OK flag, e.g. length out of range
        return ENC28J60_RX_ERROR;
    return m_rxHeader.nSize;
}

uint16_t ENC28J60::RxGetData(byte* pBuffer, uint16_t nLen)
{
    uint16_t nQuant = min(nLen, m_rxHeader.nSize - m_nRxOffset); //Limit data length to available buffer
    SPIReadBuf(pBuffer, nQuant); //Get data from NIC
    m_nRxOffset += nQuant;
    return nQuant;
}

uint16_t ENC28J60::RxGetData(byte* pBuffer, uint16_t nLen, uint16_t nOffset)
{
    if(nOffset >= m_rxHeader.nSize)
        return 0; //Requested start beyond end of packet
    if(RX_BUFFER_END - m_nRxPacketPtr < nOffset) //Offset wraps receive circular buffer
        WriteRegWord(ERDPT, RX_BUFFER_START + nOffset - (RX_BUFFER_END - m_nRxPacketPtr) - 1);
    else
        WriteRegWord(ERDPT, m_nRxPacketPtr + nOffset);
    m_nRxOffset = nOffset;
    return RxGetData(pBuffer, nLen);
}

byte ENC28J60::RxGetByte()
{
    byte nValue;
    RxGetData((byte*)&nValue, 1);
    return nValue;
}

byte ENC28J60::RxGetByte(uint16_t nOffset)
{
    byte nValue;
    RxGetData((byte*)&nValue, 1, nOffset);
    return nValue;
}

uint16_t ENC28J60::RxGetWord()
{
    uint16_t nValue;
    RxGetData((byte*)&nValue, 2);
    return SwapBytes(nValue);
}

uint16_t ENC28J60::RxGetWord(uint16_t nOffset)
{
    uint16_t nValue;
    RxGetData((byte*)&nValue, 2, nOffset);
    return SwapBytes(nValue);
}

void ENC28J60::RxEnd()
{
    if(0 == m_rxHeader.nSize)
        return; //Not currently processing packet
    //erata B7.14 - ensure recieve buffer read pointer points to odd address

    //Need to set unavailable space boundary to odd value
    if(m_rxHeader.nNextPacket & 0x0001)
    {
        //Next packet starts at odd address so protect by setting ERXRDPT to it
        WriteRegWord(ERXRDPT, m_rxHeader.nNextPacket);
    }
    else
    {
        //Next packet starts at even address so protect by setting ERXRDPT to one less
        if(RX_BUFFER_START == m_rxHeader.nNextPacket)
            WriteRegWord(ERXRDPT, RX_BUFFER_END - 2); //Ensure RX_BUFFER_END - X is odd
        else
            WriteRegWord(ERXRDPT, m_rxHeader.nNextPacket - 1);
    }
    SPISetBits(ECON2, ECON2_PKTDEC); //Decrement the packet count
    m_rxHeader.nSize = 0; //Clear packet size - used to indicate we are processing a packet
    m_nRxPacketPtr = m_rxHeader.nNextPacket;
}

uint16_t ENC28J60::PacketReceive(byte* pBuffer, uint16_t nSize)
{
    uint16_t nLen = 0;
    if(RxBegin() > 0)
        nLen = RxGetData(pBuffer, nSize);
    RxEnd();
    return nLen;
}

uint16_t ENC28J60::RxGetPacketSize()
{
    return m_rxHeader.nSize;
}

uint16_t ENC28J60::RxGetStatus()
{
    return m_rxHeader.nStatus;
}

bool ENC28J60::RxIsBroadcast()
{
    return m_rxHeader.nStatus & ENC28J60_RX_BROADCAST;
}

bool ENC28J60::RxIsMulticast()
{
    return m_rxHeader.nStatus & ENC28J60_RX_MULTICAST;
}

byte ENC28J60::RxGetPacketCount()
{
    return SPIReadRegister(EPKTCNT);
}


uint16_t ENC28J60::RxGetFreeSpace()
{
    byte nCount = SPIReadRegister(EPKTCNT); //get packet count to allow checking the reading of write pointer is atomic
    uint16_t nWrpt = ReadRegWord(ERXWRPT); //read write pointer
    if(SPIReadRegister(EPKTCNT) != nCount) //chance of change of MSB/LSB parts of write pointer
        nWrpt = ReadRegWord(ERXWRPT); //re-read write pointer
    //!@todo Check free space calculation is accurate
    if(nWrpt > m_rxHeader.nNextPacket)
        return (RX_BUFFER_END - RX_BUFFER_START) - (nWrpt - m_rxHeader.nNextPacket);
    else if(nWrpt == m_rxHeader.nNextPacket)
        return (RX_BUFFER_END - RX_BUFFER_START);
    else
        return m_rxHeader.nNextPacket - nWrpt - 1;
}

//***Packet transmission functions***

void ENC28J60::TxBegin(byte* pMac, uint16_t nEthertype)
{
    while(SPIReadRegister(ECON1) & ECON1_TXRTS) //!@todo Consider erata B7.12 which says this may block indefinitely
        ; //Wait for previous transmission (if any) to complete
    if(SPIReadRegister(EIR) & EIR_TXERIF) //Tx aborted
    {
        //Reset Tx, i.e. abort transmission
        SPISetBits(ECON1, ECON1_TXRST); //Transmit reset only
        SPIClearBits(ECON1, ECON1_TXRST); //Return to normal operation
    }
    WriteRegWord(EWRPT, TX_BUFFER_PPCB); //Reset start of Tx buffer
    SPIWriteReg(ENC28J60_SPI_WBM, 0); //Reset control word to use default send configuration
    byte pBuffer[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; //Write destination MAC
    SPIWriteBuf(pMac?pMac:pBuffer, 6);
    GetMac(pBuffer);
    SPIWriteBuf(pBuffer, 6); //Write source MAC
    uint16_t nType = SwapBytes(nEthertype);
    SPIWriteBuf((byte*)&nType, 2); //Write Ethertype or length in network byte order
    m_nTxLen = 14;

    //Tx write pointer is pointing to first byte of Ethernet payload
}

bool ENC28J60::TxAppend(byte* pData, uint16_t nLen)
{
    if(m_nTxLen + nLen > MAX_FRAMELEN)
        return false; //Insufficient space in Tx buffer
    //Write to Tx buffer
    SPIWriteBuf(pData, nLen); //Write data to Tx buffer
    m_nTxLen += nLen;
    return true;
}

bool ENC28J60::TxAppendByte(byte nData)
{
    return TxAppend(&nData, 1);
}

bool ENC28J60::TxAppendWord(uint16_t nData)
{
    uint16_t nByteOrder = SwapBytes(nData);
    return TxAppend((byte*)&nByteOrder, 2);
}

void ENC28J60::TxWrite(uint16_t nOffset, byte* pData, uint16_t nLen)
{
    WriteRegWord(EWRPT, TX_BUFFER_START + nOffset); //packet data starts at offset +1, after 'per packet control byte'
    SPIWriteBuf(pData, nLen);
    WriteRegWord(EWRPT, TX_BUFFER_START + m_nTxLen); //Reset pointer to where it was
    m_nTxLen = max(m_nTxLen, nOffset - 1 + nLen);
}

void ENC28J60::TxWriteByte(uint16_t nOffset, byte nData)
{
    WriteRegWord(EWRPT, TX_BUFFER_START + nOffset); //packet data starts at offset +1, after 'per packet control byte'
    SPIWriteBuf(&nData, 1);
    WriteRegWord(EWRPT, TX_BUFFER_START + m_nTxLen); //Reset pointer to where it was
    m_nTxLen = max(m_nTxLen, nOffset - 1);
}

void ENC28J60::TxWriteWord(uint16_t nOffset, uint16_t nData)
{
    uint16_t nByteOrder = SwapBytes(nData);
    WriteRegWord(EWRPT, TX_BUFFER_START + nOffset); //packet data starts at offset +1, after 'per packet control byte'
    SPIWriteBuf((byte*)&nByteOrder, 2);
    WriteRegWord(EWRPT, TX_BUFFER_START + m_nTxLen); //Reset pointer to where it was
    m_nTxLen = max(m_nTxLen, nOffset);
}

void ENC28J60::TxSwap(uint16_t nOffset1, uint16_t nOffset2, uint16_t nLen)
{
    byte pBuffer1[nLen];
    byte pBuffer2[nLen];
    WriteRegWord(ERDPT, TX_BUFFER_START + nOffset1);
    SPIReadBuf(pBuffer1, nLen);
    WriteRegWord(ERDPT, TX_BUFFER_START + nOffset2);
    SPIReadBuf(pBuffer2, nLen);
    TxWrite(nOffset1, pBuffer2, nLen);
    TxWrite(nOffset2, pBuffer1, nLen);
    WriteRegWord(ERDPT, m_nRxPacketPtr + m_nRxOffset); //Recover Rx read position
}


byte ENC28J60::TxGetStatus()
{
    if(SPIReadRegister(ECON1) & ECON1_TXRTS)
        return ENC28J60_TX_IN_PROGRESS;
    if(SPIReadRegister(EIR) & EIR_TXERIF)
        return ENC28J60_TX_FAILED;
    return ENC28J60_TX_SUCCESS;
}

byte ENC28J60::TxGetError()
{
    byte pBuffer[7];
    //!@todo Set read pointer to start of Tx error
    WriteRegWord(ERDPT, ReadRegWord(ETXND) + 3); //Position read pointer at Tx Status Vector + 2 (skip Tx byte count)
    SPIReadBuf(pBuffer, 5); //Read remaining TSV
    byte nError = 0;
    //!@todo Populate each bit of nError
    //!@todo Define error bits as Constants
    nError |= ((pBuffer[0] & 112) >> 4);
    nError |= ((pBuffer[1] & 124) << 3);

    WriteRegWord(ERDPT, m_nRxPacketPtr); //Restore read pointer
    return nError;
}

void ENC28J60::TxClearError()
{
    SPIClearBits(ESTAT, ESTAT_LATECOL | ESTAT_TXABRT);
}

uint16_t ENC28J60::TxGetSize()
{
    return m_nTxLen;
}

void ENC28J60::TxEnd(uint16_t nLen)
{
    if(nLen)
        WriteRegWord(ETXND, TX_BUFFER_START + nLen - 1); //Define packet length by setting ETXND to point to last byte of frame
    else
        WriteRegWord(ETXND, TX_BUFFER_START + m_nTxLen - 1); //Define packet length by setting ETXND to point to last byte of frame
    SPISetBits(ECON1, ECON1_TXRTS); //Start transmission
}

bool ENC28J60::PacketSend(byte* pBuffer, uint16_t nLen)
{
    TxBegin();
    if(TxAppend(pBuffer, nLen))
    {
        TxEnd();
        return true;
    }
    return false;
}

//***Misc functions***

void ENC28J60::DMACopy(uint16_t nDestination, uint16_t nStart, uint16_t nLen)
{
    /*Appropriately program the EDMAST, EDMAND and EDMADST register pairs. The EDMAST registers should point to the first byte to copy from, the EDMAND registers should point to the
    last byte to copy and the EDMADST registers should point to the first byte in the destination range. The destination range will always be linear, never wrapping at any values except from
    8191 to 0 (the 8-Kbyte memory boundary). Extreme care should be taken when programming the Start and End Pointers to prevent a never ending DMA operation which would overwrite the entire 8-Kbyte buffer.
    */
    if(nLen < 1 || nStart >= m_rxHeader.nSize || nStart + nLen > m_rxHeader.nSize)
        return; //Check request fits within available data range. Also check at least one copy (ENC28J60 design does not permit zero)
    bool bRx = DisableReception();
    //Set the DMA source start
    if(RX_BUFFER_END - m_nRxPacketPtr < nStart) //wraps receive circular buffer
        WriteRegWord(EDMAST, RX_BUFFER_START + nStart - (RX_BUFFER_END - m_nRxPacketPtr) - 1);
    else
        WriteRegWord(EDMAST, m_nRxPacketPtr + nStart);
    //Set the DMA source end
    if(RX_BUFFER_END - m_nRxPacketPtr + 1 < nStart + nLen) //wraps receive circular buffer
        WriteRegWord(EDMAND, RX_BUFFER_START + nStart + nLen - (RX_BUFFER_END - m_nRxPacketPtr) - 2);
    else
        WriteRegWord(EDMAND, m_nRxPacketPtr + nStart + nLen - 1);
    //Set DMA destination start
    WriteRegWord(EDMADST, TX_BUFFER_START + nDestination);
    //Verify that ECON1.CSUMEN is clear.
    SPIClearBits(ECON1, ECON1_CSUMEN);
    //Start the DMA copy by setting ECON1.DMAST.
    SPISetBits(ECON1, ECON1_DMAST);
    //When the copy is complete, the DMA hardware will clear the DMAST bit, set the DMAIF bit and generate an interrupt (if enabled). The pointers and the EDMACS registers will not be modified.
    while(SPIReadRegister(ECON1) & ECON1_DMAST)
        ;
    if(bRx)
        EnableReception();
    m_nTxLen = max(m_nTxLen, nDestination + nLen);
}

uint16_t ENC28J60::SwapBytes(uint16_t nValue)
{
    return (((nValue & 0x00FF) << 8) + ((nValue & 0xFF00) >> 8));
}

void ENC28J60::GetMac(byte* pMac)
{
    pMac[0] = SPIReadRegister(MAADR5);
    pMac[1] = SPIReadRegister(MAADR4);
    pMac[2] = SPIReadRegister(MAADR3);
    pMac[3] = SPIReadRegister(MAADR2);
    pMac[4] = SPIReadRegister(MAADR1);
    pMac[5] = SPIReadRegister(MAADR0);
}

uint16_t ENC28J60::GetChecksum(uint16_t nStart, uint16_t nLength)
{
    //Range check
    if(nStart > TX_BUFFER_END)
        return 0;
    if(nStart + nLength > TX_BUFFER_END)
        return 0;
    bool bRx = DisableReception(); //Errata B7.17 says do not use checksum whilst receiving data
    //Set EDMAST to first byte and EDMAND to last byte
    WriteRegWord(EDMAST, TX_BUFFER_START + nStart); //Packet starts at offset +1, after 'per packet offset'
    WriteRegWord(EDMAND, TX_BUFFER_START + nStart + nLength - 1);
    //To generate interrupt on completion, clear EIR.DMAIF, set EIE.DMAIE and set EIE.INTIE.
    /* Uncomment to enable interrupt
    SPIClearBits(EIR, EIR_DMAIF);
    SPISetBits(EIE, EIE_DMAIE);
    SPISetBits(EIE, EIE_INTIE);
    */
    //Start calculation by setting ECON1.CSUMEN and ECON1.DMAST.
    SPISetBits(ECON1, ECON1_CSUMEN);
    SPISetBits(ECON1, ECON1_DMAST);
    //Check for completion: DMAST bit cleared, DMAIF bit set and an interrupt generated if enabled.
    while((SPIReadRegister(ECON1) & ECON1_DMAST) && !(SPIReadRegister(EIR) | EIR_DMAIF))
        delay(1);
    if(bRx)
        EnableReception(); //Errata B7.17 says do not use checksum whilst receiving data
    //EDMACSH and EDMACSL registers will contain the calculated checksum.
    uint16_t nResult = ReadRegWord(EDMACS);
    return SwapBytes(nResult); //!@todo Should we return network byte order or host byte order? Should be consistent with other functions.
}

bool ENC28J60::BIST(byte nTest)
{
    //!@todo BIST stops NIC from sending packets
    /*Four tests:
        ENC29J60_BIST_RDFM      = Random data fill
        ENC29J60_BIST_RDFM_RACE = Random data fill with race
        ENC29J60_BIST_AFM       = Address fill
        ENC29J60_BIST_PSFM      = Pattern shift fill
    */

    //Initialise and reset interface to ensure we are in a known state
    SPIInit();
    Reset();

    //Store MAC to allow reseting of nic
    byte pMac[6];
    GetMac(pMac);

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

    Initialize(pMac, m_nSelectPin);
    return bResult;
}
