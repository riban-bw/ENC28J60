// Microchip ENC28J60 Ethernet Interface Driver
// Author: Pascal Stang
// Modified by: Guido Socher
// Copyright: GPL V2
//
// This driver provides initialization and transmit/receive
// functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//
// 2010-05-20 <jc@wippler.nl>

#pragma once

// Tx status
//!@todo move this to abstract Ethernet class
static const byte ENC28J60_TX_SUCCESS       = 0x00;
static const byte ENC28J60_TX_IN_PROGRESS   = 0x01;
static const byte ENC28J60_TX_FAILED        = 0x02;
//Built in self test modes
static const byte ENC28J60_BIST_RDFM        = 0b0000; //!< Random data fill mode
static const byte ENC28J60_BIST_RDFM_RACE   = 0b1100; //!< Random data fill mode with race
static const byte ENC28J60_BIST_AFM         = 0b0100; //!< Address fill mode
static const byte ENC28J60_BIST_PSFM        = 0b1000; //!< Pattern shift fill mode
//Recieve header status flags
static const uint16_t ENC28J60_RX_DROP              = 0x0001; //!< Indicates a packet over 50,000 bit times occurred or that a packet was dropped since the last receive
static const uint16_t ENC28J60_RX_CARRIER_SEEN      = 0x0004; //!< Indicates that at some time since the last receive, a carrier event was detected. The carrier event is not associated with this packet. A carrier event is activity on the receive channel that does not result in a packet receive attempt being made
static const uint16_t ENC28J60_RX_CRC_ERROR         = 0x0010; //!< Indicates that frame CRC field value does not match the CRC calculated by the MAC
static const uint16_t ENC28J60_RX_LEN_ERROR         = 0x0020; //!< Indicates that frame length field value in the packet does not match the actual data byte length and specifies a valid length
static const uint16_t ENC28J60_RX_LEN_RANGE_ERROR   = 0x0040; //!< Indicates that frame type/length field was larger than 1500 bytes (type field)
static const uint16_t ENC28J60_RX_OK                = 0x0080; //!< Indicates that at the packet had a valid CRC and no symbol errors
static const uint16_t ENC28J60_RX_MULTICAST         = 0x0100; //!< Indicates packet received had a valid multicast address
static const uint16_t ENC28J60_RX_BROADCAST         = 0x0200; //!< Indicates packet received had a valid broadcast address
static const uint16_t ENC28J60_RX_DRIBBLE           = 0x0400; //!< Indicates that after the end of this packet, an additional 1 to 7 bits were received. The extra bits were thrown away
static const uint16_t ENC28J60_RX_CONTROL_FRAME     = 0x0800; //!< Current frame was recognized as a control frame for having a valid type/length designating it as a control frame
static const uint16_t ENC28J60_RX_PAUSE_FRAME       = 0x1000; //!< Current frame was recognized as a control frame containing a valid pause frame opcode and a valid destination address
static const uint16_t ENC28J60_RX_UNKNOWN_OPCODE    = 0x2000; //!< Current frame was recognized as a control frame but it contained an unknown opcode
static const uint16_t ENC28J60_RX_VLAN              = 0x4000; //!< Current frame was recognized as a VLAN tagged frame

struct ENC28J60_RX_HEADER
{
    uint16_t nNextPacket;
    uint16_t nSize;
    uint16_t nStatus;
};

/** This class provide low-level interfacing with the ENC28J60 network interface.*/
class ENC28J60
{
    public:
        /**   @brief  Initialise network interface
        *     @param  macaddr Pointer to 4 byte hardware (MAC) address
        *     @param  nChipSelectPin Arduino pin used for chip select (enable network interface SPI bus). Default = 8
        *     @return <i>uint8_t</i> ENC28J60 firmware version or zero on failure.
        */
        byte Initialize(const byte* pMac, byte nChipSelectPin = 8);

        /**   @brief  Check if network link is connected
        *     @return <i>bool</i> True if link is up
        */
        bool IsLinkUp();

        /** @brief  Sends whole packet of data
        *   @param  pBuffer Pointer to the data buffer
        *   @param  nLen Quantity of bytes of data to send
        *   @note   Only supports transmitting one packet at a time. Blocks until previous packet transmission is complete
        */
        void PacketSend(byte* pBuffer, uint16_t nLen);

        /** @brief  Copy recieved packet to data buffer
        *   @param  pBuffer Pointer to a buffer to recieve data
        *   @param  nSize Maximum quantity of bytes to read
        *   @return <i>uint16_t</i> Quantity of bytes actually copied
        *   @note   Use this function to handle whole packets (with suitable sized buffer). Use Rx transaction functions to access NIC recieve buffer directly
        */
        uint16_t PacketReceive(byte* pBuffer, uint16_t nSize);

        /** @brief  Put ENC28J60 in sleep mode
        */
        void PowerDown();  // contrib by Alex M.

        /** @brief  Wake ENC28J60 from sleep mode
        */
        void PowerUp();    // contrib by Alex M.

        /** @brief  Enables unicast match filter
        *   @note   This is default behaviour. Accepts packets targetted at local host MAC address
        */
        void EnableUnicast();

        /** @brief  Disables unicast match filter
        *   @note   Silently drops (ignores) packets targetted at local host MAC address.
        */
        void DisableUnicast();

        /** @brief  Enable reception of broadcast messages
        *   @param  bTemporary Set true to temporarily enable broadcast - subsequent call to DisableBroadcast(false) will restore previous (or subsequently configured) state
        *   @note   This will increase load on recieved data handling
        *   @note   bTemporary may be used to enable broadcasts where required for just one session without affecting others, e.g. awaiting a DHCP response
        */
        void EnableBroadcast(bool bTemporary = false);

        /** @brief  Disable reception of broadcast messages
        *   @param  bTemporary Set true to disable only if temporarily enabled
        *   @note   This will reduce load on recieved data handling
        */
        void DisableBroadcast(bool bTemporary = false);

        /** @brief  Enables reception of mulitcast messages
        *   @note   This will increase load on recieved data handling
        */
        void EnableMulticast ();

        /** @brief  Disable reception of mulitcast messages
        *   @note   This will reduce load on recieved data handling
        */
        void DisableMulticast();

        /** @brief  Enables pattern match filter
        *   @param  nOffset Offset of start of pattern matching, starting from first octet of destination address, i.e. start of Ethernet packet
        *   @param  nChecksum Checksum of the 64 byte pattern to match.
        *   @param  nMask 64-bit mask. Set a bit to enable matching of corresponding octet Default is to match all 64 bytes
        *   @note   The pattern match filter selects up to 64 bytes from the incoming packet and calculates an IP checksum of the bytes. If this does not match the configured checksum, the packet is silently discarded.
        *   @note   The pattern match filter may be useful for filtering packets which have expected data inside them.
        */
        void EnablePatternMatch(uint16_t nOffset, uint16_t nChecksum,  uint64_t nMask = 0xFFFFFFFF);

        /** @brief  Disables pattern match filter
        */
        void DisablePatternMatch();

        /** @brief  Enable the hash table match filter
        *   @param  nHashFlags 64-bit hash flags
        *   @note   The hash table receive filter performs a CRC over the six destination address bytes in the packet.
        *   @note   The CRC is then used as a pointer into the bits of the configured 64-bit flag.
        *   @note   If the pointer points to a bit which is clear, the packet will be silenetly dropped (ignored).
        *   @note   For example, if the CRC is calculated to be 0x5, bit 5 in the hash table willbe checked. If it is set, the hash table filter criteria will be met and the packet processed.
        *   @note   If every bit is clear in the hash table, the filter criteria will never be met. Similarly, if every bit is set inthe hash table, the filter criteria will always be met.
        */
        void EnableHashFilter(uint64_t nHashFlags);

        /** @brief  Disable hash table filter
        */
        void DisableHashFilter();

        /** @brief  Enable the Magic Packet(TM) match filter
        *   @param  pAddress Pointer to 6 byte destination (MAC) address identifying Magic Packet(TM)
        */
        void EnableMagicPacket(byte* pAddress);

        /** @brief  Disables the Magic Packet(TM) match filter
        */
        void DisableMagicPacket();

        //!@todo Add filters for hash table, unicast?, Magic Packet

        /** @brief  Perform built in self test
        *   @param  nTest Bitwise flag specifying which tests to perform: ENC29J60_BIST_RDFM | ENC29J60_BIST_ENC29J60_BIST_RDFM_RACE | ENC29J60_BIST_AFM | ENC29J60_BIST_PSFM
        *   @return <i>bool</i> True on success
        *   @note   Default is perform all tests
        */
        bool BIST(byte nTest = ENC28J60_BIST_RDFM | ENC28J60_BIST_RDFM_RACE | ENC28J60_BIST_AFM | ENC28J60_BIST_PSFM);

        //Reception functions
        /** @brief  Start handling next received packet
        *   @return <i>int16_t</i> Size of packet. Zero if new packet not available. -1 if error recieving packet
        *   @note   Call GetRxStatus to find error
        *   @note   Call RxEnd to finish processing recieved packet (and release recieve buffer space)
        *   @note   Calling RxBegin will end current packet processing
        */
        int16_t RxBegin();

        /** @brief  Finishes handling recieved packet
        *   @note   This call frees the space within the recieve buffer used by the packet
        */
        void RxEnd();

        /** @brief  Gets data from recieve buffer
        *   @param  pBuffer Pointer to a buffer to hold data
        *   @param  nLen Maximum quantity of bytes to read
        *   @return <i>uint16_t</i> Quantity of bytes read
        *   @note   Advances ERDPT read pointer position by nymber of bytes read
        */
        uint16_t RxGetData(byte* pBuffer, uint16_t nLen);

        /** @brief  Gets data from recieve buffer
        *   @param  pBuffer Pointer to a buffer to hold data
        *   @param  nLen Maximum quantity of bytes to read
        *   @param  nOffset Position within packet to read
        *   @return <i>uint16_t</i> Quantity of bytes read
        *   @note   Advances ERDPT read pointer position by nymber of bytes read
        */
        uint16_t RxGetData(byte* pBuffer, uint16_t nLen, uint16_t nOffset);

        /** @brief  Gets the size of the recieved packet
        *   @return <i>uint16_t</i> Quantity of bytes in recieved packet
        *   @note   ERDPT read pointer set to start of Ethernet packet (destination address)
        */
        uint16_t GetRxPacketSize();

        /** @brief  Get status of recieved packet
        *   @return <i>uint16_t</i> Status represented as 32 bit flags
        *   @note   Flags represented by constants prefixed with ENC28J60_RX_
        *   @todo   Link to flag documentation
        */
        uint16_t RxGetStatus();

        //Transmission functions
        /** @brief  Starts a transmission transaction
        *   @note   Call TxAppend to append data to the transmission transaction
        *   @note   Call TxEnd to close transaction and send packet
        */
        void TxBegin();

        /** @brief  Ends a transmission transaction and sends packet
        *   @note   Call TxBegin to start transaction
        *   @note   Call TxAppend to append data to the transmission transaction
        */
        void TxEnd();

        /** @brief  Appends data to a transmission transaction
        *   @param  pData Pointer to data to append
        *   @param  nLen Quantity of bytes to append
        *   @note   Call TxBegin before appending data
        *   @note   Call TxEnd to complete transaction and send packet
        */
        void TxAppend(byte* pData, uint16_t nLen);

        /** @brief  Check if last transmission was successful
        *   @return <i>byte</i> Result: ENC28J60_TX_SUCCESS | ENC28J60_TX_IN_PROGRESS | ENC28J60_TX_FAILED
        */
        byte TxGetStatus();

        /** @brief  Clears all transmit error flags
        */
        void TxClearError();

    private:
        //SPI functions
        /** @brief  Initialise SPI interface
        *   @note   Configures Arduino pins as input / output, configures SPI interface, etc.
        */
        void SPIInit ();

        /** @brief  Transfer one byte of data to and from interface
        *   @param  nData Value to transfer to interface. Optional - omit to just recieve data
        *   @return <i>byte</i> Value recieved from interface
        *   @note   Combines send and recieve in to single function (to reduce program memory)
        */
        byte SPITransfer(byte nData = 0);

        /** @brief  Read control register from SPI
        *   @param  nRegister Control register to read
        *   @return <i>byte</i> Value read from interface
        */
        byte SPIReadRegister(byte nRegister);

        /** @brief  Read buffer memory from SPI
        *   @param  pData Pointer to buffer to populate with read data
        *   @param  nLen Quantity of bytes to read
        */
        void SPIReadBuf(byte* pData, uint16_t nLen);

        /** @brief  Write to SPI control register
        *   @param  nRegister Register to write value to
        *   @param  nData Value to write to register
        */
        void SPIWriteReg(byte nRegister, byte nData);

        /** @brief  Write to SPI buffer memory
        *   @param  pData Pointer to data buffer to write
        *   @param  nLen Quantity of bytes to write
        */
        void SPIWriteBuf(byte* pData, uint16_t nLen);

        /** @brief  Set bit(s) in SPI control register
        *   @param  nRegister Register to modify
        *   @param  nBits Bits to set
        *   @note   nBits is logically ORed with content of register
        *   @note   May only be used for Ethernet registers <b>not</b> MAC or MII registers
        */
        void SPISetBit(byte nRegister, byte nBits);

        /** @brief  Clear bit in SPI control register
        *   @param  nRegister Register to modify
        *   @param  nBits Bits to clear
        *   @note   nBits is logically NOTANDed with the content of register
        *   @note   May only be used for Ethernet registers <b>not</b> MAC or MII registers
        */
        void SPIClearBit(byte nRegister, byte nBits);

        /** @brief  Reset NIC
        */
        void SPIReset();

        /** @brief  Enable interface chip
        */
        void EnableChip();

        /** @brief  Disable interface chip
        */
        void DisableChip();

        /** @brief  Set bank in ENC28J60
        *   @param  nAddress Bank address
        */
        void SetBank(byte nAddress);

        /** @brief  Read byte from register
        *   @param  nAddress Register address
        *   @return <i>byte</i> Register value
        */
        byte ReadRegByte(byte nAddress);

        /** @brief  Read 16-bit word from register
        *   @param  nAddress Register address
        *   @return <i>uint16_t</i> Register value
        */
        uint16_t ReadRegWord(byte nAddress);

        /** @brief  Write 8-bit byte to register
        *   @param  nAddress register address
        *   @param  nData Register value
        */
        void WriteRegByte(byte nAddress, byte nData);

        /** @brief  Write 16-bit word to register
        *   @param  nAddress register address
        *   @param  nData Register value
        */
        void WriteRegWord(byte nAddress, uint16_t nData);

        /** @brief  Read value from PHY register
        *   @param  nAddress Register address
        *   @return <i>uint16_t</i> Register value
        */
        uint16_t ReadPhyWord(byte nAddress);

        /** @brief  Write value to PHY register
        *   @param  nAddress Register address
        *   @param  nData Register value
        */
        void WritePhyWord(byte nAddress, uint16_t nData);

        bool m_bBroadcastEnabled; //!< True if broadcasts enabled (used to allow temporary disable of broadcast for DHCP or other internal functions)
        byte m_nSelectPin; //!< Arduino pin used as chip select
        byte m_nBank; //!< Currently selected register bank (stored in most significant 3 bits)
        uint16_t m_nTxLen; //!< Size of transmission packet
        uint16_t m_nRxPacketPtr; //!< Pointer to (address of) the current Rx packet in ENC28J60 ring buffer
        uint16_t m_nRxOffset; //!< Pointer to postion within recieved packet used for random access
        uint16_t m_nRxSize; //!< Quantity of bytes in recieve buffer
        ENC28J60_RX_HEADER m_rxHeader; //!< Details of current recieve packet

};
