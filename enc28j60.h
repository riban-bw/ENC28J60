// Microchip ENC28J60 Ethernet Interface Driver
// Original Author: Pascal Stang
// Modified by: Guido Socher, Brian Walton
// Copyright: GPL V2
//
// This driver provides initialization and transmit/receive
// functions for the Microchip ENC28J60 10Mb Ethernet Controller and PHY.
// This chip is novel in that it is a full MAC+PHY interface all in a 28-pin
// chip, using an SPI interface to the host processor.
//
// Based on documentation from Microchip:
//   39662e - Stand-Alone Ethernet Controller with SPI Interface
//   80349c - ENC28J60 Silicon Errata and Data Sheet Clarification (B7)
//
// 2010-05-20 <jc@wippler.nl>
// 2014-10-17 <brian@riban.co.uk>

//!@todo Add version to doxygen
//!@todo Add ability to configure and use interupts

/** @note   ENC28J60 memory allocation is made thus:
*               0x0000 .. 0x1A0C Packet recieve circular buffer (RX_BUFFER_START .. TX_BUFFER_START - 1)
*               0x1A0C .. 0x1FFF Transmission buffer (TX_BUFFER_START .. TX_BUFFER_END)
*                   ETXST = 0x1A0E = Per packet control byte (should be an even address)
*                           0x1A0F = First byte of packet data
*                   ETXND points to last byte of packet        (Max = 0x1FF8)
*                   EXTND + 1 = Transmit Status Vector [7:0]   (0x1FF9)
*                   EXTND + 2 = Transmit Status Vector [15:8]  (0x1FFA)
*                   EXTND + 3 = Transmit Status Vector [23:16] (0x1FFB)
*                   EXTND + 4 = Transmit Status Vector [31:24] (0x1FFC)
*                   EXTND + 5 = Transmit Status Vector [39:32] (0x1FFD)
*                   EXTND + 6 = Transmit Status Vector [47:40] (0x1FFE)
*                   EXTND + 7 = Transmit Status Vector [55:48] (0x1FFF)
*/

#pragma once

// Tx status
//!@todo move this to abstract Ethernet class
static const byte ENC28J60_TX_SUCCESS       = 0x00;
static const byte ENC28J60_TX_IN_PROGRESS   = 0x01;
static const byte ENC28J60_TX_FAILED        = 0x02;
static const int16_t ENC28J60_RX_ERROR      = -1; //!< Indicates error receiving packet
//Error flags
static const byte ENC28J60_TXERROR_CRC          = 0x01; //!< Error bit asserted if last Tx packet had CRC error
static const byte ENC28J60_TXERROR_LEN          = 0x02; //!< Error bit asserted if last Tx packet had length error
static const byte ENC28J60_TXERROR_SIZE         = 0x04; //!< Error bit asserted if last Tx packet data was > 1500 bytes
static const byte ENC28J60_TXERROR_DEFER        = 0x08; //!< Error bit asserted if last Tx packet was deferred
static const byte ENC28J60_TXERROR_EXCESS_DEFER = 0x10; //!< Error bit asserted if last Tx packet was deferred in excess of 24,287 bit times (2.4287 ms)
static const byte ENC28J60_TXERROR_COLL         = 0x20; //!< Error bit asserted if last Tx packet was aborted after number of collisions exceeded retransmission maximum
static const byte ENC28J60_TXERROR_LATE_COLL    = 0x40; //!< Error bit asserted if last Tx packet collision occurred beyond the collision window
static const byte ENC28J60_TXERROR_GIANT        = 0x80; //!< Error bit asserted if last Tx packet frame byte count was greater than maximum frame size
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

/** @brief  Describes details of last recieved packet */
struct ENC28J60_RX_HEADER
{
    uint16_t nNextPacket;   //!< Pointer to the position of first byte in next recieved packet
    uint16_t nSize;         //!< Quantity of bytes in this packet
    uint16_t nStatus;       //!< Error status of last packet reception - see ENC28J60_RX_ range of constant values, e.g. ENC28J60_RX_OK
};

/** @brief  This class provide low-level interfacing with the ENC28J60 network interface.*/
class ENC28J60
{
    public:
        /**   @brief  Initialise network interface
        *     @param  macaddr Pointer to 6 byte hardware (MAC) address
        *     @param  nChipSelectPin Arduino pin used for chip select (enable network interface SPI bus). Default = 10
        *     @return <i>uint8_t</i> ENC28J60 firmware version or zero on failure.
        */
        byte Initialize(const byte* pMac, byte nChipSelectPin = 10);

        /** @brief  Reset NIC
        *   @note   Will power up NIC if in powersave mode
        */
        void Reset();

        /** @brief  Wake ENC28J60 from sleep mode
        *   @note   This also enables packet reception
        */
        void PowerUp();    // contrib by Alex M.

        /** @brief  Put ENC28J60 in sleep mode
        */
        void PowerDown();  // contrib by Alex M.

        /** @brief  Enable reception of packets
        */
        void EnableReception();

        /** @brief  Disable reception of packets
        *   @return <i>bool</i> True if state changed
        */
        bool DisableReception();

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
        void EnableMulticast();

        /** @brief  Disable reception of mulitcast messages
        *   @note   This will reduce load on recieved data handling
        */
        void DisableMulticast();

        /** @brief  Enables pattern match filter
        *   @param  nOffset Offset of start of pattern matching, starting from first octet of destination address, i.e. start of Ethernet packet. Must be even (see note)
        *   @param  nChecksum Checksum of the 64 byte pattern to match.
        *   @param  nMask 64-bit mask. Set a bit to enable matching of corresponding octet Default is to match all 64 bytes
        *   @note   The pattern match filter selects up to 64 bytes from the incoming packet and calculates an IP checksum of the bytes. If this does not match the configured checksum, the packet is silently discarded.
        *   @note   The pattern match filter may be useful for filtering packets which have expected data inside them.
        *   @note   nOffset must be even. To ensure this, the least significant bit is cleared which may result in searching from one octet lower than specified.
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

        /** @brief  Sets duplex to full
        *   @note   ENC28J60 does not support auto negotiate and presents as half duplex. Both ends of link must be manually configured to use full duplex.
        */
        void SetFullDuplex();

        /** @brief  Sets duplex to half
        */
        void SetHalfDuplex();

        /** @brief  Enable flow control
        */
        void EnableFlowControl();

        /** @brief  Disable flow control
        */
        void DisableFlowControl();

        /**   @brief  Check if network link is connected
        *     @return <i>bool</i> True if link is up
        */
        bool IsLinkUp();

        /** @brief  Sets LED mode
        *   @param  nMode LED mode
        */
        void SetLedMode(uint16_t nMode);

        //Reception functions
        /** @brief  Start handling next received packet
        *   @return <i>int16_t</i> Size of packet. Zero if new packet not available. ENC28J60_RX_ERROR if error recieving packet
        *   @note   Call GetRxStatus to find error
        *   @note   Call RxEnd to finish processing recieved packet (and release recieve buffer space)
        *   @note   Calling RxBegin will end current packet processing
        *   @note   ENC28J60 pads packet to minimum packet size of 60.
        */
        int16_t RxBegin();

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
        *   @note   Advances ERDPT read pointer position by nymber of bytes read beyond nOffset
        */
        uint16_t RxGetData(byte* pBuffer, uint16_t nLen, uint16_t nOffset);

        /** @brief  Gets one byte from recieve buffer
        *   @return <i>byte</i> Value
        *   @note   Advances ERDPT read pointer position by one
        */
        byte RxGetByte();

        /** @brief  Gets one byte from specific location within recieve buffer
        *   @param  nOffset Position within buffer to read byte
        *   @return <i>byte</i> Value
        *   @note   Advances ERDPT read pointer to immediately after byte read
        */
        byte RxGetByte(uint16_t nOffset);

        /** @brief  Gets 16-bit word from recieve buffer
        *   @return <i>uint16_t</i> 16-bit value in host byte order
        *   @note   Advances ERDPT read pointer position by two
        */
        uint16_t RxGetWord();

        /** @brief  Gets 16-bit word from specific location within recieve buffer
        *   @param  nOffset Postion within buffer to read word
        *   @return <i>uint16_t</i> 16-bit value in host byte order
        *   @note   Advances ERDPT read pointer to immediately after word read
        */
        uint16_t RxGetWord(uint16_t nOffset);

        /** @brief  Finishes handling recieved packet
        *   @note   This call frees the space within the recieve buffer used by the packet
        */
        void RxEnd();

        /** @brief  Copy recieved packet to data buffer
        *   @param  pBuffer Pointer to a buffer to recieve data
        *   @param  nSize Maximum quantity of bytes to read
        *   @return <i>uint16_t</i> Quantity of bytes actually copied
        *   @note   Use this function to handle whole packets (with suitable sized buffer). Use Rx transaction functions to access NIC recieve buffer directly
        */
        uint16_t PacketReceive(byte* pBuffer, uint16_t nSize);

        /** @brief  Gets the size of the recieved packet
        *   @return <i>uint16_t</i> Quantity of bytes in recieved packet
        *   @note   ERDPT read pointer set to start of Ethernet packet (destination address)
        */
        uint16_t RxGetPacketSize();

        /** @brief  Get status of recieved packet
        *   @return <i>uint16_t</i> Status represented as 32 bit flags
        *   @note   Flags represented by constants prefixed with ENC28J60_RX_
        *   @note   ENC28J60 pads packet to minimum packet size of 60.
        *   @todo   Link to flag documentation
        */
        uint16_t RxGetStatus();

        /** @brief  Checks for recieved broadcast packet
        *   @return <i>bool</i> True if last recieved packet was a broadcast packet
        */
        bool RxIsBroadcast();

        /** @brief  Checks for recieved multicast packet
        *   @return <i>bool</i> True if last recieved packet was a multicast packet
        */
        bool RxIsMulticast();

        /** @brief  Get the quantity of unprocessed received packets
        *   @return <i>byte</i> Quantity of packets
        */
        byte RxGetPacketCount();

        /** @brief  Get the unused space in the recieve buffer
        *   @return <i>uint16_t</i> Quantity of unused bytes
        */
        uint16_t RxGetFreeSpace();

        //Transmission functions
        /** @brief  Starts a transmission transaction
        *   @param  pMac Optional pointer to byte array representing remote host MAC address. Default is broadcast address FF:FF:FF:FF:FF:FF
        *   @param  nEthertype Optional Ethertype or length of this Ethernet packet. Default is 0x0800 (IPV4)
        *   @note   Call TxAppend to append data to the transmission transaction
        *   @note   Call TxEnd to close transaction and send packet
        *   @note   Tx write pointer points to first byte of Ethernet payload
        */
        void TxBegin(byte* pMac = NULL, uint16_t nEthertype = 0x0800);

        /** @brief  Appends data to a transmission transaction
        *   @param  pData Pointer to data to append
        *   @param  nLen Quantity of bytes to append
        *   @return <i>bool</i> True on success. False if insufficient space left in Tx buffer
        *   @note   Call TxBegin before appending data
        *   @note   Call TxEnd to complete transaction and send packet
        */
        bool TxAppend(byte* pData, uint16_t nLen);

        /** @brief  Appends a single byte to a transmission transaction
        *   @param  nData Value to append
        *   @return <i>bool</i> True on success. False if insufficient space left in Tx buffer
        */
        bool TxAppendByte(byte nData);

        /** @brief  Appends a 16-bit word to a transmission transaction
        *   @param  nData Value to append
        *   @return <i>bool</i> True on success. False if insufficient space left in Tx buffer
        */
        bool TxAppendWord(uint16_t nData);

        /** @brief  Write data to specific position in write buffer
        *   @param  nOffset Position offset from start of Tx buffer, i.e. 0=first byte of buffer
        *   @param  pData Pointer to data to be written
        *   @param  nLen Quantity of bytes to write to buffer
        *   @note   Leaves append buffer cursor and Tx packet size counter unchanged.
        *   @note   Offset is relative to start of packet frame data, not the 'per packet control byte' used by the ENC28J60
        */
        void TxWrite(uint16_t nOffset, byte* pData, uint16_t nLen);

        /** @brief  Write a single byte to specific position in write buffer
        *   @param  nOffset Position offset from start of Tx buffer, i.e. 0=first byte of buffer
        *   @param  nData Data to write
        *   @note   Leaves append buffer cursor and Tx packet size counter unchanged.
        *   @note   Offset is relative to start of packet frame data, not the 'per packet control byte' used by the ENC28J60
        *   @todo   Add unit test for TxWriteByte
        */
        void TxWriteByte(uint16_t nOffset, byte nData);

        /** @brief  Write a two byte word to specific position in write buffer
        *   @param  nOffset Position offset from start of Tx buffer, i.e. 0=first byte of buffer
        *   @param  nData Data to write
        *   @note   nData is host byte order, word is written network byte order, i.e. bytes are swapped before writing to buffer
        *   @note   Leaves append buffer cursor and Tx packet size counter unchanged.
        *   @note   Offset is relative to start of packet frame data, not the 'per packet control byte' used by the ENC28J60
        *   @todo   Add unit test for TxWriteWord
        */
        void TxWriteWord(uint16_t nOffset, uint16_t nData);

        /** @brief  Swap two ranges of bytes in TxBuffer
        *   @param  nOffset1 Position of first range
        *   @param  nOffset2 Position of second range
        *   @param  nLen Quantity of bytes to swap
        *   @todo   Add unit test for TxSwap
        */
        void TxSwap(uint16_t nOffset1, uint16_t nOffset2, uint16_t nLen);

        /** @brief  Check if last transmission was successful
        *   @return <i>byte</i> Result: ENC28J60_TX_SUCCESS | ENC28J60_TX_IN_PROGRESS | ENC28J60_TX_FAILED
        */
        byte TxGetStatus();

        //!@todo Add function to get individual TxStatus
        /** @brief  Get error flags
        *   @return <i>byte</i> Bitwise flags
        */
        byte TxGetError();

        /** @brief  Clears all transmit error flags
        */
        void TxClearError();

        /** @brief  Get the size of the Tx packet / frame
        *   @return <i>uint16_t</i>
        */
        uint16_t TxGetSize();

        /** @brief  Ends a transmission transaction and sends packet
        *   @param  nLen Optional quantity of bytes in packet. Default is to use TxAppend pointer
        *   @note   Call TxBegin to start transaction
        *   @note   Call TxAppend to append data to the transmission transaction
        */
        void TxEnd(uint16_t nLen = 0);

        /** @brief  Sends whole packet of data
        *   @param  pBuffer Pointer to the data buffer
        *   @param  nLen Quantity of bytes of data to send
        *   @return <i>bool</i> True on success. Will fail if too much data for single packet.
        *   @note   Only supports transmitting one packet at a time. Blocks until previous packet transmission is complete.
        *   @note   Return value does not guarantee successful transmission, only that the data was transfered to Tx buffer and Tx requested to start.
        *   @todo   Should we return true on success? Should be consistent with other functions.
        */
        bool PacketSend(byte* pBuffer, uint16_t nLen);

        //Misc functions
        /** @brief  Performs driect memory access transfer of data from Rx buffer to Tx buffer
        *   @param  nDestination Offset in Tx buffer of first byte to write
        *   @param  nStart Offset in Rx buffer of first byte to copy
        *   @param  nLen Quantity of bytes to copy
        *   @note   After the DMA module has been initialized and has begun its copy, two main ENC28J60 clock cycles will be required for each byte copied. As a result, if a maximum size 1518-byte packet was copied, the DMA module would require slightly more than 121.44us to complete. The time required to copy a minimum size packet of 64 bytes would be dominated by the time required to configure the DMA.
        */
        void DMACopy(uint16_t nDestination, uint16_t nStart, uint16_t nLen);

        /** @brief  Swaps the MSB and LSB of a 16-bit integer
        *   @param  nValue 16-bit integer value
        *   @return <i>uint16_t</i> Modified integer
        *   @note   Converts 16-bit integer between host byte order and network byte order (either direction)
        *   @note   Same as htons or ntohs
        */
        static uint16_t SwapBytes(uint16_t nValue);

        /** @brief  Get the MAC address from the NIC
        *   @param  pMac Pointer to 6 byte array
        */
        void GetMac(byte* pMac);

        /** @brief  Calculate a checksum of a range of the Tx buffer
        *   @param  nStart Position of first byte of Tx buffer to checksum
        *   @param  nLength Quantity of bytes to checksum
        *   @return <i>uint16_t</i> Resulting checksum value in network byte order
        */
        uint16_t GetChecksum(uint16_t nStart, uint16_t nLength);

        /** @brief  Perform built in self test
        *   @param  nTest Bitwise flag specifying which tests to perform: ENC29J60_BIST_RDFM | ENC29J60_BIST_ENC29J60_BIST_RDFM_RACE | ENC29J60_BIST_AFM | ENC29J60_BIST_PSFM
        *   @return <i>bool</i> True on success
        *   @note   Default is perform all tests
        */
        bool BIST(byte nTest = ENC28J60_BIST_RDFM | ENC28J60_BIST_RDFM_RACE | ENC28J60_BIST_AFM | ENC28J60_BIST_PSFM);

    protected:
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

        /** @brief  Set bank in ENC28J60
        *   @param  nAddress Bank encoded in bits 5-6 of address
        */
        void SPISetBank(byte nAddress);

        /** @brief  Read control register from SPI
        *   @param  nRegister Control register to read coded as: MII/MAC flag[7], Bank[5-6], register[0-4]
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

        /** @brief  Set bits in SPI control register
        *   @param  nRegister Register to modify
        *   @param  nBits Bits to set
        *   @note   nBits is logically ORed with content of register
        *   @note   May only be used for Ethernet registers <b>not</b> MAC or MII registers
        */
        void SPISetBits(byte nRegister, byte nBits);

        /** @brief  Clear bits in SPI control register
        *   @param  nRegister Register to modify
        *   @param  nBits Bits to clear
        *   @note   nBits is logically NOTANDed with the content of register
        *   @note   May only be used for Ethernet registers <b>not</b> MAC or MII registers
        */
        void SPIClearBits(byte nRegister, byte nBits);

        //Register manipulation
        /** @brief  Read 16-bit word from register
        *   @param  nAddress Register address
        *   @return <i>uint16_t</i> Register value
        */
        uint16_t ReadRegWord(byte nAddress);

        /** @brief  Write 16-bit word to register
        *   @param  nAddress Register address
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

        /** @brief  Enable interface chip
        */
        void EnableChip();

        /** @brief  Disable interface chip
        */
        void DisableChip();


        bool m_bBroadcastEnabled; //!< True if broadcasts enabled (used to allow temporary disable of broadcast for DHCP or other internal functions)
        bool m_bFlowControl; //!< True if flow control used to limit reception rate
        bool m_bRxPause; //!< True if flow control pause asserted
        byte m_nSelectPin; //!< Arduino pin used as chip select
        byte m_nBank; //!< Currently selected register bank (stored in most significant 3 bits)
        uint16_t m_nTxLen; //!< Size of transmission packet
        uint16_t m_nRxPacketPtr; //!< Pointer to (address of) the current Rx packet in ENC28J60 ring buffer
        uint16_t m_nRxOffset; //!< Pointer to postion within recieved packet used for random access
        uint16_t m_nRxSize; //!< Quantity of bytes in recieve buffer
        ENC28J60_RX_HEADER m_rxHeader; //!< Details of current recieve packet
};
