ENC28J60
========

Arduino library for ENC28J60 chipset

This library provides low-level control of the Microchip ENC28J60 network interface chip. This library does not provide the higher level access a programmer typically wants. That is implemented in another library (see https://github.com/riban-bw/ribanEthernet).

Before considering buying a ENC28J60 based network interface board, consider the pros and cons of this against another board such as the Wiznet W5100.

Pros:
  Cheaper (approx. 1/3 cost of W5100 based).
  Smaller (most W5100 boards are Uno shield type whereas ENC28J60 may be physically smaller).
  
Cons:
  Smaller buffer memory - more prone to packet loss during bursts of network traffic.
  No network stack - all network interfacing must be provided by program code (such as this library plus ipv4 library, e.g. ribanEthernet) using valuable program memory.
  Uses more memory - due to lack of built-in network stack code, program code may use more SRAM.
  
I bought an ENC28J60 based board from ebay for approx. £3 because it was cheaper than other boards but then found it more awkward to program, hence this library.
In hindsight, I may have bought the slightly more expensive W5100 based board from ebay for approx. £7.50 and save lots of time. You may benefit from my effort though!

This library is derived from code originally written by Pascal Stang which has been through several developers. I re-worked much of it to the point that it is unlikely to fit in to a project based on the original code.

There are other libriaries for the ENC28J60 including the ethercard library available from <http://jeelabs.net/projects/ethercard/wiki>. These may provide IP stack implementation. I chose to develop this library to provide the following features which were not (all) available from any of the existing libraries (that I could find):

* Abstract hardware driver code from network protocol (see ribanEthernet for such protocols)
* Implement as many features as the ENC28J60 chip supports
* Separate functions to allow smaller application code (assuming compiler optimises for size, excluding unused functions)
* Access ENC28J60 memory directly, reducing size of application buffers (microcontrollers generally have less SRAM than that required to process Ethernet packets of up to 1518 octets)
* Structure code in a familiar (to me) style to ease support

The ENC28J60 has 8KB memory shared between packet recieve and transmit. This library assigns 1518 bytes to the transmit buffer, enough for one packet (max 1518 bytes) and associated ENC28H60 header. This leaves 6668 bytes for the circular recieve buffer, enough for at least 4 packets (more if shorter packets).

Access to recieve and transmit buffers is provided by a transaction style interface. The application should start access by calling the Begin function and complete access by calling the End function.

To send a packet:

* Call TxBegin() which waits for any outstanding transmision to complete then resets the transmission registers and buffer, ready for the next transmission.
* Call TxAppend() to append data to the transmission buffer. Check result is false which means there is sufficient space for data in Tx buffer. Each call will append data to the buffer, allowing packet data to be defined by a single call or multiple calls, effectively building up the packet. The latter allows for abstraction or segmented programming, e.g. provide Ethernet header, provide IP header, provide IP payload, etc.
* Call TxEnd() which starts the transmission of the packet.

There is a helper function, PacketSend() which performs each transmission transaction step for a single packet buffer, i.e. if the calling application has constructed the whole packet in a byte buffer, it may call PacketSend.

To recieve a packet:

* Call RxBegin() which checks if there is a new packet received (since last transaction) and returns the size of the new packet, zero if no packet available and ENC28J60_RX_ERROR if an error occured receiving packet. Any previous receive transaction is aborted and unprocessed data is lost.
* Call RxGetData to receive chunks of data. This may be called several times, allowing abstraction or semented programming, e.g. get Ethernet header, get IP header, get IP payload, etc. Two forms of the function call are availble, providing consecutive data access and random data access, i.e. the calling application may request consecutive data chunks or specify the start position for the data it requires.
* Call RxEnd() to complete the recieve transaction, freeing the space used by this received packet back to the circular receive buffer.

Note: Transactional access to the recieve buffers should be peformed as rapidly and regularly as possible to reduce the risk of packet loss. If there is insufficient space in the circular recieve buffer, packets will be dropped (ignored and permanently lost). The application may check for dropped packets by calling RxGetStatus(). If full duplex is enabled (default) then a connected network switch may buffer packets which may reduce packet loss. The ENC29J60 will buffer several packets but sustained network traffic above the rate it is being processed will result in packet loss.

