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
  No netowrk stack - all network interfacing must be provided by program code (such as this library) using valuable program memory.
  Uses more memory - due to lack of built-in network stack code, program code may use more SRAM.
  
I bought an ENC28J60 based board from ebay for approx. £3 because it was cheaper than other boards but then found it more awkward to program, hence this library.
In hindsight, I may have bought the slightly more expensive W5100 based board from ebay for approx. £7.50 and save lots of time. You may benefit from my effort though!

This library is derived from code originally written by Pascal Stang which has been through several developers. I re-worked much of it to the point that it is unlikely to fit in to a project based on the original code.
