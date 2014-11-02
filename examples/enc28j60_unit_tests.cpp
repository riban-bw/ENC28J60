/*  This code provides a menu driven set of unit tests.
    Its purpose is to allow testing of each feature of the ENC28J80 library
*/

#include "Arduino.h"
#include "../enc28j60.h"

static const byte ETHERNET_CS_PIN   = 10;

static const byte TEST_NONE         = -1;
static const byte TEST_MENU         = ' ';
static const byte TEST_LED          = '0';
static const byte TEST_LINK         = '1';
static const byte TEST_RX_PACKET    = '2';
static const byte TEST_TX_MIN_PACKET    = '3';
static const byte TEST_TX_MAX_PACKET    = '4';
static const byte TEST_DUPLEX       = '5';
static const byte TEST_BIST         = '6';
static const byte TEST_FREE         = '7';
static const byte TEST_CRC          = 'c';
static const byte TEST_DMA          = 'd';
static const byte TEST_RESET        = 'r';
static const byte TEST_POWER        = 'p';
static const byte TEST_UNICAST      = 'u';
static const byte TEST_MULTICAST    = 'm';
static const byte TEST_BROADCAST    = 'b';
static const byte TEST_FLOW         = 'f';
static const byte TEST_INIT         = 'i';


static const uint16_t LED_MODE_NORMAL   = 0x3422;
static const uint16_t LED_MODE_TEST     = 0x0AB0;

byte pMac[6] = {0x12,0x34,0x56,0x78,0x9A,0xBC};

ENC28J60 nic; //!< Instance of the NIC
byte nTest; //!< Current test
byte nRevision; //!< ENC28J60 firmware revision. 0 = not connected.
bool bLinkStatus; //!< Link status
bool bPower; //!< Power status
bool bDuplex; //!< True for full duplex
bool bUnicast; //!< True to enable unicast reception
bool bMulticast; //!< True to enable multicast reception
bool bBroadcast; //!< True to enable broadcast reception
bool bFlow; //!< True if flow control enabled

/////** Show the content of the ENC28J60 registers */
//void DumpRegisters()
//{
//    Serial.println(" - Dump registers:");
//    for(byte nRegister = 0; nRegister < 0x20; ++nRegister)
//    {
//        if(0x1A == nRegister)
//            continue; //Do not access reserved registers
//        Serial.print("0x");
//        if(nRegister < 0x10)
//            Serial.print("0");
//        Serial.print(nRegister, HEX);
//        Serial.print(": ");
//        for(byte nBank = 0; nBank < 3; ++nBank)
//        {
//            Serial.print("\t");
//            byte nAddress, nResult;
//            if(nBank == 2 || (nBank == 3 && (nRegister < 0x06 || nRegister == 0x0A)))
//                nAddress = nRegister | (nBank * 0x20) | 0x80; //M register
//            else
//                nAddress = nRegister | (nBank * 0x20);
//            nResult = nic.ReadRegByte(nAddress);
//            Serial.print("[");
//            if(nAddress < 0x10)
//                Serial.print("0x0");
//            else
//                Serial.print("0x");
//            Serial.print(nAddress, HEX);
//            Serial.print("] ");
//            if(nResult < 0x10)
//                Serial.print("0x0");
//            else
//                Serial.print("0x");
//            Serial.print(nResult, HEX);
//        }
//        Serial.println();
//    }
//    Serial.println();
//
//}

/** Display the menu */
void ShowMenu()
{
    Serial.println(F("Tests for [12:34:56:78:9A:BC]"));
    Serial.println(F("SPACE shows menu and clears test mode"));
    Serial.println(F("0. Toggle LEDs"));
    Serial.println(F("1. Show link status"));
    Serial.println(F("2. Recieve packets"));
    Serial.println(F("3. *Send minimal packet"));
    Serial.println(F("4. *Send maximum packet"));
    Serial.println(F("5. *Toggle duplex"));
    Serial.println(F("6. *Built-in self tests"));
    Serial.println(F("7. *Show free Rx buffer space"));
    Serial.println(F("c. *CRC test"));
    Serial.println(F("d. *DMA test"));
    Serial.println(F("r. *Reset NIC"));
    Serial.println(F("p. *Toggle power"));
    Serial.println(F("u. *Toggle unicast reception"));
    Serial.println(F("m. *Toggle multicast reception"));
    Serial.println(F("b. *Toggle broadcast reception"));
    Serial.println(F("f. *Toggle flow control"));
    Serial.println(F("i. *Initialise"));
    Serial.println(F("* Does not change current test mode"));
}

/** Initialise NIC */
void initialse()
{
    nTest = TEST_NONE;
    bFlow = true;
    bBroadcast = true;
    bMulticast = false;
    bUnicast = true;
    bDuplex = false;
    nRevision = nic.Initialize(pMac, ETHERNET_CS_PIN);
    if(nRevision)
    {
        nic.DisableReception();
        Serial.print("Initialise NIC. Firmware version ");
        Serial.println(nRevision);
        ShowMenu();
    }
    else
        Serial.println("!!!NIC Initialisation error - check SPI and CS connections then press 'i'");
}

/** Show the last detected link status */
void ShowLinkStatus()
{
    Serial.print("Link is ");
    if(bLinkStatus)
        Serial.println("up");
    else
        Serial.println("down");
}

/** Initialisation */
void setup()
{
    Serial.begin(9600);
    Serial.println(F("ENC28J60 unit tests"));
    initialse();
}

/** Main program loop */
void loop()
{
    int nInput = Serial.read();
    if(nInput == TEST_INIT)
        initialse();
    if(nRevision && TEST_NONE != nInput)
    {
        switch(nInput)
        {
            case TEST_MENU:
                nTest = TEST_NONE;
                ShowMenu();
                break;
            case TEST_LED:
                if(TEST_LED == nTest)
                {
                    nTest = TEST_NONE;
                    nic.SetLedMode(LED_MODE_NORMAL);
                }
                else
                {
                    nic.SetLedMode(LED_MODE_TEST);
                    nTest = TEST_LED;
                }
                break;
            case TEST_LINK:
                nTest = nInput;
                bLinkStatus = nic.IsLinkUp();
                ShowLinkStatus();
                break;
            case TEST_RX_PACKET:
                nTest = nInput;
                if(!nic.DisableReception())
                    nic.EnableReception();
                break;
            case TEST_TX_MIN_PACKET:
            case TEST_TX_MAX_PACKET:
            {
                //Send a UDP packet to global broadcast address port 2000
                //Start Ethernet header
                nic.TxBegin();
                //Start IPV4 header
                byte pBuffer[20];
                memset(pBuffer, 0, sizeof(pBuffer)); //Cleare buffer ready for IPV4 header
                pBuffer[0] = 0x45; //IPV4
                pBuffer[3] = 20 + 8; //IP length = header (20) + payload (8) = 28 (0x1C)
                pBuffer[8] = 64; //TTL 0x40
                pBuffer[9] = 17; //Protocol = UDP (0x11)
                memset(pBuffer + 16, 0xFF, 4); //Set IP global broadcast address
                nic.TxAppend(pBuffer, 20); //Populate IP header
                //Start UDP header
                memset(pBuffer, 0, sizeof(pBuffer)); //Clear buffer ready for UDP header
                pBuffer[2] = 0x20; //Destination port 8192
                pBuffer[5] = 8; //UDP minimal length (change later)
                nic.TxAppend(pBuffer, 8); //Populate UDP header
                //Populate UDP payload
                //Populate UDP payload with maximum quantity of dummy data
                byte cData = 0;
                if(TEST_TX_MAX_PACKET == nInput)
                {
//                    for(int i = 0; i < 1474; ++i) //!@todo fails for udp payload > 1468
//                        nic.TxAppend(&cData, 1);
                    while(nic.TxAppend(&cData, 1))
                        ++cData;
                }
                //Calculate UDP length
                uint16_t nLen = ENC28J60::SwapBytes(nic.TxGetSize() - 34);
                nic.TxWrite(14+20+4, (byte*)&nLen, 2); //UDP size
                //Calculate IP total length
                nLen = ENC28J60::SwapBytes(nic.TxGetSize() - 14);
                nic.TxWrite(14+2, (byte*)&nLen, 2); //IP size
                //Calculate IP checksum
                uint16_t nChecksum = nic.GetChecksum(14, 20);
                nic.TxWrite(14+10, (byte*)&nChecksum, 2);
                nic.TxEnd(); //Send packet

                Serial.print(F("UDP packet sent, size="));
                Serial.println(nic.TxGetSize());
                break;
            }
            case TEST_CRC:
                {
                    uint16_t nBufferSize = 100;
                    byte pBuffer[nBufferSize];
                    for(byte i = 0; i < sizeof(pBuffer); ++i)
                        pBuffer[i] = i;
                    nic.TxWrite(0, pBuffer, sizeof(pBuffer));
                    uint16_t nChecksum = nic.GetChecksum(0, sizeof(pBuffer));
                    uint32_t nResult = 0;
                    do
                    {
                        for(byte i = 0; i < sizeof(pBuffer); i+=2)
                        {
                            uint16_t nValue = ((pBuffer[i]) << 8) + (pBuffer[i+1] & 0xFF);
                            nResult += nValue;
                        }
                        Serial.println(nResult, HEX);
                        nResult = (nResult & 0xFFFF) + (nResult >> 16);
                    } while(nResult > 0xFFFF);
                    uint16_t nCalcChecksum = nResult;
                    nCalcChecksum = ~nCalcChecksum;
                    nCalcChecksum = ENC28J60::SwapBytes(nCalcChecksum);
                    Serial.println((nChecksum == nCalcChecksum)?"CRC Pass":"CRC Fail"); //!@todo Bigger buffer and correct checksum value
                }
                break;
            case TEST_DMA:
                {
                    uint16_t nLen = 0;
                    Serial.println("Waiting to recieve packet..");
                    while(0 == nLen)
                        nLen = nic.RxBegin();
                    byte pBuffer[nLen];
                    nic.RxGetData(pBuffer, nLen, 0);
                    Serial.println("Bouncing recieved packet");
                    nic.TxBegin(NULL, 3 + nLen); //Prepare Ethernet II LLC broadcast
                    byte pLlcHeader[] = {0x00,0x00,0x03};
                    nic.TxAppend(pLlcHeader, 3);
                    nic.DMACopy(17, 0, nLen);
                    nic.TxEnd();
                }
                break;
            case TEST_RESET:
                nic.Reset();
                Serial.println(" - Reset");
                break;
            case TEST_POWER:
                bPower = !bPower;
                if(bPower)
                {
                    nic.PowerDown();
                    Serial.println(" - Power down");
                }
                else
                {
                    nic.PowerUp();
                    Serial.println(" - Power up");
                }
                break;
            case TEST_DUPLEX:
                bDuplex = !bDuplex;
                if(bDuplex)
                    nic.SetFullDuplex();
                else
                    nic.SetHalfDuplex();
                Serial.println(bDuplex?" - Full duplex":" - Half duplex");
                break;
            case TEST_UNICAST:
                bUnicast = !bUnicast;
                Serial.print(F(" - Unicast reception "));
                if(bUnicast)
                {
                    nic.EnableUnicast();
                    Serial.println(" enabled");
                }
                else
                {
                    nic.DisableUnicast();
                    Serial.println("disabled");
                }
                break;
            case TEST_MULTICAST:
                bMulticast = !bMulticast;
                Serial.print(F(" - Multicast reception "));
                if(bMulticast)
                {
                    nic.EnableMulticast();
                    Serial.println(" enabled");
                }
                else
                {
                    nic.DisableMulticast();
                    Serial.println("disabled");
                }
                break;
            case TEST_BROADCAST:
                bBroadcast = !bBroadcast;
                Serial.print(F(" - Broadcast reception "));
                if(bBroadcast)
                {
                    nic.EnableBroadcast();
                    Serial.println(" enabled");
                }
                else
                {
                    nic.DisableBroadcast();
                    Serial.println("disabled");
                }
                break;
            case TEST_FLOW:
                bFlow = !bFlow;
                if(bFlow)
                    nic.EnableFlowControl();
                else
                    nic.DisableFlowControl();
                Serial.print(F(" - Flow control "));
                Serial.println(bFlow?"enabled":"disabled");
                break;
            case TEST_BIST:
                Serial.print(F("BIST "));
                Serial.println(nic.BIST()?"Pass":"Fail");
                break;
            case TEST_FREE:
                Serial.print(F("Free Rx space = "));
                Serial.println(nic.RxGetFreeSpace());
                break;
        }
    }

    switch(nTest)
    {
        case TEST_NONE:
            break;
        case TEST_LINK:
        {
            //Link status
            bool bStatus = nic.IsLinkUp();
            if(bLinkStatus != bStatus)
            {
                bLinkStatus = bStatus;
                ShowLinkStatus();
            }
            break;
        }
        case TEST_RX_PACKET:
        {
            //!@todo check for long packets
            while(int16_t nRx = nic.RxBegin()) //Start a recieve transaction and get size of next pending packet
            {
                if(nRx > 0 && nRx < 16)
                {
                    Serial.println("!!Error recieving packet - length < 16");
                }
                else if(-1 == nRx)
                {
                    Serial.print("Recieve error ");
                    Serial.println(nic.RxGetStatus());
                }
                else
                {
                    byte pBuffer[14];
                    nic.RxGetData(pBuffer, 14); //Get the first 14 bytes of data from NIC
                    Serial.print("Recieved packet from ");
                    for(byte i = 6; i < 12; ++i)
                    {
                        //Print the source hardware address
                        if(pBuffer[i] < 0x10)
                            Serial.print("0");
                        Serial.print(pBuffer[i], HEX);
                        if(i < 11)
                            Serial.print(":");
                    }
                    if((int16_t)nic.RxGetPacketSize() != nRx)
                        Serial.print(" !!Packet size error: ");
                    else
                        Serial.print(" size=");
                    Serial.print(nRx);
                    if(nic.RxIsBroadcast())
                        Serial.print(" Broadcast");
                    if(nic.RxIsMulticast())
                        Serial.print(" Multicast");
                    Serial.println();
                }
                nic.RxEnd();
            }
            break;
        }
    }
}

