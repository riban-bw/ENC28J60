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
static const byte TEST_TX_PACKET    = '3';
static const byte TEST_DUPLEX       = '4';
static const byte TEST_DUMP         = 'd';
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

///** Show the content of the ENC28J60 registers */
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
//            nResult = nic.SPIReadRegister(nAddress);
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

///** Reset the SPI interface */
//void Reset()
//{
//    nic.SPIReset();
//}

/** Display the menu */
void ShowMenu()
{
    Serial.println(F("Tests for [12:34:56:78:9A:BC]"));
    Serial.println(F("SPACE shows menu and clears test mode"));
    Serial.println(F("0. Toggle LEDs"));
    Serial.println(F("1. Show link status"));
    Serial.println(F("2. Recieve packets"));
    Serial.println(F("3. *Send packet"));
    Serial.println(F("4. *Toggle duplex"));
//    Serial.println(F("d. *Dump registers"));
//    Serial.println(F("r. *Reset NIC"));
    Serial.println(F("p. *Toggle power"));
    Serial.println(F("u. *Toggle unicast reception"));
    Serial.println(F("m. *Toggle multicast reception"));
    Serial.println(F("b. *Toggle broadcast reception"));
    Serial.println(F("f. *Toggle flow control"));
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
    bDuplex = true;
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
            case TEST_TX_PACKET:
            {
                //Send a minimal UDP packet to global broadcast address port 2000
                byte pBuffer[22] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
                nic.TxBegin();
                nic.TxAppend(pBuffer, 6);
                nic.TxAppend(pMac, 6);
                memset(pBuffer, 0, sizeof pBuffer);
                pBuffer[0] = 0x08; //Ether type IP
                pBuffer[2] = 0x45; //IPV4
                pBuffer[5] = 20 + 8; //IP length = header (20) + payload (8) 0x1C
                pBuffer[10] = 64; //TTL 0x40
                pBuffer[11] = 17; //Protocol = UDP 0x11
                pBuffer[12] = 0x72; //Checksum
                pBuffer[13] = 0xD2;
                memset(pBuffer + 18, 0xFF, 4); //Set IP global broadcast address
                nic.TxAppend(pBuffer, 22);
                memset(pBuffer, 0, sizeof(pBuffer));
                pBuffer[2] = 0x20; //Destination port 8192
                pBuffer[5] = 8; //UDP length
                nic.TxAppend(pBuffer, 8);
                nic.TxEnd(); //Send packet
                Serial.println(" - minimal UDP packet sent");
                break;
            }
//            case TEST_DUMP:
//                DumpRegisters();
//                break;
//            case TEST_RESET:
//                Reset();
//                Serial.println(" - Reset");
//                break;
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
                nic.SetFullDuplex(bDuplex);
                Serial.println(bDuplex?" - Full duplex":" - Half duplex");
                break;
            case TEST_UNICAST:
                bUnicast = !bUnicast;
                Serial.print(" - Unicast reception ");
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
                Serial.print(" - Multicast reception ");
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
                Serial.print(" - Broadcast reception ");
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
                nic.EnableFlowControl(bFlow);
                Serial.print(" - Flow control ");
                Serial.println(bFlow?"enabled":"disabled");
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
            int16_t nRx = nic.RxBegin(); //Start a recieve transaction and get size of next pending packet
            if(0 == nRx)
                return; // No packet available
            else if(nRx > 0 && nRx < 16)
            {
                Serial.println("!!Error recieving packet - length < 16");
            }
            else if(-1 == nRx)
            {
                Serial.print("Recieve error ");
                Serial.println(nic.GetRxStatus());
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
                if((int16_t)nic.GetRxPacketSize() != nRx)
                    Serial.print(" !!Packet size error: ");
                else
                    Serial.print(" size=");
                Serial.println(nRx);
            }
            nic.RxEnd();
            break;
        }
    }
}

