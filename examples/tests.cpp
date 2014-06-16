#include "Arduino.h"
#include "../enc28j60.h"

#define _RIBANDEBUG_

static const byte ETHERNET_CS_PIN   = 10;
static const byte TEST_NONE         = -1;
static const byte TEST_MENU         = ' ';
static const byte TEST_LED          = '0';
static const byte TEST_LINK         = '1';
static const byte TEST_RX_PACKET    = '2';
static const byte TEST_TX_PACKET    = '3';
static const byte TEST_DUMP         = 'd';
static const byte TEST_RESET        = 'r';
static const byte TEST_POWER        = 'p';

static const uint16_t LED_MODE_NORMAL   = 0x3422;
static const uint16_t LED_MODE_TEST     = 0x0AB0;

ENC28J60 nic; //!< Instance of the NIC
byte nTest; //!< Current test
bool bLinkStatus; //!< Link status
bool bPower; //!< Power on status

void ShowMenu()
{
    Serial.println("Tests");
    Serial.println("0. Toggle LEDs");
    Serial.println("1. Show link status");
    Serial.println("2. Send packet");
    Serial.println("3. Recieve packets");
    Serial.println("d. Dump registers");
    Serial.println("r. Reset NIC");
    Serial.println("p. Toggle power");
}

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
    byte pMac[6] = {0x12,0x34,0x56,0x78,0x9A,0xBC};
    byte nRevision = nic.Initialize(pMac, ETHERNET_CS_PIN);
    Serial.print("Initialise NIC. Firmware version ");
    Serial.println(nRevision);
    nTest = TEST_NONE;
    ShowMenu();
}

/** Main program loop */
void loop()
{
    int nInput = Serial.read();
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
            break;
        case TEST_TX_PACKET:
            nTest = nInput;
            break;
        case TEST_DUMP:
            Serial.println(" - Dump registers:");
            for(byte nRegister = 0; nRegister < 0x20; ++nRegister)
            {
                if(0x1A == nRegister)
                    continue; //Do not access reserved registers
                Serial.print("0x");
                if(nRegister < 0x10)
                    Serial.print("0");
                Serial.print(nRegister, HEX);
                Serial.print(": ");
                for(byte nBank = 0; nBank < 2; ++nBank)
                {
                    Serial.print("\t");
                    byte nAddress, nResult;
                    if(nBank > 1)
                        nAddress = nRegister | (nBank * 0x20) | 0x80;
                    else
                        nAddress = nRegister | (nBank * 0x20);
                    nResult = nic.SPIReadRegister(nAddress);
                    Serial.print("[");
                    if(nAddress < 0x10)
                        Serial.print("0x0");
                    else
                        Serial.print("0x");
                    Serial.print(nAddress, HEX);
                    Serial.print("] ");
                    if(nResult < 0x10)
                        Serial.print("0x0");
                    else
                        Serial.print("0x");
                    Serial.print(nResult, HEX);
                }
                Serial.println();
            }
            Serial.println();
            break;
        case TEST_RESET:
            nic.SPIReset();
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
    }
    switch(nTest)
    {
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
            byte pBuffer[64];
            if(nic.PacketReceive(pBuffer, 64))
                Serial.println("Packet recieved");
            break;
        }
        case TEST_TX_PACKET:
            break;
    }
}

