
#include <SPI.h>
#include "mcp_can.h"
#include "crc8_table.h"
#include <avr/wdt.h>

#define TXBUFFERSIZE 10

INT8U registers[] = { //list of mcp2515 registers for debugging
MCP_RXF0SIDH,
MCP_RXF0SIDL,
MCP_RXF0EID8,
MCP_RXF0EID0,
MCP_RXF1SIDH,
MCP_RXF1SIDL,
MCP_RXF1EID8,
MCP_RXF1EID0,
MCP_RXF2SIDH,
MCP_RXF2SIDL,
MCP_RXF2EID8,
MCP_RXF2EID0,
MCP_CANSTAT,
MCP_CANCTRL,
MCP_RXF3SIDH,
MCP_RXF3SIDL,
MCP_RXF3EID8,
MCP_RXF3EID0,
MCP_RXF4SIDH,
MCP_RXF4SIDL,
MCP_RXF4EID8,
MCP_RXF4EID0,
MCP_RXF5SIDH,
MCP_RXF5SIDL,
MCP_RXF5EID8,
MCP_RXF5EID0,
MCP_TEC  ,
MCP_REC  ,
MCP_RXM0SIDH,
MCP_RXM0SIDL,
MCP_RXM0EID8,
MCP_RXM0EID0,
MCP_RXM1SIDH,
MCP_RXM1SIDL,
MCP_RXM1EID8,
MCP_RXM1EID0,
MCP_CNF3 ,
MCP_CNF2 ,
MCP_CNF1 ,
MCP_CANINTE,
MCP_CANINTF,
MCP_EFLG ,
MCP_TXB0CTRL,
MCP_TXB1CTRL,
MCP_TXB2CTRL,
MCP_RXB0CTRL,
MCP_RXB0SIDH,
MCP_RXB1CTRL,
MCP_RXB1SIDH
};

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int LED = 8;
boolean ledON = 1;
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

unsigned char stmp0[8] = {0, 0, 0, 0, 0, 0, 0, 0};
void setup()
{
  Serial.begin(1000000);
  pinMode(LED, OUTPUT);

START_INIT:

  if (CAN_OK == CAN.begin(CAN_1000KBPS))                  
  {
    //Serial.println("CAN BUS Shield init ok!");
  }
  else
  {
    //Serial.println("CAN BUS Shield init fail");
    //Serial.println("Init CAN BUS Shield again");
    delay(100);
    goto START_INIT;
  }

  CAN.init_Mask(0, 1, 0); //set message filter to let everything through
  CAN.init_Filt(0, 1, 0);


	wdt_enable(WDTO_1S);
  //CAN.sendMsgBuf( (INT32U)0x2040000, 1, 8, stmp0); //some random activity to make the talon go into CAN mode
}

struct RXData { //struct for storing received CAN messages
  unsigned char size = 0;
  unsigned char packetcount = 0;
  INT32U canId;
  unsigned char bytes[8];
  unsigned char checksum = 42;
};

struct TXData { //struct for storing CAN messages to send received from the serial port
  unsigned char size = 0;
  unsigned char index = 0;
  long periodMs = -1;
  INT32U canId;
  unsigned char bytes[8];
  unsigned char checksum = 42;
};

struct TXCANData { //stores messages for periodic sending
  unsigned char size = 0;
  unsigned char bytes[8];
  INT32U canId;
  long periodMs = -1;
};
TXCANData txarr[TXBUFFERSIZE];

RXData rxData;
TXData txData;

#define RXBUFFERSIZE 20

RXData RXDataBuffer[RXBUFFERSIZE];
char RXDataBufferWriteIndex = 0;
char RXDataBufferReadIndex = 0;
unsigned char prevRXDataBufferReadIndex = 0;
unsigned char RXDataBufferReadCount = 0;
unsigned char RXDataBufferWriteCount = 0;
unsigned char RXDataBytestosend = 0;

unsigned char keepalive = 0;
unsigned char sendmessages = 0;
unsigned char command = 0;

unsigned char stream = 0;
unsigned char send2hostonce=0;

unsigned long periodicMessageLastMillis = 0;

int j = 0;


void loop()
{

	wdt_reset();


if (Serial.available())
  {
    command = Serial.read();
    switch (command)
    {
      case 'd': // send received CAN messages to host
        send2hostonce = 2;
        keepalive = 255;
        break;

      case 'r': //resend message in case there is an error
        if (RXDataBufferReadIndex < 5)
        {
          RXDataBufferReadIndex = RXBUFFERSIZE-5 + RXDataBufferReadIndex;
          //Serial.write((unsigned char*) &RXDataBuffer[RXBUFFERSIZE-1 + RXDataBufferReadIndex], 15);
        }
        else
        {
          //Serial.write((unsigned char*) &RXDataBuffer[RXDataBufferReadIndex - 1], 15);
          RXDataBufferReadIndex -= 5;
        }
        break;

      case 'q': // start streaming received messages
        stream = 1;
        break;

      case 'w': // stop message stream
        stream = 0;
        break;

      case 'g': // dump configuration of mcp2515 to serial. Used for debugging the filters
	CAN.mcp2515_setCANCTRL_Mode(MODE_CONFIG);
	delay(10);
	for(int i = 0; i < sizeof(registers); i++)
	{
		Serial.print(i);
		Serial.print("\t");
		Serial.print(CAN.mcp2515_readRegister(registers[i]),HEX);
		Serial.print("\t");
		Serial.println(CAN.mcp2515_readRegister(registers[i]),BIN);
	}
	CAN.mcp2515_setCANCTRL_Mode(MODE_NORMAL);
        break;

	case 'h':
		Serial.println("\n");
	    for (j = 0; j < TXBUFFERSIZE; j++)		// print all periodic messages
	    {
	    	Serial.print("per ");
	      Serial.print(txarr[j].periodMs);
	      Serial.print("\tcanId ");
	      Serial.print(txarr[j].canId);
	      Serial.print("\tsize ");
	      Serial.print(txarr[j].size);
	      Serial.print(" data ");
	      for (int i=0; i<8; i++)
	      {
	      	Serial.print(txarr[j].bytes[i]);
		Serial.print(" ");
	}
		Serial.println();
	    }
		Serial.println("\n");
		break;

      case 0: // host wants to send a message
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
      case 7:
      case 8: // 1:size 1:index 4:period 4:arbID 8:data 1:checksum  19 total
      send2hostonce = 0;
      
        txData.size = command;
        while (Serial.available() < 18);
        Serial.readBytes((char*)&txData.index, 18);

        //if (txData.size > 8) break; // size greater than 8 is an error
        if (txData.checksum != crc_update(0, &txData.size, 18)) // checksum check
	{ // there was an error
		while(Serial.available()>0) Serial.read(); // clear input buffer
		Serial.write(21); //nak
		//Serial.print("\nchecksum ");
		//Serial.print(txData.checksum);
		//Serial.println("\n");
		break;
	}
	else // data is good
	{
		if (txData.periodMs == 0) // send one-shot message
		{
		  CAN.sendMsgBuf(txData.canId, 1, txData.size, txData.bytes);
		}
		else // put message in transmit buffer for periodic sending
		{
		  txarr[txData.index].size = txData.size;
		  txarr[txData.index].periodMs = txData.periodMs;
		  txarr[txData.index].canId = txData.canId;
		  for (j = 0; j < txData.size; j++) txarr[txData.index].bytes[j] = txData.bytes[j];
		}
		Serial.write(6); //ack
	}


        keepalive = 255;

#ifdef DEBUG
        Serial.println();
        Serial.print("size:\t");
        Serial.println(txarr[txData.index].size);
        Serial.print("index:\t");
        Serial.println(txData.index);
        Serial.print("chksum:\t");
        Serial.println(txData.checksum);
        Serial.print("period:\t");
        Serial.println(txarr[txData.index].periodMs);
        Serial.print("arbID:\t");
        Serial.println(txarr[txData.index].canId);

        Serial.print("bytes:\t");
        for (j = 0 ; j < txarr[txData.index].size; j++)
        {
          Serial.print(txData.bytes[j]);
          Serial.print("\t");
        }
        Serial.println();
#endif

        break;

	case '.': // test the watchdog
		while(true);
		break;

	case '?': // respond with name
		Serial.println("CANBRIDGE");
		break;
    }
  }
  

  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {						// put received CAN messages in the buffer
    CAN.readMsgBuf(&rxData.size, rxData.bytes);
  
  //populate data struct
    rxData.canId = CAN.getCanId();
    rxData.checksum = crc_update(0, &rxData.canId, 12);
    rxData.packetcount++;
  //put data in buffer
    RXDataBuffer[RXDataBufferWriteIndex].size = rxData.size;
    RXDataBuffer[RXDataBufferWriteIndex].packetcount = rxData.packetcount;
    RXDataBuffer[RXDataBufferWriteIndex].canId = rxData.canId;
    for (j = 0; j < 8; j++) RXDataBuffer[RXDataBufferWriteIndex].bytes[j] = rxData.bytes[j];
    RXDataBuffer[RXDataBufferWriteIndex].checksum = rxData.checksum;

    if (RXDataBufferWriteIndex < RXBUFFERSIZE-1) RXDataBufferWriteIndex++;
    else RXDataBufferWriteIndex = 0;
    
    if (RXDataBufferWriteIndex == RXDataBufferReadIndex) {
      if (RXDataBufferReadIndex < RXBUFFERSIZE-1) RXDataBufferReadIndex++;
      else RXDataBufferReadIndex = 0;
    }
    
    RXDataBufferWriteCount++;
    if(RXDataBytestosend <= RXBUFFERSIZE) RXDataBytestosend++;


  }

  if (keepalive) // send messages only if the host computer has talked recently
  {
    for (j = 0; j < TXBUFFERSIZE; j++)		// send all periodic messages
    {
      if (txarr[j].periodMs >= 0) // do not send messages with a -1 period
      {
        CAN.sendMsgBuf(txarr[j].canId, 1, txarr[j].size, txarr[j].bytes);
        if (txarr[j].periodMs == 0) txarr[j].periodMs = -1;
      }
    }
    keepalive--;
  }
  else
  {
    for (j = 0; j < TXBUFFERSIZE; j++) // set all message periods to -1 if no communications from host
    {
      txarr[j].periodMs = -1;
    }
  }

  // send received CAN messages to computer if stream mode is enabled    ||  RXDataBufferReadIndex != RXDataBufferWriteIndex
  if ( (stream || send2hostonce) && (RXDataBufferReadIndex != RXDataBufferWriteIndex) && millis()-periodicMessageLastMillis >= 10) //if indexes are the same, there are no unread messages in buffer
  {
    Serial.write((unsigned char*) &RXDataBuffer[RXDataBufferReadIndex], 15);
    prevRXDataBufferReadIndex = RXDataBufferReadIndex;
    if (RXDataBufferReadIndex < RXBUFFERSIZE-1) RXDataBufferReadIndex++;
    else RXDataBufferReadIndex = 0;
    send2hostonce--;
    RXDataBufferReadCount++;
    RXDataBytestosend--;
  }

  
}
