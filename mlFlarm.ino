/*
	~/Arduino/RP2040/ThingPlus/mlFlarm/mlFlarm.ino
	Patrick Eaton
	2024/07/01
*/

// ~/.arduino15/packages/rp2040/hardware/rp2040/3.9.2/cores/rp2040

// A useful utility for converting an ASCII hex file to binary:
// $ xxd -r -ps file.hex file.bin

#include <Adafruit_NeoPixel.h>
// #define NP_Pwr 16
#define NP_Pin 8

Adafruit_NeoPixel pixels(1, NP_Pin, NEO_GRB + NEO_KHZ800);

#define DEBUG 0		   // 0=OFF, 1=MSGID,CKSUM 2=PRTMSG
#define HDR_SIZE 6
#define MSG_SIZE 320
#define initMsg "Arduino/RP2040/ThingPlus/mlFlarm/mlFlarm.ino starting!"

#include <Arduino.h>
#include <LibPrintf.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include "mlUtil.h"
#include "mlCRC.h"

// Global Variables
uint8_t hdrBuf[HDR_SIZE];	// mavLink buffer
uint8_t __attribute__ ((aligned (4))) msgBuf[MSG_SIZE];	// mavLink buffer
int		bufIdx = 0;			// buffer index
int		msgId;
int		length = 0;
enum eSTATE_t STATE;

// OwnShip Position and Altitude
struct sOwnShip_t ownShip;

// Target Array
struct sTarget_t target[32];

// Traffic Array
struct sTrafficReport_t traffic[32];

// Function prototypes
void parseDataStreamRequest(void *pMlMsg);
void parseIdentification(void* pMlMsg);
void parseNavData(void* pMlMsg);
void parseOwnship(void* pMlMsg);
void parseScaledPressure(void* pMlMsg);
void parseStatic(void* pMlMsg);
void parseStatus(void* pMlMsg);
void parseTraffic(void* pMlMsg);


void setup()
{
	int timeout;

    // Initialize NeoPixel power pin
    // pinMode(NP_Pwr, OUTPUT);
    // digitalWrite(NP_Pwr, HIGH);

	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);
	pixels.begin();		// Initialize the neopixel
  setNeoPixel(32,32,0);

	// Setup USB Serial port to print incoming mavlink messages.
	Serial.begin(115200);		// USB Serial
	// Init RX/TX pins on XIAO
	Serial1.begin(57600);		// Input from ping2020 (mavlink) GPIO0,1
    Serial2.setTX(20);          // GPIO4 for ItsyBitsy, GPIO20 for ThingPlus
    Serial2.setRX(21);          // GPIO5 for ItsyBitsy, GPIO21 for ThingPlus
    Serial2.begin(115200);      // Output FLARM messages to S80
	// Wait for hardware serial to appear
	timeout = 20000000;
	while (timeout--)
	{
		if (Serial)
		{
            delay(2000);
			// printf("Timeout: %d\n", timeout);
			printf("%s\r\n", initMsg);
			break;
		}
	}
	delay(500);
	digitalWrite(LED_BUILTIN, LOW);













	timeout = 20000000;
	while (timeout--)
	{
		if (Serial1)
		{
			printf("ping2020 serial port initialized!\r\n");
			break;
		}
	}
    Serial2.write("S80 output serial port initialized!\r\n");
	delay(2000);
	Serial.write("mlFlarm starting on /dev/ttyACM0\r\n");
	Serial1.write("mlFlarm waiting for mavlink input on /dev/ttyACM1 (Serial1)!\r\n");
    Serial2.write("Outputing FLARM messages on /dev/ttyUSB0 (Serial2)!\r\n");
    delay(2000);
	clrscr();
}


void loop()
{
	uint8_t chr;

	chr = getCh();
	switch (STATE)
	{
		case SYNC:
			// SYNC state until 0xFE
			if (chr == 0xFE)
			{
				hdrBuf[0] = chr;
				bufIdx = 1;
				STATE = HEADER;
			}
			break;
		case HEADER:
			// HEADER state
			hdrBuf[bufIdx++] = chr;
			if (bufIdx == 6)
			{
				bufIdx = 0;
				length = (int)hdrBuf[1];
				msgId = (int)hdrBuf[5];
				STATE = DATA;
			}
			break;
		case DATA:
			// DATA state until length + 2
			msgBuf[bufIdx++] = chr;
			if (bufIdx == (length + 2))
			{
				parseMessage();
				STATE = SYNC;
			}
			break;
		default:
			// ERROR!
			STATE = SYNC;
			break;
	}
}


uint8_t getCh(void)
{
    // Wait for a mavlink character
    while (!Serial1.available()) { ; }
    return (Serial1.read());
}


void parseMessage(void)
{
	int       crcExtraChar;
	uint16_t	crcRx;
	uint16_t	crcTx;

	// check CRC
	crc_init(&crcRx);
	crc_accumulate_buffer(&crcRx, (const char *)&hdrBuf[1], 5);
	crc_accumulate_buffer(&crcRx, (const char *)msgBuf, length);
	crcExtraChar = crc_get_extra(msgId, length);
	crc_accumulate(crcExtraChar, &crcRx);
	crcTx = msgBuf[length] + (msgBuf[length + 1] << 8);

	if (DEBUG > 0)
	{
		// Print the msgId, length, crcRx, crcTx, PASS/FAIL
		printf("msgID:%3d LENGTH:%3d CRC_Rx:%04X CRC_Tx:%04X", msgId, length, crcRx, crcTx);
		if (crcRx == crcTx)
		{
			printf("  PASS!\r\n");
		}
		else
		{
			printf("  FAIL!\r\n");
			return;
		}
	}

	if (crcRx == crcTx)
	{
		// printf("MSGID: %d\r\n", msgId);
		switch (msgId)
		{
			case 29:
				// SCALED PRESSURE, In, 14, 115
				parseScaledPressure(msgBuf);
				break;
			case 66:
				// DATA_STREAM_REQUEST, Out, 6, 148
				parseDataStreamRequest(msgBuf);
				break;
			case 201:
				// STATIC, , 0xC9 19 126
				parseStatic(msgBuf);
				break;
			case 202:
				// Ownship, Out, 0xCA, 42, 7
				// Navigation, In, 51, 11
				if (length == 42)
				{
					// OWNSHIP,  Out length=42, CRC Extra=7
					parseOwnship(msgBuf);
				}
				else if (length == 51) 
				{
					// NAVIGATION,	Out length=51, CRC Extra=11
					parseNavData(msgBuf);
				}
				break;
			case 203:
				// STATUS, Out, 0xCB, 1, 85
				parseStatus(msgBuf);
				break;
			case 246:
				// TRAFFIC_REPORT, 0xF6, Out, 38, 148
				parseTraffic(msgBuf);
				break;
			case 248:		// 0xF8 69	 8
				// IDENTIFICATION, In,
				parseIdentification(msgBuf);
				break;
			default:
				// printf("ERROR Parsing MsgID: %3d\n", msgId);
				break;
		}
	}

	return;
}


// MSG_ID 66 Data Stream Request
void parseDataStreamRequest(void *pMlMsg)
{
  int i;
  uint8_t *pByte;

  pByte = (uint8_t *)pMlMsg;

  if (0)
  {
    printf("DataStreamRequest: ");
    for (i=0; i<6; i++)
    {
      printf("%02X", pByte[i]);
    }
    printf("\r\n");
  }

	return;
}


// MSG_ID 248 Identification
void parseIdentification(void* pMlMsg)
{
  struct sIdentification_t* pIdent;
  pIdent = (sIdentification_t *)pMlMsg;

	printf("Identification SN: %lld\n", pIdent->primarySerialNumber);

	return;
}


// MSG_ID 202 Navigation
void parseNavData(void* pMlMsg)
{
	struct sNavData_t* pNavData;
	pNavData = (struct sNavData_t*)pMlMsg;

	printf("Navigation,%lu,%ld,%ld,%ld,%ld\n",
		pNavData->utcTime,
		pNavData->lat,
		pNavData->lon,
		pNavData->altHAE,
		pNavData->altPress
	);

	return;
}


// MSG_ID 202 Ownship
void parseOwnship(void* pMlMsg)
{
	time_t mlTime;
	struct sOwnShip_t* pOwnShip;			// typecast the 'void*' pointer
	pOwnShip = (struct sOwnShip_t*)pMlMsg;	// to an OwnShip structure pointer

  // Just copy the buffer into the ownShip global structure
  memcpy(&ownShip, pOwnShip, sizeof(struct sOwnShip_t));

	// Convert epoch from 1980/01/06 to 1970/01/01
	mlTime = pOwnShip->utcTime + 315964800;
  ownShip.utcTime = mlTime;   // Adjust time to UNIX epoch

	if (ownShip.fixType > 1)
	{
    // Status Green
	  setNeoPixel(0,64,0);
  }
  else
  {
    // Status Red
    setNeoPixel(64,64,0);
  }
  prtOwnShip ();

	return;
}


// MSG_ID 29 Scaled Pressure
void parseScaledPressure(void* pMlMsg)
{
  struct sScaledPressure_t* pScaledPressure;
  pScaledPressure = (struct sScaledPressure_t*)pMlMsg;

	printf("ScaledPressure: %f mb\n", pScaledPressure->press_abs);

	return;
}


// MSG_ID 201 Parse Static
void parseStatic(void* pMlMsg)
{
  // typecast the 'void*' pointer
  struct sStatic_t* pStatic;
	pStatic = (struct sStatic_t*)pMlMsg;

  printf("Static Msg, Stall Speed: %d\n", pStatic->stallSpeed);

	return;
}


// MSG_ID 203 Parse Status
void parseStatus(void* pMlMsg)
{
	uint8_t Status;
	
	Status = *(uint8_t*)pMlMsg;

	// printf("Status:");
	if (Status & 0x1E)      // If any faults are reported
	{ 
	    printf("Status: ");
	    // if (Status & 0x01) { printf(" OK"); }
	    if (Status & 0x02) { printf(" TX_FAIL_1090ES"); }
	    if (Status & 0x04) { printf(" RX_FAIL_1090ES"); }
	    if (Status & 0x08) { printf(" TX_FAIL_UAT"); }
	    if (Status & 0x10) { printf(" RX_FAIL_UAT"); }
	    printf("\r\n");
    }

	return;
}


// MSG_ID 246 Traffic Report
void parseTraffic(void* pMlMsg)
{
	struct sTrafficReport_t* pTraffic;

	pTraffic = (struct sTrafficReport_t *)pMlMsg;
  memcpy(&traffic[0], pTraffic, sizeof(struct sTrafficReport_t));

	// Calculate dist and brg from ownship position
	if (ownShip.fixType >= 2)
	{
		brgDstAlt(pTraffic, &target[0]); 
	}
  prtTraffic(0);

  return;
}


// Set NeoPixel Color
void setNeoPixel(int Red, int Green, int Blue)
{
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(Red, Green, Blue));
    pixels.show();   
}


void s2print(char *chrString)
{
	Serial2.print(chrString);
}
