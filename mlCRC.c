// gcc -g -Wall -c mlEchoCRC.c -o mlEchoCRC.o

#include "mlCRC.h"

// CRC EXTRA
// MSG_ID	Description			Length	CRC Extra
// 29,		Scaled Pressure,	14,		115
// 66,		DataStream Request,	6,		148
// 201,		Static,				19,		126
// 202,		Ownship,			42,		7
// 203,		Status,				1,		85
// 246,		Traffic,			38,		184
// 248,		Identification,		69,		

int crc_get_extra(int MSGID, int length)
{
    int CRC_EXTRA;

    // return sizeof(MSGID);    // size 4
    // return (int)MSGID;

	switch (MSGID)
	{
		case 29:
		    CRC_EXTRA = 115;
			break;
		case 66:
            CRC_EXTRA = 148;
			break;
        case 128:
            CRC_EXTRA = 226;
            break;
        case 134:
            CRC_EXTRA = 229;
            break;
        case 152:
            CRC_EXTRA = 208;
            break;
        case 158:
            CRC_EXTRA = 134;
            break;
		case 201:
            CRC_EXTRA = 126;
			break;
		case 202:
            if (length == 42) CRC_EXTRA = 7;
            else CRC_EXTRA = 11;
			break;
		case 203:
            CRC_EXTRA = 85;
			break;
        case 224:
            CRC_EXTRA = 102;
            break;
        case 230:
            CRC_EXTRA = 163;
            break;
		case 246:
            CRC_EXTRA = 184;
			break;
		case 248:
            CRC_EXTRA = 8;
			break;
        case 254:
            CRC_EXTRA = 46;
            break;
		default:
            CRC_EXTRA = 255;
	}
    return CRC_EXTRA;
}

// uint16_t *crcAccum;
// uint8_t data;
// uint16_t *msg;

// functions
// @param 2 (&msg + 1) to start CRC at LEN not STX.
// @param 3 Must add 5 to the payload message length to perform checksum across LEN, SEQ, SYS, COMP, MSGID and PAYLOAD.
// crc_accumulate_buffer(&msg->checksum, &msg + 1, msg->len + 5);
// Note CRC_EXTRA is defined for each individual packet in the document.
// crc_accumulate(CRC_EXTRA, &msg->checksum);
// ck_a(msg) = (uint8_t)(msg->checksum & 0xFF);
// ck_b(msg) = (uint8_t)(msg->checksum >> 8);


/**
  * @brief Accumulate the X.25 CRC by adding one char at a time.
  *
  * The checksum function adds the hash of one char at a time to the
  * 16 bit checksum (uint16_t).
  *
  * @param data - New char to hash
  * @param crcAccum - Already accumulated checksum
**/
void crc_accumulate(uint8_t data, uint16_t *crcAccum)
{
	// Accumulate one byte of data into the CRC
	uint8_t tmp;

	tmp = data ^ (uint8_t)(*crcAccum & 0xff);
	tmp ^= (tmp << 4);
	*crcAccum = (*crcAccum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}


/**
  * @brief Initialize the buffer for the X.25 CRC
  *
  * @param crcAccum - 16 bit X.25 CRC
**/
void crc_init(uint16_t *crcAccum)
{
	*crcAccum = X25_INIT_CRC;
}


/**
* @brief Calculates the X.25 checksum on a byte buffer
*
* @param pBuffer - buffer containing the byte array to hash
* @param length - length of the byte array
* @return the checksum over the buffer bytes
**/
uint16_t crc_calculate(const uint8_t *pBuffer, uint16_t length)
{
	uint16_t crcTmp;
	crc_init(&crcTmp);
	while (length--) crc_accumulate(*pBuffer++, &crcTmp);
	return crcTmp;
}


/**
* @brief Accumulate the X.25 CRC by adding an array of bytes
*
* The checksum function adds the hash of one char at a time to the
* 16 bit checksum (uint16_t).
*
* @param data - New bytes to hash
* @param crcAccum - Already accumulated checksum
**/
void crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length)
{
	const uint8_t *p = (const uint8_t *)pBuffer;
	while (length--) crc_accumulate(*p++, crcAccum);
}
