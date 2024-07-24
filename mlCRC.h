// Patrick Eaton

#ifndef MLCRC_H
#define MLCRC_H    1

#ifdef __cplusplus
extern "C"
{
#endif

#define X25_INIT_CRC 0xffff
#define X25_VALIDATE_CRC 0xf0b8

#include <stdint.h>

int         crc_get_extra(int msgId, int length);
void	    crc_accumulate(uint8_t data, uint16_t *crcAccum);
void	    crc_init(uint16_t *crcAccum);
uint16_t    crc_calculate(const uint8_t *pBuffer, uint16_t length);
void	    crc_accumulate_buffer(uint16_t *crcAccum, const char *pBuffer, uint16_t length);

#ifdef __cplusplus
} 
#endif

#endif  /* mlCRC.h */
