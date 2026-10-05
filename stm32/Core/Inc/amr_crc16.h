#ifndef AMR_CRC16_H
#define AMR_CRC16_H

#include <stddef.h>
#include <stdint.h>

uint16_t AmrCrc16_CcittFalse(const uint8_t *data, size_t length);

#endif
