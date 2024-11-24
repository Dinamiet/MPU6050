#ifndef _INTERNALS_H_
#define _INTERNALS_H_

#include "mpu6050.h"

#include <stdint.h>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define BIG_ENDIAN_16(x)    __builtin_bswap16(x)
#define LITTLE_ENDIAN_16(x) (x)
#define BIG_ENDIAN_32(x)    __builtin_bswap32(x)
#define LITTLE_ENDIAN_32(x) (x)
#else
#define BIG_ENDIAN_16(x)    (x)
#define LITTLE_ENDIAN_16(x) __builtin_bswap16(x)
#define BIG_ENDIAN_32(x)    (x)
#define LITTLE_ENDIAN_32(x) __builtin_bswap32(x)
#endif

bool setRegister(const MPU* mpu, const uint8_t reg, const uint8_t value);
void programDMP(const MPU* mpu, MPU_ReadDMPFirmwareInterface read_interface);

#endif
