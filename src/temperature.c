#include "internals.h"

#define TEMPERATURE_REG 0x41

void MPU_RequestTemperature(const MPU* mpu, const MPU_Complete complete) { reqData(mpu, TEMPERATURE_REG, sizeof(int16_t), complete); }

float MPU_Temperature(const MPU* mpu)
{
	int16_t rawTemp;
	mpu->Read(&rawTemp, sizeof(rawTemp));
	return BIG_ENDIAN_16(rawTemp) / 340.0f + 36.53f;
}
