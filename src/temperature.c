#include "internals.h"

#define TEMPERATURE_REG 0x41

bool MPU_RequestTemperature(const MPU* mpu, const MPU_CompleteHandler complete_handler)
{
	return mpu->Request(mpu, TEMPERATURE_REG, sizeof(int16_t), complete_handler);
}

float MPU_Temperature(const MPU* mpu)
{
	int16_t rawTemp;
	mpu->Read(mpu, &rawTemp, sizeof(rawTemp));
	return ((int16_t)BIG_ENDIAN_16(rawTemp)) / 340.0f + 36.53f;
}
