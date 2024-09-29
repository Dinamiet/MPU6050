#include "mpu6050.h"
#include "private.h"

#define TEMPERATURE_REG 0x41

void MPU_RequestTemperature(const MPU* mpu, const MPU_Complete ready)
{
	const uint8_t reg = TEMPERATURE_REG;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(sizeof(int16_t), ready));
}

float MPU_Temperature(const MPU* mpu)
{
	int16_t rawTemp;
	mpu->Read(&rawTemp, sizeof(rawTemp));
	return BIG_ENDIAN_16(rawTemp) / 340.0f + 36.53f;
}
