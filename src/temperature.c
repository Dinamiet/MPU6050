#include "mpu6050.h"
#include "private.h"

#define TEMPERATURE_REG 0x41

void MPU_RequestTemperature(const MPU* mpu, const MPU_Complete complete)
{
	const uint8_t reg = TEMPERATURE_REG;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(mpu, sizeof(int16_t), complete));
}

float MPU_Temperature(const MPU* mpu)
{
	int16_t rawTemp;
	mpu->Read(&rawTemp, sizeof(rawTemp));
	return BIG_ENDIAN_16(rawTemp) / 340.0f + 36.53f;
}
