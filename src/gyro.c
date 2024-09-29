#include "mpu6050.h"
#include "private.h"

#include <string.h>

#define GYRO_OFFSET_REG 0x13
#define GYRO_RAW_REG    0x43

void MPU_SetGyroOffset(const MPU* mpu, const MPUOffset offset)
{
	MPUOffset conv;
	conv.X = BIG_ENDIAN_16(offset.X);
	conv.Y = BIG_ENDIAN_16(offset.Y);
	conv.Z = BIG_ENDIAN_16(offset.Z);

	uint8_t data[1 + sizeof(conv)] = {GYRO_OFFSET_REG}; // Gyro offset register
	memcpy(&data[1], &conv, sizeof(conv));
	while (!mpu->Write(data, sizeof(data)));
}

void MPU_RequestGyroOffset(const MPU* mpu, const MPU_Complete ready)
{
	const uint8_t reg = GYRO_OFFSET_REG;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(sizeof(MPUOffset), ready));
}

MPUOffset MPU_GyroOffset(const MPU* mpu)
{
	MPUOffset offset;
	mpu->Read(&offset, sizeof(offset));

	offset.X = BIG_ENDIAN_16(offset.X);
	offset.Y = BIG_ENDIAN_16(offset.Y);
	offset.Z = BIG_ENDIAN_16(offset.Z);

	return offset;
}

void MPU_RequestRawGyro(const MPU* mpu, const MPU_Complete ready)
{
	const uint8_t reg = GYRO_RAW_REG;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(sizeof(MPURaw), ready));
}

MPURaw MPU_RawGyro(const MPU* mpu)
{
	MPURaw raw;
	mpu->Read(&raw, sizeof(raw));

	raw.X = BIG_ENDIAN_16(raw.X);
	raw.Y = BIG_ENDIAN_16(raw.Y);
	raw.Z = BIG_ENDIAN_16(raw.Z);

	return raw;
}
