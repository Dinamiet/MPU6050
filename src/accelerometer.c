#include "internals.h"

#include <string.h>

#define ACCEL_OFFSET_REG 0x06
#define ACCEL_RAW_REG    0x3B

void MPU_SetAccelOffset(const MPU* mpu, const MPUOffset offset)
{
	MPUOffset conv;
	conv.X = BIG_ENDIAN_16(offset.X);
	conv.Y = BIG_ENDIAN_16(offset.Y);
	conv.Z = BIG_ENDIAN_16(offset.Z);

	uint8_t data[1 + sizeof(conv)] = {ACCEL_OFFSET_REG}; // Gyro offset register
	memcpy(&data[1], &conv, sizeof(conv));
	while (!mpu->Write(data, sizeof(data)));
}

void MPU_RequestAccelOffset(const MPU* mpu, const MPU_Complete complete) { reqData(mpu, ACCEL_OFFSET_REG, sizeof(MPUOffset), complete); }

MPUOffset MPU_AccelOffset(const MPU* mpu)
{
	MPUOffset offset;
	mpu->Read(&offset, sizeof(offset));

	offset.X = BIG_ENDIAN_16(offset.X);
	offset.Y = BIG_ENDIAN_16(offset.Y);
	offset.Z = BIG_ENDIAN_16(offset.Z);

	return offset;
}

void MPU_RequestRawAccel(const MPU* mpu, const MPU_Complete complete) { reqData(mpu, ACCEL_RAW_REG, sizeof(MPURaw), complete); }

MPURaw MPU_RawAccel(const MPU* mpu)
{
	MPURaw raw;
	mpu->Read(&raw, sizeof(raw));

	raw.X = BIG_ENDIAN_16(raw.X);
	raw.Y = BIG_ENDIAN_16(raw.Y);
	raw.Z = BIG_ENDIAN_16(raw.Z);

	return raw;
}
