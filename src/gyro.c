#include "internals.h"

#include <string.h>

#define GYRO_OFFSET_REG 0x13
#define GYRO_RAW_REG    0x43

bool MPU_SetGyroOffset(const MPU* mpu, const MPUOffset offset)
{
	MPUOffset conv;
	conv.X = BIG_ENDIAN_16(offset.X);
	conv.Y = BIG_ENDIAN_16(offset.Y);
	conv.Z = BIG_ENDIAN_16(offset.Z);

	return mpu->Write(GYRO_OFFSET_REG, &conv, sizeof(conv));
}

bool MPU_RequestGyroOffset(const MPU* mpu, const MPU_Complete complete) { return mpu->Request(GYRO_OFFSET_REG, sizeof(MPUOffset), complete); }

MPUOffset MPU_GyroOffset(const MPU* mpu)
{
	MPUOffset offset;
	mpu->Read(&offset, sizeof(offset));

	offset.X = BIG_ENDIAN_16(offset.X);
	offset.Y = BIG_ENDIAN_16(offset.Y);
	offset.Z = BIG_ENDIAN_16(offset.Z);

	return offset;
}

bool MPU_RequestRawGyro(const MPU* mpu, const MPU_Complete complete) { return mpu->Request(GYRO_RAW_REG, sizeof(MPURaw), complete); }

MPURaw MPU_RawGyro(const MPU* mpu)
{
	MPURaw raw;
	mpu->Read(&raw, sizeof(raw));

	raw.X = BIG_ENDIAN_16(raw.X);
	raw.Y = BIG_ENDIAN_16(raw.Y);
	raw.Z = BIG_ENDIAN_16(raw.Z);

	return raw;
}
