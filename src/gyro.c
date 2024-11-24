#include "internals.h"

#include <string.h>

#define GYRO_OFFSET_REG 0x13
#define GYRO_RAW_REG    0x43

bool MPU_SetGyroOffset(const MPU* mpu, const MPUOffset offset)
{
	MPUOffset conv;
	conv.X = (int16_t)BIG_ENDIAN_16(offset.X);
	conv.Y = (int16_t)BIG_ENDIAN_16(offset.Y);
	conv.Z = (int16_t)BIG_ENDIAN_16(offset.Z);

	return mpu->Write(mpu, GYRO_OFFSET_REG, &conv, sizeof(conv));
}

bool MPU_RequestGyroOffset(const MPU* mpu, const MPU_CompleteHandler complete_handler)
{
	return mpu->Request(mpu, GYRO_OFFSET_REG, sizeof(MPUOffset), complete_handler);
}

MPUOffset MPU_GyroOffset(const MPU* mpu)
{
	MPUOffset offset;
	mpu->Read(mpu, &offset, sizeof(offset));

	offset.X = (int16_t)BIG_ENDIAN_16(offset.X);
	offset.Y = (int16_t)BIG_ENDIAN_16(offset.Y);
	offset.Z = (int16_t)BIG_ENDIAN_16(offset.Z);

	return offset;
}

bool MPU_RequestRawGyro(const MPU* mpu, const MPU_CompleteHandler complete_handler)
{
	return mpu->Request(mpu, GYRO_RAW_REG, sizeof(MPURaw), complete_handler);
}

MPURaw MPU_RawGyro(const MPU* mpu)
{
	MPURaw raw;
	mpu->Read(mpu, &raw, sizeof(raw));

	raw.X = (int16_t)BIG_ENDIAN_16(raw.X);
	raw.Y = (int16_t)BIG_ENDIAN_16(raw.Y);
	raw.Z = (int16_t)BIG_ENDIAN_16(raw.Z);

	return raw;
}
