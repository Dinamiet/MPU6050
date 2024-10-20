#include "internals.h"

#include <string.h>

#define ACCEL_OFFSET_REG 0x06
#define ACCEL_RAW_REG    0x3B

bool MPU_SetAccelOffset(const MPU* mpu, const MPUOffset offset)
{
	MPUOffset conv;
	conv.X = (int16_t)BIG_ENDIAN_16(offset.X);
	conv.Y = (int16_t)BIG_ENDIAN_16(offset.Y);
	conv.Z = (int16_t)BIG_ENDIAN_16(offset.Z);

	return mpu->Write(mpu, ACCEL_OFFSET_REG, &conv, sizeof(conv));
}

bool MPU_RequestAccelOffset(const MPU* mpu, const MPU_Complete complete) { return mpu->Request(mpu, ACCEL_OFFSET_REG, sizeof(MPUOffset), complete); }

MPUOffset MPU_AccelOffset(const MPU* mpu)
{
	MPUOffset offset;
	mpu->Read(mpu, &offset, sizeof(offset));

	offset.X = (int16_t)BIG_ENDIAN_16(offset.X);
	offset.Y = (int16_t)BIG_ENDIAN_16(offset.Y);
	offset.Z = (int16_t)BIG_ENDIAN_16(offset.Z);

	return offset;
}

bool MPU_RequestRawAccel(const MPU* mpu, const MPU_Complete complete) { return mpu->Request(mpu, ACCEL_RAW_REG, sizeof(MPURaw), complete); }

MPURaw MPU_RawAccel(const MPU* mpu)
{
	MPURaw raw;
	mpu->Read(mpu, &raw, sizeof(raw));

	raw.X = (int16_t)BIG_ENDIAN_16(raw.X);
	raw.Y = (int16_t)BIG_ENDIAN_16(raw.Y);
	raw.Z = (int16_t)BIG_ENDIAN_16(raw.Z);

	return raw;
}
