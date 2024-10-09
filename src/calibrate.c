#include "mpu6050.h"
#include "pid.h"

#include <math.h>

#define CALIBRATION_STEP 25

void MPU_CalibrateGyro(const MPU* mpu, const MPU_TransferBusy transferBusy)
{
	const float maxErr = 3.0f;
	const float Ki     = 0.07f;
	float       avgError;
	PID         pidX;
	PID         pidY;
	PID         pidZ;

	PID_Init(&pidX, 0, Ki, 0);
	PID_Init(&pidY, 0, Ki, 0);
	PID_Init(&pidZ, 0, Ki, 0);

	PID_Target(&pidX, 0);
	PID_Target(&pidY, 0);
	PID_Target(&pidZ, 0);

	do {
		avgError = 0;
		for (uint8_t i = 0; i < CALIBRATION_STEP; i++)
		{
			MPU_RequestRawGyro(mpu, NULL);
			while (transferBusy(mpu));

			MPURaw    raw = MPU_RawGyro(mpu);
			MPUOffset offset;
			offset.X = PID_Output(&pidX, raw.X, 1.0f);
			offset.Y = PID_Output(&pidY, raw.Y, 1.0f);
			offset.Z = PID_Output(&pidZ, raw.Z, 1.0f);

			MPU_SetGyroOffset(mpu, offset);
			while (transferBusy(mpu));

			avgError += fabsf(PID_Error(&pidX));
			avgError += fabsf(PID_Error(&pidY));
			avgError += fabsf(PID_Error(&pidZ));
		}
		avgError /= CALIBRATION_STEP;
	} while (avgError > maxErr);
}

void MPU_CalibrateAccel(const MPU* mpu, const Vector gravity, const MPU_TransferBusy transferBusy)
{
	const float maxErr = 100.0f;
	const float Ki     = 0.05f;
	float       avgError;
	PID         pidX;
	PID         pidY;
	PID         pidZ;

	PID_Init(&pidX, 0, Ki, 0);
	PID_Init(&pidY, 0, Ki, 0);
	PID_Init(&pidZ, 0, Ki, 0);

	PID_Target(&pidX, gravity.X * 16384.0f);
	PID_Target(&pidY, gravity.Y * 16384.0f);
	PID_Target(&pidZ, gravity.Z * 16384.0f);

	do {
		avgError = 0;
		for (uint8_t i = 0; i < CALIBRATION_STEP; i++)
		{
			MPU_RequestRawAccel(mpu, NULL);
			while (transferBusy(mpu));

			MPURaw    raw = MPU_RawAccel(mpu);
			MPUOffset offset;
			offset.X = PID_Output(&pidX, raw.X, 1.0f);
			offset.Y = PID_Output(&pidY, raw.Y, 1.0f);
			offset.Z = PID_Output(&pidZ, raw.Z, 1.0f);

			MPU_SetAccelOffset(mpu, offset);
			while (transferBusy(mpu));

			avgError += fabsf(PID_Error(&pidX));
			avgError += fabsf(PID_Error(&pidY));
			avgError += fabsf(PID_Error(&pidZ));
		}
		avgError /= CALIBRATION_STEP;
	} while (avgError > maxErr);
}
