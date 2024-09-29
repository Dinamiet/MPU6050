#include "mpu6050.h"
#include "pid.h"

#include <math.h>

#define CALIBRATION_STEP 6

void MPU_CalibrateGyro(const MPU* mpu, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy)
{
	float Kp, Ki;
	Kp = 0.1f;
	Ki = 45.0f;
	PID   pidX;
	PID   pidY;
	PID   pidZ;
	float errorSum;

	PID_Init(&pidX, Kp, Ki, 0);
	PID_Init(&pidY, Kp, Ki, 0);
	PID_Init(&pidZ, Kp, Ki, 0);

	PID_Target(&pidX, 0);
	PID_Target(&pidY, 0);
	PID_Target(&pidZ, 0);

	for (uint8_t i = 0; i < CALIBRATION_STEP; i++)
	{
		for (size_t j = 0; j < minItt || errorSum > maxErr; j++)
		{
			errorSum = 0;
			MPU_RequestRawGyro(mpu, NULL);
			while (transferBusy(mpu));

			MPURaw    raw = MPU_RawGyro(mpu);
			MPUOffset offset;
			offset.X = PID_Output(&pidX, raw.X, 0.001f);
			offset.Y = PID_Output(&pidY, raw.Y, 0.001f);
			offset.Z = PID_Output(&pidZ, raw.Z, 0.001f);

			MPU_SetGyroOffset(mpu, offset);
			while (transferBusy(mpu));
			errorSum += fabsf(PID_Error(&pidX));
			errorSum += fabsf(PID_Error(&pidY));
			errorSum += fabsf(PID_Error(&pidZ));
		}
		// Smaller steps
		Kp *= 0.75f;
		Ki *= 0.75f;
		PID_SetProportional(&pidX, Kp);
		PID_SetProportional(&pidY, Kp);
		PID_SetProportional(&pidZ, Kp);
		PID_SetIntegral(&pidX, Ki);
		PID_SetIntegral(&pidY, Ki);
		PID_SetIntegral(&pidZ, Ki);
	}
}

void MPU_CalibrateAccel(const MPU* mpu, const Vector gravity, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy)
{
	float Kp, Ki;
	Kp = 0.1f;
	Ki = 10.0f;
	PID   pidX;
	PID   pidY;
	PID   pidZ;
	float errorSum;

	PID_Init(&pidX, Kp, Ki, 0);
	PID_Init(&pidY, Kp, Ki, 0);
	PID_Init(&pidZ, Kp, Ki, 0);

	PID_Target(&pidX, gravity.X * 16384.0f);
	PID_Target(&pidY, gravity.Y * 16384.0f);
	PID_Target(&pidZ, gravity.Z * 16384.0f);

	for (uint8_t i = 0; i < CALIBRATION_STEP; i++)
	{
		for (size_t j = 0; j < minItt || errorSum > maxErr; j++)
		{
			errorSum = 0;
			MPU_RequestRawAccel(mpu, NULL);
			while (transferBusy(mpu));

			MPURaw    raw = MPU_RawAccel(mpu);
			MPUOffset offset;
			offset.X = PID_Output(&pidX, raw.X, 0.001f);
			offset.Y = PID_Output(&pidY, raw.Y, 0.001f);
			offset.Z = PID_Output(&pidZ, raw.Z, 0.001f);

			MPU_SetAccelOffset(mpu, offset);
			while (transferBusy(mpu));

			errorSum += fabsf(PID_Error(&pidX));
			errorSum += fabsf(PID_Error(&pidY));
			errorSum += fabsf(PID_Error(&pidZ));
		}
		// Smaller steps
		Kp *= 0.75f;
		Ki *= 0.75f;
		PID_SetProportional(&pidX, Kp);
		PID_SetProportional(&pidY, Kp);
		PID_SetProportional(&pidZ, Kp);
		PID_SetIntegral(&pidX, Ki);
		PID_SetIntegral(&pidY, Ki);
		PID_SetIntegral(&pidZ, Ki);
	}
}
