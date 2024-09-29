#ifndef _MPU6050_H_
#define _MPU6050_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*MPU_Complete)(bool success, uint8_t deviceID, size_t size);
typedef size_t (*MPU_DataTransfer)(void* data, size_t size);
typedef bool (*MPU_DataRequest)(size_t size, MPU_Complete completed);

typedef struct _MPU_
{
	MPU_DataTransfer Read;
	MPU_DataTransfer Write;
	MPU_DataRequest  Request;
} MPU;

void MPU_Init(MPU* mpu, MPU_DataTransfer read, MPU_DataTransfer write, MPU_DataRequest request);
void MPU_Deinit(MPU* mpu);

void MPU_Configure(MPU* mpu);
void MPU_Enable(MPU* mpu);
void MPU_Disable(MPU* mpu);

void MPU_SetGyroOffset(MPU* mpu);
void MPU_SetAccelOffset();

void MPU_RequestGyroOffset(MPU* mpu);
void MPU_GyroOffset(MPU* mpu);

void MPU_RequestAccelOffset();
void MPU_AccelOffset();

void MPU_RequestRawGyro();
void MPU_RawGyro();

void MPU_RequestRawAccel();
void MPU_RawAccel();

void MPU_RequestTemperature();
void MPU_Temperature();

void MPU_RequestAvailablePackets();
void MPU_AvailablePackets();

void MPU_RequestPacket();
void MPU_Accel();
void MPU_Gyro();
void MPU_YawPitchRoll();

void MPU_CalibrateAccel();
void MPU_CalibrateGyro();

#endif
