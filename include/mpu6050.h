#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "quaternion.h"
#include "vector.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*MPU_Complete)(bool success, uint8_t deviceID, size_t size);
typedef size_t (*MPU_DataTransfer)(void* data, size_t size);
typedef bool (*MPU_DataRequest)(size_t size, MPU_Complete completed);

typedef struct _MPUOffset_
{
	int16_t X;
	int16_t Y;
	int16_t Z;
} MPUOffset;

typedef struct _MPURaw_
{
	int16_t X;
	int16_t Y;
	int16_t Z;
} MPURaw;

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

void      MPU_SetGyroOffset(MPU* mpu, MPUOffset offset);
void      MPU_RequestGyroOffset(MPU* mpu, MPU_Complete ready);
MPUOffset MPU_GyroOffset(MPU* mpu);
void      MPU_RequestRawGyro(MPU* mpu, MPU_Complete ready);
MPURaw    MPU_RawGyro(MPU* mpu);
void      MPU_CalibrateGyro(MPU* mpu, uint8_t minItt, uint8_t maxErr, void (*progress)(char));

void      MPU_SetAccelOffset(MPU* mpu, MPUOffset offset);
void      MPU_RequestAccelOffset(MPU* mpu, MPU_Complete ready);
MPUOffset MPU_AccelOffset(MPU* mpu);
void      MPU_RequestRawAccel(MPU* mpu, MPU_Complete ready);
MPURaw    MPU_RawAccel(MPU* mpu);
void      MPU_CalibrateAccel(MPU* mpu, uint8_t minItt, uint8_t maxErr, void (*progress)(char));

void  MPU_RequestTemperature(MPU* mpu, MPU_Complete ready);
float MPU_Temperature(MPU* mpu);

void     MPU_RequestAvailablePackets(MPU* mpu, MPU_Complete ready);
uint16_t MPU_AvailablePackets(MPU* mpu);

void       MPU_RequestPacket(MPU* mpu, MPU_Complete ready);
Vector     MPU_PacketAccel(MPU* mpu);
Vector     MPU_PacketGyro(MPU* mpu);
Quaternion MPU_PacketQuaternion(MPU* mpu);

#endif
