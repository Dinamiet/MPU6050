#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "quaternion.h"
#include "vector.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct _MPU_ MPU;

typedef void (*MPU_Complete)(bool success, uint8_t deviceID, size_t size);
typedef size_t (*MPU_DataRead)(void* data, const size_t size);
typedef size_t (*MPU_DataWrite)(const void* data, const size_t size);
typedef bool (*MPU_DataRequest)(const size_t size, const MPU_Complete completed);
typedef bool (*MPU_TransferBusy)(const MPU* mpu);

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
	MPU_DataRead    Read;
	MPU_DataWrite   Write;
	MPU_DataRequest Request;
} MPU;

void MPU_Init(MPU* mpu, const MPU_DataRead read, const MPU_DataWrite write, const MPU_DataRequest request);
void MPU_Deinit(const MPU* mpu);

void MPU_Configure(const MPU* mpu);
void MPU_Enable(const MPU* mpu);
void MPU_Disable(const MPU* mpu);

void      MPU_SetGyroOffset(const MPU* mpu, const MPUOffset offset);
void      MPU_RequestGyroOffset(const MPU* mpu, const MPU_Complete ready);
MPUOffset MPU_GyroOffset(const MPU* mpu);
void      MPU_RequestRawGyro(const MPU* mpu, const MPU_Complete ready);
MPURaw    MPU_RawGyro(const MPU* mpu);
void      MPU_CalibrateGyro(const MPU* mpu, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy);

void      MPU_SetAccelOffset(const MPU* mpu, const MPUOffset offset);
void      MPU_RequestAccelOffset(const MPU* mpu, const MPU_Complete ready);
MPUOffset MPU_AccelOffset(const MPU* mpu);
void      MPU_RequestRawAccel(const MPU* mpu, const MPU_Complete ready);
MPURaw    MPU_RawAccel(const MPU* mpu);
void      MPU_CalibrateAccel(const MPU* mpu, const Vector gravity, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy);

void  MPU_RequestTemperature(const MPU* mpu, const MPU_Complete ready);
float MPU_Temperature(const MPU* mpu);

void     MPU_RequestAvailablePackets(const MPU* mpu, const MPU_Complete ready);
uint16_t MPU_AvailablePackets(const MPU* mpu);

void       MPU_RequestPacket(const MPU* mpu, const MPU_Complete ready);
Vector     MPU_PacketAccel(const MPU* mpu);
Vector     MPU_PacketGyro(const MPU* mpu);
Quaternion MPU_PacketQuaternion(const MPU* mpu);

#endif
