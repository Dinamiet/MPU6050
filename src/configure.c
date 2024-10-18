#include "internals.h"

#include <assert.h>

static void enableReadClear(const MPU* mpu, const bool success);

static void enableReadClear(const MPU* mpu, const bool success)
{
	(void)success;

	mpu->Read(NULL, 1);
}

void MPU_Init(MPU* mpu, const MPU_DataRead read, const MPU_DataWrite write, const MPU_DataRequest request)
{
	assert(mpu != NULL);
	assert(read != NULL);
	assert(write != NULL);
	assert(request != NULL);

	mpu->Read    = read;
	mpu->Write   = write;
	mpu->Request = request;

	while (!setRegister(mpu, 0x6B, 0x80)); // Reset IMU
}

void MPU_Deinit(const MPU* mpu)
{
	(void)mpu;
	while (!setRegister(mpu, 0x6B, 0x40)); // Sleep IMU
}

void MPU_Configure(const MPU* mpu, MPU_ReadDMPFirmware firmwareRead)
{
	while (!setRegister(mpu, 0x6B, 0x01)); // Setup clock source
	while (!setRegister(mpu, 0x23, 0x00)); // No Fifo
	while (!setRegister(mpu, 0x38, 0x00)); // No INT
	while (!setRegister(mpu, 0x37, 0x80)); // INT active Low
	while (!setRegister(mpu, 0x1C, 0x00)); // Accelerometer Scale
	while (!setRegister(mpu, 0x1B, 0x18)); // Gyro Scale
	while (!setRegister(mpu, 0x19, 0x04)); // Sample Rate
	while (!setRegister(mpu, 0x1A, 0x01)); // Digital Filter

	programDMP(mpu, firmwareRead);
}

void MPU_Enable(const MPU* mpu)
{
	while (!setRegister(mpu, 0x6A, 0xC4)); // Enable DMP & Fifo, also reset Fifo

	// Read interrupt status register to clear all interrupts
	while (!mpu->Request(0x3A, 1, enableReadClear));
}

void MPU_Disable(const MPU* mpu)
{
	while (!setRegister(mpu, 0x6A, 0x04)); // Disable DMP & Fifo, also reset Fifo
}
