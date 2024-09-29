#include "mpu6050.h"
#include "private.h"

#include <assert.h>

static void setRegister(const MPU* mpu, const uint8_t reg, const uint8_t value);

static void setRegister(const MPU* mpu, const uint8_t reg, const uint8_t value)
{
	const uint8_t data[] = {reg, value};
	while (!mpu->Write(data, sizeof(data)));
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

	setRegister(mpu, 0x6B, 0x80); // Reset IMU
}

void MPU_Deinit(const MPU* mpu)
{
	(void)mpu;
	/** TODO: Poweroff device */
}

void MPU_Configure(const MPU* mpu)
{
	setRegister(mpu, 0x6B, 0x01); // Setup clock source
	setRegister(mpu, 0x23, 0x00); // No Fifo
	setRegister(mpu, 0x38, 0x00); // No INT
	setRegister(mpu, 0x37, 0x80); // INT active Low
	setRegister(mpu, 0x1C, 0x00); // Accelerometer Scale
	setRegister(mpu, 0x1B, 0x18); // Gyro Scale
	setRegister(mpu, 0x19, 0x04); // Sample Rate
	setRegister(mpu, 0x1A, 0x01); // Digital Filter

	programDMP(mpu);
}

void MPU_Enable(const MPU* mpu)
{
	setRegister(mpu, 0x6A, 0xC4); // Enable DMP & Fifo, also reset Fifo

	// Read interrupt status register to clear all interrupts
	const uint8_t reg = 0x3A;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(1, NULL)); /** TODO: This requested byte needs to be read */
}

void MPU_Disable(const MPU* mpu)
{
	setRegister(mpu, 0x6A, 0x04); // Disable DMP & Fifo, also reset Fifo
}
