#include "internals.h"

#define DMP_MEM_SIZE   3062
#define DMP_CHUNK_SIZE 16
#define DMP_BANK_SIZE  256

#define DMP_BANK_REGISTER    0x6D
#define DMP_ADDRESS_REGISTER 0x6E
#define DMP_PROGRAM_REGISTER 0x6F
#define DMP_PROGRAM_START_REGISTER 0x70

bool setRegister(const MPU* mpu, const uint8_t reg, const uint8_t value) { return mpu->Write(mpu, reg, &value, sizeof(value)); }

void programDMP(const MPU* mpu, MPU_ReadDMPFirmwareInterface read_interface)
{
	uint8_t  buff[DMP_CHUNK_SIZE];
	uint16_t location = 0x00;

	while (!setRegister(mpu, DMP_BANK_REGISTER, (location & 0xFF00) >> 8));
	while (!setRegister(mpu, DMP_ADDRESS_REGISTER, location & 0xFF));
	while (location < DMP_MEM_SIZE)
	{
		uint8_t chunkSize = DMP_CHUNK_SIZE;

		// Make sure we don't go past the data size
		if (location + chunkSize > DMP_MEM_SIZE)
			chunkSize = DMP_MEM_SIZE - location + 1;

		// Make sure this chunk does not go past bank boundry
		if (chunkSize > DMP_BANK_SIZE - (location & 0xFF))
			chunkSize = DMP_BANK_SIZE - (location & 0xFF);

		// Read data to working buffer
		read_interface(&buff, location, chunkSize);

		// Write to MPU
		while (!mpu->Write(mpu, DMP_PROGRAM_REGISTER, buff, chunkSize));

		location += chunkSize;
		if ((location & 0xFF) == 0 && location < DMP_MEM_SIZE)
		{
			while (!setRegister(mpu, DMP_BANK_REGISTER, (location & 0xFF00) >> 8));
			while (!setRegister(mpu, DMP_ADDRESS_REGISTER, location & 0xFF));
		}
	}

	uint16_t startAddress = BIG_ENDIAN_16(0x0400); // Program Start address
	while (!mpu->Write(mpu, DMP_PROGRAM_START_REGISTER, &startAddress, sizeof(startAddress)));

	while (!setRegister(mpu, 0x38, 0x02)); // Enable DMP INT;
}
