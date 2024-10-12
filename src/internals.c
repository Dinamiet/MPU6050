#include "internals.h"

#define DMP_MEM_SIZE   3062
#define DMP_CHUNK_SIZE 16
#define DMP_BANK_SIZE  256

#define DMP_BANK_REGISTER    0x6D
#define DMP_ADDRESS_REGISTER 0x6E
#define DMP_PROGRAM_REGISTER 0x6F

void setRegister(const MPU* mpu, const uint8_t reg, const uint8_t value)
{
	const uint8_t data[] = {reg, value};
	while (!mpu->Write(data, sizeof(data)));
}

void reqData(const MPU* mpu, const uint8_t reg, const size_t size, MPU_Complete complete)
{
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(mpu, size, complete));
}

void programDMP(const MPU* mpu, MPU_ReadDMPFirmware read)
{
	uint8_t  buff[DMP_CHUNK_SIZE + 1] = {DMP_PROGRAM_REGISTER};
	uint16_t location                 = 0x00;

	setRegister(mpu, DMP_BANK_REGISTER, (location & 0xFF00) >> 8);
	setRegister(mpu, DMP_ADDRESS_REGISTER, location & 0xFF);
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
		read(&buff[1], location, chunkSize);

		// Write to MPU
		while (!mpu->Write(buff, chunkSize));

		location += chunkSize;
		if ((location & 0xFF) == 0 && location < DMP_MEM_SIZE)
		{
			setRegister(mpu, DMP_BANK_REGISTER, (location & 0xFF00) >> 8);
			setRegister(mpu, DMP_ADDRESS_REGISTER, location & 0xFF);
		}
	}

	uint8_t startAddress[] = {0x70, 0x04, 0x00}; // Program Start address
	while (!mpu->Write(startAddress, sizeof(startAddress)));

	setRegister(mpu, 0x38, 0x02); // Enable DMP INT;
}
