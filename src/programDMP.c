#include "mpu6050.h"
#include "private.h"

#define DMP_MEM_SIZE   3062
#define DMP_CHUNK_SIZE 16
#define DMP_BANK_SIZE  256

#define DMP_MEMORY_REGISTER  0x6D
#define DMP_PROGRAM_REGISTER 0x6F

#define MEMORY_LOCATION_SIZE 3

typedef struct _Memory_
{
	uint8_t Register;
	union
	{
		uint16_t Location;
		struct
		{
			uint8_t Bank;
			uint8_t Address;
		};
	};
} Memory;

const uint8_t program[DMP_MEM_SIZE] = {
#include "motionApp.txt"
};

void programDMP(const MPU* mpu)
{
	uint8_t buff[DMP_CHUNK_SIZE + 1];
	// Set memory bank
	Memory working;
	working.Register = DMP_MEMORY_REGISTER;
	working.Location = 0x00;
	buff[0]          = DMP_PROGRAM_REGISTER;

	while (!mpu->Write(&working, MEMORY_LOCATION_SIZE));
	while (working.Location < DMP_MEM_SIZE)
	{
		uint8_t chunkSize = DMP_CHUNK_SIZE;

		// Make sure we don't go past the data size
		if (working.Location + chunkSize > DMP_MEM_SIZE)
			chunkSize = DMP_MEM_SIZE - working.Location;

		// Make sure this chunk does not go past bank boundry
		if (chunkSize > DMP_BANK_SIZE - working.Address)
			chunkSize = DMP_BANK_SIZE - working.Address;

		// Copy data to working buffer
		memcpy(&buff[1], &program[working.Location], chunkSize);

		// Write to MPU
		while (!mpu->Write(buff, chunkSize));

		working.Location += chunkSize;
		if (working.Address == 0 && working.Location < DMP_MEM_SIZE)
		{
			working.Location = BIG_ENDIAN_16(working.Location);
			while (!mpu->Write(&working, MEMORY_LOCATION_SIZE));
			working.Location = BIG_ENDIAN_16(working.Location);
		}
	}

	uint8_t startAddress[] = {0x70, 0x04, 0x00}; // Program Start address
	while (!mpu->Write(startAddress, sizeof(startAddress)));

	uint8_t dmpINT[] = {0x38, 0x02}; // Enable DMP INT;
	while (!mpu->Write(dmpINT, sizeof(dmpINT)));
}
