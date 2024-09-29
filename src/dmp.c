#include "mpu6050.h"
#include "private.h"

#define PACKET_REQUEST       0x74
#define PACKET_AVAILABLE_REG 0x72

typedef struct _DMPPacket_
{
	int32_t Qw : 16;
	int32_t    : 16;
	int32_t Qx : 16;
	int32_t    : 16;
	int32_t Qy : 16;
	int32_t    : 16;
	int32_t Qz : 16;
	int32_t    : 16;
	int16_t Gx;
	int16_t Gy;
	int16_t Gz;
	int16_t Ax;
	int16_t Ay;
	int16_t Az;
} DMPPacket;

void MPU_RequestAvailablePackets(const MPU* mpu, const MPU_Complete ready)
{
	const uint8_t reg = PACKET_AVAILABLE_REG;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(sizeof(uint16_t), ready));
}

uint16_t MPU_AvailablePackets(const MPU* mpu)
{
	uint16_t packets;
	mpu->Read(&packets, sizeof(packets));
	return BIG_ENDIAN_16(packets) / sizeof(DMPPacket);
}

void MPU_RequestPacket(const MPU* mpu, const MPU_Complete ready)
{
	const uint8_t reg = PACKET_REQUEST;
	while (!mpu->Write(&reg, sizeof(reg)));
	while (!mpu->Request(sizeof(DMPPacket), ready));
}

Vector MPU_PacketAccel(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(&packet, sizeof(packet));

	Vector accel = Vector_Create(BIG_ENDIAN_16(packet.Ax), BIG_ENDIAN_16(packet.Ay), BIG_ENDIAN_16(packet.Az));

	return Vector_Scale(accel, 1.0f / 16384.0f);
}

Vector MPU_PacketGyro(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(&packet, sizeof(packet));

	Vector gyro = Vector_Create(BIG_ENDIAN_16(packet.Gx), BIG_ENDIAN_16(packet.Gy), BIG_ENDIAN_16(packet.Gz));

	return Vector_Scale(gyro, 1.0f / 16.4f);
}

Quaternion MPU_PacketQuaternion(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(&packet, sizeof(packet));

	Quaternion q = Quaternion_Make(BIG_ENDIAN_16(packet.Qw), BIG_ENDIAN_16(packet.Qx), BIG_ENDIAN_16(packet.Qy), BIG_ENDIAN_16(packet.Qz));

	return Quaternion_Scale(q, 1.0f / 1638.0f);
}
