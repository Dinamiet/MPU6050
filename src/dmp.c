#include "internals.h"

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

bool MPU_RequestAvailablePackets(const MPU* mpu, const MPU_Complete complete)
{
	return mpu->Request(mpu, PACKET_AVAILABLE_REG, sizeof(uint16_t), complete);
}

uint16_t MPU_AvailablePackets(const MPU* mpu)
{
	uint16_t packets;
	mpu->Read(mpu, &packets, sizeof(packets));
	return BIG_ENDIAN_16(packets) / sizeof(DMPPacket);
}

bool MPU_RequestPacket(const MPU* mpu, const MPU_Complete complete) { return mpu->Request(mpu, PACKET_REQUEST, sizeof(DMPPacket), complete); }

Vector MPU_PacketAccel(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(mpu, &packet, sizeof(packet));

	Vector accel = Vector_Create((int16_t)BIG_ENDIAN_16(packet.Ax), (int16_t)BIG_ENDIAN_16(packet.Ay), (int16_t)BIG_ENDIAN_16(packet.Az));

	return Vector_Scale(accel, 1.0f / 16384.0f);
}

Vector MPU_PacketGyro(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(mpu, &packet, sizeof(packet));

	Vector gyro = Vector_Create((int16_t)BIG_ENDIAN_16(packet.Gx), (int16_t)BIG_ENDIAN_16(packet.Gy), (int16_t)BIG_ENDIAN_16(packet.Gz));

	return Vector_Scale(gyro, 1.0f / 16.4f);
}

Quaternion MPU_PacketQuaternion(const MPU* mpu)
{
	DMPPacket packet;
	mpu->Read(mpu, &packet, sizeof(packet));

	Quaternion q = Quaternion_Make(
			(int16_t)BIG_ENDIAN_16(packet.Qw),
			(int16_t)BIG_ENDIAN_16(packet.Qx),
			(int16_t)BIG_ENDIAN_16(packet.Qy),
			(int16_t)BIG_ENDIAN_16(packet.Qz));

	return Quaternion_Unit(q);
}
