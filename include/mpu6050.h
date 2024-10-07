#ifndef _MPU6050_H_
#define _MPU6050_H_

/**
 * \file
 * MPU6050
 *
 * MPU6050 interface library
 */

#include "quaternion.h"
#include "vector.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * MPU6050
 */
typedef struct _MPU_ MPU;

/**
 * Callback function when a transaction completes and data is available for processing
 * \param success Was the transaction successful
 * \param mpu Device for which the transaction completed
 * \param size The size of the transaction
 */
typedef void (*MPU_Complete)(const bool success, const MPU* mpu, const size_t size);

/**
 * Data read interface, implementation specifies how data is read from the MPU
 * \param data Read data should be copied to this buffer
 * \param size The number of bytes to read
 * \return The number of bytes read
 */
typedef size_t (*MPU_DataRead)(void* data, const size_t size);

/**
 * Data write interface, implementation specifies how data is written to the MPU
 * \param data Data to be written to the device
 * \param size The number of bytes to write to the device
 * \return The number of bytes written
 */
typedef size_t (*MPU_DataWrite)(const void* data, const size_t size);

/**
 * Data request interface, implementation specifies how data is requestde from the MPU
 * \param size Number of bytes to request from the MPU
 * \param completed Callback to notify when request is completed and data is available for reading
 * \return Should return true if request could be done
 */
typedef bool (*MPU_DataRequest)(const MPU* mpu, const size_t size, const MPU_Complete completed);

/**
 * Checking if a transfer is still in progress
 * \param mpu MPU6050 to check
 * \return True if a transfer is in progress
 */
typedef bool (*MPU_TransferBusy)(const MPU* mpu);

/**
 * Reading DMP Firmware from storage - can be external storage
 * \param data Location to place read DMP firmware data
 * \param offset Byte offset of DMP firmware
 * \param size Number of bytes to read
 */
typedef size_t (*MPU_ReadDMPFirmware)(void* data, const size_t offset, const size_t size);

/**
 * MPU Offset structure
 */
typedef struct _MPUOffset_
{
	int16_t X; /** X axis offset */
	int16_t Y; /** Y axis offset */
	int16_t Z; /** Z axis offset */
} MPUOffset;

/**
 * MPU Raw structure
 */
typedef struct _MPURaw_
{
	int16_t X; /** X axis */
	int16_t Y; /** Y axis */
	int16_t Z; /** Z axis */
} MPURaw;

/**
 * MPU6050
 */
typedef struct _MPU_
{
	MPU_DataRead    Read;    /** Device data read specification */
	MPU_DataWrite   Write;   /** Device data write specification */
	MPU_DataRequest Request; /** Device data request specification */
} MPU;

/**
 * Initialize MPU device structure. Binds the mpu to a hardware interface by providing the required read, write and request function templates.
 * MPU device is also reset to provide a clean startup state.
 * \note MPU requires some time to startup after the reset, give it some time (~10ms).
 * \param mpu The MPU to initialize
 * \param read Interface data read specification/implementation
 * \param write Interface data write specification/implementation
 * \param request Interface data requset specification/implementation
 */
void MPU_Init(MPU* mpu, const MPU_DataRead read, const MPU_DataWrite write, const MPU_DataRequest request);

/**
 * Deinitializes the MPU
 * \param mpu Device to deinitialize/power-off
 */
void MPU_Deinit(const MPU* mpu);

/**
 * Configures required registers of the MPU, programs DMP
 * \note This function may take some time to complete as it is semi-blocking.
 * \param mpu Device to configure
 * \param firmwareRead Function to read DMP firmware.
 * \note DMP firmware is defined in mpu6050_dmpfw.txt - This was done to allow the firmware to be loaded on external/larger storage since some
 * controllers have limited memory
 */
void MPU_Configure(const MPU* mpu, MPU_ReadDMPFirmware firmwareRead);

/**
 * Enables device motion processing and resets Fifo
 * \param mpu Device to enable
 */
void MPU_Enable(const MPU* mpu);

/**
 * Disables device motion processing and resets Fifo
 * \param mpu Device to disable
 */
void MPU_Disable(const MPU* mpu);

/**
 * Set the Gyro offsets
 * \param mpu Device's offsets to adjust/change
 * \param offset New offset
 */
void MPU_SetGyroOffset(const MPU* mpu, const MPUOffset offset);

/**
 * Retrieve existing gyro offsets
 * \note This function needs to be called before MPU_GyroOffset to ensure the data is available.
 * \param mpu Devices offsets to retrieve
 * \param complete Called when offset retrieval is done and can be read
 */
void MPU_RequestGyroOffset(const MPU* mpu, const MPU_Complete complete);

/**
 * Read retrieved gyro offsets
 * \note This function needs to be called after MPU_RequestGyroOffset to ensure the data is available.
 * \param mpu Device's offsets to read
 * \return Device's current offsets
 */
MPUOffset MPU_GyroOffset(const MPU* mpu);

/**
 * Retieve raw gyro data
 * \note This function needs to be called before MPU_RawGyro to ensure the data is available.
 * \param mpu Device's raw data to retrieve
 * \param complete Called when raw data retrieval is done and can be read
 */
void MPU_RequestRawGyro(const MPU* mpu, const MPU_Complete complete);

/**
 * Read raw gyro data
 * \note This function needs to be called after MPU_RequestRawGyro to ensure the data is available.
 * \param mpu Device's raw data to read
 * \return Raw gyro data
 */
MPURaw MPU_RawGyro(const MPU* mpu);

/**
 * Calibrates the gyro using PID control loop
 * \note This function might take some time as it is blocking until calibration is done
 * \param mpu Device to calibrate
 * \param minItt Minimum itterations to ensure stable calibration
 * \param maxErr Maximum allowed error to ensure stable calibration
 * \param transferBusy Interface function to ensure calibration is done with adjusted values
 */
void MPU_CalibrateGyro(const MPU* mpu, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy);

/**
 * Set the accelerometer offsets
 * \param mpu Device's offsets to adjust/change
 * \param offset New offset
 */
void MPU_SetAccelOffset(const MPU* mpu, const MPUOffset offset);

/**
 * Retrieve existing accelerometer offsets
 * \note This function needs to be called before MPU_AccelOffset to ensure the data is available.
 * \param mpu Devices offsets to retrieve
 * \param complete Called when offset retrieval is done and can be read
 */
void MPU_RequestAccelOffset(const MPU* mpu, const MPU_Complete complete);

/**
 * Read retrieved accelerometer offsets
 * \note This function needs to be called after MPU_RequestAccelOffset to ensure the data is available.
 * \param mpu Device's offsets to read
 * \return Device's current offsets
 */
MPUOffset MPU_AccelOffset(const MPU* mpu);

/**
 * Retieve raw accelerometer data
 * \note This function needs to be called before MPU_RawAccel to ensure the data is available.
 * \param mpu Device's raw data to retrieve
 * \param complete Called when raw data retrieval is done and can be read
 */
void MPU_RequestRawAccel(const MPU* mpu, const MPU_Complete complete);

/**
 * Read raw accelerometer data
 * \note This function needs to be called after MPU_RequestRawAccel to ensure the data is available.
 * \param mpu Device's raw data to read
 * \return Raw gyro data
 */
MPURaw MPU_RawAccel(const MPU* mpu);

/**
 * Calibrates the accelerometer using PID control loop
 * \note This function might take some time as it is blocking until calibration is done
 * \param mpu Device to calibrate
 * \param gravity Unit vector indicating the acceleration component of gravity of each axis
 * \param minItt Minimum itterations to ensure stable calibration
 * \param maxErr Maximum allowed error to ensure stable calibration
 * \param transferBusy Interface function to ensure calibration is done with adjusted values
 */
void MPU_CalibrateAccel(const MPU* mpu, const Vector gravity, const uint8_t minItt, const uint8_t maxErr, const MPU_TransferBusy transferBusy);

/**
 * Retieve the temperature reading of the devices temperature sensor
 * \note This function needs to be called before MPU_Temperature to ensure the data is available
 * \param mpu Device's temperature to retrieve
 * \param complete Called when temperature retrieval is done and can be read
 */
void MPU_RequestTemperature(const MPU* mpu, const MPU_Complete complete);

/**
 * Read the temperature data
 * \note This function needs to be called after MPU_RequestTemperature to ensure data is available
 * \param mpu Device's temperature to read
 * \return Degrees celcius
 */
float MPU_Temperature(const MPU* mpu);

/**
 * Request the available Motion app packets
 * \note This function needs to be called before MPU_AvailablePackets to ensure data is available
 * \param mpu Device's packets to retrieve
 * \param complete Called when available packet retrieval is done and can be read
 */
void MPU_RequestAvailablePackets(const MPU* mpu, const MPU_Complete complete);

/**
 * Read the available packets data
 * \note This function needs to be called after MPU_RequestAvailablePackets to ensure data is available
 * \param mpu Devices available packets to read
 * \return Number of motion app packets available
 */
uint16_t MPU_AvailablePackets(const MPU* mpu);

/**
 * Retrieve a motion app packet from device
 * \note This function needs to be called before MPU_PacketAccel or MPU_PacketGyro or MPU_PacketQuaternion to ensure data is available
 * \param mpu Device to retrieve a packet from
 * \param complete Called when packet retrieval is done and data can be read
 */
void MPU_RequestPacket(const MPU* mpu, const MPU_Complete complete);

/**
 * Reads the acceleration information from the motion app packet
 * \note This function needs to be called after MPU_RequestPacket to ensure data is available
 * \param mpu Device's packet to read
 * \return Acceleration vector (G)
 */
Vector MPU_PacketAccel(const MPU* mpu);

/**
 * Reads the gyro information from the motion app packet
 * \note This function needs to be called after MPU_RequestPacket to ensure data is available
 * \param mpu Device's packet to read
 * \return Rotation vector (rad/s)
 */
Vector MPU_PacketGyro(const MPU* mpu);

/**
 * Reads the orientation information from the motion app packet
 * \note This function needs to be called after MPU_RequestPacket to ensure data is available
 * \param mpu Device's packet to read
 * \return Orientation quaterion
 */
Quaternion MPU_PacketQuaternion(const MPU* mpu);

#endif
