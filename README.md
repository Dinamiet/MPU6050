# MPU6050

Interface library for the MPU6050.

The MPU6050 is a integrated 6-axis MotionTracking device. It combines a 3-axis gyro and 3-axis accelerometer with a Digital Motion Processor.

The library is aimed to be non-blocking and not linked to specific hardware interfaces. This is achieved by specifying interface data read, data write and data request functions to bind the device to a specific hardware interface.

The non-blocking implementation requires that data is requested before it can be read. Requests should provide a complete/data ready callback to notify the higher system that the data is available.
