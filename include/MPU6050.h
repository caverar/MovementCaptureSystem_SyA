#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif


#define MPU_X_ACCEL_OFFSET_ADDRESS 0x06
#define MPU_Y_ACCEL_OFFSET_ADDRESS 0x08
#define MPU_Z_ACCEL_OFFSET_ADDRESS 0x0A
#define MPU_X_GYRO_OFFSET_ADDRESS 0x13
#define MPU_Y_GYRO_OFFSET_ADDRESS 0x15
#define MPU_Z_GYRO_OFFSET_ADDRESS 0x17

unsigned char* dummyMPURead;

void initMPUPart1(unsigned char i2cPort, unsigned char mpu6050Address);
void initMPUPart2(unsigned char i2cPort, unsigned char mpu6050Address, 
                short MPU_X_ACCEL_OFFSET, short MPU_Y_ACCEL_OFFSET, short MPU_Z_ACCEL_OFFSET, 
                short MPU_X_GYRO_OFFSET, short MPU_Y_GYRO_OFFSET, short MPU_Z_GYRO_OFFSET);
void initMPUPart3(unsigned char i2cPort, unsigned char mpu6050Address, unsigned char interruptPin);

void getMPUData(unsigned char i2cPort, unsigned char deviceAddress, unsigned char* MPUInterruptionCounter, unsigned char* MPUInterruptionFlag,
                int* MPUwQuat, int* MPUxQuat, int* MPUyQuat, int* MPUzQuat,
                short* MPUxGyro, short* MPUyGyro, short* MPUzGyro,
                short* MPUxAccel, short* MPUyAccel, short* MPUzAccel,
                unsigned char** MPURawData, unsigned char* MPUSampleReadyFlag, struct Quaternion* MPUQuaternion, struct Gravity* MPUGravity,
                float* MPUYaw, float* MPUPitch, float* MPURoll,
                float* averageMPUYaw, float* averageMPUPitch, float* averageMPURoll, 
                int* counter, float* offsetMPUYaw, float* offsetMPUPitch, float* offsetMPURoll);




#ifdef __cplusplus
}
#endif
#endif