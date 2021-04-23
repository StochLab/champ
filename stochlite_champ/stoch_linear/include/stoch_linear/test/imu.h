#ifndef __IMU_H__
#define __IMU_H__




typedef struct {
	float roll;
	float pitch;
	float yaw;
}sRPY;

void *readIMU(void *d);


#endif // __IMU_H_#ifndef __IMU_H__
