#ifndef __ICM2068X_H
#define __ICM2068X_H
#include "myiic.h"   												  	  
#include "Filter.h"



#define IMU_ICM20689_ID  0x98
//****************************************
// MPU6050



#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG  0x1C
#define ACCEL_CONFIG2 0x1D
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I		  0x75
#define USER_CTRL		  0x6A
#define INT_PIN_CFG		0x37

#define ICM20689_ADRESS		0x68


void IMU_Calibration(void);
uint8_t ICM20689_Init(void);
void ICM20689_Read_Data(vector3f *gyro,vector3f *accel,float *temperature);
extern Vector3f gyro_offset;
extern lpf_param accel_lpf_param;
extern lpf_param gyro_lpf_param;
extern lpf_param cal_lpf_param,ins_lpf_param,accel_fb_lpf_param;



extern uint8_t imu_address;
extern IMU_ID_READ IMU_ID;

#endif




































