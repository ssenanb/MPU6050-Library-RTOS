/*
 * MPU6050.h
 *
 *  Created on: Aug 14, 2025
 *      Author: sssen
 */

#ifndef INC_RTOS_MPU6050_H_
#define INC_RTOS_MPU6050_H_

#include "main.h"

/* MPU6050 registers */
#define MPU6050_DEVICE_ADDRESS 		0xD0 // 0x68 << 1
#define WHO_AM_I               		0x75
#define WHO_AM_I_VALUE				0x68
#define PWR_MGMT_1					0x6B
#define ACCEL_CONFIG				0x1C
#define GYRO_CONFIG					0x1B
#define ACCEL_MEASUREMENT			0x3B
#define GYRO_MEASUREMENT			0x43
#define CONFIG						0x1A
#define SMPRT_DIV					0x19
#define SAMPLE_RATE					0x07

/* Accelerometer Full Scale Range */
#define AFS_SEL_2G					0X00
#define AFS_SEL_4G					0x01
#define AFS_SEL_8G					0x02
#define AFS_SEL_16G					0x03

/* Gyroscope Full Scale Range */
#define FS_SEL_250					0x00
#define FS_SEL_500					0x01
#define AFS_SEL_1000				0x02
#define AFS_SEL_2000				0x03

/* Digital Low Pass Filter (DLPF) */
#define DLPF_CFG_0					0x00
#define DLPF_CFG_1					0x01
#define DLPF_CFG_2					0x02
#define DLPF_CFG_3					0x03
#define DLPF_CFG_4					0x04
#define DLPF_CFG_5					0x05
#define DLPF_CFG_6					0x06
#define DLPF_CFG_7 					0x07

/* Accelerometer Sensitivities */
#define ACC_SENSIVITY_2G			(float)16384.0
#define ACC_SENSIVITY_4G			(float)8192.0
#define ACC_SENSIVITY_8G			(float)4096.0
#define ACC_SENSIVITY_16G			(float)2048.0

/* Gyroscope Sensitivities */
#define GYRO_SENSIVITY_250			(float)131.0
#define GYRO_SENSIVITY_500			(float)65.5
#define GYRO_SENSIVITY_1000			(float)32.8
#define GYRO_SENSIVITY_2000			(float)16.4

/* Offset Time */
#define SAMPLES 					150


typedef enum{
	READ_SUCCESS = 1,
	READ_FAIL = 0,
}MPU6050ReadStatus_t;

typedef enum{
	WRITE_SUCCESS = 1,
	WRITE_FAIL = 0,
}MPU6050WriteStatus_t;

typedef enum{
	INIT_SUCCESS = 1,
	INIT_FAIL = 0,
}MPU6050InitStatus_t;

typedef enum{
	GET_SUCCESS = 1,
	GET_FAIL = 0,
}MPU6050GetStatus_t;

typedef enum{
	CALIBRATION_SUCCESS = 1,
	CALIBRATION_FAIL = 0,
}MPU6050CalibrationStatus_t;

/* PWR_MGMT_1 bits*/
typedef struct{
	uint8_t ClkSel: 3;
	uint8_t Temp_Dis: 1;
	uint8_t Reserved: 1;
	uint8_t Cycle: 1;
	uint8_t Sleep: 1;
	uint8_t Device_Reset: 1;
}PowerManagmentRegister;

/* CONFIG bits */
typedef struct{
	uint8_t DLPF_CFG: 3;
	uint8_t EXT_SYNC_SET: 3;
	uint8_t Reserved: 2;
}ConfigRegister;

/* ACCEL_CONFIG bits */
typedef struct{
	uint8_t Reserved: 3;
	uint8_t AFS_SEL: 2;
	uint8_t ZA_ST: 1;
	uint8_t YA_ST: 1;
	uint8_t XA_ST: 1;
}AccelometerConfig;

/* GYRO_CONFIG bits */
typedef struct{
	uint8_t Reserved: 3;
	uint8_t FS_SEL: 2;
	uint8_t ZG_ST: 1;
	uint8_t YG_ST: 1;
	uint8_t XG_ST: 1;
}GyroscopeConfig;

/* for offset variables */
typedef struct{
	int16_t x_acc_offset, y_acc_offset, z_acc_offset;
	int16_t x_gyro_offset, y_gyro_offset, z_gyro_offset;
}MPU6050_Calibration_Data_t;

/* for MPU6050 variables */
typedef struct{
	int16_t acc_x_raw, acc_y_raw, acc_z_raw;
	int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
	float acc_x, acc_y, acc_z;
	float gyro_x, gyro_y, gyro_z;
}MPU6050_Data_t;

MPU6050InitStatus_t MPU6050_Init(void);
MPU6050GetStatus_t MPU6050_getAccelandGyroValue(MPU6050_Data_t *values);

#endif /* INC_RTOS_MPU6050_H_ */
