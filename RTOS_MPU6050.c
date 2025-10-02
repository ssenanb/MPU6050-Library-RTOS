/*
 * MPU6050.c
 *
 *  Created on: Aug 14, 2025
 *      Author: sssen
 *
 *  This file is an I2C-based driver for the MPU6050 accelerometer/gyroscope sensor.
 */

#include "RTOS_MPU6050.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;

extern osMutexId_t i2cMutexHandle;

static MPU6050_Calibration_Data_t data;

static MPU6050ReadStatus_t MPU6050_ReadRegisterData(uint8_t registerAddress, uint8_t sizeofData, uint8_t *dataBuffer){
	if(osMutexAcquire(i2cMutexHandle, 1000) == osOK){
		if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEVICE_ADDRESS, registerAddress, 1, dataBuffer, sizeofData, 1000) == HAL_OK) {
			osMutexRelease(i2cMutexHandle);
			return READ_SUCCESS;
		}
	}
		return READ_FAIL;
}

static MPU6050WriteStatus_t MPU6050_WriteRegisterData(uint8_t registerAddress, uint8_t value){
	if(osMutexAcquire(i2cMutexHandle, 1000) == osOK){
		if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEVICE_ADDRESS, registerAddress, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000) == HAL_OK) {
			osMutexRelease(i2cMutexHandle);
			return WRITE_SUCCESS;
		}
	}
		return WRITE_FAIL;
}

static MPU6050CalibrationStatus_t calibrateIMU(){
	 int16_t acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;

	 uint8_t acc_data[6] = {0};
	 uint8_t gyro_data[6] = {0};

	 data.x_acc_offset = 0, data.y_acc_offset = 0, data.z_acc_offset = 0;
	 data.x_gyro_offset = 0, data.y_gyro_offset = 0, data.z_gyro_offset = 0;

	for(int i = 0; i < SAMPLES; i++){
		if(MPU6050_ReadRegisterData(ACCEL_MEASUREMENT, 6, acc_data) != READ_SUCCESS) return CALIBRATION_FAIL;

		acc_x = ((int16_t)acc_data[0] << 8) | acc_data[1];
		acc_y = ((int16_t)acc_data[2] << 8) | acc_data[3];
		acc_z = ((int16_t)acc_data[4] << 8) | acc_data[5];

		if(MPU6050_ReadRegisterData(GYRO_MEASUREMENT, 6, gyro_data) != READ_SUCCESS) return CALIBRATION_FAIL;

		gyro_x = ((int16_t)gyro_data[0] << 8) | gyro_data[1];
		gyro_y = ((int16_t)gyro_data[2] << 8) | gyro_data[3];
		gyro_z = ((int16_t)gyro_data[4] << 8) | gyro_data[5];


		data.x_acc_offset += acc_x;
		data.y_acc_offset += acc_y;
		data.z_acc_offset += acc_z;

		data.x_gyro_offset += gyro_x;
		data.y_gyro_offset += gyro_y;
		data.z_gyro_offset += gyro_z;

		osDelay(5); // reading range
	}

		data.x_acc_offset /= SAMPLES;
		data.y_acc_offset /= SAMPLES;
		data.z_acc_offset /= SAMPLES;


		data.x_gyro_offset /= SAMPLES;
		data.y_gyro_offset /= SAMPLES;
		data.z_gyro_offset /= SAMPLES;

		return CALIBRATION_SUCCESS;
}

MPU6050InitStatus_t MPU6050_Init(void){
	uint8_t checkData = 0;

	PowerManagmentRegister powerManagment = {0};
	ConfigRegister configRegister = {0};
	AccelometerConfig accelometerConfig = {0};
	GyroscopeConfig gyroscopeConfig = {0};

	uint8_t tempReg = 0;

	// Verify the MPU6050
	if(MPU6050_ReadRegisterData(WHO_AM_I, 1, &checkData) != READ_SUCCESS) return INIT_FAIL;

	if(checkData != WHO_AM_I_VALUE) return INIT_FAIL;

	// Wake up the MPU6050
	powerManagment.Temp_Dis = 0x00;
	tempReg = *((uint8_t*)&powerManagment);

	if(MPU6050_WriteRegisterData(PWR_MGMT_1, tempReg) != WRITE_SUCCESS) return INIT_FAIL;

	osDelay(5);

	// DLPF configuration
	configRegister.DLPF_CFG = DLPF_CFG_3;
	tempReg = *((uint8_t*)&configRegister);

	if(MPU6050_WriteRegisterData(CONFIG, tempReg) != WRITE_SUCCESS) return INIT_FAIL;

	osDelay(5);

	// Accelerometer configuraiton
	accelometerConfig.AFS_SEL = AFS_SEL_4G;
	tempReg = *((uint8_t*)&accelometerConfig);

	if(MPU6050_WriteRegisterData(ACCEL_CONFIG, tempReg) != WRITE_SUCCESS) return INIT_FAIL;

	osDelay(5);

	// Gyroscope configuration
	gyroscopeConfig.FS_SEL = FS_SEL_500;
	tempReg = *((uint8_t*)&gyroscopeConfig);

	if(MPU6050_WriteRegisterData(GYRO_CONFIG, tempReg) != WRITE_SUCCESS) return INIT_FAIL;

	osDelay(5);

	/* Gyroscope Output Rate 1kHz when the DLPF is enabled.
	 * SampleRate = GyroOutputRate/(1+SMPLRT_DIV)
	 * 1kHz/(1+7)=125 Hz
	 */

	if(MPU6050_WriteRegisterData(SMPRT_DIV, SAMPLE_RATE) != WRITE_SUCCESS) return INIT_FAIL;

	osDelay(5);

	if(calibrateIMU() != CALIBRATION_SUCCESS) return INIT_FAIL;

	return INIT_SUCCESS;
}

/*
 * @brief  Reading accelerometer and gyroscope data from the MPU6050 .
 * @param  values: An instance of the struct that holds the sensor readings.
 * @retval MPU6050GetStatus_t: SUCCES or FAIL
 */

MPU6050GetStatus_t MPU6050_getAccelandGyroValue(MPU6050_Data_t *values){

	uint8_t acc_data[6] = {0};
	uint8_t gyro_data[6] = {0};

	if(MPU6050_ReadRegisterData(ACCEL_MEASUREMENT, 6, acc_data) != READ_SUCCESS) return GET_FAIL;

	values->acc_x_raw = (int16_t)(acc_data[0] << 8) | acc_data[1];
	values->acc_y_raw = (int16_t)(acc_data[2] << 8) | acc_data[3];
	values->acc_z_raw = (int16_t)(acc_data[4] << 8) | acc_data[5];

	values->acc_x = (float)((values->acc_x_raw - data.x_acc_offset) / ACC_SENSIVITY_4G);
	values->acc_y = (float)((values->acc_y_raw - data.y_acc_offset) / ACC_SENSIVITY_4G);
	values->acc_z = (float)((values->acc_z_raw - data.z_acc_offset) / ACC_SENSIVITY_4G);

	osDelay(10);

	if(MPU6050_ReadRegisterData(GYRO_MEASUREMENT, 6, gyro_data) != READ_SUCCESS) return GET_FAIL;

	values->gyro_x_raw = (int16_t)(gyro_data[0] << 8) | gyro_data[1];
	values->gyro_y_raw = (int16_t)(gyro_data[2] << 8) | gyro_data[3];
	values->gyro_z_raw = (int16_t)(gyro_data[4] << 8) | gyro_data[5];

	values->gyro_x = (float)((values->gyro_x_raw - data.x_gyro_offset) / GYRO_SENSIVITY_500);
	values->gyro_y = (float)((values->gyro_y_raw - data.y_gyro_offset) / GYRO_SENSIVITY_500);
	values->gyro_z = (float)((values->gyro_z_raw - data.z_gyro_offset) / GYRO_SENSIVITY_500);

	return GET_SUCCESS;
}

