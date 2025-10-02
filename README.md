# MPU6050-Library

MPU6050 library: thread-safe (RTOS) and standard versions included. 

Developed an MPU6050 library providing HAL I2C access, accelerometer/gyroscope calibration, normalized sensor data output (g and °/s), and support for both RTOS (thread-safe) and standard applications.

# How To Run?

**Standard Version**

Firstly, include the **MPU6050.h** header file in your main.c. Sensor data is stored in the **MPU6050_Data_t** struct, so an object of this type is created. The **MPU6050_Init** function should be called once at the beginning of main. Inside the main loop, call **MPU6050_getAccelandGyroValue(&values)** to read the latest accelerometer and gyroscope data. This allows you to monitor the sensor readings continuously.

```c
#include "MPU6050.h"

MPU6050_Data_t data;

int main(void) {
    MPU6050_Init(); // Initialize and calibrate
    while(1) {
        MPU6050_getAccelandGyroValue(&data);
        // Now you can use data.acc_x, data.gyro_x, etc.
    }
}
```

**Thread-Safe Version**

In the RTOS environments, use mutex to prevent data corruption when multi-tasks access the sensor concurrently. 

```c
extern osMutexId_t i2cMutexHandle;
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
```
Before using the library in an RTOS environment, create a task and a mutex to protect I2C access. Then, include the **RTOS_MPU6050.h** header file in your main.c. Sensor data is stored in the **MPU6050_Data_t** struct, so create an object of this type. Call **MPU6050_Init()** once at the beginning of the task. 
Inside the task loop, call **MPU6050_getAccelandGyroValue(&values)** to read the latest accelerometer and gyroscope data. This allows you to monitor sensor readings continuously and safely in a multi-tasking environment.

```c
#include "RTOS_MPU6050.h"

MPU6050_Data_t data;

void StartIMUTask(void *argument)
{
	MPU6050_Init(); // Initialize and calibrate
  for(;;)
  {
	  MPU6050_getAccelandGyroValue(&data);
      osDelay(10); // A short delay prevents overwhelming the I2C bus
      // Now you can use data.acc_x, data.gyro_x, etc.
  }
}
```




  

