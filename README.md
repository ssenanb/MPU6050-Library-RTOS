# MPU6050-Library

MPU6050 library: thread-safe (RTOS) and standard versions included. 

Developed an MPU6050 library providing HAL I2C access, accelerometer/gyroscope calibration, normalized sensor data output (g and °/s), and support for both RTOS (thread-safe) and standard applications.

# How To Run?

**Standard Version**

Firstly, include the **MPU6050.h** header file in your main.c. Sensor data is stored in the **MPU6050_Data_t"** struct, so an object of this type is created. The **MPU6050_Init** function should be called once at the beginning of main. Inside the main loop, call **MPU6050_getAccelandGyroValue(&values)** to read the latest accelorometer and gyroscope data. This allows you to monitor the sensor readings continuously.

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
```c





  

