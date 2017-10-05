#ifndef _SENSORS_TASK_
#define _SENSORS_TASK_
#include "../MessageTypes/messages.h"
#include "../MessageTypes/basic_types.h"
void mpu6000Callback(void);
void hmc5883lCallback(void);
void ms5611Callback(void);
void sensorsTaskInit(void);
void margAcquire(marg_t *marg);
void baroAcquire(baro_t *baro);
void gyro_calibrate(vec3f_t* avr_gyr);
#define I2C_DEV_MS5611 1
#define I2C_DEV_HMC5883 0
#define ACC_SCALE (9.81f/8192.0f)
#define GYR_SCALE (7509.9f * 0.5f)
#endif
