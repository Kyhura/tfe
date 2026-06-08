/*
 *  Montefiore Robocup
 *  ==================
 *
 *  imu_sensor.h: Inertial mesurement sensor (AS5045B) setup and operations.
 */

#ifndef _IMU_SENSOR_H
#define _IMU_SENSOR_H

#include <stdint.h>

/*** Definitions  ***/

#define I2C_IMU_ADDR         0x6b
#define IMU_WHO_AM_I         0x0f
#define IMU_CTRL1            0x10
#define IMU_CTRL2            0x11

#define GYR_OUT_X_L           0x22
#define GYR_OUT_X_H           0x23
#define GYR_OUT_Y_L           0x24
#define GYR_OUT_Y_H           0x25
#define GYR_OUT_Z_L           0x26
#define GYR_OUT_Z_H           0x27

#define ACC_OUT_X_L          0x28
#define ACC_OUT_X_H          0x29
#define ACC_OUT_Y_L          0x2a
#define ACC_OUT_Y_H          0x2b
#define ACC_OUT_Z_L          0x2c
#define ACC_OUT_Z_H          0x2d

#define GYR_X                0x1
#define GYR_Y                0x2
#define GYR_Z                0x3
#define ACC_X                0x4
#define ACC_Y                0x5
#define ACC_Z                0x6


#define IMU_PORT             GPIOA
#define SCL                  GPIO9
#define SDA                  GPIO10

/***  Prototypes of public functions  ***/

void imu_sensor_setup(void);
int16_t read_ang_rate_X_sensor(void);
int16_t read_ang_rate_Y_sensor(void);
int16_t read_ang_rate_Z_sensor(void);
int16_t read_lin_acc_X_sensor(void);
int16_t read_lin_acc_Y_sensor(void);
int16_t read_lin_acc_Z_sensor(void);


#endif  /* _IMU_SENSOR_H */
