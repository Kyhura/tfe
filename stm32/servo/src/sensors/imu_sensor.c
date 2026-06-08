/*
 *  Montefiore Robocup
 *  ==================
 *
 *  imu_sensor.c: Inertial mesurement sensor (AS5045B) setup and operations.
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <semphr.h>


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "imu_sensor.h"

/*** Private variable ***/

static SemaphoreHandle_t sem;

/***  Prototypes of private functions  ***/

static int16_t read_sensor(uint8_t);

/***  Public functions  ***/

void imu_sensor_setup(void) {

  /* Enable GPIO clocks. */

  rcc_periph_clock_enable(RCC_GPIOA);

  /* I2C setup. */

  rcc_periph_clock_enable(RCC_I2C2);
  rcc_set_i2c_clock_hsi(I2C2);

  rcc_periph_reset_pulse(RST_I2C2);

  gpio_mode_setup(IMU_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, SCL | SDA);
  gpio_set_af(IMU_PORT, GPIO_AF4, SCL | SDA);

  i2c_peripheral_disable(I2C2);

  i2c_enable_analog_filter(I2C2);
  i2c_set_digital_filter(I2C2, 0);

  i2c_set_speed(I2C2, i2c_speed_sm_100k, 8);

  i2c_set_7bit_addr_mode(I2C2);
  i2c_peripheral_enable(I2C2);

  /* Semaphore */

  sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sem);

  /* Initate the IMU regsiter */

  xSemaphoreTake(sem, portMAX_DELAY);

  unsigned char i2c_cmd[2], i2c_data;

  i2c_cmd[0] = IMU_CTRL1;
  i2c_cmd[1] = 0x80;

  i2c_transfer7(I2C2, I2C_IMU_ADDR, i2c_cmd, 2, &i2c_data, 0);

  i2c_cmd[0] = IMU_CTRL2;
  i2c_cmd[1] = 0x84;

  i2c_transfer7(I2C2, I2C_IMU_ADDR, i2c_cmd, 2, &i2c_data, 0);

  xSemaphoreGive(sem);

}

static int16_t read_sensor(uint8_t mode) {
  
  int16_t  v;
  unsigned char i2c_cmd[2], i2c_data;

  xSemaphoreTake(sem, portMAX_DELAY);


  if(mode == GYR_X) {
    i2c_cmd[0] = GYR_OUT_X_L;
  } else if(mode == GYR_Y) {
    i2c_cmd[0] = GYR_OUT_Y_L;
  } else if(mode == GYR_Z) {
    i2c_cmd[0] = GYR_OUT_Z_L;
  } else if(mode == ACC_X) {
    i2c_cmd[0] = ACC_OUT_X_L;
  } else if(mode == ACC_Y) {
    i2c_cmd[0] = ACC_OUT_Y_L;
  } else if(mode == ACC_Z){
    i2c_cmd[0] = ACC_OUT_Z_L;
  }
  
  i2c_transfer7(I2C2, I2C_IMU_ADDR, i2c_cmd, 1, &i2c_data, 1);
  v = i2c_data;

  if(mode == GYR_X) {
    i2c_cmd[0] = GYR_OUT_X_H;
  } else if(mode == GYR_Y) {
    i2c_cmd[0] = GYR_OUT_Y_H;
  } else if(mode == GYR_Z) {
    i2c_cmd[0] = GYR_OUT_Z_H;
  } else if(mode == ACC_X) {
    i2c_cmd[0] = ACC_OUT_X_H;
  } else if(mode == ACC_Y) {
    i2c_cmd[0] = ACC_OUT_Y_H;
  } else if(mode == ACC_Z) {
    i2c_cmd[0] = ACC_OUT_Z_H;
  }
  
  i2c_transfer7(I2C2, I2C_IMU_ADDR, i2c_cmd, 1, &i2c_data, 1);
  v += i2c_data << 8;

  xSemaphoreGive(sem);

  return v;

}


int16_t read_lin_acc_X_sensor(void) {
  return read_sensor(ACC_X);
}

int16_t read_lin_acc_Y_sensor(void) {
  return read_sensor(ACC_Y);
}

int16_t read_lin_acc_Z_sensor(void) {
  return read_sensor(ACC_Z);
}

int16_t read_ang_rate_X_sensor(void) {
  return read_sensor(GYR_X);
}

int16_t read_ang_rate_Y_sensor(void) {
  return read_sensor(GYR_Y);
}

int16_t read_ang_rate_Z_sensor(void) {
  return read_sensor(GYR_Z);
}



