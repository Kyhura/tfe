/*
 *  Montefiore Robocup
 *  ==================
 *
 *  mag_sensor.h: Magnetic sensor (AS5045B) setup and operations.
 */

#ifndef _MAG_SENSOR_H
#define _MAG_SENSOR_H

#include <stdint.h>

/*** Definitions  ***/

#define MAG_PORT             GPIOA
#define MAG_CS_MASK          GPIO4
#define MAG_CLK_MASK         GPIO5
#define MAG_DATA_MASK        GPIO6
#define MAG_PROG_MASK        GPIO7

/***  Prototypes of public functions  ***/

void mag_sensor_setup(void);
uint16_t read_mag_sensor(void);

#endif  /* _MAG_SENSOR_H */
