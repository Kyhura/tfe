/*
 *  Montefiore Robocup
 *  ==================
 *
 *  vbat_sensor.h: Battery voltage sensor setup and operations.
 */

#ifndef _VBAT_SENSOR_H
#define _VBAT_SENSOR_H

#include <stdint.h>

/*** Definitions  ***/

#define VBAT_PORT              GPIOB
#define VBAT_ADC_MASK          GPIO4

/***  Prototypes of public functions  ***/

void vbat_sensor_setup(void);
uint16_t read_vbat_sensor(void);

#endif  /* _VBAT_SENSOR_H */
