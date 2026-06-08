/*
 *  Montefiore Robocup
 *  ==================
 *
 *  curr_sensor.h: Current amplifier sensor (INA240) setup and operations.
 */

#ifndef _CURR_SENSOR_H
#define _CURR_SENSOR_H

#include <stdint.h>

/*** Definitions  ***/

#define DRIVER_PORT          GPIOA
#define LEFT_IN              GPIO8
#define RIGHT_IN             GPIO11
#define SD                   GPIO12
#define CURRENT_SENSE        GPIO2

#define NB_SAMPLES 1000


/***  Prototypes of public functions  ***/

void curr_sensor_setup(void);
int32_t read_curr_sensor(void);

#endif  /* _MAG_SENSOR_H */
