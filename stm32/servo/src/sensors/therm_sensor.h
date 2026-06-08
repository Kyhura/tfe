/*
 *  Montefiore Robocup
 *  ==================
 *
 *  therm_sensor.h: Thermistor sensor setup and operations.
 */

#ifndef _THERM_SENSOR_H
#define _THERM_SENSOR_H

#include <stdint.h>


/***  Prototypes of public functions  ***/

void therm_sensor_setup(void);
int16_t read_therm_sensor(void);

#endif  /* _THERM_SENSOR_H */
