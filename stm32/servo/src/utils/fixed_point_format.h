/*
 *  Montefiore Robocup
 *  ==================
 *
 *  fixed_point_format.h: Format raw value to fixed point number and conversely.
 */


#ifndef _FIXED_POINT_FORMAT_H
#define _FIXED_POINT_FORMAT_H

#include <stdint.h>

/***  Prototypes of public functions  ***/

uint32_t format_16_16(double);
double read_16_16(uint32_t);

#endif  /* _FIXED_POINT_FORMAT_H */
