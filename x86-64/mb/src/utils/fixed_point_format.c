/*
 *  Montefiore Robocup
 *  ==================
 *
 *  fixed_point_format.c: Format raw value to fixed point number and conversely.
 */


#include <stdint.h>
#include <math.h>


#include "fixed_point_format.h"

/***  Public functions  ***/

uint32_t format_16_16(double raw_value) {

    uint16_t unit  = floor(raw_value);
    uint16_t digit = round((raw_value - unit)*65536.0);

    uint32_t formated_value = (unit << 16) | digit;

    return formated_value;

}

double read_16_16(uint32_t formated_value) {

    double value_16_16 = formated_value/65536.0;

    return value_16_16;
}

