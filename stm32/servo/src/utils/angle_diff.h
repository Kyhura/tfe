/*
 *  Montefiore Robocup
 *  ==================
 *
 *  angle_diff.h: Angle difference operation.
 */

#ifndef _ANGLE_DIFF_H
#define _ANGLE_DIFF_H

#include <stdint.h>

/*** Structure ***/

struct angle_diffences
{
    double small_diff;
    double big_diff;
};



/***  Prototypes of public functions  ***/

struct angle_diffences compute_angle_diff(double, double);
double angle_referential_conversion(double);

#endif  /* _ANGLE_DIFF_H */
