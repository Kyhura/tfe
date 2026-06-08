/*
 *  Montefiore Robocup
 *  ==================
 *
 *  angle_diff.c: Angle difference operation.
 */


#include <stdint.h>
#include <math.h>


#include "angle_diff.h"
#include "parameters.h"

/***  Public functions  ***/

struct angle_diffences compute_angle_diff(double a1, double a2) {
  
  double small_diff, big_diff;

  double angle_diff = a1 - a2;

  double a = 360;

  // If the difference is negative then we need to do +360
  if(angle_diff < 0) {
    a *= -1;
  }

  // Take the smaller in absolute value
  if(fabs(angle_diff) > fabs(angle_diff - a)) {
    big_diff = angle_diff;
    small_diff = angle_diff - a;
  } else {
    big_diff = angle_diff - a;
    small_diff = angle_diff;
  }

  struct angle_diffences ad;

  ad.small_diff = small_diff;
  ad.big_diff = big_diff;

  return ad;
}

/* Convert angle into referential range 
  * where 0 correspond to CCW angle limit
  */

double angle_referential_conversion(double angle) {

  double converted_angle = angle - CCW_angle_limit;

  /* Convert back into [0; 360] */

  while(converted_angle < 0) {
    converted_angle += 360;
  }

  return converted_angle;
}

