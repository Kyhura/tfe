/*
 *  Montefiore Robocup
 *  ==================
 *
 *  parameters.h: Definition of the servomotors's parameters.
 */

#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include <stdint.h>
#include <stdbool.h>

/***  Public variable  ***/

/* Measure */

extern double servo_angle;
extern double servo_linear_acc_x;
extern double servo_linear_acc_y;
extern double servo_linear_acc_z;
extern double servo_angular_spe_x;
extern double servo_angular_spe_y;
extern double servo_angular_spe_z;
extern double torque;
extern int32_t current;
extern double batt_voltage;
extern double motor_temp;
extern double max_motor_temp;
extern int16_t therm_v;

/* Derivative */

extern double servo_motor_speed;

/* Limit */

extern const double CW_angle_limit;
extern const double CCW_angle_limit;
extern bool ccw_bigger;
extern uint16_t max_torque;

/* PID parameter */

extern volatile double pos_ref;
extern double pos_tol;
extern volatile int16_t pos_pid_output;
extern bool pos_pid_enabled;
extern bool pos_has_been_set;

extern volatile double spe_ref;
extern uint8_t spe_tol;
extern volatile int16_t spe_pid_output;
extern bool spe_pid_enabled;
extern bool spe_has_been_set;

extern volatile double tor_ref;
extern uint8_t tor_tol;
extern volatile int16_t tor_pid_output;
extern bool tor_pid_enabled;
extern bool tor_has_been_set;

/* Flag */

extern uint8_t indication_flags;


#endif /* _PARAMETERS_H */
