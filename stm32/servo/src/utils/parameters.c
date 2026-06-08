/*
 *  Montefiore Robocup
 *  ==================
 *
 *  parameters.c: Definition of the servomotors's parameters.
 */

#include "parameters.h"

/***  Public variable  ***/

/* Measure */

double servo_angle;
double servo_linear_acc_x;
double servo_linear_acc_y;
double servo_linear_acc_z;
double servo_angular_spe_x;
double servo_angular_spe_y;
double servo_angular_spe_z;
double torque;
int32_t current;
double batt_voltage;
double motor_temp;
double max_motor_temp = 80;
int16_t therm_v;

/* Derivative */

double servo_motor_speed;

/* Limit */

// // SERVO1
// const double CW_angle_limit = 30;
// const double CCW_angle_limit = 280;

// Servo3
const double CW_angle_limit = 230;
const double CCW_angle_limit = 70;


bool ccw_bigger = (CW_angle_limit < CCW_angle_limit) ? true : false;
uint16_t max_torque = 0;

/* PID parameter */

volatile double pos_ref;
double pos_tol = 5;
volatile int16_t pos_pid_output = 0;
bool pos_pid_enabled = false;
bool pos_has_been_set = false;

volatile double spe_ref;
uint8_t spe_tol = 5;
volatile int16_t spe_pid_output = 0;
bool spe_pid_enabled = false;
bool spe_has_been_set = false;

volatile double tor_ref;
uint8_t tor_tol = 5;
volatile int16_t tor_pid_output = 0;
bool tor_pid_enabled = false;
bool tor_has_been_set = false;

/* Flag */

uint8_t indication_flags = 0x00;


