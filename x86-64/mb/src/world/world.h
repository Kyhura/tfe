/*
 *  Montefiore Robocup
 *  ==================
 *
 *  world.h: Description of the world from the mb point of view.
 */

#ifndef _WORLD_H
#define _WORLD_H


#include <stdint.h>


/*** Definition  ***/

// Mapping from series number to servo ID
#define SERVO_1     0x02
#define SERVO_2     0x03
#define SERVO_3     0x01

#define LIN_ACC_X   0x01
#define LIN_ACC_Y   0x02
#define LIN_ACC_Z   0x03


/***  Prototypes of public functions  ***/

void init_world(void);
void update_polarity(uint8_t);
uint8_t get_polarity(uint8_t);

void update_collision(uint8_t, bool);
bool get_collision(uint8_t);

void update_position(uint8_t, double);
double get_position(uint8_t);
void update_ref_position(uint8_t, double);
double get_ref_position(uint8_t);
void update_pos_kp(uint8_t, double);
void update_pos_ki(uint8_t, double);
void update_pos_kd(uint8_t, double);
void update_pos_int_limit(uint8_t, uint16_t);

void update_speed(uint8_t, double);
double get_speed(uint8_t);
void update_ref_speed(uint8_t, double);
double get_ref_speed(uint8_t);
void update_spe_kp(uint8_t, double);
void update_spe_ki(uint8_t, double);
void update_spe_kd(uint8_t, double);
void update_spe_int_limit(uint8_t, uint16_t);

void update_torque(uint8_t, double);
double get_torque(uint8_t);
void update_ref_torque(uint8_t, double);
double get_ref_torque(uint8_t);
void update_tor_kp(uint8_t, double);
void update_tor_ki(uint8_t, double);
void update_tor_kd(uint8_t, double);
void update_tor_int_limit(uint8_t, uint16_t);

void update_max_torque(uint8_t, uint16_t);
void update_pos_mod_fact(uint8_t, uint8_t);
void update_spe_mod_fact(uint8_t, uint8_t);
void update_tor_mod_fact(uint8_t, uint8_t);

void update_lin_acc(uint8_t, double, double, double);
double get_lin_acc(uint8_t, uint8_t);


#endif /* _WORLD_H */
