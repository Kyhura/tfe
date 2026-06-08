/*
 *  Montefiore Robocup
 *  ==================
 *
 *  world.c: Description of the world from the mb point of view.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include "world.h"
#include "../log/log.h"

/*** Structure ***/



struct servo {
    uint8_t polarity;
    bool collision_detected;

    double angle;
    double ref_angle;
    double pos_kp;
    double pos_ki;
    double pos_kd;
    uint16_t pos_integral_limit;
    uint8_t pos_mod;

    double speed;
    double ref_speed;
    double spe_kp;
    double spe_ki;
    double spe_kd;
    uint16_t spe_integral_limit;
    uint8_t spe_mod;

    double torque;
    double ref_torque;
    double tor_kp;
    double tor_ki;
    double tor_kd;
    uint16_t tor_integral_limit;
    uint8_t tor_mod;

    uint16_t max_torque;

    double lin_acc_x, lin_acc_y, lin_acc_z;
};

/*** Private definition ***/

#define POSITION_PID 0x01
#define SPEED_PID    0x02
#define TORQUE_PID   0x03

#define KP           0x01
#define KI           0x02
#define KD           0x03

/*** Private variable ***/

struct servo s1;
struct servo s2;
struct servo s3;

/***  Prototypes of private functions  ***/

static void init_servo(struct servo*);
static struct servo* get_servo(uint8_t);
void update_gain(struct servo *, double, uint8_t, uint8_t);
void update_int_limit(struct servo *, uint16_t, uint8_t);
void update_mod_fact(struct servo *, uint8_t, uint8_t);

/***  Private functions  ***/

void init_servo(struct servo* s) {

    s->polarity = 0;

    s->collision_detected = false;

    s->angle = 0;
    s->ref_angle = 0;
    s->pos_kp = 0;
    s->pos_ki = 0;
    s->pos_kd = 0;
    s->pos_integral_limit = 0;
    s->pos_mod = 100;

    s->speed = 0;
    s->ref_speed = 0;
    s->spe_kp = 0;
    s->spe_ki = 0;
    s->spe_kd = 0;
    s->spe_integral_limit = 0;
    s->spe_mod = 100;

    s->torque = 0;
    s->ref_torque = 0;
    s->tor_kp = 0;
    s->tor_ki = 0;
    s->tor_kd = 0;
    s->tor_integral_limit = 0;
    s->tor_mod = 100;

    s->max_torque = 0; 
    
    s->lin_acc_x = 0;
    s->lin_acc_y = 0;
    s->lin_acc_z = 0;
}

struct servo* get_servo(uint8_t servo_addr) {

    struct servo *s;

    switch (servo_addr) {
        case SERVO_1:
            s = &s1;
            break;
        case SERVO_2:
            s = &s2;
            break;
        case SERVO_3:
            s = &s3;
            break;
        default:
            printf("Error: update for gain from unknown adress : %d\n", servo_addr);
            return NULL;
    }

    return s;
}

void update_gain(struct servo *s, double new_gain, uint8_t pid_type, uint8_t gain) {

    switch (pid_type)
    {
    case POSITION_PID:
        switch (gain)
        {
        case KP:
            s->pos_kp = new_gain;
            break;
        case KI:
            s->pos_ki = new_gain;
            break;
        case KD:
            s->pos_kd = new_gain;
            break;
        default:
            printf("Error when updating the gain: unknown gain: %d\n", gain);
            break;
        }
        break;
    case SPEED_PID:
        switch (gain)
        {
        case KP:
            s->spe_kp = new_gain;
            break;
        case KI:
            s->spe_ki = new_gain;
            break;
        case KD:
            s->spe_kd = new_gain;
            break;
        default:
            printf("Error when updating the gain: unknown gain: %d\n", gain);
            break;
        }
        break;
    case TORQUE_PID:
        switch (gain)
        {
        case KP:
            s->tor_kp = new_gain;
            break;
        case KI:
            s->tor_ki = new_gain;
            break;
        case KD:
            s->tor_kd = new_gain;
            break;
        default:
            printf("Error when updating the gain: unknown gain: %d\n", gain);
            break;
        }
        break;
    
    default:
        printf("Error when updating the gain: unknown pid type: %d\n", pid_type);
        break;
    }
}

void update_int_limit(struct servo *s, uint16_t new_integral_limit, uint8_t pid_type) {

    switch (pid_type)
    {
    case POSITION_PID:
        s->pos_integral_limit = new_integral_limit;
        break;
    case SPEED_PID:
        s->spe_integral_limit = new_integral_limit;
        break;
    case TORQUE_PID:
        s->tor_integral_limit = new_integral_limit;
        break;
    
    default:
        printf("Error when updating the integral limit: unkown pid type: %d\n", pid_type);
        break;
    }
}

void update_mod_fact(struct servo *s, uint8_t pid_type, uint8_t new_mod_fact) {

     switch (pid_type)
    {
    case POSITION_PID:
        s->pos_mod = new_mod_fact;
        break;
    case SPEED_PID:
        s->spe_mod = new_mod_fact;
        break;
    case TORQUE_PID:
        s->tor_mod = new_mod_fact;
        break;
    
    default:
        printf("Error when updating the modulation factor: unkown pid type: %d\n", pid_type);
        break;
    }
    
}

/*** Public functions  ***/

void init_world() {

    init_servo(&s1);
    init_servo(&s2);
    init_servo(&s3);

}


void update_polarity(uint8_t servo_addr) {

    struct servo *s = get_servo(servo_addr);

    s->polarity = 1 - s->polarity;

}

uint8_t get_polarity(uint8_t servo_addr) {

    struct servo *s = get_servo(servo_addr);

    return s->polarity;
}


void update_collision(uint8_t servo_addr, bool collision_state) {

    struct servo *s = get_servo(servo_addr);

    s->collision_detected = collision_state;

}

bool get_collision(uint8_t servo_addr) {

    struct servo *s = get_servo(servo_addr);

    return s->collision_detected;
}



/* Position PID update */

void update_position(uint8_t servo_addr, double new_pos) {
    
    struct servo *s = get_servo(servo_addr);

    s->angle = new_pos;
}


void update_ref_position(uint8_t servo_addr, double new_ref_pos) {
    
    struct servo *s = get_servo(servo_addr);

    s->ref_angle = new_ref_pos;
}


void update_pos_kp(uint8_t servo_addr, double new_kp) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kp, POSITION_PID, KP);
}

void update_pos_ki(uint8_t servo_addr, double new_ki) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_ki, POSITION_PID, KI);
}

void update_pos_kd(uint8_t servo_addr, double new_kd) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kd, POSITION_PID, KD);
}

void update_pos_int_limit(uint8_t servo_addr, uint16_t new_integral_limit) {

    struct servo *s = get_servo(servo_addr);

    update_int_limit(s, new_integral_limit, POSITION_PID);
}



/* Speed PID update */

void update_speed(uint8_t servo_addr, double new_spe) {
    
    struct servo *s = get_servo(servo_addr);

    s->speed = new_spe;
}

void update_ref_speed(uint8_t servo_addr, double new_ref_spe) {
    
    struct servo *s = get_servo(servo_addr);

    s->ref_speed = new_ref_spe;
}


void update_spe_kp(uint8_t servo_addr, double new_kp) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kp, SPEED_PID, KP);
}

void update_spe_ki(uint8_t servo_addr, double new_ki) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_ki, SPEED_PID, KI);
}

void update_spe_kd(uint8_t servo_addr, double new_kd) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kd, SPEED_PID, KD);
}

void update_spe_int_limit(uint8_t servo_addr, uint16_t new_integral_limit) {

    struct servo *s = get_servo(servo_addr);

    update_int_limit(s, new_integral_limit, SPEED_PID);
}



/* Torque PID update */

void update_torque(uint8_t servo_addr, double new_tor) {
    
    struct servo *s = get_servo(servo_addr);

    s->torque = new_tor;
}

void update_ref_torque(uint8_t servo_addr, double new_ref_tor) {
    
    struct servo *s = get_servo(servo_addr);

    s->ref_torque = new_ref_tor;
}

void update_tor_kp(uint8_t servo_addr, double new_kp) {
    
    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kp, TORQUE_PID, KP);
}   

void update_tor_ki(uint8_t servo_addr, double new_ki) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_ki, TORQUE_PID, KI);
}

void update_tor_kd(uint8_t servo_addr, double new_kd) {

    struct servo *s = get_servo(servo_addr);

    update_gain(s, new_kd, TORQUE_PID, KD);
}

void update_tor_int_limit(uint8_t servo_addr, uint16_t new_integral_limit) {

    struct servo *s = get_servo(servo_addr);

    update_int_limit(s, new_integral_limit, TORQUE_PID);
}


/* Get data */

double get_position(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->angle;
    
}

double get_ref_position(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->ref_angle;
    
}

double get_speed(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->speed;
    
}

double get_ref_speed(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->ref_speed;
    
}

double get_torque(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->torque;
    
}

double get_ref_torque(uint8_t servo_addr) {
    
    struct servo *s = get_servo(servo_addr);
    
    return s->ref_torque;
    
}


/* Limit update */

void update_max_torque(uint8_t servo_addr, uint16_t new_max_torque) {

    struct servo *s = get_servo(servo_addr);
    
    s->max_torque = new_max_torque;
}

/* Modulation factor update */

void update_pos_mod_fact(uint8_t servo_addr, uint8_t new_mod_fact) {

    struct servo *s = get_servo(servo_addr);

    update_mod_fact(s, POSITION_PID, new_mod_fact);

}

void update_spe_mod_fact(uint8_t servo_addr, uint8_t new_mod_fact) {

    struct servo *s = get_servo(servo_addr);

    update_mod_fact(s, SPEED_PID, new_mod_fact);
    
}

void update_tor_mod_fact(uint8_t servo_addr, uint8_t new_mod_fact) {

    struct servo *s = get_servo(servo_addr);

    update_mod_fact(s, TORQUE_PID, new_mod_fact);
    
}

void update_lin_acc(uint8_t servo_addr, double new_lin_acc_x, double new_lin_acc_y, double new_lin_acc_z) {

    struct servo *s = get_servo(servo_addr);

    s->lin_acc_x = new_lin_acc_x;
    s->lin_acc_y = new_lin_acc_y;
    s->lin_acc_z = new_lin_acc_z;
    
}

double get_lin_acc(uint8_t servo_addr, uint8_t lin_acc) {

    struct servo *s = get_servo(servo_addr);

    switch (lin_acc)
    {
    case (LIN_ACC_X):
        return s->lin_acc_x;
    case (LIN_ACC_Y):
        return s->lin_acc_y;
    case (LIN_ACC_Z):
        return s->lin_acc_z;
    default:
        log_critical("Unknown linear acceleration axis asked\n");
        return INT64_MAX;
    }


}
