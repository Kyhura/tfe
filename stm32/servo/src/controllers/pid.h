/*
 *  Montefiore Robocup
 *  ==================
 *
 *  pid.h: PID controller 
 */

#ifndef _POS_H
#define _POS_H

#include <stdint.h>

/*** Structure ***/

struct pid {
    double kp;
    double ki;
    double kd;
    uint16_t integral_limit;
    double integral_term;
    uint8_t mod_fact;
    double prev_e;
};

/***  Public variable  ***/

extern struct pid pos_pid;
extern struct pid spe_pid;
extern struct pid tor_pid;

/***  Prototypes of public functions  ***/

void init_pid(void);
void disable_motor(void);
int16_t pos_pid_controller(void); 
int16_t spe_pid_controller(void); 
int16_t tor_pid_controller(void); 
void update_kp(struct pid*, double);
void update_ki(struct pid*, double);
void update_kd(struct pid*, double);
double get_kp(struct pid*);
double get_ki(struct pid*);
double get_kd(struct pid*);
void update_int_limit(struct pid*, uint16_t);
void update_mod_fact(struct pid*, uint8_t);

#endif  /* _POS_H */
