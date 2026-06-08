/*
 *  Montefiore Robocup
 *  ==================
 *
 *  pwm.h: PWM setup and operations. 
 */

#ifndef _PWM_H
#define _PWM_H

#include <stdint.h>
#include <stdbool.h>


/*** Definitions  ***/

#define DRIVER_PORT          GPIOA
#define LEFT_IN              GPIO8
#define RIGHT_IN             GPIO11
#define SD                   GPIO12
#define CURRENT_SENSE        GPIO2


/***  Prototypes of public functions  ***/
void pwm_setup(void);
void pwm_update(int16_t);
void pwm_motor_stop(void);


#endif  /* _PWM_H */
