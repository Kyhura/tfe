/*
 *  Montefiore Robocup
 *  ==================
 *
 *  pid.c: PID controller 
 */


#include <stdint.h>
#include <math.h>

#include "pid.h"
#include "../utils/parameters.h"
#include "../../../../common/constant.h"
#include "../utils/angle_diff.h"
#include "../utils/board.h"
#include "../utils/flags.h"


/***  Prototypes of private functions  ***/

static int16_t pid_controller(double, struct pid*);

/*** Private functions ***/

int16_t pid_controller(double e, struct pid* pid_controller) {

  pid_controller->integral_term += e*cycle_period;

  // Antiwindup
  if(fabs(pid_controller->integral_term) > pid_controller->integral_limit) {
    
    board_set_leds(BOARD_LED_YELLOW);
    
    if(pid_controller == &pos_pid) set_flag(POS_WINDUP_FLAG);
    else if(pid_controller == &spe_pid) set_flag(SPE_WINDUP_FLAG);
    else set_flag(TOR_WINDUP_FLAG);

    if(pid_controller->integral_term < 0) pid_controller->integral_term = - pid_controller->integral_limit;
    else pid_controller->integral_term = pid_controller->integral_limit;
  
  } else {
    clear_flag(POS_WINDUP_FLAG);
    clear_flag(SPE_WINDUP_FLAG);
    clear_flag(TOR_WINDUP_FLAG);
  }


  double der_e = (e - pid_controller->prev_e)/cycle_period;

  pid_controller->prev_e = e;


  int16_t u = pid_controller->kp*e + pid_controller->ki*pid_controller->integral_term + pid_controller->kd*der_e;

  
  double modulation = pid_controller->mod_fact/100.0;

  int16_t u_mod = u*modulation;


  return u_mod;

}



/***  Public variable  ***/

struct pid pos_pid;
struct pid spe_pid;
struct pid tor_pid;


/***  Public functions  ***/

void init_pid(void) {

  /* Position pid */
  pos_pid.kp = 0.0;
  pos_pid.ki = 0.0;
  pos_pid.kd = 0.0;
  pos_pid.integral_limit = 0;
  pos_pid.integral_term = 0;
  pos_pid.mod_fact = 100;
  pos_pid.prev_e = 0;


  /* Speed pid */
  spe_pid.kp = 0.0;
  spe_pid.ki = 0.0;
  spe_pid.kd = 0.0;
  spe_pid.integral_limit = 0;
  spe_pid.integral_term = 0;
  spe_pid.mod_fact = 100;
  spe_pid.prev_e = 0;

  /* Tor pid */
  tor_pid.kp = 0.0;
  tor_pid.ki = 0.0;
  tor_pid.kd = 0.0;
  tor_pid.integral_limit = 0;
  tor_pid.integral_term = 0;
  tor_pid.mod_fact = 100;
  tor_pid.prev_e = 0;

}


int16_t pos_pid_controller() {

  double error, bigger_error;
  double converted_pos_ref, converted_servo_angle, converted_CCW, converted_CW;


  converted_CCW = 0; // By definition of the referential system

  /* Angle frame conversion */
  converted_pos_ref = angle_referential_conversion(pos_ref);
  converted_servo_angle = angle_referential_conversion(servo_angle);
  converted_CW = angle_referential_conversion(CW_angle_limit);
  
  /* Reference out of range check */

  bool oor = false;
      
  if(converted_servo_angle > converted_CW)  {
    board_set_leds(BOARD_LED_WHITE);
    
    oor = true; 

    struct angle_diffences ref_cw_ad = compute_angle_diff(converted_pos_ref, converted_CW-5);
    struct angle_diffences ref_ccw_ad = compute_angle_diff(converted_pos_ref, converted_CCW+5);

    /*** Position reference becomes the closest angular limit ***/
    converted_pos_ref = fabs(ref_cw_ad.small_diff) < fabs(ref_ccw_ad.small_diff) ? ref_cw_ad.small_diff : ref_ccw_ad.small_diff;
  }

  /* Compute error */

  struct angle_diffences ad = compute_angle_diff(converted_pos_ref, converted_servo_angle);

  error = ad.small_diff;
  bigger_error = ad.big_diff;


  /* Limit check */

  if(!oor) {
    
    if((error < 0 && converted_servo_angle+error < converted_CCW && converted_servo_angle >= converted_CCW) ||
       (error > 0 && converted_servo_angle+error > converted_CW && converted_servo_angle <= converted_CW)) {
        
        // If smaller path will go out of limite, take the longer path
        board_set_leds(BOARD_LED_BLUE);
        error = bigger_error;

    }
  } 


  return pid_controller(error, &pos_pid);
}

int16_t spe_pid_controller() {
  
  double error;

  error = spe_ref - servo_motor_speed;


  return pid_controller(error, &spe_pid);
}

int16_t tor_pid_controller() {
  
  double error;

  error = tor_ref - torque;

  return pid_controller(error, &tor_pid);
}

void update_kp(struct pid *controller, double new_kp) {
  controller->kp = new_kp;
}

void update_ki(struct pid *controller, double new_ki) {
  controller->ki = new_ki;
}

void update_kd(struct pid *controller, double new_kd) {
  controller->kd = new_kd;
}

double get_kp(struct pid* controller) {
  return controller->kp;
}

double get_ki(struct pid* controller) {
  return controller->ki;
}

double get_kd(struct pid* controller) {
  return controller->kd;
}

void update_int_limit(struct pid* controller, uint16_t new_int_limit) {
  controller->integral_limit = new_int_limit;
}

void update_mod_fact(struct pid* controller, uint8_t new_mod_fact) {
  controller->mod_fact = new_mod_fact;
}