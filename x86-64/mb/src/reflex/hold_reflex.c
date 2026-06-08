/*
 *  Montefiore Robocup
 *  ==================
 *
 *  hold_reflex.c : Hold reflex logic 
 *                      
 */


#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "hold_reflex.h"
#include "../utils/bytes.h"
#include "../utils/information_id.h"
#include "../utils/communication.h"
#include "../utils/pid_visualisation.h" 
#include "../utils/protocol.h"
#include "../world/world.h"
#include "../utils/constant.h"
#include "../log/log.h"

/*** Private variable ***/

static uint8_t low_max_torque = 5;
//static uint8_t mid_max_torque = 30; Used for the intermediate state where only one leg touches the ground 
static uint8_t high_max_torque = 100;

static uint8_t low_integral_limit = 10;
//static uint8_t mid_integral_limit = 30; Used for the intermediate state where only one leg touches the ground 
static uint8_t high_integral_limit = 50;

/*** Prototypes of private function ***/

static void main_loop(void);


/*** Private functions ***/

static void main_loop() {
  static uint8_t  buffer[8];
  
  static bool PID_enable = false;
  static bool PID_stopped = false;

  static bool touched_ground = false;

  // Wait for first signal
  pthread_cond_wait(&cond_hold, &lock_hold);


  // Modulation factor

  uint8_t mod_fact = 100.0;
  write_uint8(buffer, POS_MOD_FACT);
  write_uint16(buffer+1, mod_fact);

  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1) ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  update_polarity(SERVO_1);

  mod_fact = 100.0;
  write_uint8(buffer, POS_MOD_FACT);
  write_uint16(buffer+1, mod_fact);

  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  update_polarity(SERVO_3);


  // Proportionnal gain

  double kp = 30.0;
  write_uint8(buffer, POS_PROP_GAIN);
  write_uint32(buffer+1, format_16_16(kp));

  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 5);
      
  update_polarity(SERVO_1);

  kp = 10.0;
  write_uint8(buffer, POS_PROP_GAIN);
  write_uint32(buffer+1, format_16_16(kp));

  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 5);
      
  update_polarity(SERVO_3);


  // Integral gain

  double ki = 30.0;
  write_uint8(buffer, POS_INT_GAIN);
  write_uint32(buffer+1, format_16_16(ki));

  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 5);
      
  update_polarity(SERVO_1);

  ki = 5.0;
  write_uint8(buffer, POS_INT_GAIN);
  write_uint32(buffer+1, format_16_16(ki));

  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 5);
      
  update_polarity(SERVO_3);


  // Integral limit

  uint8_t integral_limit = low_integral_limit;
  write_uint8(buffer, POS_INT_LIMIT);
  write_uint16(buffer+1, integral_limit);

  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  update_polarity(SERVO_1);

  integral_limit = low_integral_limit;
  write_uint8(buffer, POS_INT_LIMIT);
  write_uint16(buffer+1, integral_limit);

  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  update_polarity(SERVO_3);



  // Max torque

  uint8_t torque_limit_percentage = low_max_torque;
  write_uint8(buffer, MAX_TORQUE);
  write_uint16(buffer+1, torque_limit_percentage);

  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  update_polarity(SERVO_1);

  torque_limit_percentage = low_max_torque;
  write_uint8(buffer, MAX_TORQUE);
  write_uint16(buffer+1, torque_limit_percentage);

  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
      
  
  update_polarity(SERVO_3);



  // Angular position 

  write_uint8(buffer, MOTOR_POSITION);
  com_queue_order(0, SERVO_1, (PROTOCOL_ORDER_GET),
              (false), buffer, 1);

  write_uint8(buffer, MOTOR_POSITION);
  com_queue_order(0, SERVO_3, (PROTOCOL_ORDER_GET),
              (false), buffer, 1);

  // Linear acceleration

  write_uint8(buffer, LINEAR_ACC);
  com_queue_order(0, SERVO_1, (PROTOCOL_ORDER_GET),
              (false), buffer, 1);

  write_uint8(buffer, LINEAR_ACC);
  com_queue_order(0, SERVO_3, (PROTOCOL_ORDER_GET),
              (false), buffer, 1);



  pthread_cond_wait(&cond_hold, &lock_hold);
  pthread_cond_wait(&cond_hold, &lock_hold);
  pthread_cond_wait(&cond_hold, &lock_hold);


  // WRITE order to change the angle reference

  double angle1 = get_position(SERVO_1);
  double angle3 = get_position(SERVO_3);

  uint16_t angle_to_send = angle1/angle_resolution;

  write_uint8(buffer, POSITION_REFERENCE);
  write_uint16(buffer+1, angle_to_send);
  
  com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);

  
  update_polarity(SERVO_1);

  angle_to_send = angle3/angle_resolution;

  write_uint8(buffer, POSITION_REFERENCE);
  write_uint16(buffer+1, angle_to_send);
  
  com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
              (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);

  
  update_polarity(SERVO_3);

  // Enable the PID position

  PID_enable = true;
  write_uint8(buffer, PID_enable);
  com_queue_order(0, SERVO_1, (PROTOCOL_ORDER_PID_POS),
              (false), buffer, 1);

  PID_enable = true;
  write_uint8(buffer, PID_enable);
  com_queue_order(0, SERVO_3, (PROTOCOL_ORDER_PID_POS),
              (false), buffer, 1);



  int t = 0; // timestep in ms
  for(;;) {
    angle1 = get_position(SERVO_1);
    angle3 = get_position(SERVO_3);
    double angle_ref_1 = get_ref_position(SERVO_1);
    double angle_ref_3 = get_ref_position(SERVO_3);
    

    if(!PID_stopped) {

      // Angular position

      write_uint8(buffer, MOTOR_POSITION);
      com_queue_order(0, SERVO_1, (PROTOCOL_ORDER_GET),
                  (false), buffer, 1);

      write_uint8(buffer, MOTOR_POSITION);
      com_queue_order(0, SERVO_3, (PROTOCOL_ORDER_GET),
                  (false), buffer, 1);

      // Linear acceleration
  
      write_uint8(buffer, LINEAR_ACC);
      com_queue_order(0, SERVO_1, (PROTOCOL_ORDER_GET),
                  (false), buffer, 1);

      write_uint8(buffer, LINEAR_ACC);
      com_queue_order(0, SERVO_3, (PROTOCOL_ORDER_GET),
                  (false), buffer, 1);


      // Robot did not touch the ground yet
      if(!touched_ground) {

        // Check for collision detected, for now we use a shared variable
        if(get_collision(SERVO_1)) {

          // Robot touched the ground
          touched_ground = true;
          log_normal("Hold reflex determines that the robot touched the ground \n");

          // Update to a low compliance -> high max torque

          torque_limit_percentage = high_max_torque;
          write_uint8(buffer, MAX_TORQUE);
          write_uint16(buffer+1, torque_limit_percentage);

          com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
              
          
          update_polarity(SERVO_1);

          torque_limit_percentage = high_max_torque;
          write_uint8(buffer, MAX_TORQUE);
          write_uint16(buffer+1, torque_limit_percentage);

          com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
              
          
          update_polarity(SERVO_3);


          // Update integral limit to a higher value

          integral_limit = high_integral_limit;
          write_uint8(buffer, POS_INT_LIMIT);
          write_uint16(buffer+1, integral_limit);

          com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
              
          
          update_polarity(SERVO_1);

          integral_limit = high_integral_limit;
          write_uint8(buffer, POS_INT_LIMIT);
          write_uint16(buffer+1, integral_limit);

          com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);
              
          
          update_polarity(SERVO_3);

        } else {

          // If the robot did not touch the ground now, we update the reference

          angle_to_send = angle1/angle_resolution;

          write_uint8(buffer, POSITION_REFERENCE);
          write_uint16(buffer+1, angle_to_send);
          
          com_queue_order(0, SERVO_1, (get_polarity(SERVO_1)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);

          
          update_polarity(SERVO_1);


          angle_to_send = angle3/angle_resolution;

          write_uint8(buffer, POSITION_REFERENCE);
          write_uint16(buffer+1, angle_to_send);
          
          com_queue_order(0, SERVO_3, (get_polarity(SERVO_3)  ? (PROTOCOL_ORDER_WRITE_1) :
                      (PROTOCOL_ORDER_WRITE_0)), (false), buffer, 3);

          
          update_polarity(SERVO_3);
        }      
      }
    }


    t += 1;

    // Wait for signal
    pthread_cond_wait(&cond_hold, &lock_hold);
  }

}

/*** Public variable ***/

pthread_cond_t cond_hold = PTHREAD_COND_INITIALIZER;
pthread_mutex_t lock_hold = PTHREAD_MUTEX_INITIALIZER;


/***  Public functions  ***/

void* hold_reflex_main(void *arg) {

  printf("Hold reflex started\n");
  fflush(stdout);
  log_debug("Hold reflex started\n");
 
  main_loop();

  printf("Hold reflex terminated\n");
  fflush(stdout);
  log_debug("Hold reflex terminated\n");
  return NULL;
}


