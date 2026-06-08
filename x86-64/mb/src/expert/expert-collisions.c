/*
 *  Montefiore Robocup
 *  ==================
 *
 *  expert-collisions.c : Expert responsible for detecting collisions.
 *                        Inspired by Montefiore Team code for Eurobot
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "expert-collisions.h"
#include "../world/world.h"
#include "../log/log.h"

/*** Prototypes of private function ***/

static void main_loop(void);


/*** Private functions ***/

static void main_loop() {

  double lin_acc_x, lin_acc_y, lin_acc_z, prev_lin_acc_x, prev_lin_acc_y, prev_lin_acc_z;

  double tol = 4;

  // Wait for two signals to wait for servomotor initialisation
  pthread_cond_wait(&cond_collisions, &lock_collisions);
  pthread_cond_wait(&cond_collisions, &lock_collisions);

  lin_acc_x = get_lin_acc(SERVO_1, LIN_ACC_X);
  lin_acc_y = get_lin_acc(SERVO_1, LIN_ACC_Y);
  lin_acc_z = get_lin_acc(SERVO_1, LIN_ACC_Z);

  prev_lin_acc_x = lin_acc_x;
  prev_lin_acc_y = lin_acc_y;
  prev_lin_acc_z = lin_acc_z;

  for(;;) {
    // Determine if a collisions has been sensed
    
    lin_acc_x = get_lin_acc(SERVO_1, LIN_ACC_X);
    lin_acc_y = get_lin_acc(SERVO_1, LIN_ACC_Y);
    lin_acc_z = get_lin_acc(SERVO_1, LIN_ACC_Z);

    // Condition to prevent uncorrect collisions detection at start
    if(prev_lin_acc_x != 0 && prev_lin_acc_y != 0 && prev_lin_acc_z != 0) {
      if(fabs(lin_acc_x - prev_lin_acc_x) >= tol ||
         fabs(lin_acc_y - prev_lin_acc_y) >= tol ||
         fabs(lin_acc_z - prev_lin_acc_z) >= tol ) {

          log_normal("Expert detects a collision for servomotor 1 \n");
          printf("Choc detected\n");
          fflush(stdout);

          update_collision(SERVO_1, true);

          // Acts as an one time detection, no mechanism to set the variable back to false for now
      } 
    }
    
    // Wait for signal
    pthread_cond_wait(&cond_collisions, &lock_collisions);

    // Update previous
    prev_lin_acc_x = lin_acc_x;
    prev_lin_acc_y = lin_acc_y;
    prev_lin_acc_z = lin_acc_z;
  }
}

/*** Public variable ***/

pthread_cond_t cond_collisions = PTHREAD_COND_INITIALIZER;
pthread_mutex_t lock_collisions = PTHREAD_MUTEX_INITIALIZER;
bool collision_detected = false;

/*** Public functions ***/

/* Thread entry point */

void* expert_collisions_main(void *arg) { 
  
  printf("Expert collisions started\n");
  log_debug("Expert collisions started\n");

  main_loop();

  printf("Expert collisions terminated\n");
  log_debug("Expert collisions terminated\n");

  return NULL;
}


