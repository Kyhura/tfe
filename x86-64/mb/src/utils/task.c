/*
 *  Montefiore Robocup
 *  ==================
 *
 *  task.c : Management of non real-time tasks.
 *           Greatly inspired by Montefiore Team code for Eurobot
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <pthread.h>
#include <sched.h>

#include "task.h"


/***  Public functions  ***/

/*
 *  Creates a new (non real-time) task with the entry point main
 */
int8_t task_create(void *(*main)(void *)) {
  
  pthread_t thread1;

  if(pthread_create(&thread1, NULL, main, NULL)) {
    printf("Error while creating a thread\n");
    return -1;
  }

  return 0;
}

