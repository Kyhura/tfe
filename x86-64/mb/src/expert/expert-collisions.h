/*
 *  Montefiore Robocup
 *  ==================
 *
 *  expert-collisions.h : Expert responsible for detecting collisions.
 *                        Inspired by Montefiore Team code for Eurobot
 */


#ifndef _EXPERT_COLLISIONS_H
#define _EXPERT_COLLISIONS_H

#include <pthread.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>

/*** Public variable ***/

extern pthread_cond_t cond_collisions;
extern pthread_mutex_t lock_collisions;
extern bool collision_detected;

/***  Prototypes of public functions  ***/

void* expert_collisions_main(void *);


#endif /* _EXPERT_COLLISIONS_H */

