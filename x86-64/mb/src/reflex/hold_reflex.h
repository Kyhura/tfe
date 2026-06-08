/*
 *  Montefiore Robocup
 *  ==================
 *
 *  hold_reflex.h : Hold reflex logic 
 *                      
 */


#ifndef _HOLD_REFLEX_H
#define _HOLD_REFLEX_H


#include <stdbool.h>
#include <stdint.h>

/*** Public variable ***/

extern pthread_cond_t cond_hold;
extern pthread_mutex_t lock_hold;


/***  Prototypes of public functions  ***/

void* hold_reflex_main(void *);


#endif /* _HOLD_REFLEX_H */

