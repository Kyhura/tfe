/*
 *  Montefiore Robocup
 *  ==================
 *
 *  task.h : Management of non real-time tasks.
 *           Greatly inspired by Montefiore Team code for Eurobot
 */

#ifndef _TASK_H
#define _TASK_H


#include <stdint.h>



/***  Prototypes of public functions  ***/

int8_t task_create(void *(*)(void *));

#endif /* _TASK_H */