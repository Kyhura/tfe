/*
 *  Montefiore Robocup
 *  ==================
 *
 *  flags.h: Flags utility functions 
 */

#ifndef _FLAG_H
#define _FLAG_H

#include <stdint.h>

/*** Definition ***/

#define MAGN_SENSOR_FLAG     0x01
#define POS_WINDUP_FLAG      0x02
#define SPE_WINDUP_FLAG      0x04
#define TOR_WINDUP_FLAG      0x08
#define MAX_TOR_REACHED_FLAG 0x10
#define OOR_PID_FLAG         0x20
#define MAX_TEMP_FLAG        0x40

/***  Prototypes of public functions  ***/

void set_flag(uint8_t);
void clear_flag(uint8_t);
void clear_all_flags();

#endif  /* _FLAG_H */
