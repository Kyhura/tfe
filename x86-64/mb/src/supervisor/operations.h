/*
 *  Montefiore Robocup
 *  ==================
 *
 *  operations.h: Operations of the motherboard.
 */

#ifndef _OPERATIONS_H
#define _OPERATIONS_H

#include <stdint.h>

/***  Prototypes of public functions  ***/

void  op_new_period(uint32_t);
void  op_recv_frame(uint8_t, uint8_t *, uint32_t);

#endif  /* _OPERATIONS_H */
