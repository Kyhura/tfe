/*
 *  Montefiore Robocup
 *  ==================
 *
 *  diag.c: Diagnostic LEDs.
 */


#ifndef _DIAG_H
#define _DIAG_H

/***  Prototypes of public functions  ***/

void  diag_setup(void);
void  diag_set_led_pattern(uint8_t, uint8_t, uint8_t, uint8_t);

#endif  /* _DIAG_H */
