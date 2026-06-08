/*
 *  Montefiore Robocup
 *  ==================
 *
 *  flags.c: Flags utility functions 
 */


#include <stdint.h>

#include "flags.h"
#include "parameters.h"


/***  Public functions  ***/

void set_flag(uint8_t flag_to_set) {
  indication_flags |= flag_to_set; 
}

void clear_flag(uint8_t flag_to_clear) {
    indication_flags &= ~flag_to_clear;
}

void clear_all_flags() {
  indication_flags = 0x00;
}

