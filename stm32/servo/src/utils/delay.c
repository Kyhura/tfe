/*
 *  Montefiore Robocup
 *  ==================
 *
 *  delay.c: Servomotor delays.
 */

#include <stdint.h>

#include "delay.h"

/***  Public functions  ***/

void delay_mag(void)
{
  int i;

  for (i = 0; i < 10; i++) // about 1.18 µs.
    __asm__("nop");
}

void delay_leds(void)
{
  int i;

  for (i = 0; i < 5000000; i++) // about 590 ms.
    __asm__("nop");
}

void delay_adc(void)
{
  int i;

  for (i = 0; i < 10000; i++) // about 1.18 ms.
    __asm__("nop");
}