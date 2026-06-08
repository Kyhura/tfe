/*
 *  Montefiore Robocup
 *  ==================
 *
 *  diag.c: Diagnostic LEDs.
 */

#include <stdint.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

#include "board.h"
#include "diag.h"

/***  Private global variable  ***/

static volatile uint32_t  led_pattern = 0;

/***  Public functions  ***/

void tim2_isr(void)
{
  static uint8_t  state = 0;

  uint32_t  pattern;

  pattern = led_pattern >> ((state++ % 4) * 8);
  board_set_leds(pattern);

  TIM_SR(TIM2) &= ~TIM_SR_UIF;
}

void  diag_setup(void)
{
  rcc_periph_clock_enable(RCC_TIM2);

  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE,
    TIM_CR1_DIR_UP);

  timer_set_period(TIM2, 0x01000000);
  timer_enable_counter(TIM2);
  timer_enable_irq(TIM2, TIM_DIER_UIE);

  nvic_set_priority(NVIC_TIM2_IRQ, 0x10);
  nvic_enable_irq(NVIC_TIM2_IRQ);
}

 void  diag_set_led_pattern(uint8_t p1, uint8_t p2, uint8_t p3,
                             uint8_t p4)
{
  led_pattern = p1 + (p2 << 8) + (p3 << 16) + (p4 << 24);
}
