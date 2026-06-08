/*
 *  Montefiore Robocup
 *  ==================
 *
 *  vbat_sensor.c: Battery voltage sensor setup and operations.
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "../utils/delay.h"
#include "vbat_sensor.h"

/*** Private variable ***/

static SemaphoreHandle_t sem;

/***  Public functions  ***/

void vbat_sensor_setup(void) {

  /* Enable GPIO clocks. */

  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_ADC12);

  /* VBat ADC */
  
  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2); 
  
  adc_power_off(ADC2);
  // IMPORTANT ADC1 for the pair ADC1, ADC2
  adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2); 
  adc_set_single_conversion_mode(ADC2);
  adc_disable_external_trigger_regular(ADC2);
  adc_set_right_aligned(ADC2);

  adc_set_sample_time_on_all_channels(ADC2, ADC_SMPR_SMP_61DOT5CYC);
  uint8_t channel_array[] = { 12 }; /* ADC2_IN12, (PB2) */
  adc_set_regular_sequence(ADC2, 1, channel_array);
  adc_set_resolution(ADC2, ADC_CFGR1_RES_12_BIT);

  adc_enable_regulator(ADC2);
  delay_adc();  // overestimated, should be > 10 µs.

  adc_calibrate(ADC2);

  adc_power_on(ADC2);

  /* Semaphore */

  sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sem);
}

uint16_t read_vbat_sensor(void) {

  xSemaphoreTake(sem, portMAX_DELAY);

  adc_start_conversion_regular(ADC2);
  while (!(adc_eoc(ADC2)));

  uint16_t v = adc_read_regular(ADC2);

  xSemaphoreGive(sem);

  return v;
}

