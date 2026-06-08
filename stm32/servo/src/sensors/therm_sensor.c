/*
 *  Montefiore Robocup
 *  ==================
 *
 *  therm_sensor.c: Thermistor sensor setup and operations.
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "../utils/delay.h"
#include "therm_sensor.h"

/*** Private variable ***/

static SemaphoreHandle_t sem;

/***  Public functions  ***/

void therm_sensor_setup(void) {

  /* Enable GPIO clocks. */

  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_ADC34);

  /* Thermistor */

  gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); 

  /* Setup ADC. */

  adc_power_off(ADC3);
  adc_set_clk_prescale(ADC3, ADC_CCR_CKMODE_DIV2); 
  adc_set_single_conversion_mode(ADC3);
  adc_disable_external_trigger_regular(ADC3);
  adc_set_right_aligned(ADC3);

  adc_set_sample_time_on_all_channels(ADC3, ADC_SMPR_SMP_61DOT5CYC);
  uint8_t channel_array[] = { 12 }; /* ADC3_IN12, (PB0) */
  adc_set_regular_sequence(ADC3, 1, channel_array);
  adc_set_resolution(ADC3, ADC_CFGR1_RES_12_BIT);

  adc_enable_regulator(ADC3);
  delay_adc();  // overestimated, should be > 10 µs.

  adc_calibrate(ADC3);

  adc_power_on(ADC3);

  /* Semaphore */

  sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sem);
}
int16_t read_therm_sensor(void) {
    
  xSemaphoreTake(sem, portMAX_DELAY);

  adc_start_conversion_regular(ADC3);
  while (!(adc_eoc(ADC3)));

  int16_t v = adc_read_regular(ADC3);

  xSemaphoreGive(sem);

  return v;
}