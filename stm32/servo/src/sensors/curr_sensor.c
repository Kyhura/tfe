/*
 *  Montefiore Robocup
 *  ==================
 *
 *  curr_sensor.c: Current amplifier sensor (INA240) setup and operations.
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <stddef.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>

#include <stdint.h>
#include "curr_sensor.h"
#include "../utils/delay.h"
#include "../utils/board.h"
#include "../utils/constant.h"
#include "../utils/parameters.h"
#include "../utils/flags.h"

/*** Private variable ***/

static SemaphoreHandle_t sem;

/***  Public functions  ***/

void curr_sensor_setup(void) {

  /* Enable GPIO clocks. */

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_ADC12);


  /* Current sense */
  
  gpio_mode_setup(DRIVER_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_SENSE);

  /* Setup PWM. */

  gpio_mode_setup(DRIVER_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SD);
  gpio_set(DRIVER_PORT, SD);
  gpio_mode_setup(DRIVER_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LEFT_IN | RIGHT_IN);
  gpio_set_af(DRIVER_PORT, GPIO_AF6, LEFT_IN);
  gpio_set_af(DRIVER_PORT, GPIO_AF11, RIGHT_IN);


  /* Setup ADC. */

  adc_power_off(ADC1);
  adc_set_clk_prescale(ADC1, ADC_CCR_CKMODE_DIV2); 
  adc_set_single_conversion_mode(ADC1);
  adc_disable_external_trigger_regular(ADC1);
  adc_set_right_aligned(ADC1);

  adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_61DOT5CYC);
  uint8_t channel_array[] = { 3 }; /* ADC1_IN3, (PA2) */
  adc_set_regular_sequence(ADC1, 1, channel_array);
  adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);

  adc_enable_regulator(ADC1);
  delay_adc();  // overestimated, should be > 10 µs.

  adc_calibrate(ADC1);
  adc_power_on(ADC1);

  /* Semaphore */

  sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sem);

}

int32_t read_curr_sensor(void) {

  xSemaphoreTake(sem, portMAX_DELAY);
  
  int32_t v;
  int32_t sum = 0;
  int32_t mean = 0;
  
  adc_start_conversion_regular(ADC1);

  for(uint32_t cnt = 0; cnt<NB_SAMPLES; cnt++) {
    

    while (!(adc_eoc(ADC1)));
    v = adc_read_regular(ADC1);
    v = v & 0x00000FFF;
    sum += v;

    adc_start_conversion_regular(ADC1);

  }

  mean = (int32_t) (sum / NB_SAMPLES);

  xSemaphoreGive(sem);

  return mean;
}
