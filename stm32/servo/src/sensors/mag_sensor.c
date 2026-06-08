/*
 *  Montefiore Robocup
 *  ==================
 *
 *  mag_sensor.c: Magnetic sensor (AS5045B) setup and operations.
 */

#include <FreeRTOS.h>
#include <stdint.h>
#include <stddef.h>
#include <semphr.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "mag_sensor.h"
#include "../utils/delay.h"
#include "../utils/board.h"
#include "../utils/constant.h"
#include "../utils/parameters.h"
#include "../utils/flags.h"

/*** Private variable ***/

static SemaphoreHandle_t sem;


/*** Prototypes of private functions  ***/

static void mag_sensor_clock_cycle(void);


/*** Private functions  ***/

static void mag_sensor_clock_cycle(void) {
  gpio_clear(MAG_PORT, MAG_CLK_MASK);
  delay_mag();

  gpio_set(MAG_PORT, MAG_CLK_MASK);
  delay_mag();
}

/***  Public functions  ***/

void mag_sensor_setup(void) {
  rcc_periph_clock_enable(RCC_GPIOA);

  gpio_mode_setup(MAG_PORT, GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE, MAG_CS_MASK | MAG_CLK_MASK | MAG_PROG_MASK);

  gpio_clear(MAG_PORT, MAG_CLK_MASK);
  //gpio_set(MAG_PORT, MAG_PROG_MASK);
  gpio_set(MAG_PORT, MAG_CS_MASK);


  gpio_mode_setup(MAG_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
                  MAG_DATA_MASK);

  /* Semaphore */

  sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sem);
}

uint16_t read_mag_sensor(void) {

  /* Semaphore block to prevent simultaneous multiple read (since to read the value, we need to write on the sensor pins) */

  xSemaphoreTake(sem, portMAX_DELAY);

  uint32_t v, i;

  gpio_clear(MAG_PORT, MAG_CS_MASK);

  for (i = 0, v = 0; i < 18; i++)
  {
      mag_sensor_clock_cycle();

      v <<= 1;

      if (gpio_port_read(MAG_PORT) & MAG_DATA_MASK)
      v++;
      
  }
  delay_mag();
  gpio_set(MAG_PORT, MAG_CS_MASK);

  /* End of critic part */

  xSemaphoreGive(sem);

  uint32_t data;
  uint32_t status;

  /* The 6 first bits are status bits */
  status = v & 0x3f; 

  /* The 12 left bits are data bits */
  data = (v >> 6) & 0x0fff;
  

  /* Status check */

  if(!status >> 5 & 0x01) {
    /* Error: the Offset Compensation Algorithm is not yet finished */
    diag_set_led_pattern((BOARD_LED_RED), (BOARD_LED_OFF),
                         (BOARD_LED_OFF), (BOARD_LED_RED));
    set_flag(MAGN_SENSOR_FLAG);
  }

  if(status >> 4 & 0x01) {
    /* Error CORDIC Overflow: invalid data */
    diag_set_led_pattern((BOARD_LED_RED), (BOARD_LED_OFF),
                         (BOARD_LED_OFF), (BOARD_LED_RED));
    set_flag(MAGN_SENSOR_FLAG);
  }

  if(status >> 3 & 0x01) {
    /* Error Linearity Alarm: data can still be used, but can contain invalid data */
    diag_set_led_pattern((BOARD_LED_RED), (BOARD_LED_OFF),
                         (BOARD_LED_OFF), (BOARD_LED_RED));
    set_flag(MAGN_SENSOR_FLAG);
  }
  
  uint32_t parity = __builtin_parity(v >> 1);
  if(parity != (status & 0x01)) {
    /* Transmission error detected: incorrect parity bit */
    diag_set_led_pattern((BOARD_LED_RED), (BOARD_LED_OFF),
                         (BOARD_LED_OFF), (BOARD_LED_RED));
    set_flag(MAGN_SENSOR_FLAG);
  }


  
  return data;
}