/*
 *  Montefiore Robocup
 *  ==================
 *
 *  Software of servo board, to be flashed and
 *  launched by bootloader.
 */

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>


#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "./utils/bytes.h"
#include "./utils/canbus.h"
#include "./utils/protocol.h"
#include "./utils/information_id.h"
#include "./utils/board.h"
#include "./utils/diag.h"
#include "./utils/delay.h"
#include "./sensors/mag_sensor.h"
#include "./utils/parameters.h"
#include "./controllers/pwm.h"
#include "./utils/constant.h"
#include "./sensors/imu_sensor.h"
#include "./sensors/vbat_sensor.h"
#include "./sensors/therm_sensor.h"
#include "./sensors/curr_sensor.h"
#include "./controllers/pid.h"
#include "./utils/angle_diff.h"
#include "./utils/fixed_point_format.h"
#include "./utils/flags.h"

/***  Definitions  ***/

#define  DB_FLASH_CONFIG_PAGE_ADDR  0x8001800
#define CONFIG_REGISTER(r)  ((uint32_t *) (DB_FLASH_CONFIG_PAGE_ADDR))[r]
#define CANBUS_RX_BUFFER_SIZE  0x08


/*** Private global variables ***/

static uint8_t   rx_buffer[CANBUS_RX_BUFFER_SIZE];
static uint32_t  rx_buffer_nb_bytes;

static volatile TaskHandle_t  task_order_handler = NULL;
static volatile TaskHandle_t  task_main_handler = NULL;

static SemaphoreHandle_t pos_pid_sem;
static SemaphoreHandle_t spe_pid_sem;

static bool prev_angle_set = false;

/***  Prototypes of private functions  ***/

static void  init_board(void);
static void  process_debug_order(uint8_t *, uint8_t);
static void  process_get_order(uint8_t *, uint8_t);
static void  process_pid_pos_order(uint8_t *, uint8_t);
static void  process_pid_spe_order(uint8_t *, uint8_t);
static void  process_pid_tor_order(uint8_t *, uint8_t);
static void  task_orders(void *);


/***  Private functions  ***/

static void  init_board(void)
{
  /* Setup board. */

  board_setup();

  /* CAN bus. */

  canbus_setup(CONFIG_REGISTER(1) & (PROTOCOL_CAN_DB_ADDR_MASK));

  /* Magnetic sensor setup. */

  mag_sensor_setup();

  /* IMU sensor setup */

  imu_sensor_setup();

  /* Current sensor setup */

  curr_sensor_setup();

  /* Battery voltage sensor setup */

  vbat_sensor_setup();

  /* Thermistor sensor setup */

  therm_sensor_setup();

  /* PWM setup */

  pwm_setup();

  /* Diagnotic setup */

  diag_setup();

  /* PID controller initiation */

  init_pid();

}


static void process_sync_order(uint8_t *bytes, uint8_t nb_bytes) {

  /* Notify measurements task */

  if (task_main_handler) {
    xTaskNotifyGive(task_main_handler);
  }

  /* Update pwm */

  int16_t output = 0;

  /** Heterogeneous actions fusion **/

  if(pos_pid_enabled && pos_has_been_set) {
    output += pos_pid_output;
  }

  if(spe_pid_enabled && spe_has_been_set) {
    output += spe_pid_output;
  }

  if(tor_pid_enabled && tor_has_been_set) {
    output += tor_pid_output;
  }

  /** Actuator update **/

  pwm_update(output);


}


static void  process_debug_order(uint8_t *bytes, uint8_t nb_bytes)
{
  uint32_t  v;

  board_set_leds(BOARD_LED_WHITE);

  if (nb_bytes != 5)
    return;

  v = read_uint32(bytes + 1);
  write_uint32(bytes + 1, v * 10);

  // /* Note: only send the following order(s) after the last
  //    received message for the current frame! */

  canbus_send_order(bytes, nb_bytes);
}


static void process_get_order(uint8_t *bytes, uint8_t nb_bytes) {

  uint8_t information_id;

  if (nb_bytes != 2)
    return;
  
  information_id = read_uint8(bytes + 1);
  switch (information_id) {
    case MOTOR_POSITION:
      uint16_t angle = servo_angle/angle_resolution;
      write_uint8(bytes + 1, MOTOR_POSITION);
      write_uint16(bytes + 2, angle);
      nb_bytes = 4;
      break;
    case LINEAR_ACC:
      int16_t linear_acc_x = servo_linear_acc_x/(imu_acc_sensitivity*g);
      int16_t linear_acc_y = servo_linear_acc_y/(imu_acc_sensitivity*g);
      int16_t linear_acc_z = servo_linear_acc_z/(imu_acc_sensitivity*g);

      write_uint8(bytes + 1, LINEAR_ACC);
      write_uint16(bytes + 2, linear_acc_x);
      write_uint16(bytes + 4, linear_acc_y);
      write_uint16(bytes + 6, linear_acc_z);

      nb_bytes = 8;
      break;
    case ANGULAR_SPEED:
      int16_t ang_spe_x = servo_angular_spe_x/imu_gyr_sensitivity;
      int16_t ang_spe_y = servo_angular_spe_y/imu_gyr_sensitivity;
      int16_t ang_spe_z = servo_angular_spe_z/imu_gyr_sensitivity;

      write_uint8(bytes + 1, ANGULAR_SPEED);
      write_uint16(bytes + 2, ang_spe_x);
      write_uint16(bytes + 4, ang_spe_y);
      write_uint16(bytes + 6, ang_spe_z);

      nb_bytes = 8;
      break;
    case TORQUE:
      
      write_uint8(bytes + 1, TORQUE);
      write_uint32(bytes + 2, current);

      nb_bytes = 6;
      break;
    case VOLTAGE:
      int16_t voltage = (batt_voltage*10/(50+10))*4095/3.3;

      write_uint8(bytes + 1, VOLTAGE);
      write_uint16(bytes + 2, voltage);

      nb_bytes = 4;
      break;
    case MOTOR_TEMP:
      write_uint8(bytes + 1, MOTOR_TEMP);
      write_uint16(bytes + 2, therm_v);

      nb_bytes = 4;
      break;
    case MOTOR_SPEED:

      int32_t motor_speed = format_16_16(servo_motor_speed);
      write_uint8(bytes + 1, MOTOR_SPEED);
      write_uint32(bytes + 2, motor_speed);

      nb_bytes = 6;
      break;
    case POS_PROP_GAIN:
      double kp = get_kp(&pos_pid);
      write_uint8(bytes + 1, POS_PROP_GAIN);
      write_uint32(bytes + 2, format_16_16(kp));

      nb_bytes = 6;
      break;
    case POS_INT_GAIN:
      double ki = get_ki(&pos_pid);
      write_uint8(bytes + 1, POS_INT_GAIN);
      write_uint32(bytes + 2, format_16_16(ki));

      nb_bytes = 6;
      break;
    case POS_DER_GAIN:
      double kd = get_kd(&pos_pid);
      write_uint8(bytes + 1, POS_DER_GAIN);
      write_uint32(bytes + 2, format_16_16(kd));

      nb_bytes = 6;
      break;
    case FLAGS:
      write_uint8(bytes + 1, FLAGS);
      write_uint8(bytes + 2, indication_flags);

      nb_bytes = 3;
      break;
    default:
      write_uint8(bytes + 1, 0xff);
      write_uint8(bytes + 2, information_id);
      nb_bytes = 3;
  }
  

  /* Note: only send the following order(s) after the last
     received message for the current frame! */

  canbus_send_order(bytes, nb_bytes);
}

static void process_pid_pos_order(uint8_t *bytes, uint8_t nb_bytes) {

  pos_pid_enabled = (bool)read_uint8(bytes + 1);

  /* Reset the output and set flag if PID disabled */
  if(!pos_pid_enabled) {
    pos_pid_output = 0;
    pos_has_been_set = false;
  }

}

static void process_pid_spe_order(uint8_t *bytes, uint8_t nb_bytes) {
  
  spe_pid_enabled = (bool)read_uint8(bytes + 1);

  /* Reset the output and set flag if PID disabled */
  if(!spe_pid_enabled) {
    spe_pid_output = 0;
    spe_has_been_set = false;
  }

}

static void process_pid_tor_order(uint8_t *bytes, uint8_t nb_bytes) {
  
  tor_pid_enabled = (bool)read_uint8(bytes + 1);

  /* Reset the output and set flag if PID disabled */
  if(!tor_pid_enabled) {
    tor_pid_output = 0;
    tor_has_been_set = false;
  }

}

static void process_write_order(uint8_t *bytes, uint8_t nb_bytes) {
  uint8_t info_id = read_uint8(bytes + 1);

  switch (info_id) {
    case POSITION_REFERENCE:
      pos_ref = (double)read_uint16(bytes + 2)*angle_resolution;

      /* If the angle ref is not yet set */
      if(!pos_has_been_set) pos_has_been_set = true;

      uint16_t pos_to_send = pos_ref/angle_resolution;

      write_uint16(bytes + 2, pos_to_send);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPEED_REFERENCE:
      spe_ref = read_16_16(read_int32(bytes + 2));

      /* If the speed ref is not yet set */
      if(!spe_has_been_set) spe_has_been_set = true;

      int32_t speed_to_send = format_16_16(spe_ref);

      write_uint32(bytes + 2, speed_to_send);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TORQUE_REFERENCE:
      tor_ref = read_int16(bytes + 2);

      /* If the torque ref is not yet set */
      if(!tor_has_been_set) tor_has_been_set = true;

      write_uint16(bytes + 2, tor_ref);

      canbus_send_order(bytes, nb_bytes);
      break;
    
    case MAX_TORQUE:
      uint8_t torque_limit_percentage = read_uint8(bytes + 2);

      if(torque_limit_percentage < 0) max_torque = 0;
      else if(torque_limit_percentage > 100) max_torque = 512;
      else 
        max_torque = 512*torque_limit_percentage/100;

      write_uint8(bytes + 2, torque_limit_percentage);

      canbus_send_order(bytes, nb_bytes);
      break;
    case POS_PROP_GAIN:
      uint32_t pos_kp = read_uint32(bytes + 2);

      update_kp(&pos_pid, read_16_16(pos_kp));

      write_uint32(bytes + 2, pos_kp);

      canbus_send_order(bytes, nb_bytes);
      break;
    case POS_INT_GAIN:
      uint32_t pos_ki = read_uint32(bytes + 2);

      update_ki(&pos_pid, read_16_16(pos_ki));

      write_uint32(bytes + 2, pos_ki);

      canbus_send_order(bytes, nb_bytes);
      break;
    case POS_DER_GAIN:
      uint32_t pos_kd = read_uint32(bytes + 2);

      update_kd(&pos_pid, read_16_16(pos_kd));

      write_uint32(bytes + 2, pos_kd);

      canbus_send_order(bytes, nb_bytes);
      break;
    case POS_INT_LIMIT:
      uint8_t pos_integral_limit = read_uint8(bytes + 2);

      update_int_limit(&pos_pid, pos_integral_limit);

      write_uint8(bytes + 2, pos_integral_limit);

      canbus_send_order(bytes, nb_bytes);
      break;
    case POS_MOD_FACT:
      uint8_t pos_mod_fact = read_uint8(bytes + 2);

      update_mod_fact(&pos_pid, pos_mod_fact);

      write_uint8(bytes + 2, pos_mod_fact);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPE_PROP_GAIN:
      uint32_t spe_kp = read_uint32(bytes + 2);

      update_kp(&spe_pid, read_16_16(spe_kp));

      write_uint32(bytes + 2, spe_kp);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPE_INT_GAIN:
      uint32_t spe_ki = read_uint32(bytes + 2);

      update_ki(&spe_pid, read_16_16(spe_ki));

      write_uint32(bytes + 2, spe_ki);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPE_DER_GAIN:
      uint32_t spe_kd = read_uint32(bytes + 2);

      update_kd(&spe_pid, read_16_16(spe_kd));

      write_uint32(bytes + 2, spe_kd);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPE_INT_LIMIT:
      uint8_t spe_integral_limit = read_uint8(bytes + 2);

      update_int_limit(&spe_pid, spe_integral_limit);

      write_uint8(bytes + 2, spe_integral_limit);

      canbus_send_order(bytes, nb_bytes);
      break;
    case SPE_MOD_FACT:
      uint8_t spe_mod_fact = read_uint8(bytes + 2);

      update_mod_fact(&spe_pid, spe_mod_fact);

      write_uint8(bytes + 2, spe_mod_fact);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TOR_PROP_GAIN:
      uint32_t tor_kp = read_uint32(bytes + 2);

      update_kp(&tor_pid, read_16_16(tor_kp));


      write_uint32(bytes + 2, tor_kp);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TOR_INT_GAIN:
      uint32_t tor_ki = read_uint32(bytes + 2);

      update_ki(&tor_pid, read_16_16(tor_ki));

      write_uint32(bytes + 2, tor_ki);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TOR_DER_GAIN:
      uint32_t tor_kd = read_uint32(bytes + 2);

      update_kd(&tor_pid, read_16_16(tor_kd));

      write_uint32(bytes + 2, tor_kd);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TOR_INT_LIMIT:
      uint8_t tor_integral_limit = read_uint8(bytes + 2);

      update_int_limit(&tor_pid, tor_integral_limit);

      write_uint8(bytes + 2, tor_integral_limit);

      canbus_send_order(bytes, nb_bytes);
      break;
    case TOR_MOD_FACT:
      uint8_t tor_mod_fact = read_uint8(bytes + 2);

      update_mod_fact(&tor_pid, tor_mod_fact);

      write_uint8(bytes + 2, tor_mod_fact);

      canbus_send_order(bytes, nb_bytes);
      break;
    default:
      // Unknown information 
      board_set_leds(BOARD_LED_RED);
  }
}

void  usb_lp_can1_rx0_isr(void) {
  
  BaseType_t hptw = pdFALSE;

  
  if (can_fifo_pending(CAN1, 0) <= 0)
    return;

  if (canbus_receive_order(rx_buffer, &rx_buffer_nb_bytes) ||
      rx_buffer_nb_bytes < 1)
    return;

  if (task_order_handler) {
    vTaskNotifyGiveFromISR(task_order_handler, &hptw);
  }

  portYIELD_FROM_ISR(hptw);
}



static void task_orders(void *p __attribute__ ((unused)))
{
  static uint8_t polarity = 0;


  for (;;)
    {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  

      switch (rx_buffer[0])
        {
        case PROTOCOL_ORDER_DEBUG:
          process_debug_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        case PROTOCOL_ORDER_GET:
          process_get_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        case PROTOCOL_ORDER_PID_POS:
          process_pid_pos_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        case PROTOCOL_ORDER_PID_SPE:
          process_pid_spe_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        case PROTOCOL_ORDER_PID_TOR:
          process_pid_tor_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        case PROTOCOL_ORDER_WRITE_0:
          if(!polarity) {
            process_write_order(rx_buffer, rx_buffer_nb_bytes);
            polarity = 1 - polarity;
          }
          break;
        case PROTOCOL_ORDER_WRITE_1:
          if(polarity) {
            process_write_order(rx_buffer, rx_buffer_nb_bytes);
            polarity = 1 - polarity;
          }
         break;
        case PROTOCOL_ORDER_SYNC:
          process_sync_order(rx_buffer, rx_buffer_nb_bytes);
          break;
        default:
          // Unkown order
          board_set_leds(BOARD_LED_RED);
          continue;
        }      
    }
}

static void task_main_loop(void *p __attribute__ ((unused)))
{

  

  for(;;) {

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 


    /* Reset flags */
    
    clear_all_flags();

    
    /* Measure sensors */

    /** Angle **/
    
    double prev_servo_angle;
    if (!prev_angle_set)
    {
      prev_servo_angle = (double)read_mag_sensor()*angle_resolution;
      prev_angle_set = true;
      servo_angle = prev_servo_angle;
    } else {
      prev_servo_angle = servo_angle;
      servo_angle = (double)read_mag_sensor()*angle_resolution;
    }

    /***  Compute the difference between the angles ***/
    struct angle_diffences ad = compute_angle_diff(servo_angle, prev_servo_angle);
    servo_motor_speed = ad.small_diff/cycle_period;

    /** Linear acceleration **/

    servo_linear_acc_x = read_lin_acc_X_sensor()*imu_acc_sensitivity*g;
    servo_linear_acc_y = read_lin_acc_Y_sensor()*imu_acc_sensitivity*g;
    servo_linear_acc_z = read_lin_acc_Z_sensor()*imu_acc_sensitivity*g;

    /** Angular rate **/

    servo_angular_spe_x = read_ang_rate_X_sensor()*imu_gyr_sensitivity;
    servo_angular_spe_y = read_ang_rate_Y_sensor()*imu_gyr_sensitivity;
    servo_angular_spe_z = read_ang_rate_Z_sensor()*imu_gyr_sensitivity;

    /** Torque **/

    current = read_curr_sensor();
    torque = (double)-current*2.521 + 5155.445; // torque in mNm

    /** Battery voltage **/

    batt_voltage = read_vbat_sensor(); 
    batt_voltage = ((batt_voltage/4095.0)*3.3)*((50+10)/10);

    /** Motor temperature **/

    therm_v  = read_therm_sensor();
    double v_0 = (therm_v / 4095.0)*3.3;
    double r_therm = 4.7*(3.3-v_0)/v_0;
    double r_ref = 10;  

    motor_temp = 1/(3.354016E-03 + 2.569850E-04*log(r_therm/r_ref) + 2.620131E-06*pow(log(r_therm/r_ref), 2) + 6.383091E-08*pow(log(r_therm/r_ref), 3));

    motor_temp -= 273.15;


    /* Hard limit check */

    /** Temperature check **/

    if(motor_temp > max_motor_temp) {

      set_flag(MAX_TEMP_FLAG);

      disable_motor();
    }



    /* PID controller */

    if(pos_pid_enabled) {
      pos_pid_output = pos_pid_controller();
    } else if(spe_pid_enabled) {
      spe_pid_output = spe_pid_controller();
    } else if(tor_pid_enabled) {
      tor_pid_output = tor_pid_controller();
    }

  }

  
}


/***  Entry point  ***/

int main(void)
{

  /* Initialize board. */

  init_board();

  /* Create task. */

  xTaskCreate(task_orders, "orders", 512, NULL, 4, &task_order_handler);
  xTaskCreate(task_main_loop, "main_loop", 512, NULL, 3, &task_main_handler);

  /* Start OS. */

  vTaskStartScheduler();

  /* We should never get here. */

  for (;;);
}