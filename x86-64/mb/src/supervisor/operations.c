/*
 *  Montefiore Robocup
 *  ==================
 *
 *  operations.c: Operations of the motherboard.
 */

#include <stdint.h>
#include <stdio.h>  /* debug */
#include <stdbool.h>
#include <math.h>

#include "operations.h"
#include "../utils/bytes.h"
#include "../utils/protocol.h"
#include "../utils/communication.h"
#include "../utils/information_id.h"
#include "../utils/crc.h"
#include "../utils/constant.h"
#include "../world/world.h"
#include "test_program.h"
#include "../utils/pid_visualisation.h"
#include "../utils/fixed_point_format.h"
#include "../expert/expert-collisions.h"
#include "../log/log.h"
#include "../reflex/hold_reflex.h"


/***  Public functions  ***/

void  op_new_period(uint32_t  period_nb)
{

  log_debug("new period: %u\n", (unsigned) period_nb);

  // Signal the reflex
  pthread_cond_signal(&cond_hold);

  // Signal the expert
  pthread_cond_signal(&cond_collisions);
  
}


void  op_recv_frame(uint8_t addr, uint8_t *bytes, uint32_t nb_bytes)
{
  uint32_t  v;

  if (nb_bytes == 5 && bytes[0] == (PROTOCOL_ORDER_DEBUG)) {
    log_normal("Debug order received \n");

    v = read_uint32(bytes + 1);

    printf("DEBUG : MB received value = %u from servo ID %d\n", (unsigned) v, addr);
    log_normal("DEBUG : MB received value = %u from servo ID %d\n", (unsigned) v, addr);

    fflush(stdout);

  } else if (bytes[0] == (PROTOCOL_ORDER_GET)) {

    uint8_t information_id = read_uint8(bytes + 1);
    double angle;
    double lin_acc_x, lin_acc_y, lin_acc_z;
    double ang_spe_x, ang_spe_y, ang_spe_z;
    double voltage;
    double current;
    double v, v_0, r_therm, r_ref, motor_temp;
    double motor_speed;
    uint32_t formated_kp, formated_ki, formated_kd;
    uint8_t flags;

    switch (information_id) {
      case (MOTOR_POSITION):
        
        angle = read_uint16(bytes + 2)*angle_resolution;

        update_position(addr, angle);
        
        log_normal("GET: MB received angle = %f° from servo ID %d\n", angle, addr);
        break;
      case (LINEAR_ACC):
        
        lin_acc_x = read_int16(bytes + 2)*(imu_acc_sensitivity*g);
        lin_acc_y = read_int16(bytes + 4)*(imu_acc_sensitivity*g);
        lin_acc_z = read_int16(bytes + 6)*(imu_acc_sensitivity*g);

        update_lin_acc(addr, lin_acc_x, lin_acc_y, lin_acc_z);

        // Signal the expert
        pthread_cond_signal(&cond_collisions);

        log_normal("GET: MB received lin_acc x = %f m/s² from servo ID %d \n", lin_acc_x, addr);
        log_normal("GET: MB received lin_acc y = %f m/s² from servo ID %d\n", lin_acc_y, addr);
        log_normal("GET: MB received lin_acc z = %f m/s² from servo ID %d\n", lin_acc_z, addr);
        break;
      case (ANGULAR_SPEED):
        
        ang_spe_x = read_int16(bytes + 2)*imu_gyr_sensitivity;
        ang_spe_y = read_int16(bytes + 4)*imu_gyr_sensitivity;
        ang_spe_z = read_int16(bytes + 6)*imu_gyr_sensitivity;

        log_normal("GET: MB received ang_spe_x = %f dps from servo ID %d\n", ang_spe_x, addr);
        log_normal("GET: MB received ang_spe_y = %f dps from servo ID %d\n", ang_spe_y, addr);
        log_normal("GET: MB received ang_spe_z = %f dps from servo ID %d\n", ang_spe_z, addr);
        break;
      case (TORQUE):
        
        current = read_uint32(bytes + 2);
        double torque = (double)-current*2.521 + 5155.445;

        update_torque(addr, torque);

        log_normal("GET: MB received torque = %lf mNm from servo ID %d\n", torque, addr);

        break;
      case (VOLTAGE):
        
        voltage = read_uint16(bytes + 2);
        voltage = ((voltage/4095.0)*3.3)*((50+10)/10);

        log_normal("GET: MB received bat voltage = %lf V from servo ID %d\n", voltage, addr);

        break;
      case (MOTOR_TEMP):
        
        v = read_uint16(bytes + 2);
        v_0 = (v / 4095.0)*3.3;

        r_therm = 4.7*(3.3-v_0)/v_0;
        r_ref = 10;  

        motor_temp = 1/(3.354016E-03 + 2.569850E-04*log(r_therm/r_ref) + 2.620131E-06*pow(log(r_therm/r_ref), 2) + 6.383091E-08*pow(log(r_therm/r_ref), 3));

        log_debug("GET: MB received motor temperature = %f°C from servo ID %d\n", motor_temp - 273.15, addr);
        break;
      case (MOTOR_SPEED):
        
        motor_speed = read_16_16(read_int32(bytes + 2));

        update_speed(addr, motor_speed);

        log_normal("GET: MB received motor_speed = %f dps from servo ID %d\n", motor_speed, addr);
        break;
      case (POS_PROP_GAIN):
        
        formated_kp = read_uint32(bytes + 2);

        log_normal("GET: MB received kp = %lf from servo ID %d\n", read_16_16(formated_kp), addr);
        break;
      case (POS_INT_GAIN):
        
        formated_ki = read_uint32(bytes + 2);

        log_normal("GET: MB received ki = %lf from servo ID %d\n", read_16_16(formated_ki), addr);
        break;
      case (POS_DER_GAIN):
        
        formated_kd = read_uint32(bytes + 2);

        log_normal("GET: MB received kd = %lf from servo ID %d\n", read_16_16(formated_kd), addr);
        break;
      case (FLAGS):
        
        flags = read_uint8(bytes + 2);

        log_normal("GET: MB received flag = %d from servo ID %d\n", flags, addr);
        break;
      default:
        
        v = read_uint32(bytes + 2);
        printf("  Information ID %u received is unknown and of value = %u from servo ID %d\n", (unsigned) information_id, (unsigned) v, addr);
        log_warn("  Information ID %u received is unknown and of value = %ufrom servo ID %d\n", (unsigned) information_id, (unsigned) v, addr);
    }

    fflush(stdout);

  } else if (bytes[0] == (PROTOCOL_ORDER_PID_POS)) {

    v = read_uint8(bytes + 1);

    log_normal("PID position: Servo ID %d returned = %u\n", addr,(unsigned) v);


  } else if (bytes[0] == (PROTOCOL_ORDER_PID_SPE)) {

    v = read_uint8(bytes + 1);
    log_normal("PID velocity: Servo ID %d returned = %u\n", addr, (unsigned) v);


  } else if (bytes[0] == (PROTOCOL_ORDER_PID_TOR)) {

    v = read_uint8(bytes + 1);
    log_normal("PID torque: Servo ID %d returned = %u\n", addr, (unsigned) v);


  } else if (bytes[0] == (PROTOCOL_ORDER_WRITE_0) || bytes[0] == (PROTOCOL_ORDER_WRITE_1)) {
    
    uint8_t information_id = read_uint8(bytes + 1);
    uint32_t v;
    double angle_ref, speed_ref;
    int16_t torque_ref;
    uint16_t max_torque;
    uint32_t pos_formated_kp, pos_formated_ki, pos_formated_kd, spe_formated_kp, spe_formated_ki, spe_formated_kd, tor_formated_kp, tor_formated_ki, tor_formated_kd;
    double pos_kp, pos_ki, pos_kd, spe_kp, spe_ki, spe_kd, tor_kp, tor_ki, tor_kd;
    uint8_t pos_mod_fact, spe_mod_fact, tor_mod_fact;
    uint8_t pos_int_limit, spe_int_limit, tor_int_limit;

    switch (information_id) {
      case (POSITION_REFERENCE):
        
        angle_ref = (double)read_uint16(bytes + 2)*angle_resolution;

        update_ref_position(addr, angle_ref);

        log_normal("WRITE: Servo ID %d received the angle reference = %lf°\n", addr, angle_ref);
        break;
      case (SPEED_REFERENCE):
        
        speed_ref = read_16_16(read_int32(bytes + 2));

        update_ref_speed(addr, speed_ref);

        log_normal("WRITE: Servo ID %d received the speed reference = %lf dps\n", addr, speed_ref);
        break;
      case (TORQUE_REFERENCE):
        
        torque_ref = read_int16(bytes + 2);

        update_ref_torque(addr, torque_ref);

        log_normal("WRITE: Servo ID %d received the torque reference = %ld mNm\n", addr, torque_ref);
        break;
      case (MAX_TORQUE):
        
        max_torque = read_uint8(bytes + 2)*512/100;
 
        // Update the knowledge of motherboard
        update_max_torque(addr, max_torque);

        log_normal("WRITE: Servo ID %d received max torque = %d mNm\n", addr, max_torque);
        break;
      case (POS_PROP_GAIN):
        
        pos_formated_kp = read_uint32(bytes + 2);
        pos_kp = read_16_16(pos_formated_kp);

        // Update the knowledge of motherboard
        update_pos_kp(addr, pos_kp);

        log_normal("WRITE: Servo ID %d received position proportional gain = %lf\n", addr, pos_kp);
        break;
      case (POS_INT_GAIN):
        
        pos_formated_ki = read_uint32(bytes + 2);
        pos_ki = read_16_16(pos_formated_ki);

        // Update the knowledge of motherboard
        update_pos_ki(addr, pos_ki);

        log_normal("WRITE: Servo ID %d received position integral gain = %lf\n", addr, pos_ki);
        break;
      case (POS_DER_GAIN):
        
        pos_formated_kd = read_uint32(bytes + 2);
        pos_kd = read_16_16(pos_formated_kd);

        // Update the knowledge of motherboard
        update_pos_kd(addr, pos_kd);

        log_normal("WRITE: Servo ID %d received positon derivative gain = %lf\n", addr, pos_kd);
        break;
      case (POS_MOD_FACT):
        
        pos_mod_fact = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_pos_mod_fact(addr, pos_mod_fact);

        log_normal("WRITE: Servo ID %d received position modulation factor = %d %\n",addr, pos_mod_fact);
        break;
      case (POS_INT_LIMIT):
        
        pos_int_limit = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_pos_int_limit(addr, pos_int_limit);

        log_normal("WRITE: Servo ID %d received position integral limit = %u\n", addr, pos_int_limit);
        break;
      case (SPE_PROP_GAIN):
        
        spe_formated_kp = read_uint32(bytes + 2);
        spe_kp = read_16_16(spe_formated_kp);

        // Update the knowledge of motherboard
        update_spe_kp(addr, spe_kp);

        log_normal("WRITE: Servo ID %d received speed proportional gain = %lf\n",addr, spe_kp);
        break;
      case (SPE_INT_GAIN):
        
        spe_formated_ki = read_uint32(bytes + 2);
        spe_ki = read_16_16(spe_formated_ki);

        // Update the knowledge of motherboard
        update_spe_ki(addr, spe_ki);

        log_normal("WRITE: Servo ID %d received speed integral gain = %lf\n", addr, spe_ki);
        break;
      case (SPE_DER_GAIN):
        
        spe_formated_kd = read_uint32(bytes + 2);
        spe_kd = read_16_16(spe_formated_kd);

        // Update the knowledge of motherboard
        update_spe_kd(addr, spe_kd);

        log_normal("WRITE: Servo ID %d received speed derivative gain = %lf\n", addr, spe_kd);
        break;
      case (SPE_INT_LIMIT):
        
        spe_int_limit = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_spe_int_limit(addr, spe_int_limit);

        log_normal("WRITE: Servo ID %d received speed integral limit = %u\n",addr, spe_int_limit);
        break;
      case (SPE_MOD_FACT):
        
        spe_mod_fact = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_spe_mod_fact(addr, spe_mod_fact);

        log_normal("WRITE: Servo ID %d received speed modulation factor = %d %\n", addr, spe_mod_fact);
        break;
      case (TOR_PROP_GAIN):
        
        tor_formated_kp = read_uint32(bytes + 2);
        tor_kp = read_16_16(tor_formated_kp);

        // Update the knowledge of motherboard
        update_tor_kp(addr, tor_kp);

        log_normal("WRITE: Servo ID %d received torque proportional gain = %lf\n", addr, tor_kp);
        break;
      case (TOR_INT_GAIN):
        
        tor_formated_ki = read_uint32(bytes + 2);
        tor_ki = read_16_16(tor_formated_ki);

        // Update the knowledge of motherboard
        update_tor_ki(addr, tor_ki);

        log_normal("WRITE: Servo ID %d received torque integral gain = %lf\n",addr,  tor_ki);
        break;
      case (TOR_DER_GAIN):
        
        tor_formated_kd = read_uint32(bytes + 2);
        tor_kd = read_16_16(tor_formated_kd);

        // Update the knowledge of motherboard
        update_tor_kd(addr, tor_kd);

        log_normal("WRITE: Servo ID %d received torque derivative gain = %lf\n", addr, tor_kd);
        break;
      case (TOR_INT_LIMIT):
        
        tor_int_limit = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_tor_int_limit(addr, tor_int_limit);

        log_normal("WRITE: Servo ID %d received torque integral limit = %u\n", addr, tor_int_limit);
        break;
      case (TOR_MOD_FACT):
        
        tor_mod_fact = read_uint8(bytes + 2);

        // Update the knowledge of motherboard
        update_tor_mod_fact(addr, tor_mod_fact);

        log_normal("WRITE: Servo ID %d received torque modulation factor = %d % \n", addr, tor_mod_fact);
        break;
      default:
        v = read_uint32(bytes + 2);
        log_warn("WRITE: Servo ID %d received unknown value = %u\n", addr, (unsigned) v);
    } 

    

  } else {
    printf("Servo ID %d received unknown message = %u\n", addr, bytes[0]);
    log_critical(" Servo ID %d received unknown message = %u\n",addr, bytes[0]);
  }

  fflush(stdout);
}
