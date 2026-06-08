/*
 *  Montefiore Robocup
 *  ==================
 *
 *  information_id.h: ID of the data stored in the daughter boards
 */

#ifndef _INFORMATION_ID_H
#define _INFORMATION_ID_H

/***  Definitions  ***/

/* Data name */

#define ID                   0x00
#define SERIAL_NB            0x01
#define MOTOR_POSITION       0x02
#define ANGULAR_SPEED        0x03
#define LINEAR_ACC           0x04
#define TORQUE               0x05
#define MOTOR_SPEED          0x06
#define VOLTAGE              0x07
#define MOTOR_TEMP           0x08
#define MC_TEMP              0x09
#define LED                  0x0a
#define PWM_COMMAND          0x0b
#define CW_ANGLE_LIMIT       0x0c
#define CCW_ANGLE_LIMIT      0x0d
#define MAX_MOTOR_SPEED      0x0e
#define MAX_TORQUE           0x0f
#define MOTOR_TEMP_LIMIT     0x10
#define MC_TEMP_LIMIT        0x11
#define MIN_VOLTAGE_LIMIT    0x12
#define MAX_VOLTAGE_LIMIT    0x13
#define POSITION_REFERENCE   0x14
#define SPEED_REFERENCE      0x15
#define TORQUE_REFERENCE     0x16
#define POS_PROP_GAIN        0x17
#define POS_INT_GAIN         0x18
#define POS_DER_GAIN         0x19
#define SPE_PROP_GAIN        0x1a
#define SPE_INT_GAIN         0x1b
#define SPE_DER_GAIN         0x1c
#define TOR_PROP_GAIN        0x1d
#define TOR_INT_GAIN         0x1e
#define TOR_DER_GAIN         0x1f
#define PID_POS_OUTPUT       0x20
#define PID_SPE_OUTPUT       0x21
#define PID_TOR_OUTPUT       0x22
#define POS_INT_LIMIT        0x23
#define SPE_INT_LIMIT        0x24
#define TOR_INT_LIMIT        0x25
#define POS_MOD_FACT         0x26
#define SPE_MOD_FACT         0x27
#define TOR_MOD_FACT         0x28
#define POS_TOL              0x29
#define SPE_TOL              0x2a
#define TOR_TOL              0x2b
#define FLAGS                0x2c


#endif  /* _INFORMATION_ID_H */
