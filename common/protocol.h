/*
 *  Montefiore Robocup
 *  ==================
 *
 *  protocol.h: Communication protocols (USB MB <-> MF, CAN MF <-> DBs).
 */

#ifndef _PROTOCOL_H
#define _PROTOCOL_H

/***  Definitions  ***/

/*  USB protocol parameters  */

#define PROTOCOL_USB_BROADCAST_ALL_BUSSES  0x00
#define PROTOCOL_USB_BROADCAST_ONE_BUS     0x01
#define PROTOCOL_USB_UNICAST               0x03

/*  CAN protocol parameters  */

#define PROTOCOL_CAN_MIN_DB_ADDR   0x01
#define PROTOCOL_CAN_MAX_DB_ADDR   0x1f
#define PROTOCOL_CAN_DB_ADDR_MASK  0x1f

#define PROTOCOL_CAN_BROADCAST_BIT    0
#define PROTOCOL_CAN_UNICAST_BIT      1

/*  Orders (--> daughter boards)  */

#define PROTOCOL_ORDER_BOOT        0x01
#define PROTOCOL_ORDER_UPDATE      0x02
#define PROTOCOL_ORDER_DATA_0      0x03
#define PROTOCOL_ORDER_DATA_1      0x04
#define PROTOCOL_ORDER_UPDATE_ID   0x05
#define PROTOCOL_ORDER_SYNC        0x06
#define PROTOCOL_ORDER_GET         0x07
#define PROTOCOL_ORDER_PID_POS     0x08
#define PROTOCOL_ORDER_PID_SPE     0x09
#define PROTOCOL_ORDER_PID_TOR     0x0a
#define PROTOCOL_ORDER_WRITE_0      0x0b
#define PROTOCOL_ORDER_WRITE_1      0x0c
#define PROTOCOL_ORDER_MODE        0x0d

#define PROTOCOL_ORDER_GO          0xfd
#define PROTOCOL_ORDER_DEBUG       0xfe

/*  Order parameters  */

#define PROTOCOL_TIME_ASAP (-1)

#define PROTOCOL_BOOT_MODE_NOMINAL  0x00
#define PROTOCOL_BOOT_MODE_ERROR    0x01
#define PROTOCOL_BOOT_MODE_ADDR     0x02

#endif  /* _PROTOCOL_H */
