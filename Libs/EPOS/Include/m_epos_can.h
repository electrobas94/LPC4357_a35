#ifndef CAN_H
#define CAN_H

#include "lpc43xx_can.h"

//Epos controller ID
#define EPOS_HORIZONTAL 2
#define EPOS_VERTICAL 1

#define CAN_COB_ID_SDO_SEND                       0x0600
#define CAN_COB_ID_SDO_RECEIVE                    0x0580
#define CAN_COB_ID_SDO_EMERGENCY                  0x0080

//SDO CMD BYTE
#define CAN_CMD_SDO_WRITE_SEND_1_BYTE             0x2F
#define CAN_CMD_SDO_WRITE_SEND_2_BYTE             0x2B
#define CAN_CMD_SDO_WRITE_SEND_4_BYTE             0x23
#define CAN_CMD_SDO_WRITE_SEND_UNDEFINED          0x22
#define CAN_CMD_SDO_WRITE_SEND_N_BYTE_INIT        0x21
#define CAN_CMD_SDO_WRITE_SEND_N_BYTE_SEGMENT     0x00
#define CAN_CMD_SDO_WRITE_RECEIVE                 0x60

#define CAN_CMD_SDO_READ_RECEIVE_1_BYTE           0x4F
#define CAN_CMD_SDO_READ_RECEIVE_2_BYTE           0x4B
#define CAN_CMD_SDO_READ_RECEIVE_4_BYTE           0x43
#define CAN_CMD_SDO_READ_RECEIVE_UNDEFINED        0x42
#define CAN_CMD_SDO_READ_RECEIVE_N_BYTE_INIT      0x41
#define CAN_CMD_SDO_READ_RECEIVE_N_BYTE_SEGMENT   0x60
#define CAN_CMD_SDO_READ_SEND                     0x40

#define CAN_CMD_SDO_ABORT                         0xC0

//ERROR CODE
#define CAN_ERROR_NONE                            0 // Success
#define CAN_ERROR_CONFIG                          1 // CAN configuration error
#define CAN_ERROR_SEND                            2 // Failed to send CAN message
#define CAN_ERROR_RECEIVE                         3 // Failed to receive CAN message

void can_init(void);

int can_device_send_message(uint32_t node_id, message_object* message);



#endif
