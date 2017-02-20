#ifndef EPOS_DEVICE_H
#define EPOS_DEVICE_H

#include "m_epos_can.h"

//Object index
#define EPOS_DEVICE_INDEX_ERROR_REGISTER        0x1001
#define EPOS_DEVICE_INDEX_ERROR_HISTORY        	0x1003
#define EPOS_DEVICE_INDEX_STORE                 0x1010
#define EPOS_DEVICE_SUBINDEX_STORE              0x01
#define EPOS_DEVICE_INDEX_RESTORE               0x1011
#define EPOS_DEVICE_SUBINDEX_RESTORE            0x01
#define EPOS_DEVICE_INDEX_ID                    0x2000
#define EPOS_DEVICE_INDEX_CAN_BIT_RATE          0x2001
#define EPOS_DEVICE_INDEX_RS232_BAUD_RATE       0x2002
#define EPOS_DEVICE_INDEX_VERSION               0x2003
#define EPOS_DEVICE_INDEX_MISC_CONFIGURATION    0x2008
#define EPOS_DEVICE_SUBINDEX_SOFTWARE_VERSION   0x01
#define EPOS_DEVICE_SUBINDEX_HARDWARE_VERSION   0x02
#define EPOS_DEVICE_INDEX_CONTROL               0x6040
#define EPOS_DEVICE_INDEX_STATUS                0x6041
#define EPOS_DEVICE_INDEX_MODES	                0x6060
#define EPOS_DEVICE_INDEX_MODES_DISPLAY	        0x6061

#define EPOS_DEVICE_INDEX_ACTUAL_POSITION	      0x6064
#define EPOS_DEVICE_INDEX_ACTUAL_SPEED		      0x606C

//###VEL PROFILE
#define EPOS_VELPROF_SPEED	        						0x60FF
#define EPOS_VELPROF_ACCSELERATIONS	    				0x6083
#define EPOS_STOP_ACCELERATION									0x6085
#define EPOS_VELPROF_DURATIONS       						0x6084

// Control word
#define EPOS_DEVICE_CONTROL_SWITCH_ON           0x0001
#define EPOS_DEVICE_CONTROL_ENABLE_VOLTAGE      0x0003
#define EPOS_DEVICE_CONTROL_SHUTDOWN            0x0006
#define EPOS_DEVICE_CONTROL_QUICK_STOP          0x0007
#define EPOS_DEVICE_CONTROL_ENABLE_OPERATION    0x000F
#define EPOS_DEVICE_CONTROL_FAULT_RESET         0x0080


#define EPOS_DEVICE_TYPE_MASK                   0xFFF0


#define EPOS_DEVICE_ERROR_NONE                  0
//!< Success
#define EPOS_DEVICE_ERROR_INVALID_SIZE          1
//!< Invalid EPOS object size
#define EPOS_DEVICE_ERROR_SEND                  2
//!< Failed to send to EPOS device
#define EPOS_DEVICE_ERROR_RECEIVE               3
//!< Failed to receive from EPOS device
#define EPOS_DEVICE_ERROR_ABORT                 4
//!< EPOS communication error (abort)
#define EPOS_DEVICE_ERROR_INTERNAL              5
//!< EPOS internal device error
#define EPOS_DEVICE_ERROR_WAIT_TIMEOUT          6
//!< EPOS device timeout

#define EPOS_PROFILE_POSITION 									0
#define EPOS_PROFILE_HOME												6
#define EPOS_PROFILE_VELOSITY										3


int32_t epos_get_actual_position(uint32_t dev);
int32_t epos_get_actual_speed(uint32_t dev);

//### Velosity mode functions

int epos_set_velosity_profile(uint32_t dev); 

int epos_velprof_set_speed(uint32_t dev, int16_t speed); //EPOS_DEVICE_INDEX_CONTROL

int epos_velprof_set_accselerations(uint32_t dev, int32_t accs); //EPOS_VELPROF_DURATIONS

int epos_velprof_set_durations(uint32_t dev, int32_t dur); //EPOS_VELPROF_DURATIONS

int epos_velprof_halt(uint32_t dev); //EPOS_DEVICE_INDEX_CONTROL

int epos_velprof_move(uint32_t dev); //EPOS_DEVICE_INDEX_CONTROL

// ## velosity mode funktions



int epos_device_open(uint32_t dev) ;

int epos_device_close(uint32_t dev) ;

int epos_device_read(uint32_t dev, short index, 
												unsigned char subindex,
												unsigned char* data, uint32_t num) ;
												
int epos_device_write(uint32_t dev, short index, unsigned char subindex,
    unsigned char* data, uint32_t num) ;

uint32_t epos_device_get_status(uint32_t dev) ;

int epos_device_wait_status(uint32_t dev, short status, double timeout);

short epos_device_get_control(uint32_t dev) ;

int epos_device_set_control(uint32_t dev, short control) ;

int epos_device_set_profile(uint32_t dev, uint8_t profile);

int8_t epos_device_get_profile(uint32_t dev) ;


int epos_device_move_rel(uint32_t dev, int32_t pos);

int epos_device_move_abs(uint32_t dev, int32_t pos);

int epos_device_preset_speed(uint32_t dev, uint16_t speed);

//
int epos_device_set_speed(uint32_t dev, int16_t speed);

unsigned char epos_device_get_error(uint32_t dev) ;

int epos_device_shutdown(uint32_t dev) ;

int epos_device_enable_operation(uint32_t dev) ;
//

int epos_device_enable(uint32_t dev) ;

int epos_device_reset(uint32_t dev);

void RX_callback(uint32_t msg_no);

void TX_callback(uint32_t msg_no);

#endif
