#ifndef CMD_ADUNOK
#define CMD_ADUNOK

// Command
//#define BU_EXEC_CAN						0x50
#define BU_SET_SPEED_H 				0x11
#define BU_SET_SPEED_V 				0x12
#define BU_PRESET_SPEED_H 		0x13
#define BU_PRESET_SPEED_V 		0x14
#define BU_SET_POS_REL_H 			0x15
#define BU_SET_POS_REL_V 			0x16
#define BU_SET_POS_ABS_H 			0x17
#define BU_SET_POS_ABS_V 			0x18

#define BU_READ_PRESS_OUT 			0x1B
#define BU_READ_TEMP_OUT	 			0x19
#define BU_READ_U								0x29
#define BU_ACTION_SHOOT     		0x1E
#define BU_READ_TYPE_ARM				0x24
#define BU_SET_SHOOT_MAX_TIME		0x1F
#define BU_SET_SHOOT_MAX_CUR		0x20
#define BU_READ_SHOOT_MAX_TIME	0x21
#define BU_READ_SHOOT_MAX_CUR		0x22
//#define BU_READ_SHOOT_CUR				0x23
#define BU_SET_ENABLE_FIER			0x30
#define BU_SET_FIER							0x31
#define BU_SET_SCOR_CART				0x1C
#define BU_READ_SCOR_CART				0x1D
#define BU_SET_GIRO							0x32

//Command for BNIC
#define BU_READ_RANGE					0xF6
#define BU_SET_CAMERA					0x25
#define BU_READ_CAMERA				0x26
#define BU_SET_TV							0x27
#define BU_READ_TEMT_BNIC			0x1A
#define BU_OPEN_GATE					0x67
#define BU_CLOSE_GATE					0x65

//Aditional command
#define BU_READ_POS_H					0xF7
#define BU_READ_POS_V					0xF8
#define BU_READ_ACT_SPEED_H		0xF9
#define BU_READ_ACT_SPEED_V		0xFA
#define BU_READ_MOVE_FLAG			0xFB
#define BU_SET_FIRE_TIME			0xFC

#define BU_SET_MOVIE_LIMIT		0x51


#endif // CMD_ADUNOK
