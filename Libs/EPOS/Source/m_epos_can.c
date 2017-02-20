#include "m_epos_can.h"

message_object recv_obj;

void can_init()
{
	recv_obj.id = 0x581;
	recv_obj.dlc=8;
}

int can_device_send_message(uint32_t node_id, message_object* send_obj)
{
	send_obj->id = CAN_COB_ID_SDO_SEND + node_id;
	recv_obj.id=	CAN_COB_ID_SDO_RECEIVE + node_id;
	send_obj->dlc = 8;
	
	CAN_Recv(1, (uint32_t*)&recv_obj, FALSE);
	CAN_Send(17, (uint32_t *)send_obj);
	
	return 0;
}
