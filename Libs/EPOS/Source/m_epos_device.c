#include "m_epos_device.h"

message_object message;
message_object message_recv;
volatile uint8_t flag_recv;
volatile uint32_t last_error;

volatile int32_t pre_speed=1000;
//####### Function foor UART

int epos_device_can_exec(uint8_t* data) 
{
	int test_p;
	
	for(test_p=0;test_p<8;test_p++)
		message.data[test_p]=data[test_p];

	flag_recv=0;
	//can_device_send_message(dev, &message);
					
	test_p=10000;
	while(flag_recv==0)
	{
		if(test_p!=0)
			test_p--;
		else
			break;
	}
	
	CAN_ReadMsg(0, &message_recv);
	//data=message_recv.data;
	
	return 0;
}


int epos_device_move_rel(uint32_t dev, int32_t pos) // ##########################################
{
	int32_t pos_abs;
	int32_t pos_tmp;
	int32_t pre_speedt=pre_speed;
	
	if(pos==0)
		return 0;
	
	if(pos<0)
		pre_speedt*=-1;
	
	epos_velprof_set_speed(dev, pre_speedt);

	pos_abs=epos_get_actual_position(dev);
	pos_abs+=pos;
	
	epos_velprof_move(dev);
	
	pos_tmp=epos_get_actual_position(dev);
	
	if(pos>0){
			while(pos_abs-pos_tmp>0)
			{
				pos_tmp=epos_get_actual_position(dev);
				if(!(epos_device_get_status(dev) &(1<<1)==(1<<1)))
        {
            epos_device_enable(dev);
            epos_set_velosity_profile(dev);
						epos_velprof_set_speed(dev, pre_speedt);
        }
			}
	}
	else
			while(pos_tmp-pos_abs>0 )
			{
				pos_tmp=epos_get_actual_position(dev);
				if(!(epos_device_get_status(dev) &(1<<1)==(1<<1)))
        {
            epos_device_enable(dev);
            epos_set_velosity_profile(dev);
						epos_velprof_set_speed(dev, pre_speedt);
        }
			}


	
	epos_velprof_halt(dev);
	
  return 0;
}

int epos_device_move_abs(uint32_t dev, int32_t pos) // ##########################################
{
	int32_t pos_abs;
	int32_t pos_tmp;
	int32_t pre_speedt=pre_speed;
	
	pos_tmp=epos_get_actual_position(dev);
	
	if(pos_tmp>pos)
		pre_speedt*=-1;
	
	epos_velprof_set_speed(dev, pre_speedt);

	pos_abs=pos;
	
	epos_velprof_move(dev);
	
	pos_tmp=epos_get_actual_position(dev);
	
	if(pre_speedt>0)
	{
			while(pos_abs-pos_tmp>0)
				pos_tmp=epos_get_actual_position(dev);
	}
	else
			while(pos_tmp-pos_abs>0 )
				pos_tmp=epos_get_actual_position(dev);


	
	epos_velprof_halt(dev);
	
  return 0;
}

int epos_device_preset_speed(uint32_t dev, uint16_t speed) // ##########################################
{
	pre_speed=speed;
	
	return 0;
}


int epos_device_set_speed(uint32_t dev, int16_t speed) // ##########################################
{
	int32_t tmp=speed;
	
  epos_device_write(dev, EPOS_VELPROF_SPEED, 0,
    (unsigned char*)&tmp, 4);
	
  return 0;
}

// ##### Position function
int32_t epos_get_actual_position(uint32_t dev)
{
	int32_t tmp=0;
	epos_device_read(dev,EPOS_DEVICE_INDEX_ACTUAL_POSITION, 0,(unsigned char*)&tmp, 4);
	
	return tmp;
}

int32_t epos_get_actual_speed(uint32_t dev)
{
	int16_t tmp=0;
	epos_device_read(dev,EPOS_DEVICE_INDEX_ACTUAL_SPEED, 0,(unsigned char*)&tmp, 4);
	
	return tmp;
}


//##########################VELOSYCY PROFILE###########################
int epos_set_velosity_profile(uint32_t dev) 
{
	if(epos_device_get_profile(dev)!=3)
		epos_device_set_profile(dev,3);

  return 0;
}

int epos_velprof_set_speed(uint32_t dev, int16_t speed) //EPOS_DEVICE_INDEX_CONTROL
{
	int32_t tpm=speed;
  epos_device_write(dev, EPOS_VELPROF_SPEED, 0,
    (unsigned char*)&tpm, 4);
  
  return 0;
}

int epos_velprof_set_accselerations(uint32_t dev, int32_t accs) //EPOS_VELPROF_DURATIONS
{
  epos_device_write(dev, EPOS_VELPROF_ACCSELERATIONS, 0,
    (unsigned char*)&accs, 4);
  
  return 0;
}

int epos_velprof_set_durations(uint32_t dev, int32_t dur) //EPOS_VELPROF_DURATIONS
{
  epos_device_write(dev, EPOS_VELPROF_DURATIONS, 0,
    (unsigned char*)&dur, 4);
	
	epos_device_write(dev, EPOS_STOP_ACCELERATION, 0,
    (unsigned char*)&dur, 4);
  
  return 0;
}

int epos_velprof_halt(uint32_t dev) //EPOS_DEVICE_INDEX_CONTROL
{
	uint16_t tmp=0x010F;
	
  epos_device_write(dev, EPOS_DEVICE_INDEX_CONTROL, 0,
    (unsigned char*)&tmp, 2);
  
  return 0;
}

int epos_velprof_move(uint32_t dev) //EPOS_DEVICE_INDEX_CONTROL
{
	uint16_t tmp=0x000F;
	
  epos_device_write(dev, EPOS_DEVICE_INDEX_CONTROL, 0,
    (unsigned char*)&tmp, 2);
  
  return 0;
}

//#########################################################################



int epos_device_open(uint32_t dev) 
{  
		epos_device_reset(dev);

    epos_device_shutdown(dev);

		return EPOS_DEVICE_ERROR_NONE;
}

int epos_device_close(uint32_t dev) 
{
		return epos_device_shutdown(dev);
}

int epos_device_read(uint32_t dev, short index, 
												unsigned char subindex,
												unsigned char* data, uint32_t num) 
{
	int test_p;
	//uint32_t tmp;
	//tmp=0xFFF;
	
  message.data[0] = CAN_CMD_SDO_READ_SEND;
  message.data[1] = index;
  message.data[2] = index >> 8;
  message.data[3] = subindex;
  message.dlc = 8;
	
	flag_recv=0;
	can_device_send_message(dev, &message);
	
	//while(tmp!=0)
	{
		//tmp--;
	}
	
	test_p=13000;
	while(flag_recv==0)
	{
		if(test_p!=0)
			test_p--;
		else
		{
			CAN_Init( CAN_BITRATE1000K12MHZ, CLKDIV1 , TX_callback, RX_callback);
			break;
		}
	}
	CAN_ReadMsg(0, &message_recv);
	
	data[0]=message_recv.data[4];
	data[1]=message_recv.data[5];
	data[2]=message_recv.data[6];
	data[3]=message_recv.data[7];
	
	return 0;
}

int epos_device_write(uint32_t dev, short index, unsigned char subindex,
    unsigned char* data, uint32_t num) 
{
	int test_p;

    switch (num) 
		{
      case 1 :
        message.data[0] = CAN_CMD_SDO_WRITE_SEND_1_BYTE;
        break;
      case 2 :
        message.data[0] = CAN_CMD_SDO_WRITE_SEND_2_BYTE;
        break;
      case 4 :
        message.data[0] = CAN_CMD_SDO_WRITE_SEND_4_BYTE;
        break;
      default:
        break;
        }
   // }
    message.data[1] = index;
    message.data[2] = index >> 8;
    message.data[3] = subindex;
    message.data[4] = data[0];
    message.data[5] = (num-0 > 1) ? data[0+1] : 0x00;
    message.data[6] = (num-0 > 2) ? data[0+2] : 0x00;
    message.data[7] = (num-0 > 2) ? data[0+3] : 0x00;

	flag_recv=0;
	can_device_send_message(dev, &message);

			
	//__WFI();	
	test_p=1000000;
	while(flag_recv==0)
	{
		if(test_p!=0)
			test_p--;
		else
		{
			CAN_Init( CAN_BITRATE1000K12MHZ, CLKDIV1 , TX_callback, RX_callback);
			break;
		}
	}
	
	CAN_ReadMsg(0, &message_recv);
	
	data=message_recv.data;
	
	return 0;
}

uint32_t epos_device_get_status(uint32_t dev) 
{
  uint32_t status = 0;
  epos_device_read(dev, EPOS_DEVICE_INDEX_STATUS, 0,
    (unsigned char*)&status, sizeof(uint32_t));

  return status;
}

int epos_device_wait_status(uint32_t dev, short status, double timeout) 
{
  while (!(status & epos_device_get_status(dev)) )
	{

  }

  return 0;
}

short epos_device_get_control(uint32_t dev) 
{
  short control = 0;
  epos_device_read(dev, EPOS_DEVICE_INDEX_CONTROL, 0,
    (unsigned char*)&control, sizeof(short));

  return control;
}

int epos_device_set_control(uint32_t dev, short control) 
{
  epos_device_write(dev, EPOS_DEVICE_INDEX_CONTROL, 0,
    (unsigned char*)&control, sizeof(short));
  
  return 0;
}

///////////////
int epos_device_set_profile(uint32_t dev, uint8_t profile) // ##########################################
{
  epos_device_write(dev, EPOS_DEVICE_INDEX_MODES, 0,
    &profile, 1);
  
  return 0;
}

int8_t epos_device_get_profile(uint32_t dev) 
{
  int8_t profile;
  epos_device_read(dev, EPOS_DEVICE_INDEX_CONTROL, 0,
    &profile, 1);

  return profile;
}

 // ##########################################

///////////////////////////////////////////////////////

unsigned char epos_device_get_error(uint32_t dev) 
	{
  unsigned char error = 0;
  epos_device_read(dev, EPOS_DEVICE_INDEX_ERROR_REGISTER, 0, &error, 1);

  return error;
}

int epos_device_shutdown(uint32_t dev) 
{
  epos_device_set_control(dev, EPOS_DEVICE_CONTROL_SHUTDOWN);
	
	return 0;
}

int epos_device_enable_operation(uint32_t dev) 
{
	epos_device_set_control(dev, EPOS_DEVICE_CONTROL_ENABLE_OPERATION);
	
	return 0;
}


int epos_device_enable(uint32_t dev) 
{
	int i=0;
	
	epos_device_reset(dev);
	
	epos_device_enable_operation(dev);

	epos_device_reset(dev);
	
	epos_device_shutdown(dev);
	for(i=0;i<100;i++);
	
	epos_device_set_control(dev, 0x010F);
	for(i=0;i<100;i++);
	
	return 0;
}

int epos_device_reset(uint32_t dev)
{
  epos_device_set_control(dev, EPOS_DEVICE_CONTROL_FAULT_RESET);
	
	return 0;
}

void RX_callback(uint32_t msg_no)
{
	flag_recv=1;
}


void TX_callback(uint32_t msg_no)
{
}
