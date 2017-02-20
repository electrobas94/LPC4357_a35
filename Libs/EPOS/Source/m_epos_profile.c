#include "m_epos_profile.h"

int epos_profile_wait(epos_node_t* node, double timeout) 
{
  return epos_device_wait_status(&node->dev, EPOS_PROFILE_STATUS_REACHED, timeout);
}

int epos_profile_set_acceleration(uint32_t dev, unsigned int acceleration) 
{
  epos_device_write(dev, EPOS_PROFILE_INDEX_ACCELERATION, 0,
    (unsigned char*)&acceleration, sizeof(unsigned int));
  
  return dev->error.code;
}

int epos_profile_set_deceleration(uint32_t dev, unsigned int deceleration) 
{
  epos_device_write(dev, EPOS_PROFILE_INDEX_DECELERATION, 0,
    (unsigned char*)&deceleration, sizeof(unsigned int));
  
  return dev->error.code;
}

int epos_profile_set_type(uint32_t dev, epos_profile_type_t type) 
{
  epos_device_write(dev, EPOS_PROFILE_INDEX_TYPE, 0,
    (unsigned char*)&type, sizeof(short));
  
  return dev->error.code;
}
