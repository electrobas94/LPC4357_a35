#include "m_epos_control.h"

char epos_control_modes[] = {
   6,
   3,
   1,
  -1,
  -2,
  -3,
  -4,
  -5,
  -6,
};

epos_control_mode_t epos_control_get_mode(epos_control_t* control) 
{
  unsigned char mode = 0;
  
  if (epos_device_read(control->dev, EPOS_CONTROL_INDEX_MODE_DISPLAY, 0,
      &mode, 1) > 0) {
    int i;
    for (i = 0; i < sizeof(epos_control_modes); ++i)
      if (epos_control_modes[i] == mode)
        return i;
  }
  else
    return -1;
    
  return mode;
}

int epos_control_set_mode(epos_control_t* control, epos_control_mode_t mode) 
{
  if (epos_device_write(control->dev, EPOS_CONTROL_INDEX_MODE, 0,
      (unsigned char*)&epos_control_modes[mode], 1) > 0)
    control->mode = mode;

  return control->dev->error.code;
}

/*
int epos_control_start(epos_control_t* control) 
{
  if (!epos_device_set_control(control->dev,
      EPOS_DEVICE_CONTROL_ENABLE_OPERATION))
    timer_sleep(EPOS_CONTROL_START_SLEEP);

  return control->dev->error.code;
}

int epos_control_stop(epos_control_t* control) 
{
  return epos_device_set_control(control->dev,
    EPOS_DEVICE_CONTROL_QUICK_STOP);
}
*/