#ifndef EPOS_CONTROL_H
#define EPOS_CONTROL_H

#include "m_epos_device.h"

//#define EPOS_CONTROL_START_SLEEP             0.01

#define EPOS_CONTROL_INDEX_MODE              0x6060
#define EPOS_CONTROL_INDEX_MODE_DISPLAY      0x6061

//Operation mode values
typedef enum 
	{
  epos_control_homing,        //!< Homing operating mode.
  epos_control_profile_vel,   //!< Profile velocity operating mode.
  epos_control_profile_pos,   //!< Profile position operating mode.
  epos_control_position,      //!< Position operating mode.
  epos_control_velocity,      //!< Velocity operating mode.
  epos_control_current,       //!< Current operating mode.
  epos_control_diagnostic,    //!< Diagnostic operating mode.
  epos_control_master_enc,    //!< Master encoder operating mode.
  epos_control_step_dir       //!< Step/direction operating mode.
} epos_control_mode_t;


epos_control_mode_t epos_control_get_mode(
  epos_control_t* control);

int epos_control_set_mode(
  epos_control_t* control,
  epos_control_mode_t mode);

int epos_control_start(
  epos_control_t* control);

int epos_control_stop(
  epos_control_t* control);

#endif
