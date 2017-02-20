#ifndef EPOS_PROFILE_H
#define EPOS_PROFILE_H

//#include "m_epos_epos.h"

#define EPOS_PROFILE_INDEX_ACCELERATION       0x6083
#define EPOS_PROFILE_INDEX_DECELERATION       0x6084
#define EPOS_PROFILE_INDEX_TYPE               0x6086

#define EPOS_PROFILE_STATUS_REACHED           0x0400

typedef enum 
{
  epos_profile_linear,            //!< Linear profile.
  epos_profile_sinusoidal,        //!< Sinusoidal profile.
} epos_profile_type_t;


typedef struct epos_profile_value_t 
{
  float position;                 //!< The profile position in [rad].
  float velocity;                 //!< The profile velocity in [rad/s].
  float acceleration;             //!< The profile acceleration in [rad/s^2].
} epos_profile_value_t;

int epos_profile_set_acceleration(
  uint32_t dev,
  unsigned int acceleration);

int epos_profile_set_deceleration(
  uint32_t dev,
  unsigned int deceleration);

int epos_profile_set_type(
  uint32_t dev,
  epos_profile_type_t type);

#endif
