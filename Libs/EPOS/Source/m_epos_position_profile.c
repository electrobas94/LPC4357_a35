/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <stdio.h>

#include <timer/timer.h>

#include "position_profile.h"

#include "gear.h"
#include "macros.h"

void epos_position_profile_init(epos_position_profile_t* profile, float
    target_value, float velocity, float acceleration, float deceleration,
    epos_profile_type_t type, int relative) {
  profile->target_value = target_value;
  profile->velocity = velocity;
  profile->acceleration = acceleration;
  profile->deceleration = deceleration;

  profile->type = type;
  profile->relative = relative;

  profile->start_value = 0.0;
  profile->start_time = 0.0;
}

int epos_position_profile_start(epos_node_t* node, epos_position_profile_t*
    profile) {
  int pos = epos_gear_from_angle(&node->gear, profile->target_value);
  unsigned int vel = abs(epos_gear_from_angular_velocity(&node->gear,
    profile->velocity));
  unsigned int acc = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->acceleration));
  unsigned int dec = abs(epos_gear_from_angular_acceleration(&node->gear,
    profile->deceleration));
  short control = (profile->relative) ?
    EPOS_POSITION_PROFILE_CONTROL_SET_RELATIVE :
    EPOS_POSITION_PROFILE_CONTROL_SET_ABSOLUTE;

  if (!epos_control_set_mode(&node->control, epos_control_profile_pos) &&
      !epos_position_profile_set_velocity(&node->dev, vel) &&
      !epos_profile_set_acceleration(&node->dev, acc) &&
      !epos_profile_set_deceleration(&node->dev, dec) &&
      !epos_profile_set_type(&node->dev, profile->type) &&
      !epos_control_start(&node->control) &&
      !epos_position_profile_set_target(&node->dev, pos)) {
    profile->start_value = epos_node_get_position(node);
    error_return(&node->dev.error);
  
    timer_start(&profile->start_time);
    epos_device_set_control(&node->dev, control);
    timer_correct(&profile->start_time);
  }

  return node->dev.error.code;
}

int epos_position_profile_stop(epos_node_t* node) {
  return epos_control_stop(&node->control);
}

epos_profile_value_t epos_position_profile_eval(const epos_position_profile_t*
    profile, double time) {
  epos_profile_value_t values;
  
  float s_0 = profile->start_value;
  float s_1 = (profile->relative) ? s_0+profile->target_value :
    profile->target_value;
  double t = time-profile->start_time;
  
  if (t > 0.0) {
    float s = s_1-s_0;
    float v = copysignf(fabs(profile->velocity), s);
    float a = copysignf(fabs(profile->acceleration), s);
    float d = copysignf(fabs(profile->deceleration), s);
    
    if (profile->type == epos_profile_sinusoidal) {
      float v_c = sqrt(4.0*fabs(s)/(M_PI/fabs(a)+M_PI/fabs(d)));
      v_c = copysignf(min(fabs(v), v_c), s);
      
      double t_a = 0.5*M_PI*v_c/a;
      double t_d = 0.5*M_PI*v_c/d;
      float s_a = 0.25*sqr(v_c)*M_PI/a;
      float s_d = 0.25*sqr(v_c)*M_PI/d;
      double t_c = (fabs(s) > fabs(s_a-s_d)) ? (s-s_a-s_d)/v_c : 0.0;
      float s_c = v_c*t_c;

      if (t <= t_a) {
        values.position = s_0+0.5*v_c*(t-0.5*v_c/a*sin(2.0*a/v_c*t));
        values.velocity = 0.5*v_c*(1.0-cos(2.0*a/v_c*t));
        values.acceleration = a*sin(2.0*a/v_c*t);
      }
      else if (t <= t_a+t_c) {
        values.position = s_0+s_a+v_c*(t-t_a);
        values.velocity = v_c;
        values.acceleration = 0.0;
      }
      else if (t <= t_a+t_c+t_d) {
        values.position = s_0+s_a+s_c+0.5*v_c*(t-t_a-t_c+0.5*v_c/d*
          sin(2.0*d/v_c*(t-t_a-t_c)));
        values.velocity = 0.5*v_c*(1.0+cos(2.0*d/v_c*(t-t_a-t_c)));
        values.acceleration = -d*sin(2.0*a/v_c*(t-t_a-t_c));
      }
      else {
        values.position = s_0+s;
        values.velocity = 0.0;
        values.acceleration = 0.0;
      }
    }
    else {
      float v_c = sqrt(2.0*fabs(s)/(1.0/fabs(a)+1.0/fabs(d)));
      v_c = copysignf(min(fabs(v), v_c), s);

      double t_a = v_c/a;
      double t_d = v_c/d;
      float s_a = 0.5*sqr(v_c)/a;
      float s_d = 0.5*sqr(v_c)/d;
      double t_c = (fabs(s) > fabs(s_a+s_d)) ? (s-s_a-s_d)/v_c : 0.0;
      float s_c = v_c*t_c;

      if (t <= t_a) {
        values.position = s_0+0.5*a*sqr(t);
        values.velocity = a*t;
        values.acceleration = a;
      }
      else if (t <= t_a+t_c) {
        values.position = s_0+s_a+v_c*(t-t_a);
        values.velocity = v_c;
        values.acceleration = 0.0;
      }
      else if (t <= t_a+t_c+t_d) {
        values.position = s_0+s_a+s_c+v_c*(t-t_a-t_c)-0.5*d*sqr(t-t_a-t_c);
        values.velocity = v_c-d*(t-t_a-t_c);
        values.acceleration = -d;
      }
      else {
        values.position = s_0+s;
        values.velocity = 0.0;
        values.acceleration = 0.0;
      }
    }
  }
  else {
    values.position = s_0;
    values.velocity = 0.0;
    values.acceleration = 0.0;
  }

  return values;
}

int epos_position_profile_set_target(epos_device_t* dev, int position) {
  epos_device_write(dev, EPOS_POSITION_PROFILE_INDEX_TARGET, 0,
    (unsigned char*)&position, sizeof(int));
  
  return dev->error.code;
}

int epos_position_profile_set_velocity(epos_device_t* dev, unsigned int
    velocity) {
  return epos_device_write(dev, EPOS_POSITION_PROFILE_INDEX_VELOCITY, 0,
    (unsigned char*)&velocity, sizeof(unsigned int));
  
  return dev->error.code;
}
