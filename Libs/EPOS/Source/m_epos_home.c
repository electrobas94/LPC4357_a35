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

#include "home.h"
#include "gear.h"

char epos_home_methods[] = {
  11,
   7,
  27,
  23,
   1,
   2,
  17,
  18,
  -1,
  -2,
  -3,
  -4,
  34,
  33,
};

void epos_home_init(epos_home_t* home, epos_home_method_t method, float
    current, float velocity, float acceleration, float position) {
  home->method = method;
  home->type = epos_profile_sinusoidal;

  home->current = current;
  home->switch_vel = velocity;
  home->zero_vel = velocity;
  home->acceleration = acceleration;

  home->offset = 0.0;
  home->position = position;
}

int epos_home_start(epos_node_t* node, const epos_home_t* home) {
  unsigned int switch_vel = abs(epos_gear_from_angular_velocity(&node->gear,
    home->switch_vel));
  unsigned int zero_vel = abs(epos_gear_from_angular_velocity(&node->gear,
    home->zero_vel));
  unsigned int acc = abs(epos_gear_from_angular_acceleration(&node->gear,
    home->acceleration));
  int offset = epos_gear_from_angle(&node->gear, home->offset);
  int pos = epos_gear_from_angle(&node->gear, home->position);

  if (!epos_control_set_mode(&node->control, epos_control_homing) &&
      !epos_home_set_method(&node->dev, home->method) &&
      !epos_home_set_current_threshold(&node->dev, home->current*1e3) &&
      !epos_home_set_switch_search_velocity(&node->dev, switch_vel) &&
      !epos_home_set_zero_search_velocity(&node->dev, zero_vel) &&
      !epos_home_set_acceleration(&node->dev, acc) &&
      !epos_home_set_offset(&node->dev, offset) &&
      !epos_home_set_position(&node->dev, pos) &&
      !epos_profile_set_type(&node->dev, home->type) &&
      !epos_device_set_control(&node->dev, EPOS_DEVICE_CONTROL_SHUTDOWN) &&
      !epos_control_start(&node->control))
    epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_START);

  return node->dev.error.code;
}

int epos_home_stop(epos_node_t* node) {
  if (!epos_device_set_control(&node->dev, EPOS_HOME_CONTROL_HALT))
    epos_control_stop(&node->control);
  
  return node->dev.error.code;
}

int epos_home_wait(epos_node_t* node, double timeout) {
  return epos_device_wait_status(&node->dev, EPOS_HOME_STATUS_REACHED,
    timeout);
}

int epos_home_set_method(epos_device_t* dev, epos_home_method_t method) {
  epos_device_write(dev, EPOS_HOME_INDEX_METHOD, 0,
    (unsigned char*)&epos_home_methods[method], 1);
  
  return dev->error.code;
}

int epos_home_set_current_threshold(epos_device_t* dev, short current) {
  epos_device_write(dev, EPOS_HOME_INDEX_CURRENT_THRESHOLD, 0,
    (unsigned char*)&current, sizeof(short));
  
  return dev->error.code;
}

int epos_home_set_switch_search_velocity(epos_device_t* dev, unsigned int
    velocity) {
  epos_device_write(dev, EPOS_HOME_INDEX_VELOCITIES,
    EPOS_HOME_SUBINDEX_SWITCH_SEARCH_VELOCITY, (unsigned char*)&velocity,
    sizeof(unsigned int));
  
  return dev->error.code;
}

int epos_home_set_zero_search_velocity(epos_device_t* dev, unsigned int
    velocity) {
  epos_device_write(dev, EPOS_HOME_INDEX_VELOCITIES,
    EPOS_HOME_SUBINDEX_ZERO_SEARCH_VELOCITY, (unsigned char*)&velocity,
    sizeof(unsigned int));
  
  return dev->error.code;
}

int epos_home_set_acceleration(epos_device_t* dev, unsigned int
    acceleration) {
  epos_device_write(dev, EPOS_HOME_INDEX_ACCELERATION, 0,
    (unsigned char*)&acceleration, sizeof(unsigned int));
  
  return dev->error.code;
}

int epos_home_set_offset(epos_device_t* dev, int offset) {
  epos_device_write(dev, EPOS_HOME_INDEX_OFFSET, 0,
    (unsigned char*)&offset, sizeof(int));
  
  return dev->error.code;
}

int epos_home_set_position(epos_device_t* dev, int position) {
  epos_device_write(dev, EPOS_HOME_INDEX_POSITION, 0,
    (unsigned char*)&position, sizeof(int));
  
  return dev->error.code;
}
