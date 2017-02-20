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

#ifndef EPOS_SENSOR_H
#define EPOS_SENSOR_H

#include "m_epos_device.h"

/** \file sensor.h
  * \brief EPOS sensor functions
  */

/** \name Object Indexes
  * \brief Predefined EPOS sensor object indexes
  */
//@{
#define EPOS_SENSOR_INDEX_CONFIGURATION         0x2210
#define EPOS_SENSOR_SUBINDEX_PULSES             0x01
#define EPOS_SENSOR_SUBINDEX_TYPE               0x02
#define EPOS_SENSOR_SUBINDEX_POLARITY           0x04
#define EPOS_SENSOR_INDEX_POSITION              0x2020
//@}

/** \brief Predefined EPOS sensor error descriptions
  */
extern const char* epos_sensor_errors[];

/** \brief EPOS position sensor types
  */
typedef enum {
  epos_sensor_3chan,                //!< 3-channel incremental encoder.
  epos_sensor_2chan,                //!< 2-channel incremental encoder.
  epos_sensor_hall                  //!< Hall sensor.
} epos_sensor_type_t;

/** \brief EPOS position sensor polarity
  */
typedef enum {
  epos_sensor_normal,               //!< Normal polarity.
  epos_sensor_inverted              //!< Inverted polarity.
} epos_sensor_polarity_t;

/** \brief EPOS position sensor supervision
  */
typedef enum {
  epos_sensor_fully_supervised,     //!< Hardware and software supervised.
  epos_sensor_hardware_supervised,  //!< Hardware supervised.
  epos_sensor_software_supervised,  //!< Software supervised.
  epos_sensor_unsupervised          //!< Not supervised.
} epos_sensor_supervision_t;

/** \brief Structure defining an EPOS position sensor
  */
typedef struct epos_sensor_t {
  epos_device_t* dev;              //!< The EPOS device of the sensor.

  epos_sensor_type_t type;         //!< The position sensor type.
  epos_sensor_polarity_t polarity; //!< The position sensor polarity.
  int num_pulses;                  //!< The number of pulses per revolution.

  epos_sensor_supervision_t
    supervision;                   //!< The position sensor's supervision.
} epos_sensor_t;

/** \brief Initialize EPOS position sensor
  * \param[in] sensor The EPOS position sensor to be initialized.
  * \param[in] dev The EPOS device the position sensor is connected to.
  * \param[in] type The type of the EPOS position sensor to be initialized.
  * \param[in] polarity The polarity of the position sensor.
  * \param[in] num_pulses The sensor's number of pulses per revolution.
  * \param[in] supervision The sensor's supervision.
  */
void epos_sensor_init(
  epos_sensor_t* sensor,
  epos_device_t* dev,
  epos_sensor_type_t type,
  epos_sensor_polarity_t polarity,
  int num_pulses,
  epos_sensor_supervision_t supervision);

/** \brief Destroy EPOS position sensor
  * \param[in] sensor The EPOS position sensor to be destroyed.
  */
void epos_sensor_destroy(
  epos_sensor_t* sensor);

/** \brief Set EPOS sensor parameters
  * \param[in] sensor The EPOS sensor to set the parameters for.
  * \return The resulting device error code.
  */
int epos_sensor_setup(
  epos_sensor_t* sensor);

/** \brief Retrieve EPOS position sensor type
  * \param[in] sensor The EPOS position sensor to retrieve the type for.
  * \return The type of the specified EPOS position sensor. On error, the
  *   return value will be -1 and the error code set in sensor->dev->error.
  */
epos_sensor_type_t epos_sensor_get_type(
  epos_sensor_t* sensor);

/** \brief Set EPOS position sensor type
  * \param[in] sensor The EPOS position sensor to set the type for.
  * \param[in] type The type of the specified EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_type(
  epos_sensor_t* sensor,
  epos_sensor_type_t type);

/** \brief Retrieve EPOS position sensor polarity
  * \param[in] sensor The EPOS position sensor to retrieve the polarity for.
  * \return The polarity of the specified EPOS position sensor. On error, the
  *   return value will be -1 and the error code set in sensor->dev->error.
  */
epos_sensor_polarity_t epos_sensor_get_polarity(
  epos_sensor_t* sensor);

/** \brief Set EPOS position sensor polarity
  * \param[in] sensor The EPOS position sensor to set the polarity for.
  * \param[in] polarity The polarity of the specified EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_polarity(
  epos_sensor_t* sensor,
  epos_sensor_polarity_t polarity);

/** \brief Retrieve an EPOS position sensor's number of pulses per revolution
  * \param[in] sensor The EPOS position sensor to retrieve the number of
  *   revolutions for.
  * \return The number of pulses of the specified EPOS position sensor.
  *   On error, the return value will be zero and the error code set in
  *   sensor->dev->error.
  */
int epos_sensor_get_pulses(
  epos_sensor_t* sensor);

/** \brief Set an EPOS position sensor's number of pulses per revolution
  * \param[in] sensor The EPOS position sensor to set the number of
  *   revolutions for.
  * \param[in] num_pulses The number of pulses per revolution of the specified
  *   EPOS position sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_pulses(
  epos_sensor_t* sensor,
  int num_pulses);

/** \brief Retrieve an EPOS position sensor's supervision
  * \param[in] sensor The EPOS position sensor to retrieve the supervision for.
  * \return The number of pulses of the specified EPOS position sensor.
  *   On error, the return value will be -1 and the error code set in
  *   sensor->dev->error.
  */
epos_sensor_supervision_t epos_sensor_get_supervision(
  epos_sensor_t* sensor);

/** \brief Set an EPOS position sensor's supervision
  * \param[in] sensor The EPOS position sensor to set the supervision for.
  * \param[in] supervision The supervision of the specified EPOS position
*     sensor.
  * \return The resulting device error code.
  */
int epos_sensor_set_supervision(
  epos_sensor_t* sensor,
  epos_sensor_supervision_t supervision);

/** \brief Retrieve an EPOS position sensor's position
  * \param[in] sensor The EPOS position sensor to retrieve the position for.
  * \return The position of the specified EPOS position sensor in [steps].
  *   On error, the return value will be zero and the error code set in
  *   sensor->dev->error
  */
short epos_sensor_get_position(
  epos_sensor_t* sensor);

#endif
