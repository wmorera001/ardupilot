// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  GPS driver backend class
 */
#ifndef __AP_INERTIALSENSOR_BACKEND_H__
#define __AP_INERTIALSENSOR_BACKEND_H__

#include <GCS_MAVLink.h>
#include <AP_InertialSensor.h>

class AP_InertialSensor_Backend
{
public:
  AP_InertialSensor_Backend(AP_InertialSensor & _imu, AP_InertialSensor::AP_InertialSensor_State & _state, AP_HAL::UARTDriver *_port);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}

    // The read() method is the only one needed in each driver. It
    // should return true when the backend has successfully received a
    // valid packet from the IMU.
    virtual bool read() = 0;

protected:
    AP_HAL::UARTDriver *port;                           ///< UART we are attached to
    AP_InertialSensor &imu;                             ///< access to frontend (for parameters)
    AP_InertialSensor::InertialSensor_State &state;     ///< public state for this instance

};

#endif // __AP_INERTIALSENSOR_BACKEND_H__
