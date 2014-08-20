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
  AP_InertialSensor_Backend(AP_InertialSensor & _imu, AP_InertialSensor::AP_InertialSensor_State & _state, AP_HAL::SPIDeviceDriver *_port);

    // we declare a virtual destructor so that drivers can
    // override with a custom destructor if need be.
    virtual ~AP_InertialSensor_Backend(void) {}

    /// Perform startup initialisation.
    ///
    /// Called to initialise the state of the IMU.
    ///
    /// For COLD_START, implementations using real sensors can assume
    /// that the airframe is stationary and nominally oriented.
    ///
    /// For WARM_START, no assumptions should be made about the
    /// orientation or motion of the airframe.  Calibration should be
    /// as for the previous COLD_START call.
    ///
    /// @param style  The initialisation startup style.
    ///
    virtual void init( Start_style style,
                       Sample_rate sample_rate);

    // sensor specific init to be overwritten by descendant classes
    virtual uint16_t        _init_sensor( Sample_rate sample_rate ) = 0;


    /// Perform cold startup initialisation for just the accelerometers.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work.
    ///
    virtual void init_accel();

    /// Perform cold-start initialisation for just the gyros.
    ///
    /// @note This should not be called unless ::init has previously
    ///       been called, as ::init may perform other work
    ///
    virtual void init_gyro(void);

    /* Update the sensor data, so that getters are nonblocking.
     * Returns a bool of whether data was updated or not.
     */
    virtual bool update() = 0;

    /* get_delta_time returns the time period in seconds
     * overwhich the sensor data was collected
     */
    virtual float get_delta_time() const = 0;

    // return the maximum gyro drift rate in radians/s/s. This
    // depends on what gyro chips are being used
    virtual float get_gyro_drift_rate(void) = 0;

    // wait for a sample to be available, with timeout in milliseconds
    virtual bool wait_for_sample(uint16_t timeout_ms) = 0;

protected:
    AP_HAL::SPIDeviceDriver *port;                          ///< SPI we are attached to
    AP_InertialSensor &imu;                                 ///< access to frontend (for parameters)
    AP_InertialSensor::InertialSensor_State &state;         ///< public state for this instance

    // Most recent accelerometer reading obtained by ::update
    Vector3f _accel[INS_MAX_INSTANCES];

    // previous accelerometer reading obtained by ::update
    Vector3f _previous_accel[INS_MAX_INSTANCES];

    // Most recent gyro reading obtained by ::update
    Vector3f _gyro[INS_MAX_INSTANCES];

    // product id
    AP_Int16 _product_id;

    // accelerometer scaling and offsets
    AP_Vector3f             _accel_scale[INS_MAX_INSTANCES];
    AP_Vector3f             _accel_offset[INS_MAX_INSTANCES];
    AP_Vector3f             _gyro_offset[INS_MAX_INSTANCES];

    // filtering frequency (0 means default)
    AP_Int8                 _mpu6000_filter;

    // board orientation from AHRS
    enum Rotation     _board_orientation;


};

#endif // __AP_INERTIALSENSOR_BACKEND_H__
