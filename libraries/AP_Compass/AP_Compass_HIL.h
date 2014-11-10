/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include "Compass.h"

class AP_Compass_HIL : public AP_Compass_Backend
{
public:
    AP_Compass_HIL();
    AP_Compass_HIL(Compass &compass);
    bool        read(void);
    void        accumulate(void);
    void        setHIL(float roll, float pitch, float yaw);
    void        setHIL(const Vector3f &mag);
    const Vector3f&   getHIL() const;
    // detect the sensor
    static AP_Compass_Backend *detect(AP_Compass &compass);
    bool init(void);

    bool read_raw(void) {return true};
    bool re_initialise(void) {return true};
    bool read_register(uint8_t address, uint8_t *value) {return true};
    bool write_register(uint8_t address, uint8_t value) {return true};



private:
    Vector3f    _hil_mag;
    Vector3f    _Bearth;
    float		_last_declination;
    void        _setup_earth_field();
    uint8_t     _compass_instance;    
};

#endif
