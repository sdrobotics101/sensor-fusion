#ifndef DATACORE_H
#define DATACORE_H

#include <boost/shared_ptr.hpp>

#include "../Utilities/Sensor.h"

class DataCore {
    typedef boost::shared_ptr<Sensor<3>> IMUSensor;
    typedef boost::shared_ptr<Sensor<1>> PressureSensor;

    private:
        IMUSensor _accelerometer;
        IMUSensor _gyro;
        IMUSensor _magnetometer;
        PressureSensor _pressureSensor;
};

#endif //DATACORE_H
