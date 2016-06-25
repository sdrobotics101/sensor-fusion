#ifndef MULTISENSOR_H
#define MULTISENSOR_H

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include "Sensor.h"

template<int OUTPUT_DIM, int NUM_SENSORS>
class MultiSensor : public Sensor<OUTPUT_DIM> {
    public:
        typedef boost::shared_ptr<Sensor<OUTPUT_DIM>> sensor;
        MultiSensor(boost::array<sensor, NUM_SENSORS>);

        sensor getSensorByID(uint8_t ID);   //O(1)
        sensor getSensorByDescriptor(std::string descriptor);   //O(N)

        uint8_t numEnabledSensors();
    private:
        boost::unordered_map<uint8_t, sensor> _sensors;
        uint8_t _enabledCount;
};

#endif //MULTISENSOR_H
