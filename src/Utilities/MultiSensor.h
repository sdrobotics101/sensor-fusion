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
        MultiSensor(boost::array<sensor, NUM_SENSORS> sensors);

        sensor getSensorByID(uint8_t ID);   //O(1)
        sensor getSensorByDescriptor(std::string descriptor);   //O(N)

        uint8_t numEnabledSensors();

        Eigen::Matrix<double, OUTPUT_DIM, 1> getOutput();
    private:
        boost::unordered_map<uint8_t, sensor> _sensors;
        uint8_t _enabledCount;
};

template<int OUTPUT_DIM, int NUM_SENSORS>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
MultiSensor(boost::array<sensor, NUM_SENSORS> sensors) {
    for (auto s : sensors) {
        _sensors[s.ID()] = s;
    }
    _enabledCount = NUM_SENSORS;
}


template<int OUTPUT_DIM, int NUM_SENSORS>
boost::shared_ptr<Sensor<OUTPUT_DIM>>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getSensorByID(uint8_t ID) {
    auto iterator = _sensors.find(ID);
    if (iterator == _sensors.end()) {
        return NULL;
    }
    return iterator->second;
}

template<int OUTPUT_DIM, int NUM_SENSORS>
boost::shared_ptr<Sensor<OUTPUT_DIM>>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getSensorByDescriptor(std::string descriptor) {
    for (auto s : _sensors) {
        if (s->second.descriptor() == descriptor) {
            return s->second;
        }
    }
    return NULL;
}

template<int OUTPUT_DIM, int NUM_SENSORS>
uint8_t
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
numEnabledSensors() {
    return _enabledCount;
}

template<int OUTPUT_DIM, int NUM_SENSORS>
Eigen::Matrix<double, OUTPUT_DIM, 1>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getOutput() {
    uint8_t enabledCount = 0;
    Eigen::Matrix<double, OUTPUT_DIM, 1> summedOutputs = Eigen::Matrix<double, OUTPUT_DIM, 1>::Zero();
    for (auto s : _sensors) {
        if (s->second.isEnabled()) {
            summedOutputs += s->second.getOutput();
            enabledCount++;
        }
    }
    summedOutputs /= enabledCount;
    _enabledCount = enabledCount;
    return summedOutputs;
}

#endif //MULTISENSOR_H
