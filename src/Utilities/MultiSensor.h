#ifndef MULTISENSOR_H
#define MULTISENSOR_H

#include <string>
#include <atomic>

#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>

#include "Sensor.h"

template<int OUTPUT_DIM, int NUM_SENSORS>
class MultiSensor : public Sensor<OUTPUT_DIM> {
    public:
        typedef boost::shared_ptr<Sensor<OUTPUT_DIM>> sensor;

        MultiSensor(boost::array<sensor, NUM_SENSORS>* sensors,
                    uint8_t ID,
                    std::string descriptor,
                    bool isEnabled = true);

        sensor getSensorByID(uint8_t ID);   //O(1)
        sensor getSensorByDescriptor(const std::string descriptor);   //O(N)

        uint8_t numEnabledSensors();

        Eigen::Matrix<double, OUTPUT_DIM, 1> getOutput();
    private:
        boost::unordered_map<uint8_t, sensor> _sensors;
        std::atomic<uint8_t> _enabledCount;
};

template<int OUTPUT_DIM, int NUM_SENSORS>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
MultiSensor(boost::array<sensor, NUM_SENSORS>* sensors,
            uint8_t ID,
            std::string descriptor,
            bool isEnabled) :
            Sensor<OUTPUT_DIM>(ID,
                               descriptor,
                               isEnabled)
{
    int enabledCount = 0;
    for (auto s : *sensors) {
        if (!s) {
            continue;
        }
        _sensors.insert(std::make_pair(s->ID(), s));
        if (s->isEnabled()) {
            enabledCount++;
        }
    }
    _enabledCount = enabledCount;
}


template<int OUTPUT_DIM, int NUM_SENSORS>
boost::shared_ptr<Sensor<OUTPUT_DIM>>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getSensorByID(uint8_t ID) {
    auto iterator = _sensors.find(ID);
    if (iterator == _sensors.end()) {
        return nullptr;
    }
    return iterator->second;
}

template<int OUTPUT_DIM, int NUM_SENSORS>
boost::shared_ptr<Sensor<OUTPUT_DIM>>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getSensorByDescriptor(const std::string descriptor) {
    for (auto s : _sensors) {
        if (s.second->descriptor() == descriptor) {
            return s.second;
        }
    }
    return nullptr;
}

template<int OUTPUT_DIM, int NUM_SENSORS>
uint8_t
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
numEnabledSensors() {
    return _enabledCount.load();
}

template<int OUTPUT_DIM, int NUM_SENSORS>
Eigen::Matrix<double, OUTPUT_DIM, 1>
MultiSensor<OUTPUT_DIM, NUM_SENSORS>::
getOutput() {
    if (!this->isEnabled()) {
        return Eigen::MatrixXd::Zero(OUTPUT_DIM, 1);
    }
    Eigen::Matrix<double, OUTPUT_DIM, 1> summedOutputs;
    uint8_t enabledCount = 0;
    for (auto s : _sensors) {
        if (s.second->isEnabled()) {
            summedOutputs += s.second->getOutput();
            enabledCount++;
        }
    }
    _enabledCount = enabledCount;
    return (summedOutputs / _enabledCount.load());
}

#endif //MULTISENSOR_H
