#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <cstdint>

#include <eigen3/Eigen/Core>

template<int OUTPUT_DIM>
class Sensor {
    public:
        Sensor(uint8_t ID, std::string descriptor, bool isEnabled = true)
            : _ID(ID), _descriptor(descriptor), _isEnabled(isEnabled) {}

        uint8_t     ID()         const {return _ID;}
        std::string descriptor() const {return _descriptor;}
        bool        isEnabled()        {return _isEnabled;}

        void disable() {_isEnabled = false;}
        void enable()  {_isEnabled = true;}

        void setEnabled(bool enable) {_isEnabled = enable;}

        Eigen::Matrix<double, OUTPUT_DIM, 1> getOutput() = 0;
    private:
        const uint8_t     _ID;
        const std::string _descriptor;
        bool              _isEnabled;
};

#endif //SENSOR_H
