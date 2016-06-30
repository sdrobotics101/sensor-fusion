#ifndef CALIBRATION_MODULE_H
#define CALIBRATION_MODULE_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>

#include "../Utilities/Sensor.h"
#include "../Utilities/IMUSensorConfig.h"

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define PROMPT_STATIC 0
#define PROMPT_DYNAMIC 1
#define PROMPT_SUMMARIZE 2
#define PROMPT_WRITE 3

#define SAMPLE_DELAY 10 //milliseconds

//TODO magnetometer
template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
class CalibrationModule {
    public:
        typedef boost::shared_ptr<Sensor<3>> IMUSensor;
        typedef Eigen::Matrix<double, 3, 1> IMUData;
        CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                          boost::array<IMUSensor, NUM_GYRO>* gyros,
                          boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                          int numSamples,
                          std::string path);
        ~CalibrationModule() {}

        void start();

    private:
        void prompt(int which);
        void summarize();
        void writeFiles();
        void staticCalibrate();
        void dynamicCalibrate();

        boost::array<IMUSensor, NUM_ACCL>* _accelerometers;
        boost::array<IMUSensor, NUM_GYRO>* _gyros;
        boost::array<IMUSensor, NUM_MAGN>* _magnetometers;

        int _numSamples;
        std::string _path;

        boost::array<IMUData, NUM_ACCL> _acclTotals;
        boost::array<IMUData, NUM_GYRO> _gyroTotals;
};

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                  boost::array<IMUSensor, NUM_GYRO>* gyros,
                  boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                  int numSamples,
                  std::string path) :
                  _accelerometers(accelerometers),
                  _gyros(gyros),
                  _magnetometers(magnetometers),
                  _numSamples(numSamples),
                  _path(path)
{
    for (int i = 0; i < NUM_ACCL; i++) {
        _acclTotals[i] << 0,0,0;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        _gyroTotals[i] << 0,0,0;
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
start() {
    prompt(PROMPT_STATIC);
    staticCalibrate();
    prompt(PROMPT_DYNAMIC);
    dynamicCalibrate();
    prompt(PROMPT_SUMMARIZE);
    prompt(PROMPT_WRITE);
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
prompt(int which) {
    switch (which) {
        case PROMPT_STATIC:
            std::cout << "Calibrating " << NUM_ACCL << " accelerometers, " << NUM_GYRO << " gyros, " << NUM_MAGN << " magnetometers." << std::endl;
            std::cout << "Hold robot steady. Press enter to start static calibration.";
            getchar();
            std::cout << "Starting static calibration... ";
            return;
        case PROMPT_DYNAMIC:
            std::cout << "done" << std::endl;
            std::cout << "Press enter to start dynamic calibration.";
            getchar();
            std::cout << "Starting dynamic calibration... ";
            return;
        case PROMPT_SUMMARIZE:
            std::cout << "done" << std::endl;
            std::cout << "Would you like to see a calibration summary? (y/n): ";
            while (1) {
                char c;
                std::cin >> c;
                if (c == 'y') {
                    summarize();
                    return;
                } else if (c == 'n') {
                    return;
                } else {
                    std::cout << "Please enter y or n: ";
                }
            }
            return;
        case PROMPT_WRITE:
            std::cout << "Would you like to write to file? (y/n): ";
            while (1) {
                char c;
                std::cin >> c;
                if (c == 'y') {
                    std::cout << "Writing files...";
                    writeFiles();
                    std::cout << " done" << std::endl;
                    std::cout << "Exiting" << std::endl;
                    return;
                } else if (c == 'n') {
                    std::cout << "Exiting" << std::endl;
                    return;
                } else {
                    std::cout << "Please enter y or n: ";
                }
            }
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
summarize() {
    std::cout << "Calibration Summary" << std::endl;
    std::cout << "-------------------" << std::endl;
    std::cout << "ACCELEROMETERS" << std::endl;
    for (int i = 0; i < NUM_ACCL; i++) {
        std::cout << "    " << (*_accelerometers)[i]->descriptor() << ":" << std::endl;
        std::cout << "        " << "X: " << _acclTotals[i](XAXIS,0) << std::endl;
        std::cout << "        " << "Y: " << _acclTotals[i](YAXIS,0) << std::endl;
        std::cout << "        " << "Z: " << _acclTotals[i](ZAXIS,0) << std::endl;
    }
    std::cout << "GYROS" << std::endl;
    for (int i = 0; i < NUM_GYRO; i++) {
        std::cout << "    " << (*_gyros)[i]->descriptor() << ":" << std::endl;
        std::cout << "        " << "X: " << _gyroTotals[i](XAXIS,0) << std::endl;
        std::cout << "        " << "Y: " << _gyroTotals[i](YAXIS,0) << std::endl;
        std::cout << "        " << "Z: " << _gyroTotals[i](ZAXIS,0) << std::endl;
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
writeFiles() {
    for (int i = 0; i < NUM_ACCL; i++) {
        IMUSensorSettings settings;
        settings.bias[XAXIS] = _acclTotals[i](XAXIS,0);
        settings.bias[YAXIS] = _acclTotals[i](YAXIS,0);
        settings.bias[ZAXIS] = _acclTotals[i](ZAXIS,0);
        settings.save(_path+"/accelerometers/"+(*_accelerometers)[i]->descriptor());
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        IMUSensorSettings settings;
        settings.bias[XAXIS] = _gyroTotals[i](XAXIS,0);
        settings.bias[YAXIS] = _gyroTotals[i](YAXIS,0);
        settings.bias[ZAXIS] = _gyroTotals[i](ZAXIS,0);
        settings.save(_path+"/gyros/"+(*_gyros)[i]->descriptor());
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
staticCalibrate() {
    //sample sensor data, adding each sample together
    for (int i = 0; i < _numSamples; i++) {
        for (int j = 0; j < NUM_ACCL; j++) {
            _acclTotals[j] += (*_accelerometers)[j]->getOutput();
        }
        for (int j = 0; j < NUM_GYRO; j++) {
            _gyroTotals[j] += (*_gyros)[j]->getOutput();
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(SAMPLE_DELAY));
    }

    //divide by the number of samples to get an average
    for (int i = 0; i < NUM_ACCL; i++) {
        _acclTotals[i] /= _numSamples;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        _gyroTotals[i] /= _numSamples;
    }
}

//TODO
template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
dynamicCalibrate() {
    //create a dsm client
    //send data over the network for some period
    //calculate calibration offboard
    return;
}

#endif //CALIBRATION_MODULE_H
