#ifndef CALIBRATION_MODULE_H
#define CALIBRATION_MODULE_H

#include <iostream>
#include <cstdio>

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>

#include "../Utilities/Sensor.h"

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define PROMPT_STATIC 0
#define PROMPT_DYNAMIC 1
#define PROMPT_FINISHED 2

#define SAMPLE_DELAY 10 //milliseconds

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
class CalibrationModule {
    public:
        typedef boost::shared_ptr<Sensor<3>> IMUSensor;
        typedef Eigen::Matrix<double, 3, 1> IMUData;
        CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                          boost::array<IMUSensor, NUM_GYRO>* gyros,
                          boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                          int numSamples);
        ~CalibrationModule() {}

        void start();

    private:
        void prompt(int which);
        void staticCalibrate();
        void dynamicCalibrate();

        boost::array<IMUSensor, NUM_ACCL>* _accelerometers;
        boost::array<IMUSensor, NUM_GYRO>* _gyros;
        boost::array<IMUSensor, NUM_MAGN>* _magnetometers;

        int _numSamples;

        boost::array<IMUData, NUM_ACCL> _acclTotals;
        boost::array<IMUData, NUM_GYRO> _gyroTotals;
        boost::array<IMUData, NUM_MAGN> _magnTotals;
};

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                  boost::array<IMUSensor, NUM_GYRO>* gyros,
                  boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                  int numSamples) :
                  _accelerometers(accelerometers),
                  _gyros(gyros),
                  _magnetometers(magnetometers),
                  _numSamples(numSamples)
{
    for (int i = 0; i < NUM_ACCL; i++) {
        _acclTotals[i] << 0,0,0;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        _gyroTotals[i] << 0,0,0;
    }
    for (int i = 0; i < NUM_MAGN; i++) {
        _magnTotals[i] << 0,0,0;
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
    prompt(PROMPT_FINISHED);
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
prompt(int which) {
    switch (which) {
        case PROMPT_STATIC:
            std::cout << "Calibrating " << NUM_ACCL << " accelerometers, " << NUM_GYRO << " gyros, " << NUM_MAGN << " magnetometers." << std::endl;
            std::cout << "Hold robot steady. Press enter to continue.";
            getchar();
            return;
        case PROMPT_DYNAMIC:
            std::cout << "Static calibration complete. Press enter to continue.";
            getchar();
            return;
        case PROMPT_FINISHED:
            std::cout << "Dynamic calibration complete. Exiting." << std::endl;
            return;
    }
}

//TODO write to file
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

    //divide by the number of samples to get an average and write to file
    for (int i = 0; i < NUM_ACCL; i++) {
        _acclTotals[i] /= _numSamples;
        std::cout << (*_accelerometers)[i]->descriptor() << " " << _acclTotals[i] << std::endl;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        _gyroTotals[i] /= _numSamples;
        std::cout << (*_gyros)[i]->descriptor() << " " << _gyroTotals[i] << std::endl;
    }
}

//TODO
template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
dynamicCalibrate() {
    return;
}

#endif //CALIBRATION_MODULE_H
