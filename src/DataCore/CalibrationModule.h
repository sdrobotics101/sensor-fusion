#ifndef CALIBRATION_MODULE_H
#define CALIBRATION_MODULE_H

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <curses.h>

#include <boost/array.hpp>
#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>

#include "DataCoreConstants.h"
#include "../Dependencies/DistributedSharedMemory/src/Client/DSMClient.h"
#include "../Utilities/UniversalConstants.h"
#include "../Utilities/Sensor.h"
#include "../Utilities/MatrixConfig.h"

#define PROMPT_START 0
#define PROMPT_STATIC 1
#define PROMPT_DYNAMIC 2
#define PROMPT_SUMMARIZE 3
#define PROMPT_WRITE 4
#define PROMPT_EXIT 5

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
class CalibrationModule {
    public:
        typedef boost::shared_ptr<Sensor<3>> IMUSensor;
        typedef Eigen::Matrix<double, 3, 1> IMUData;
        CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                          boost::array<IMUSensor, NUM_GYRO>* gyros,
                          boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                          int numStaticSamples,
                          std::string path,
                          int serverID,
                          int clientID);
        ~CalibrationModule() {}

        void start();

    private:
        bool getYN();
        void prompt(int which);
        void summarize();
        void writeFiles();
        void staticCalibrate();
        void dynamicCalibrate();

        boost::array<IMUSensor, NUM_ACCL>* _accelerometers;
        boost::array<IMUSensor, NUM_GYRO>* _gyros;
        boost::array<IMUSensor, NUM_MAGN>* _magnetometers;

        int _numStaticSamples;
        int _numDynamicSamples;
        std::string _path;
        int _serverID;
        int _clientID;

        bool _didStaticCalibration;
        bool _didDynamicCalibration;

        boost::array<IMUData, NUM_ACCL> _acclTotals;
        boost::array<IMUData, NUM_GYRO> _gyroTotals;
};

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
CalibrationModule(boost::array<IMUSensor, NUM_ACCL>* accelerometers,
                  boost::array<IMUSensor, NUM_GYRO>* gyros,
                  boost::array<IMUSensor, NUM_MAGN>* magnetometers,
                  int numStaticSamples,
                  std::string path,
                  int serverID,
                  int clientID) :
                  _accelerometers(accelerometers),
                  _gyros(gyros),
                  _magnetometers(magnetometers),
                  _numStaticSamples(numStaticSamples),
                  _numDynamicSamples(0),
                  _path(path),
                  _serverID(serverID),
                  _clientID(clientID),
                  _didStaticCalibration(false),
                  _didDynamicCalibration(false)
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
    prompt(PROMPT_START);
    prompt(PROMPT_STATIC);
    prompt(PROMPT_DYNAMIC);
    prompt(PROMPT_SUMMARIZE);
    prompt(PROMPT_WRITE);
    prompt(PROMPT_EXIT);
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
bool
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
getYN() {
    while (1) {
        char c;
        std::cin >> c;
        if (c == 'y') {
            return true;
        } else if (c == 'n') {
            return false;
        } else {
            std::cout << "Please enter y or n: ";
        }
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
prompt(int which) {
    switch (which) {
        case PROMPT_START:
            std::cout << "Calibrating " << NUM_ACCL << " accelerometers, " << NUM_GYRO << " gyros, " << NUM_MAGN << " magnetometers." << std::endl;
            return;
        case PROMPT_STATIC:
            std::cout << "Execute static calibration? (y/n): ";
            if (getYN()) {
                std::cout << "Hold robot steady. Press enter to start static calibration.";
                std::cin.ignore(2);
                std::cout << "Starting static calibration... ";
                staticCalibrate();
                std::cout << "done." << std::endl;
                _didStaticCalibration = true;
            }
            return;
        case PROMPT_DYNAMIC:
            std::cout << "Execute dynamic calibration? (y/n): ";
            if (getYN()) {
                std::cout << "Prepare to move robot. Press enter to start dynamic calibration.";
                std::cin.ignore(2);
                std::cout << "Starting dynamic calibration... ";
                dynamicCalibrate();
                std::cout << "Starting dynamic calibration... done." << std::endl;
                _didDynamicCalibration = true;
            }
            return;
        case PROMPT_SUMMARIZE:
            if (_didStaticCalibration || _didDynamicCalibration) {
                std::cout << "Would you like to see a calibration summary? (y/n): ";
                if (getYN()) {
                    summarize();
                }
            }
            return;
        case PROMPT_WRITE:
            if (_didStaticCalibration) {
                std::cout << "Would you like to write to file? (y/n): ";
                if (getYN()) {
                    std::cout << "Writing files...";
                    writeFiles();
                    std::cout << " done." << std::endl;
                }
            }
            return;
        case PROMPT_EXIT:
            std::cout << "Exiting." << std::endl;
            return;
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
summarize() {
    std::cout << "Calibration Summary" << std::endl;
    std::cout << "-------------------" << std::endl;
    if (_didStaticCalibration) {
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
    if (_didDynamicCalibration) {
        std::cout << "MAGNETOMETERS" << std::endl;
        std::cout << "    total samples: " << _numDynamicSamples << std::endl;
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
writeFiles() {
    for (int i = 0; i < NUM_ACCL; i++) {
        MatrixSettings<3,1> settings;
        settings.name = (*_accelerometers)[i]->descriptor();
        settings.values[XAXIS][0] = _acclTotals[i](XAXIS,0);
        settings.values[YAXIS][0] = _acclTotals[i](YAXIS,0);
        settings.values[ZAXIS][0] = _acclTotals[i](ZAXIS,0);
        settings.save(_path+"/accelerometers/"+(*_accelerometers)[i]->descriptor()+".json");
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        MatrixSettings<3,1> settings;
        settings.name = (*_gyros)[i]->descriptor();
        settings.values[XAXIS][0] = _gyroTotals[i](XAXIS,0);
        settings.values[YAXIS][0] = _gyroTotals[i](YAXIS,0);
        settings.values[ZAXIS][0] = _gyroTotals[i](ZAXIS,0);
        settings.save(_path+"/gyros/"+(*_gyros)[i]->descriptor()+".json");
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
staticCalibrate() {
    //sample sensor data, adding each sample together
    for (int i = 0; i < _numStaticSamples; i++) {
        for (int j = 0; j < NUM_ACCL; j++) {
            _acclTotals[j] += (*_accelerometers)[j]->getOutput();
        }
        for (int j = 0; j < NUM_GYRO; j++) {
            _gyroTotals[j] += (*_gyros)[j]->getOutput();
        }
        boost::this_thread::sleep_for(boost::chrono::milliseconds(CALIBRATION_SAMPLE_DELAY));
    }

    //divide by the number of samples to get an average
    for (int i = 0; i < NUM_ACCL; i++) {
        _acclTotals[i] /= _numStaticSamples;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        _gyroTotals[i] /= _numStaticSamples;
    }
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN>
void
CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN>::
dynamicCalibrate() {
    struct MagnetometerData {
        double data[3][NUM_MAGN];
    } magnetometerData;
    dsm::Client client(_serverID, _clientID);
    dsm::LocalBufferKey key = dsm::Client::createLocalKey("magndata");
    client.registerLocalBuffer(key, sizeof(MagnetometerData), false);

    initscr();
    noecho();
    cbreak();
    timeout(0);
    addstr("Press q to stop.");
    int c = 0;
    while (c != 'q') {
        c = getch();
        _numDynamicSamples++;

        for (int i = 0; i < NUM_MAGN; i++) {
            Eigen::Vector3d out = (*_magnetometers)[i]->getOutput();
            magnetometerData.data[XAXIS][i] = out(XAXIS);
            magnetometerData.data[YAXIS][i] = out(YAXIS);
            magnetometerData.data[ZAXIS][i] = out(ZAXIS);
        }
        client.setLocalBufferContents(key, &magnetometerData);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(CALIBRATION_SAMPLE_DELAY));
    }
    endwin();
    return;
}

#endif //CALIBRATION_MODULE_H
