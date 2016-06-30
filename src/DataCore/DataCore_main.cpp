#include <iostream>
#include <csignal>
#include <string>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/array.hpp>
#include <boost/program_options.hpp>

#include "DataCore.h"
#include "DataCoreConfig.h"
#include "CalibrationModule.h"
#include "../Drivers/Dummy.h"
#include "../Utilities/MultiSensor.h"

#define SERVER_ID 0
#define CLIENT_ID 0

#define NUM_ACCL 9
#define NUM_GYRO 4
#define NUM_MAGN 9
#define NUM_PRES 2

#define SAMPLESIZE 10
#define CALIBRATEPATH "/Users/rahulsalvi/Desktop"

namespace po = boost::program_options;

boost::scoped_ptr<DataCore> _dataCore;

void signalHandler(int signum) {
    _dataCore->stop();
}

//TODO use program options for calibration mode and config file locations
int main(int argc, char** argv) {
    try {
        int samplesize;
        std::string calibratePath;
        std::string configFile;

        po::options_description generalOptions("Options");
        generalOptions.add_options()
            ("help,h", "print help message")
            ("calibrate", "enter calibration mode")
            ("samplesize", po::value<int>(&samplesize), "calibration sample size")
            ("calpath", po::value<std::string>(&calibratePath), "save calibration files to the provided directory")
            ("config", po::value<std::string>(&configFile), "the configuration file to use")
            ;

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                  options(generalOptions).
                  run(),
                  vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << generalOptions;
            return 0;
        }

        DatacoreSettings<NUM_ACCL, NUM_GYRO, NUM_MAGN, NUM_PRES> settings;
        if (vm.count("config")) {
            settings.load(configFile);
            if (vm.count("samplesize")) {
                settings.calibrationSampleSize = samplesize;
            }
            if (vm.count("calpath")) {
                settings.calibrationPath = calibratePath;
            }
        } else {
            settings.serverID = SERVER_ID;
            settings.clientID = CLIENT_ID;
            for (int i = 0; i < NUM_ACCL; i++) {
                settings.enabledAccl[i] = true;
            }
            for (int i = 0; i < NUM_GYRO; i++) {
                settings.enabledGyro[i] = true;
            }
            for (int i = 0; i < NUM_MAGN; i++) {
                settings.enabledMagn[i] = true;
            }
            for (int i = 0; i < NUM_PRES; i++) {
                settings.enabledPres[i] = true;
            }
            if (vm.count("calibrate")) {
                if (vm.count("samplesize")) {
                    settings.calibrationSampleSize = samplesize;
                } else {
                    settings.calibrationSampleSize = SAMPLESIZE;
                }
                if (vm.count("calpath")) {
                    settings.calibrationPath = calibratePath;
                } else {
                    settings.calibrationPath = CALIBRATEPATH;
                }
            }
        }

        boost::array<boost::shared_ptr<Sensor<3>>, NUM_ACCL> accelerometers;
        for (int i = 0; i < NUM_ACCL; i++) {
            accelerometers[i].reset(new Dummy<3>(i, "accelerometer"+std::to_string(i)));
            accelerometers[i]->setEnabled(settings.enabledAccl[i]);
        }

        boost::array<boost::shared_ptr<Sensor<3>>, NUM_GYRO> gyros;
        for (int i = 0; i < NUM_GYRO; i++) {
            gyros[i].reset(new Dummy<3>(i, "gyro"+std::to_string(i)));
            gyros[i]->setEnabled(settings.enabledGyro[i]);
        }

        boost::array<boost::shared_ptr<Sensor<3>>, NUM_MAGN> magnetometers;
        for (int i = 0; i < NUM_MAGN; i++) {
            magnetometers[i].reset(new Dummy<3>(i, "magnetometer"+std::to_string(i)));
            magnetometers[i]->setEnabled(settings.enabledMagn[i]);
        }

        boost::array<boost::shared_ptr<Sensor<1>>, NUM_PRES> pressureSensors;
        for (int i = 0; i < NUM_PRES; i++) {
            pressureSensors[i].reset(new Dummy<1>(i, "pressure"+std::to_string(i)));
            pressureSensors[i]->setEnabled(settings.enabledPres[i]);
        }

        //REMOVE THIS
        Eigen::Matrix<double, 3, 1> d;
        d << 50, 75, 100;
        boost::static_pointer_cast<Dummy<3>>(accelerometers[3])->setOutput(d);
        //END REMOVE THIS

        if (vm.count("calibrate")) {
            CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN> calibrationModule(
                    &accelerometers,
                    &gyros,
                    &magnetometers,
                    settings.calibrationSampleSize,
                    settings.calibrationPath);
            calibrationModule.start();
            return 0;
        }

        boost::shared_ptr<Sensor<3>> accelerometer(new MultiSensor<3, NUM_ACCL>(&accelerometers, 0, "multiaccl"));
        boost::shared_ptr<Sensor<3>> gyro(new MultiSensor<3, NUM_GYRO>(&gyros, 0, "multigyro"));
        boost::shared_ptr<Sensor<3>> magnetometer(new MultiSensor<3, NUM_MAGN>(&magnetometers, 0, "multimagn"));
        boost::shared_ptr<Sensor<1>> pressureSensor(new MultiSensor<1, NUM_PRES>(&pressureSensors, 0, "multipressure"));

        //REMOVE THIS
        Eigen::Matrix<double, 1, 1> pressureData;
        pressureData(0,0) = 100;
        boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByID(0)))->setOutput(pressureData);
        pressureData(0,0) = 50;
        boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByDescriptor("pressure1")))->setOutput(pressureData);
        //END REMOVE THIS

        _dataCore.reset(new DataCore(accelerometer,
                                     gyro,
                                     magnetometer,
                                     pressureSensor,
                                     settings.serverID,
                                     settings.clientID));
        std::signal(SIGINT, signalHandler);
        _dataCore->start();
        return 0;
    } catch (std::exception& e) {
        std::cout << "Error: ";
        std::cout << e.what() << std::endl;
        return 1;
    }
}
