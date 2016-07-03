#include <iostream>
#include <csignal>
#include <string>
#include <stdexcept>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/array.hpp>
#include <boost/program_options.hpp>

#include "DataCore.h"
#include "DataCoreConstants.h"
#include "DataCoreConfig.h"
#include "CalibrationModule.h"
#include "../Drivers/Dummy.h"
#include "../Utilities/MultiSensor.h"

namespace po = boost::program_options;

boost::scoped_ptr<DataCore> _dataCore;

void signalHandler(int) {
    _dataCore->stop();
}

int main(int argc, char** argv) {
    try {
        int samplesize;
        std::string calibratePath;
        std::string configFile;
        std::string generatePath;

        po::options_description generalOptions("Options");
        generalOptions.add_options()
            ("help,h",     "print help message")
            ("calibrate",  "enter calibration mode")
            ("samplesize", po::value<int>(&samplesize),            "calibration sample size")
            ("calpath",    po::value<std::string>(&calibratePath), "save calibration files to the provided directory")
            ("config",     po::value<std::string>(&configFile),    "the configuration file to use")
            ("generate",   po::value<std::string>(&generatePath),  "generate a default config file in the specified directory")
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

        DataCoreSettings<ACCL_COUNT, GYRO_COUNT, MAGN_COUNT, PRES_COUNT> settings;
        if (vm.count("config")) {
            settings.load(configFile);
            if (vm.count("samplesize")) {
                settings.calibrationSampleSize = samplesize;
            }
            if (vm.count("calpath")) {
                settings.calibrationPath = calibratePath;
            }
        } else if (vm.count("generate")) {
            settings.serverID = SERVER_ID;
            settings.clientID = CLIENT_ID;
            settings.calibrationSampleSize = SAMPLESIZE;
            settings.calibrationPath = CALIBRATEPATH;
            settings.save(generatePath+"/DataCore.json");
            return 0;
        } else {
            settings.serverID = SERVER_ID;
            settings.clientID = CLIENT_ID;
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

        if (settings.serverID < 0 || settings.serverID > 255) {
            throw std::runtime_error("Invalid server ID");
        }

        if (settings.clientID < 0 || settings.clientID > 255) {
            throw std::runtime_error("Invalid client ID");
        }

        boost::array<boost::shared_ptr<Sensor<3>>, ACCL_COUNT> accelerometers;
        for (int i = 0; i < ACCL_COUNT; i++) {
            accelerometers[i].reset(new Dummy<3>(i, "accelerometer"+std::to_string(i)));
            accelerometers[i]->setEnabled(settings.enabledAccl[i]);
        }

        boost::array<boost::shared_ptr<Sensor<3>>, GYRO_COUNT> gyros;
        for (int i = 0; i < GYRO_COUNT; i++) {
            gyros[i].reset(new Dummy<3>(i, "gyro"+std::to_string(i)));
            gyros[i]->setEnabled(settings.enabledGyro[i]);
        }

        boost::array<boost::shared_ptr<Sensor<3>>, MAGN_COUNT> magnetometers;
        for (int i = 0; i < MAGN_COUNT; i++) {
            magnetometers[i].reset(new Dummy<3>(i, "magnetometer"+std::to_string(i)));
            magnetometers[i]->setEnabled(settings.enabledMagn[i]);
        }

        boost::array<boost::shared_ptr<Sensor<1>>, PRES_COUNT> pressureSensors;
        for (int i = 0; i < PRES_COUNT; i++) {
            pressureSensors[i].reset(new Dummy<1>(i, "pressure"+std::to_string(i)));
            pressureSensors[i]->setEnabled(settings.enabledPres[i]);
        }

        //REMOVE THIS
        Eigen::Matrix<double, 3, 1> d;
        d << 50, 75, 100;
        boost::static_pointer_cast<Dummy<3>>(accelerometers[3])->setOutput(d);
        //END REMOVE THIS

        if (vm.count("calibrate")) {
            if (settings.calibrationSampleSize < 1) {
                throw std::runtime_error("Invalid calibration sample size");
            }
            CalibrationModule<ACCL_COUNT, GYRO_COUNT, MAGN_COUNT> calibrationModule(
                    &accelerometers,
                    &gyros,
                    &magnetometers,
                    settings.calibrationSampleSize,
                    settings.calibrationPath);
            calibrationModule.start();
            return 0;
        }

        boost::shared_ptr<Sensor<3>> accelerometer(new MultiSensor<3, ACCL_COUNT>(&accelerometers, 0, "multiaccl"));
        boost::shared_ptr<Sensor<3>> gyro(new MultiSensor<3, GYRO_COUNT>(&gyros, 0, "multigyro"));
        boost::shared_ptr<Sensor<3>> magnetometer(new MultiSensor<3, MAGN_COUNT>(&magnetometers, 0, "multimagn"));
        boost::shared_ptr<Sensor<1>> pressureSensor(new MultiSensor<1, PRES_COUNT>(&pressureSensors, 0, "multipressure"));

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
