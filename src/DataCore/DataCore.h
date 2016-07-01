#ifndef DATACORE_H
#define DATACORE_H

#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>

#include <boost/shared_ptr.hpp>

#include <eigen3/Eigen/Core>

#include "../Utilities/Sensor.h"
#include "../Dependencies/DistributedSharedMemory/src/Client/DSMClient.h"

#ifdef LOGGING_ENABLED
#include "../Dependencies/Log/src/Log.h"
#else
#include "../Dependencies/Log/src/LogDisabled.h"
#endif

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define ACCL 0
#define GYRO 1
#define MAGN 2
#define PRES 3

#define DATACORE_DELAY 10 //milliseconds

class DataCore {
    public:
        typedef boost::shared_ptr<Sensor<3>> IMUSensor;
        typedef boost::shared_ptr<Sensor<1>> PressureSensor;

        DataCore(IMUSensor accelerometer,
                 IMUSensor gyro,
                 IMUSensor magnetometer,
                 PressureSensor pressureSensor,
                 uint8_t serverID,
                 uint8_t clientID);
        ~DataCore() {}

        void start();
        void stop();

    private:
        IMUSensor _accelerometer;
        IMUSensor _gyro;
        IMUSensor _magnetometer;
        PressureSensor _pressureSensor;

        dsm::Client _client;
        dsm::LocalBufferKey _dataKey;

        std::atomic<bool> _isRunning;

        struct SensorData {
            bool   isEnabled[4];
            double accelerometer[3];
            double gyro[3];
            double magnetometer[3];
            double pressureSensor;
        } _sensorData;

#ifdef LOGGING_ENABLED
        logging::sources::severity_logger_mt<severityLevel> _logger;
#endif
};

#endif //DATACORE_H
