#include "DataCore.h"

DataCore::DataCore(
        IMUSensor accelerometer,
        IMUSensor gyro,
        IMUSensor magnetometer,
        PressureSensor pressureSensor,
        uint8_t serverID,
        uint8_t clientID) :
        _accelerometer(accelerometer),
        _gyro(gyro),
        _magnetometer(magnetometer),
        _pressureSensor(pressureSensor),
        _client(serverID, clientID),
        _dataKey(dsm::Client::createLocalKey("data")),
        _isRunning(false)
{
    for (int i = 0; i < 3; i++) {
        _sensorData.accelerometer[i] = 0;
        _sensorData.gyro[i] = 0;
        _sensorData.magnetometer[i] = 0;
    }
    _sensorData.pressureSensor = 0;

    _sensorData.isEnabled[ACCL] = _accelerometer->isEnabled();
    _sensorData.isEnabled[GYRO] = _gyro->isEnabled();
    _sensorData.isEnabled[MAGN] = _magnetometer->isEnabled();
    _sensorData.isEnabled[PRES] = _pressureSensor->isEnabled();

    _client.registerLocalBuffer(_dataKey, sizeof(SensorData), false);
    _client.setLocalBufferContents(_dataKey, &_sensorData);

#ifdef LOGGING_ENABLED
    initializeLog();
    setLogFilter(severityLevel::trace);
#endif

    LOG(_logger, severityLevel::startup) << "CONSTRUCTED DATACORE";
}

void DataCore::start() {
    LOG(_logger, severityLevel::info) << "DATACORE STARTED";
    _isRunning = true;
    while (_isRunning.load()) {
        LOG(_logger, severityLevel::periodic) << "POLLING SENSORS";
        Eigen::Matrix<double, 3, 1> imuData;
        Eigen::Matrix<double, 1, 1> pressureData;

        LOG(_logger, severityLevel::periodic) << "POLLING ACCELEROMETER";
        _sensorData.isEnabled[ACCL] = _accelerometer->isEnabled();
        imuData = _accelerometer->getOutput();
        _sensorData.accelerometer[XAXIS] = imuData(XAXIS, 0);
        _sensorData.accelerometer[YAXIS] = imuData(YAXIS, 0);
        _sensorData.accelerometer[ZAXIS] = imuData(ZAXIS, 0);

        LOG(_logger, severityLevel::periodic) << "POLLING GYRO";
        _sensorData.isEnabled[GYRO] = _gyro->isEnabled();
        imuData = _gyro->getOutput();
        _sensorData.gyro[XAXIS] = imuData(XAXIS, 0);
        _sensorData.gyro[YAXIS] = imuData(YAXIS, 0);
        _sensorData.gyro[ZAXIS] = imuData(ZAXIS, 0);

        LOG(_logger, severityLevel::periodic) << "POLLING MAGNETOMETER";
        _sensorData.isEnabled[MAGN] = _magnetometer->isEnabled();
        imuData = _magnetometer->getOutput();
        _sensorData.magnetometer[XAXIS] = imuData(XAXIS, 0);
        _sensorData.magnetometer[YAXIS] = imuData(YAXIS, 0);
        _sensorData.magnetometer[ZAXIS] = imuData(ZAXIS, 0);

        LOG(_logger, severityLevel::periodic) << "POLLING PRESSURE SENSOR";
        _sensorData.isEnabled[PRES] = _pressureSensor->isEnabled();
        pressureData = _pressureSensor->getOutput();
        _sensorData.pressureSensor = pressureData(0, 0);

        LOG(_logger, severityLevel::periodic) << "UPDATING GLOBAL SENSOR DATA";
        _client.setLocalBufferContents(_dataKey, &_sensorData);
        std::this_thread::sleep_for(std::chrono::milliseconds(DATACORE_DELAY));
    }
}

void DataCore::stop() {
    LOG(_logger, severityLevel::teardown) << "DATACORE STOPPING";
    _isRunning = false;
}
