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
        sensorData.accelerometer[i] = 0;
        sensorData.gyro[i] = 0;
        sensorData.magnetometer[i] = 0;
    }
    sensorData.pressureSensor = 0;

    sensorData.isEnabled[ACCL] = _accelerometer->isEnabled();
    sensorData.isEnabled[GYRO] = _gyro->isEnabled();
    sensorData.isEnabled[MAGN] = _magnetometer->isEnabled();
    sensorData.isEnabled[PRES] = _pressureSensor->isEnabled();

    _client.registerLocalBuffer(_dataKey, sizeof(sensorData), false);
    _client.setLocalBufferContents(_dataKey, &sensorData);
}

void DataCore::start() {
    _isRunning = true;
    while (_isRunning.load()) {
        Eigen::Matrix<double, 3, 1> imuData;
        Eigen::Matrix<double, 1, 1> pressureData;

        if ((sensorData.isEnabled[ACCL] = _accelerometer->isEnabled())) {
            imuData = _accelerometer->getOutput();
            sensorData.accelerometer[XAXIS] = imuData(XAXIS, 0);
            sensorData.accelerometer[YAXIS] = imuData(YAXIS, 0);
            sensorData.accelerometer[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            sensorData.accelerometer[XAXIS] = 0;
            sensorData.accelerometer[YAXIS] = 0;
            sensorData.accelerometer[ZAXIS] = 0;
        }

        if ((sensorData.isEnabled[GYRO] = _gyro->isEnabled())) {
            imuData = _gyro->getOutput();
            sensorData.gyro[XAXIS] = imuData(XAXIS, 0);
            sensorData.gyro[YAXIS] = imuData(YAXIS, 0);
            sensorData.gyro[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            sensorData.gyro[XAXIS] = 0;
            sensorData.gyro[YAXIS] = 0;
            sensorData.gyro[ZAXIS] = 0;
        }

        if ((sensorData.isEnabled[MAGN] = _magnetometer->isEnabled())) {
            imuData = _magnetometer->getOutput();
            sensorData.magnetometer[XAXIS] = imuData(XAXIS, 0);
            sensorData.magnetometer[YAXIS] = imuData(YAXIS, 0);
            sensorData.magnetometer[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            sensorData.magnetometer[XAXIS] = 0;
            sensorData.magnetometer[YAXIS] = 0;
            sensorData.magnetometer[ZAXIS] = 0;
        }

        if ((sensorData.isEnabled[PRES] = _pressureSensor->isEnabled())) {
            pressureData = _pressureSensor->getOutput();
            sensorData.pressureSensor = pressureData(0, 0);
        } else {
            sensorData.pressureSensor = 0;
        }

        _client.setLocalBufferContents(_dataKey, &sensorData);
        std::this_thread::sleep_for(std::chrono::milliseconds(DATACORE_DELAY));
    }
}

void DataCore::stop() {
    _isRunning = false;
}
