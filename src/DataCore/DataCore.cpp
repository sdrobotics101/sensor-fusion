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
}

void DataCore::start() {
    _isRunning = true;
    while (_isRunning.load()) {
        Eigen::Matrix<double, 3, 1> imuData;
        Eigen::Matrix<double, 1, 1> pressureData;

        if ((_sensorData.isEnabled[ACCL] = _accelerometer->isEnabled())) {
            imuData = _accelerometer->getOutput();
            _sensorData.accelerometer[XAXIS] = imuData(XAXIS, 0);
            _sensorData.accelerometer[YAXIS] = imuData(YAXIS, 0);
            _sensorData.accelerometer[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            _sensorData.accelerometer[XAXIS] = 0;
            _sensorData.accelerometer[YAXIS] = 0;
            _sensorData.accelerometer[ZAXIS] = 0;
        }

        if ((_sensorData.isEnabled[GYRO] = _gyro->isEnabled())) {
            imuData = _gyro->getOutput();
            _sensorData.gyro[XAXIS] = imuData(XAXIS, 0);
            _sensorData.gyro[YAXIS] = imuData(YAXIS, 0);
            _sensorData.gyro[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            _sensorData.gyro[XAXIS] = 0;
            _sensorData.gyro[YAXIS] = 0;
            _sensorData.gyro[ZAXIS] = 0;
        }

        if ((_sensorData.isEnabled[MAGN] = _magnetometer->isEnabled())) {
            imuData = _magnetometer->getOutput();
            _sensorData.magnetometer[XAXIS] = imuData(XAXIS, 0);
            _sensorData.magnetometer[YAXIS] = imuData(YAXIS, 0);
            _sensorData.magnetometer[ZAXIS] = imuData(ZAXIS, 0);
        } else {
            _sensorData.magnetometer[XAXIS] = 0;
            _sensorData.magnetometer[YAXIS] = 0;
            _sensorData.magnetometer[ZAXIS] = 0;
        }

        if ((_sensorData.isEnabled[PRES] = _pressureSensor->isEnabled())) {
            pressureData = _pressureSensor->getOutput();
            _sensorData.pressureSensor = pressureData(0, 0);
        } else {
            _sensorData.pressureSensor = 0;
        }

        _client.setLocalBufferContents(_dataKey, &_sensorData);
        std::this_thread::sleep_for(std::chrono::milliseconds(DATACORE_DELAY));
    }
}

void DataCore::stop() {
    _isRunning = false;
}
