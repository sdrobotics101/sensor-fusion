#include <csignal>
#include <atomic>

#include <boost/scoped_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include "DataCore.h"
#include "../Drivers/Dummy.h"

#define SERVER_ID 0
#define CLIENT_ID 0

boost::scoped_ptr<DataCore> _dataCore;

void signalHandler(int signum) {
    _dataCore->stop();
}

int main() {
    boost::shared_ptr<Sensor<3>> accelerometer(new Dummy<3>(0, "accelerometer"));
    boost::shared_ptr<Sensor<3>> gyro(new Dummy<3>(0, "gyro"));
    boost::shared_ptr<Sensor<3>> magnetometer(new Dummy<3>(0, "magnetometer"));
    boost::shared_ptr<Sensor<1>> pressureSensor(new Dummy<1>(0, "pressure"));

    Eigen::Matrix<double, 1, 1> pressureData;
    pressureData(0, 0) = 100;

    boost::static_pointer_cast<Dummy<1>>(pressureSensor)->setOutput(pressureData);

    _dataCore.reset(new DataCore(accelerometer,
                                 gyro,
                                 magnetometer,
                                 pressureSensor,
                                 SERVER_ID,
                                 CLIENT_ID));
    std::signal(SIGINT, signalHandler);
    _dataCore->start();
    return 0;
}
