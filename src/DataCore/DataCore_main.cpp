#include <csignal>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/array.hpp>

#include "DataCore.h"
#include "../Drivers/Dummy.h"
#include "../Utilities/MultiSensor.h"

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

    boost::array<boost::shared_ptr<Sensor<1>>, 2> pressureSensors;
    pressureSensors[0].reset(new Dummy<1>(0, "pressure0"));
    pressureSensors[1].reset(new Dummy<1>(1, "pressure1"));
    boost::shared_ptr<Sensor<1>> pressureSensor(new MultiSensor<1, 2>(pressureSensors, 0, "multipressure"));

    Eigen::Matrix<double, 1, 1> pressureData;

    pressureData(0,0) = 100;
    boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByID(0)))->setOutput(pressureData);
    pressureData(0,0) = 50;
    boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByDescriptor("pressure1")))->setOutput(pressureData);
    //boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByDescriptor("pressure1")))->disable();

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
