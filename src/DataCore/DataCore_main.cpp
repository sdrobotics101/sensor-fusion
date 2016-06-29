#include <csignal>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/array.hpp>

#include "DataCore.h"
#include "CalibrationModule.h"
#include "../Drivers/Dummy.h"
#include "../Utilities/MultiSensor.h"

#define SERVER_ID 0
#define CLIENT_ID 0

#define NUM_ACCL 9
#define NUM_GYRO 4
#define NUM_MAGN 9
#define NUM_PRES 2

boost::scoped_ptr<DataCore> _dataCore;

void signalHandler(int signum) {
    _dataCore->stop();
}

//TODO use program options for calibration mode and config file locations
int main() {
    boost::array<boost::shared_ptr<Sensor<3>>, NUM_ACCL> accelerometers;
    for (int i = 0; i < NUM_ACCL; i++) {
        accelerometers[i].reset(new Dummy<3>(i, "accelerometer"+std::to_string(i)));
    }
    boost::shared_ptr<Sensor<3>> accelerometer(new MultiSensor<3, NUM_ACCL>(&accelerometers, 0, "multiaccl"));

    boost::array<boost::shared_ptr<Sensor<3>>, NUM_GYRO> gyros;
    for (int i = 0; i < NUM_GYRO; i++) {
        gyros[i].reset(new Dummy<3>(i, "gyro"+std::to_string(i)));
    }
    boost::shared_ptr<Sensor<3>> gyro(new MultiSensor<3, NUM_GYRO>(&gyros, 0, "multigyro"));

    boost::array<boost::shared_ptr<Sensor<3>>, NUM_MAGN> magnetometers;
    for (int i = 0; i < NUM_MAGN; i++) {
        magnetometers[i].reset(new Dummy<3>(i, "magnetometer"+std::to_string(i)));
    }
    boost::shared_ptr<Sensor<3>> magnetometer(new MultiSensor<3, NUM_MAGN>(&magnetometers, 0, "multimagn"));

    boost::array<boost::shared_ptr<Sensor<1>>, NUM_PRES> pressureSensors;
    for (int i = 0; i < NUM_PRES; i++) {
        pressureSensors[i].reset(new Dummy<1>(i, "pressure"+std::to_string(i)));
    }
    boost::shared_ptr<Sensor<1>> pressureSensor(new MultiSensor<1, NUM_PRES>(&pressureSensors, 0, "multipressure"));

    Eigen::Matrix<double, 3, 1> d;
    d << 50, 75, 100;
    boost::static_pointer_cast<Dummy<3>>(accelerometers[3])->setOutput(d);

    Eigen::Matrix<double, 1, 1> pressureData;

    pressureData(0,0) = 100;
    boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByID(0)))->setOutput(pressureData);
    pressureData(0,0) = 50;
    boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByDescriptor("pressure1")))->setOutput(pressureData);
    //boost::static_pointer_cast<Dummy<1>>((boost::static_pointer_cast<MultiSensor<1,2>>(pressureSensor)->getSensorByDescriptor("pressure1")))->disable();

    CalibrationModule<NUM_ACCL, NUM_GYRO, NUM_MAGN> calibrationModule(&accelerometers, &gyros, &magnetometers, 10);
    calibrationModule.start();

    /* _dataCore.reset(new DataCore(accelerometer, */
    /*                              gyro, */
    /*                              magnetometer, */
    /*                              pressureSensor, */
    /*                              SERVER_ID, */
    /*                              CLIENT_ID)); */
    /* std::signal(SIGINT, signalHandler); */
    /* _dataCore->start(); */
    return 0;
}
