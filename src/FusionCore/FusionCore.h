#ifndef FUSIONCORE_H
#define FUSIONCORE_H

#include <cmath>
#include <atomic>

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>

#include "../Dependencies/DistributedSharedMemory/src/Client/DSMClient.h"
#include "../Dependencies/UnscentedKalmanFilter/src/UnscentedKalmanFilter.h"

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

#define ANGULAR_STATE_DIM 7
#define ANGULAR_MEASUREMENT_DIM 7
#define ANGULAR_CONTROL_DIM 0

#define DATA_DELAY 10 //milliseconds
#define ANGULAR_DELAY 10 //milliseconds

using Eigen::Matrix;
using Eigen::Vector3d;
using Eigen::Quaterniond;

class FusionCore {
    public:
        typedef Matrix<double, ANGULAR_STATE_DIM, 1> angularStateVector;
        typedef Matrix<double, ANGULAR_STATE_DIM, ANGULAR_STATE_DIM> angularStateMatrix;
        typedef Matrix<double, ANGULAR_MEASUREMENT_DIM, 1> angularMeasurementVector;
        typedef Matrix<double, ANGULAR_MEASUREMENT_DIM, ANGULAR_MEASUREMENT_DIM> angularMeasurementMatrix;
        typedef Matrix<double, ANGULAR_CONTROL_DIM, 1> angularControlVector;

        FusionCore(uint8_t serverID,
                   uint8_t clientID,
                   angularStateVector initialState,
                   angularStateMatrix initialCovariance,
                   angularStateMatrix processNoise,
                   angularMeasurementMatrix measurementNoise,
                   double kappa,
                   double alpha,
                   double beta);
        ~FusionCore();

        void start();
        void stop();

    private:
        static angularStateVector angularStateTransfer(angularStateVector state, angularControlVector control, double dt);
        static angularMeasurementVector angularMeasurementTransfer(angularStateVector state);

        void angularThreadFunction();
        /* void linearThreadFunction(); */

        static Quaterniond rotationBetweenSystems(Vector3d x1, Vector3d y1, Vector3d z1,
                                                  Vector3d x2, Vector3d y2, Vector3d z2);
        static Quaterniond eulerToQuaternion(double x, double y, double z);

        dsm::Client _client;
        dsm::LocalBufferKey _angularKey;
        dsm::LocalBufferKey _linearKey;
        dsm::LocalBufferKey _dataKey;

        boost::scoped_ptr<boost::thread> _angularThread;
        /* boost::scoped_ptr<boost::thread> _linearThread; */

        angularMeasurementVector _angularMeasurement;

        UnscentedKalmanFilter<ANGULAR_STATE_DIM, ANGULAR_MEASUREMENT_DIM, ANGULAR_CONTROL_DIM> _angularFilter;

        std::atomic<bool> _isRunning;

        struct SensorData {
            bool   isEnabled[4];
            double accelerometer[3];
            double gyro[3];
            double magnetometer[3];
            double pressureSensor;
        } _sensorData;
        boost::shared_mutex _sensorDataMutex;

        struct AngularData {
            double pos[4];
            double vel[3];
            double acc[3];
        } _angularData;
};

#endif //FUSIONCORE_H
