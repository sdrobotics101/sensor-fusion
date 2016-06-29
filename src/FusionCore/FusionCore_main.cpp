#include <csignal>

#include <boost/scoped_ptr.hpp>

#include <eigen3/Eigen/Core>

#include "FusionCore.h"

#define SERVER_ID 0
#define CLIENT_ID 1

using Eigen::Matrix;

boost::scoped_ptr<FusionCore> _fusionCore;

void signalHandler(int signum) {
    _fusionCore->stop();
}

//TODO read from config file, etc
int main() {
    Matrix<double, ANGULAR_STATE_DIM, 1> initialState;
    Matrix<double, ANGULAR_STATE_DIM, ANGULAR_STATE_DIM> initialCovariance;
    Matrix<double, ANGULAR_STATE_DIM, ANGULAR_STATE_DIM> processNoise;
    Matrix<double, ANGULAR_MEASUREMENT_DIM, ANGULAR_MEASUREMENT_DIM> measurementNoise;

    //initalize matrices

    _fusionCore.reset(new FusionCore(SERVER_ID,
                                     CLIENT_ID,
                                     initialState,
                                     initialCovariance,
                                     processNoise,
                                     measurementNoise,
                                     3 - ANGULAR_STATE_DIM, //usually set to 3 - state size
                                     0.001,                 //usually set to a small positive value
                                     2));                   //2 is optimal for gaussian distributions
    std::signal(SIGINT, signalHandler);
    _fusionCore->start();
    return 0;
}
