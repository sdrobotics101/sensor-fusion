#include "FusionCore.h"


FusionCore::FusionCore(uint8_t serverID,
                       uint8_t clientID,
                       angularStateVector initialState,
                       angularStateMatrix initialCovariance,
                       angularStateMatrix processNoise,
                       angularMeasurementMatrix measurementNoise,
                       double kappa,
                       double alpha,
                       double beta) :
                       _client(serverID, clientID),
                       _angularKey(dsm::Client::createLocalKey("angular")),
                       _linearKey(dsm::Client::createLocalKey("linear")),
                       _dataKey(dsm::Client::createLocalKey("data")),
                       _angularFilter(initialState,
                                      initialCovariance,
                                      &FusionCore::angularStateTransfer,
                                      &FusionCore::angularMeasurementTransfer,
                                      processNoise,
                                      measurementNoise,
                                      ANGULAR_DELAY,
                                      kappa,
                                      alpha,
                                      beta),
                       _isRunning(false)
{
    _client.registerLocalBuffer(_dataKey, sizeof(SensorData), false);

    for (int i = 0; i < 3; i++) {
        _angularData.pos[i] = 0;
        _angularData.vel[i] = 0;
        _angularData.acc[i] = 0;
    }
    _angularData.pos[3] = 0;
    _client.registerLocalBuffer(_angularKey, sizeof(AngularData), false);
    _client.setLocalBufferContents(_angularKey, &_angularData);
}

FusionCore::~FusionCore() {
    _isRunning = false;
    _angularThread->join();
    //_linearThread->join();
}

void FusionCore::start() {
    _isRunning = true;
    _angularThread.reset(new boost::thread(boost::bind(&FusionCore::angularThreadFunction, this)));
    while (_isRunning.load()) {
        boost::unique_lock<boost::shared_mutex> lock(_sensorDataMutex);
        _client.getLocalBufferContents(_dataKey, &_sensorData);
        lock.unlock();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(DATA_DELAY));
    }
}

void FusionCore::stop() {
    _isRunning = false;
}

FusionCore::angularStateVector FusionCore::angularStateTransfer(angularStateVector state, angularControlVector control, double dt) {
    Quaterniond quat;
    quat.w() = state(0,0);
    quat.x() = state(1,0);
    quat.y() = state(2,0);
    quat.z() = state(3,0);

    quat *= eulerToQuaternion(state(4,0)*dt, state(5,0)*dt, state(6,0)*dt);

    state(0,0) = quat.w();
    state(1,0) = quat.x();
    state(2,0) = quat.y();
    state(3,0) = quat.z();
    return state;
}

FusionCore::angularMeasurementVector FusionCore::angularMeasurementTransfer(angularStateVector state) {
    return state;
}

//TODO dt should be calculated
void FusionCore::angularThreadFunction() {
    while (_isRunning.load()) {
        //get reading from sensors
        Vector3d x,y,z;
        boost::shared_lock<boost::shared_mutex> lock(_sensorDataMutex);
        z << _sensorData.accelerometer[XAXIS], _sensorData.accelerometer[YAXIS], _sensorData.accelerometer[ZAXIS];
        x << _sensorData.magnetometer[XAXIS], _sensorData.magnetometer[YAXIS], _sensorData.magnetometer[ZAXIS];
        _angularMeasurement(4,0) = _sensorData.gyro[XAXIS];
        _angularMeasurement(5,0) = _sensorData.gyro[YAXIS];
        _angularMeasurement(6,0) = _sensorData.gyro[ZAXIS];
        lock.unlock();

        //turn into measurement vector
        y = z.cross(x);
        x = y.cross(z);

        Quaterniond measuredQuat = rotationBetweenSystems(Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
                                                          x,                 y,                 z);
        _angularMeasurement(0,0) = measuredQuat.w();
        _angularMeasurement(1,0) = measuredQuat.x();
        _angularMeasurement(2,0) = measuredQuat.y();
        _angularMeasurement(3,0) = measuredQuat.z();

        //step
        angularStateVector previous = _angularFilter.state();
        angularStateVector current = _angularFilter.step(Eigen::MatrixXd::Zero(ANGULAR_CONTROL_DIM, 1), _angularMeasurement);

        _angularData.pos[0] = current(0,0);
        _angularData.pos[1] = current(1,0);
        _angularData.pos[2] = current(2,0);
        _angularData.pos[3] = current(3,0);

        _angularData.vel[0] = current(4,0);
        _angularData.vel[1] = current(5,0);
        _angularData.vel[2] = current(6,0);

        _angularData.acc[0] = (current(4,0) - previous(4,0)) / ANGULAR_DELAY;
        _angularData.acc[1] = (current(5,0) - previous(5,0)) / ANGULAR_DELAY;
        _angularData.acc[2] = (current(6,0) - previous(6,0)) / ANGULAR_DELAY;

        //update angular state through dsm client
        _client.setLocalBufferContents(_angularKey, &_angularData);

        //sleep
        boost::this_thread::sleep_for(boost::chrono::milliseconds(ANGULAR_DELAY));
    }
}

Quaterniond FusionCore::rotationBetweenSystems(Vector3d x1, Vector3d y1, Vector3d z1,
                                               Vector3d x2, Vector3d y2, Vector3d z2) {
    Eigen::Matrix3d M = x1*x2.transpose() + y1*y2.transpose() + z1*z2.transpose();

    Eigen::Matrix4d N;
    N << M(0,0)+M(1,1)+M(2,2)   ,M(1,2)-M(2,1)          , M(2,0)-M(0,2)         , M(0,1)-M(1,0),
         M(1,2)-M(2,1)          ,M(0,0)-M(1,1)-M(2,2)   , M(0,1)+M(1,0)         , M(2,0)+M(0,2),
         M(2,0)-M(0,2)          ,M(0,1)+M(1,0)          ,-M(0,0)+M(1,1)-M(2,2)  , M(1,2)+M(2,1),
         M(0,1)-M(1,0)          ,M(2,0)+M(0,2)          , M(1,2)+M(2,1)         ,-M(0,0)-M(1,1)+M(2,2);

    Eigen::EigenSolver<Eigen::Matrix4d> N_es(N);
    Eigen::Vector4d::Index maxIndex;
    N_es.eigenvalues().real().maxCoeff(&maxIndex);

    Eigen::Vector4d ev_max = N_es.eigenvectors().col(maxIndex).real();

    Quaterniond quat(ev_max(0), ev_max(1), ev_max(2), ev_max(3));
    quat.normalize();

    return quat;
}

Quaterniond FusionCore::eulerToQuaternion(double x, double y, double z) {
    x *= (M_PI/180);
    y *= (M_PI/180);
    z *= (M_PI/180);
    Eigen::AngleAxisd roll(x, Vector3d::UnitX());
    Eigen::AngleAxisd pitch(y, Vector3d::UnitY());
    Eigen::AngleAxisd yaw(z, Vector3d::UnitZ());
    Quaterniond quat = roll*pitch*yaw;
    return quat;
}
