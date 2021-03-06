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
                                      0,
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
        _linearData.pos[i]  = 0;
        _linearData.vel[i]  = 0;
        _linearData.acc[i]  = 0;
    }
    _angularData.pos[3] = 0;

    _client.registerLocalBuffer(_angularKey, sizeof(AngularData), false);
    _client.setLocalBufferContents(_angularKey, &_angularData);

    _client.registerLocalBuffer(_linearKey, sizeof(LinearData), false);
    _client.setLocalBufferContents(_linearKey, &_linearData);

#ifdef LOGGING_ENABLED
    log::initializeLog();
    log::setLogFilter(log::severityLevel::trace);
#endif

    LOG(_logger, log::severityLevel::startup) << "CONSTRUCTED FUSIONCORE";
}

FusionCore::~FusionCore() {
    _isRunning = false;
    _angularThread->join();
    _linearThread->join();
    LOG(_logger, log::severityLevel::teardown) << "DESTRUCTED FUSIONCORE";
}

void FusionCore::start() {
    LOG(_logger, log::severityLevel::info) << "FUSIONCORE STARTED";
    _isRunning = true;
    _angularThread.reset(new boost::thread(boost::bind(&FusionCore::angularThreadFunction, this)));
    _linearThread .reset(new boost::thread(boost::bind(&FusionCore::linearThreadFunction,  this)));
    while (_isRunning.load()) {
        LOG(_logger, log::severityLevel::periodic) << "GETTING NEW DATA";
        boost::unique_lock<boost::shared_mutex> lock(_sensorDataMutex);
        _client.getLocalBufferContents(_dataKey, &_sensorData);
        lock.unlock();
        boost::this_thread::sleep_for(boost::chrono::milliseconds(DATA_DELAY));
    }
}

void FusionCore::stop() {
    LOG(_logger, log::severityLevel::info) << "FUSIONCORE STOPPED";
    _isRunning = false;
}

//TODO should this update the gyro state using the control vector?
FusionCore::angularStateVector FusionCore::angularStateTransfer(angularStateVector state, angularControlVector /*control*/, double dt) {
    Quaterniond quat;
    quat.w() = state(0,0);
    quat.x() = state(1,0);
    quat.y() = state(2,0);
    quat.z() = state(3,0);

    quat *= eulerToQuaternion(state(4,0)*dt, state(5,0)*dt, state(6,0)*dt);
    quat.normalize();

    state(0,0) = quat.w();
    state(1,0) = quat.x();
    state(2,0) = quat.y();
    state(3,0) = quat.z();
    return state;
}

FusionCore::angularMeasurementVector FusionCore::angularMeasurementTransfer(angularStateVector state) {
    return state;
}

//TODO are directions correct?
void FusionCore::angularThreadFunction() {
    LOG(_logger, log::severityLevel::startup) << "ANGULAR THREAD STARTED";
    posix_time::ptime tick = posix_time::second_clock::local_time();
    while (_isRunning.load()) {
        //get reading from sensors
        Vector3d x,y,z;
        angularMeasurementVector angularMeasurement;
        boost::shared_lock<boost::shared_mutex> dataLock(_sensorDataMutex);
        z << _sensorData.accelerometer[XAXIS], _sensorData.accelerometer[YAXIS], _sensorData.accelerometer[ZAXIS];
        x << _sensorData.magnetometer[XAXIS], _sensorData.magnetometer[YAXIS], _sensorData.magnetometer[ZAXIS];
        angularMeasurement(4,0) = _sensorData.gyro[XAXIS];
        angularMeasurement(5,0) = _sensorData.gyro[YAXIS];
        angularMeasurement(6,0) = _sensorData.gyro[ZAXIS];
        dataLock.unlock();

        //turn into measurement vector
        y = z.cross(x);
        x = y.cross(z);

        Quaterniond measuredQuat = rotationBetweenSystems(Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
                                                          x,                 y,                 z);
        angularMeasurement(0,0) = measuredQuat.w();
        angularMeasurement(1,0) = measuredQuat.x();
        angularMeasurement(2,0) = measuredQuat.y();
        angularMeasurement(3,0) = measuredQuat.z();

        //step
        posix_time::ptime now = posix_time::second_clock::local_time();
        posix_time::time_duration diff = now - tick;
        tick = posix_time::second_clock::local_time();
        double dt = diff.total_microseconds() / 1000000;    //conversion from microseconds to seconds

        angularControlVector control;

        angularStateVector previous = _angularFilter.state();
        angularStateVector current = _angularFilter.step(control, angularMeasurement, dt);

        boost::unique_lock<boost::mutex> angularLock(_angularDataMutex);
        _angularData.pos[QUAT_W] = current(0,0);
        _angularData.pos[QUAT_X] = current(1,0);
        _angularData.pos[QUAT_Y] = current(2,0);
        _angularData.pos[QUAT_Z] = current(3,0);

        _angularData.vel[XAXIS] = current(4,0);
        _angularData.vel[YAXIS] = current(5,0);
        _angularData.vel[ZAXIS] = current(6,0);

        _angularData.acc[XAXIS] = (current(4,0) - previous(4,0)) / dt;
        _angularData.acc[YAXIS] = (current(5,0) - previous(5,0)) / dt;
        _angularData.acc[ZAXIS] = (current(6,0) - previous(6,0)) / dt;
        angularLock.unlock();

        //update angular state through dsm client
        LOG(_logger, log::severityLevel::periodic) << "UPDATING GLOBAL ANGULAR STATE";
        _client.setLocalBufferContents(_angularKey, &_angularData);

        //sleep
        boost::this_thread::sleep_for(boost::chrono::milliseconds(ANGULAR_DELAY));
    }
}

void FusionCore::linearThreadFunction() {
    LOG(_logger, log::severityLevel::startup) << "LINEAR THREAD STARTED";
    posix_time::ptime tick = posix_time::second_clock::local_time();
    while (_isRunning.load()) {
        posix_time::ptime now = posix_time::second_clock::local_time();
        posix_time::time_duration diff = now - tick;
        tick = posix_time::second_clock::local_time();
        double dt = diff.total_microseconds() / 1000000;    //conversion from microseconds to seconds

        //get reading from sensors
        Vector3d acceleration;
        double pressure;
        Quaterniond orientation;

        boost::shared_lock<boost::shared_mutex> dataLock(_sensorDataMutex);
        acceleration(XAXIS,0) = _sensorData.accelerometer[XAXIS];
        acceleration(YAXIS,0) = _sensorData.accelerometer[YAXIS];
        acceleration(ZAXIS,0) = _sensorData.accelerometer[ZAXIS];
        pressure = _sensorData.pressureSensor;
        dataLock.unlock();

        boost::unique_lock<boost::mutex> angularLock(_angularDataMutex);
        orientation.w() = _angularData.pos[QUAT_W];
        orientation.x() = _angularData.pos[QUAT_X];
        orientation.y() = _angularData.pos[QUAT_Y];
        orientation.z() = _angularData.pos[QUAT_Z];
        angularLock.unlock();

        _linearData.acc[XAXIS] = acceleration(XAXIS,0);
        _linearData.acc[YAXIS] = acceleration(YAXIS,0);
        _linearData.acc[ZAXIS] = acceleration(ZAXIS,0);

        //TODO conjugate?
        acceleration -= (orientation._transformVector((9.81 * Vector3d::UnitZ())));

        _linearData.vel[XAXIS] += acceleration(XAXIS,0) * dt;
        _linearData.vel[YAXIS] += acceleration(YAXIS,0) * dt;

        _linearData.pos[XAXIS] += _linearData.vel[XAXIS] * dt;
        _linearData.pos[YAXIS] += _linearData.vel[YAXIS] * dt;

        double old = _linearData.pos[ZAXIS];
        double current = pressureToDepth(pressure);

        _linearData.vel[ZAXIS] = (current - old) / dt;
        _linearData.pos[ZAXIS] = current;

        //update linear state through dsm client
        LOG(_logger, log::severityLevel::periodic) << "UPDATING GLOBAL LINEAR STATE";
        _client.setLocalBufferContents(_linearKey, &_linearData);

        //sleep
        boost::this_thread::sleep_for(boost::chrono::milliseconds(LINEAR_DELAY));
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
    quat.normalize();

    return quat;
}

//converts pressure in Bars to depth in meters
double FusionCore::pressureToDepth(double pressure) {
    return (pressure - ATMOSPHERIC_PRESSURE_BAR) * BARS_TO_METERS;
}
