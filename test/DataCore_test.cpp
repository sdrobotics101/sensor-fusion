#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "../src/Dependencies/DistributedSharedMemory/src/Client/DSMClient.h"

#define SERVER_ID 0
#define CLIENT_ID 1

std::atomic<bool> isRunning;

void signalHandler(int signum) {
    isRunning = false;
}

int main() {
    dsm::Client _client(SERVER_ID, CLIENT_ID);
    dsm::LocalBufferKey key = dsm::Client::createLocalKey("data");

    isRunning = true;
    while(isRunning.load() && !_client.doesLocalExist(key)) {}

    struct SensorData {
        double accelerometer[3];
        double gyro[3];
        double magnetometer[3];
        double pressureSensor;
    } sensorData;

    while(isRunning.load()) {
        if (!_client.getLocalBufferContents(key, &sensorData)) {
            break;
        }
        std::cout << "P: " << sensorData.pressureSensor << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
