#ifndef DATACORE_CONFIG_H
#define DATACORE_CONFIG_H

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN, int NUM_PRES>
struct DatacoreSettings {
    DatacoreSettings();
    int serverID;
    int clientID;
    bool enabledAccl[NUM_ACCL];
    bool enabledGyro[NUM_GYRO];
    bool enabledMagn[NUM_MAGN];
    bool enabledPres[NUM_PRES];
    int calibrationSampleSize;
    std::string calibrationPath;
    void load(const std::string &filename);
    void save(const std::string &filename);
};

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN, int NUM_PRES>
DatacoreSettings<NUM_ACCL, NUM_GYRO, NUM_MAGN, NUM_PRES>::
DatacoreSettings() {
    serverID = 0;
    clientID = 0;
    for (int i = 0; i < NUM_ACCL; i++) {
        enabledAccl[i] = true;
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        enabledGyro[i] = true;
    }
    for (int i = 0; i < NUM_MAGN; i++) {
        enabledMagn[i] = true;
    }
    for (int i = 0; i < NUM_PRES; i++) {
        enabledPres[i] = true;
    }
    calibrationSampleSize = 0;
    calibrationPath = "";
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN, int NUM_PRES>
void
DatacoreSettings<NUM_ACCL, NUM_GYRO, NUM_MAGN, NUM_PRES>::
load(const std::string &filename) {
    pt::ptree tree;
    pt::read_json(filename, tree);
    serverID = tree.get_child("dsm").get<int>("serverid");
    clientID = tree.get_child("dsm").get<int>("clientid");

    for (int i = 0; i < NUM_ACCL; i++) {
        enabledAccl[i] = tree.get_child("accelerometers").
                              get_child(std::to_string(i)).
                              get_value<bool>();
    }
    for (int i = 0; i < NUM_GYRO; i++) {
        enabledGyro[i] = tree.get_child("gyros").
                              get_child(std::to_string(i)).
                              get_value<bool>();
    }
    for (int i = 0; i < NUM_MAGN; i++) {
        enabledMagn[i] = tree.get_child("magnetometers").
                              get_child(std::to_string(i)).
                              get_value<bool>();
    }
    for (int i = 0; i < NUM_PRES; i++) {
        enabledPres[i] = tree.get_child("pressure").
                              get_child(std::to_string(i)).
                              get_value<bool>();
    }
    calibrationSampleSize = tree.get_child("calibration").
                                 get<int>("samplesize");
    calibrationPath = tree.get_child("calibration").
                           get<std::string>("path");
}

template<int NUM_ACCL, int NUM_GYRO, int NUM_MAGN, int NUM_PRES>
void
DatacoreSettings<NUM_ACCL, NUM_GYRO, NUM_MAGN, NUM_PRES>::
save(const std::string &filename) {
    pt::ptree dsmNode;
    dsmNode.put("serverid", serverID);
    dsmNode.put("clientid", clientID);

    pt::ptree accelerometersNode;
    for (int i = 0; i < NUM_ACCL; i++) {
        accelerometersNode.put(std::to_string(i), enabledAccl[i]);
    }

    pt::ptree gyrosNode;
    for (int i = 0; i < NUM_GYRO; i++) {
        gyrosNode.put(std::to_string(i), enabledGyro[i]);
    }

    pt::ptree magnetometersNode;
    for (int i = 0; i < NUM_MAGN; i++) {
        magnetometersNode.put(std::to_string(i), enabledMagn[i]);
    }

    pt::ptree pressureNode;
    for (int i = 0; i < NUM_PRES; i++) {
        pressureNode.put(std::to_string(i), enabledPres[i]);
    }

    pt::ptree calibrationNode;
    calibrationNode.put("samplesize", calibrationSampleSize);
    calibrationNode.put("path", calibrationPath);

    pt::ptree tree;
    tree.add_child("dsm", dsmNode);
    tree.add_child("accelerometers", accelerometersNode);
    tree.add_child("gyros", gyrosNode);
    tree.add_child("magnetometers", magnetometersNode);
    tree.add_child("pressure", pressureNode);
    tree.add_child("calibration", calibrationNode);

    pt::write_json(filename, tree);
}

#endif //DATACORE_CONFIG_H
