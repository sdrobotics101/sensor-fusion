#ifndef FUSIONCORE_CONFIG_H
#define FUSIONCORE_CONFIG_H

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

struct FusionCoreSettings {
    FusionCoreSettings();
    int serverID;
    int clientID;
    std::string initialStatePath;
    std::string initialCovariancePath;
    std::string processNoisePath;
    std::string measurementNoisePath;

    void load(const std::string& filename);
    void save(const std::string& filename);
};

FusionCoreSettings::FusionCoreSettings() {
    serverID = 0;
    clientID = 0;
    initialStatePath      = "";
    initialCovariancePath = "";
    processNoisePath      = "";
    measurementNoisePath  = "";
}

void FusionCoreSettings::load(const std::string& filename) {
    pt::ptree tree;
    pt::read_json(filename, tree);

    serverID              = tree.get_child("dsm")  .get<int>        ("serverid");
    clientID              = tree.get_child("dsm")  .get<int>        ("clientid");
    initialStatePath      = tree.get_child("paths").get<std::string>("initialstate");
    initialCovariancePath = tree.get_child("paths").get<std::string>("initialcovariance");
    processNoisePath      = tree.get_child("paths").get<std::string>("processnoise");
    measurementNoisePath  = tree.get_child("paths").get<std::string>("measurementnoise");
}

void FusionCoreSettings::save(const std::string& filename) {
    pt::ptree dsmNode;
    dsmNode.put("serverid", serverID);
    dsmNode.put("clientid", clientID);

    pt::ptree pathsNode;
    pathsNode.put("initialstate",      initialStatePath);
    pathsNode.put("initialcovariance", initialCovariancePath);
    pathsNode.put("processnoise",      processNoisePath);
    pathsNode.put("measurementnoise",  measurementNoisePath);

    pt::ptree tree;
    tree.add_child("dsm",   dsmNode);
    tree.add_child("paths", pathsNode);

    pt::write_json(filename, tree);
}

#endif //FUSIONCORE_CONFIG_H
