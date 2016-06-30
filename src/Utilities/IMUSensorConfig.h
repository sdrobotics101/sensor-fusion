#ifndef IMUSENSOR_CONFIG_H
#define IMUSENSOR_CONFIG_H

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2

namespace pt = boost::property_tree;

struct IMUSensorSettings {
    IMUSensorSettings();
    double bias[3];
    double correction[3][3];
    void load(const std::string &filename);
    void save(const std::string &filename);
};

IMUSensorSettings::IMUSensorSettings() {
    for (int i = 0; i < 3; i++) {
        bias[i] = 0;
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            correction[i][j] = 0;
        }
    }
}

void IMUSensorSettings::load(const std::string &filename) {
    pt::ptree tree;
    pt::read_json(filename, tree);

    bias[XAXIS] = tree.get_child("bias").get<double>("X");
    bias[YAXIS] = tree.get_child("bias").get<double>("Y");
    bias[ZAXIS] = tree.get_child("bias").get<double>("Z");

    correction[0][0] = tree.get_child("correction").get<double>("11");
    correction[0][1] = tree.get_child("correction").get<double>("12");
    correction[0][2] = tree.get_child("correction").get<double>("13");

    correction[1][0] = tree.get_child("correction").get<double>("21");
    correction[1][1] = tree.get_child("correction").get<double>("22");
    correction[1][2] = tree.get_child("correction").get<double>("23");

    correction[2][0] = tree.get_child("correction").get<double>("31");
    correction[2][1] = tree.get_child("correction").get<double>("32");
    correction[2][2] = tree.get_child("correction").get<double>("33");
}

void IMUSensorSettings::save(const std::string &filename) {
    pt::ptree biasNode;
    biasNode.put("X", bias[XAXIS]);
    biasNode.put("Y", bias[YAXIS]);
    biasNode.put("Z", bias[ZAXIS]);

    pt::ptree correctionNode;
    correctionNode.put("11", correction[0][0]);
    correctionNode.put("12", correction[0][1]);
    correctionNode.put("13", correction[0][2]);

    correctionNode.put("21", correction[1][0]);
    correctionNode.put("22", correction[1][1]);
    correctionNode.put("23", correction[1][2]);

    correctionNode.put("31", correction[2][0]);
    correctionNode.put("32", correction[2][1]);
    correctionNode.put("33", correction[2][2]);

    pt::ptree tree;
    tree.add_child("bias", biasNode);
    tree.add_child("correction", correctionNode);

    pt::write_json(filename, tree);
}

#endif //IMUSENSOR_CONFIG_H
