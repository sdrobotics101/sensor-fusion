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

    int i = 0;
    for (pt::ptree::value_type &row : tree.get_child("correction")) {
        int j = 0;
        for (pt::ptree::value_type &cell : row.second) {
            correction[i][j] = cell.second.get_value<double>();
            j++;
        }
        i++;
    }
}

void IMUSensorSettings::save(const std::string &filename) {
    pt::ptree biasNode;
    biasNode.put("X", bias[XAXIS]);
    biasNode.put("Y", bias[YAXIS]);
    biasNode.put("Z", bias[ZAXIS]);

    pt::ptree correctionNode;
    for (int i = 0; i < 3; i++) {
        pt::ptree row;
        for (int j = 0; j < 3; j++) {
            pt::ptree cell;
            cell.put_value(correction[i][j]);
            row.push_back(std::make_pair("", cell));
        }
        correctionNode.push_back(std::make_pair("", row));
    }

    pt::ptree tree;
    tree.add_child("bias", biasNode);
    tree.add_child("correction", correctionNode);

    pt::write_json(filename, tree);
}

#endif //IMUSENSOR_CONFIG_H
