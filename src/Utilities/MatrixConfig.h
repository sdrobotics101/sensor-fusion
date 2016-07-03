#ifndef MATRIX_CONFIG_H
#define MATRIX_CONFIG_H

#include <string>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace pt = boost::property_tree;

template<int X_DIM, int Y_DIM>
struct MatrixSettings {
    MatrixSettings();
    std::string name;
    double values[X_DIM][Y_DIM];
    void load(const std::string &filename);
    void save(const std::string &filename);
};

template<int X_DIM, int Y_DIM>
MatrixSettings<X_DIM, Y_DIM>::
MatrixSettings() {
    name = "";
    for (int i = 0; i < X_DIM; i++) {
        for (int j = 0; j < Y_DIM; j++) {
            values[i][j] = 0;
        }
    }
}

template<int X_DIM, int Y_DIM>
void
MatrixSettings<X_DIM, Y_DIM>::
load(const std::string &filename) {
    pt::ptree tree;
    pt::read_json(filename, tree);

    name = tree.get_value<std::string>("name");

    int i = 0;
    for (pt::ptree::value_type &row : tree.get_child("values")) {
        int j = 0;
        for (pt::ptree::value_type &cell : row.second) {
            values[i][j] = cell.second.get_value<double>();
            j++;
        }
        i++;
    }
}

template<int X_DIM, int Y_DIM>
void
MatrixSettings<X_DIM, Y_DIM>::
save(const std::string &filename) {
    pt::ptree valuesNode;
    for (int i = 0; i < X_DIM; i++) {
        pt::ptree row;
        for (int j = 0; j < Y_DIM; j++) {
            pt::ptree cell;
            cell.put_value(values[i][j]);
            row.push_back(std::make_pair("", cell));
        }
        valuesNode.push_back(std::make_pair("", row));
    }

    pt::ptree tree;
    tree.put("name", name);
    tree.add_child("values", valuesNode);

    pt::write_json(filename, tree);
}

#endif //MATRIX_CONFIG_H
