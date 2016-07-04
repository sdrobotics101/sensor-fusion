#include <iostream>
#include <csignal>
#include <string>
#include <stdexcept>

#include <boost/scoped_ptr.hpp>
#include <boost/program_options.hpp>

#include <eigen3/Eigen/Core>

#include "FusionCore.h"
#include "FusionCoreConstants.h"
#include "FusionCoreConfig.h"
#include "../Utilities/MatrixConfig.h"

namespace po = boost::program_options;
using Eigen::Matrix;

boost::scoped_ptr<FusionCore> _fusionCore;

void signalHandler(int) {
    _fusionCore->stop();
}

int main(int argc, char** argv) {
    try {
        std::string configFile;
        std::string generatePath;

        po::options_description generalOptions("Options");
        generalOptions.add_options()
            ("help,h",   "print help message")
            ("config",   po::value<std::string>(&configFile),   "the configuration file to use")
            ("generate", po::value<std::string>(&generatePath), "generate default config files")
            ;

        po::variables_map vm;
        po::store(po::command_line_parser(argc, argv).
                  options(generalOptions).
                  run(),
                  vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << generalOptions;
            return 0;
        }

        FusionCoreSettings settings;
        MatrixSettings<ANGULAR_STATE_DIM,       1>                       initialStateSettings;
        MatrixSettings<ANGULAR_STATE_DIM,       ANGULAR_STATE_DIM>       initialCovarianceSettings;
        MatrixSettings<ANGULAR_STATE_DIM,       ANGULAR_STATE_DIM>       processNoiseSettings;
        MatrixSettings<ANGULAR_MEASUREMENT_DIM, ANGULAR_MEASUREMENT_DIM> measurementNoiseSettings;

        if (vm.count("config")) {
            settings.load(configFile);
        } else {
            settings.serverID = SERVER_ID;
            settings.clientID = CLIENT_ID;
            settings.initialStatePath      = ANGULAR_INITIAL_STATE;
            settings.initialCovariancePath = ANGULAR_INITIAL_COVARIANCE;
            settings.processNoisePath      = ANGULAR_PROCESS_NOISE;
            settings.measurementNoisePath  = ANGULAR_MEASUREMENT_NOISE;
            if (vm.count("generate")) {
                settings.save(generatePath+"/FusionCore.json");

                initialStateSettings     .name = "initialstate";
                initialCovarianceSettings.name = "initialcovariance";
                processNoiseSettings     .name = "processnoise";
                measurementNoiseSettings .name = "measurementnoise";

                initialStateSettings     .save(generatePath+"/Angular/initialstate.json");
                initialCovarianceSettings.save(generatePath+"/Angular/initialcovariance.json");
                processNoiseSettings     .save(generatePath+"/Angular/processnoise.json");
                measurementNoiseSettings .save(generatePath+"/Angular/measurementnoise.json");

                return 0;
            }
        }

        if (settings.serverID < 0 || settings.serverID > 255) {
            throw std::runtime_error("Invalid server ID");
        }

        if (settings.clientID < 0 || settings.clientID > 255) {
            throw std::runtime_error("Invalid client ID");
        }

        initialStateSettings.load(settings.initialStatePath);
        Matrix<double, ANGULAR_STATE_DIM, 1> initialState;
        for (int i = 0; i < ANGULAR_STATE_DIM; i++) {
            initialState(i,0) = initialStateSettings.values[i][0];
        }

        initialCovarianceSettings.load(settings.initialCovariancePath);
        Matrix<double, ANGULAR_STATE_DIM, ANGULAR_STATE_DIM> initialCovariance;
        for (int i = 0; i < ANGULAR_STATE_DIM; i++) {
            for (int j = 0; j < ANGULAR_STATE_DIM; j++) {
                initialCovariance(i,j) = initialCovarianceSettings.values[i][j];
            }
        }

        processNoiseSettings.load(settings.processNoisePath);
        Matrix<double, ANGULAR_STATE_DIM, ANGULAR_STATE_DIM> processNoise;
        for (int i = 0; i < ANGULAR_STATE_DIM; i++) {
            for (int j = 0; j < ANGULAR_STATE_DIM; j++) {
                processNoise(i,j) = processNoiseSettings.values[i][j];
            }
        }

        measurementNoiseSettings.load(settings.measurementNoisePath);
        Matrix<double, ANGULAR_MEASUREMENT_DIM, ANGULAR_MEASUREMENT_DIM> measurementNoise;
        for (int i = 0; i < ANGULAR_MEASUREMENT_DIM; i++) {
            for (int j = 0; j < ANGULAR_MEASUREMENT_DIM; j++) {
                measurementNoise(i,j) = measurementNoiseSettings.values[i][j];
            }
        }

        _fusionCore.reset(new FusionCore(settings.serverID,
                                         settings.clientID,
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
    } catch (std::exception& e) {
        std::cout << "Error: ";
        std::cout << e.what() << std::endl;
        return 1;
    }
}
