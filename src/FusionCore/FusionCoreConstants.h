#ifndef FUSIONCORE_CONSTANTS_H
#define FUSIONCORE_CONSTANTS_H

//Defaults
#define SERVER_ID 0
#define CLIENT_ID 1

#define ANGULAR_INITIAL_STATE "/Users/rahulsalvi/Dropbox/Robosub/config/Angular/initialstate"
#define ANGULAR_INITIAL_COVARIANCE "/Users/rahulsalvi/Dropbox/Robosub/config/Angular/initialcovariance"
#define ANGULAR_PROCESS_NOISE "/Users/rahulsalvi/Dropbox/Robosub/config/Angular/processnoise"
#define ANGULAR_MEASUREMENT_NOISE "/Users/rahulsalvi/Dropbox/Robosub/config/Angular/measurementnoise"

//Delays
#define DATA_DELAY 10 //milliseconds
#define ANGULAR_DELAY 10 //milliseconds
#define LINEAR_DELAY 10 //milliseconds

//General
#define ANGULAR_STATE_DIM 7
#define ANGULAR_MEASUREMENT_DIM 7
#define ANGULAR_CONTROL_DIM 0

#define ATMOSPHERIC_PRESSURE_BAR 1.01325
#define BARS_TO_METERS 10.1936

#endif //FUSIONCORE_CONSTANTS_H
