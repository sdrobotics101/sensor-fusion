#ifndef DUMMY_H
#define DUMMY_H

#include "../Utilities/Sensor.h"

template<int OUTPUT_DIM>
class Dummy : public Sensor<OUTPUT_DIM> {
    public:
        Dummy(uint8_t ID, std::string descriptor) :
            Sensor<OUTPUT_DIM>(ID, "dummy_"+descriptor+std::to_string(ID)) {
                for (int i = 0; i < OUTPUT_DIM; i++) {
                    _output(i, 0) = 0;
                }
            };
        virtual ~Dummy() {}
        Eigen::Matrix<double, OUTPUT_DIM, 1> getOutput() {return _output;}
        void setOutput(Eigen::Matrix<double, OUTPUT_DIM, 1> in) {_output = in;}
    private:
        Eigen::Matrix<double, OUTPUT_DIM, 1> _output;
};

#endif //DUMMY_H
