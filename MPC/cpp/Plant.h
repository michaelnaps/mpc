
#ifndef MPC_PLANT
#define MPC_PLANT

// #include <iostream>
// #include "Plant.cpp"
#include <string>
#include <Eigen/Dense>

namespace nap {
    class Plant
    {
    private:
        // VARIABLES:
        double time_step;
        std::string model_type;
        char mt;

    protected:
        // PUBLIC VARIABLES:
        Eigen::MatrixXd (*model)(const Eigen::MatrixXd &, const Eigen::MatrixXd &);

    public:
        // CONSTRUCTORS:
        Plant(Eigen::MatrixXd (*f)(const Eigen::MatrixXd &, const Eigen::MatrixXd &));
        Plant(Eigen::MatrixXd (*f)(const Eigen::MatrixXd &, const Eigen::MatrixXd &), const std::string &type, const double &dt);

        // ACCESSOR FUNCTIONS:
        std::string getModelType();
        double getTimeStep();

        // MEMBER FUNCTIONS:
        Eigen::MatrixXd prop(const Eigen::MatrixXd &x, const Eigen::MatrixXd &u);
    };
}

#endif