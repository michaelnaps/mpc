
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;

#ifndef MPC_PLANT
#define MPC_PLANT

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
        MatrixXd (*model)(const MatrixXd &, const MatrixXd &);

    public:
        // CONSTRUCTORS:
        Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &));
        Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), const std::string &type, const double &dt);

        // ACCESSOR FUNCTIONS:
        std::string getModelType();
        double getTimeStep();

        // MEMBER FUNCTIONS:
        MatrixXd prop(const MatrixXd &x, const MatrixXd &u);
    };
}

#endif