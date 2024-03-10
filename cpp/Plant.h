#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

#ifndef MPC_PLANT
#define MPC_PLANT

namespace nap {
    MatrixXd TaylorMethod(
        MatrixXd (*F)(MatrixXd, MatrixXd),
        MatrixXd x, MatrixXd u, double dt);

    class Plant
    {
    private:
        // VARIABLES:
        std::string model_type;
        double time_step;

    protected:
        //VARIABLES:
        MatrixXd (*model)(MatrixXd, MatrixXd);

        // PRIVATE MEMBER FUNCTIONS:
        MatrixXd templateModel(MatrixXd x, MatrixXd u);

    public:
        // VARIABLES:
        MatrixXd (*prop)(MatrixXd, MatrixXd);

        // CONSTRUCTORS:
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd));
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const std::string &type, const double &dt);

        // ACCESSOR FUNCTIONS:
        std::string getModelType();
        double getTimeStep();
    };
}

#endif