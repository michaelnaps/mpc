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
        double dt;
        std::string model_type;

    protected:
        MatrixXd (*model)(MatrixXd, MatrixXd);

    public:
        // CONSTRUCTORS:
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd));
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt);
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt, const std::string& model_type);

        // ACCESSOR FUNCTIONS:
        double getTimeStep();
        std::string getModelType();

        // MEMBER FUNCTIONS:
        MatrixXd prop(MatrixXd x, MatrixXd u);
    };
}

#endif