#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

#ifndef MPC_OBJECTIVE
#define MPC_OBJECTIVE

namespace nap {
    class Cost {
    private:
        // VARIABLES:
        double step_size;

    public:
        // VARIABLES:
        MatrixXd (*cost)(MatrixXd);

        // CONSTRUCTORS:
        Cost(MatrixXd (*g)(MatrixXd));
        Cost(MatrixXd (*g)(MatrixXd), const double& h);

        // MEMBER FUNCTIONS:
        MatrixXd gradient(MatrixXd x);
        MatrixXd hessian(MatrixXd x);
    };
}

#endif