
#include <iostream>
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using namespace std;

#ifndef MPC_OPTIMIZER
#define MPC_OPTIMIZER

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

        // SET METHODS:
        void setStepSize(const double &h);

        // MEMBER FUNCTIONS:
        MatrixXd gradient(MatrixXd x);
        MatrixXd hessian(MatrixXd x);
    };

    class Optimizer : public Cost {
    private:
        // VARIABLES:
        int max_iter;
        double epsilon;
        double alpha;
        string method;

    public:
        // CONSTRUCTORS:
        Optimizer(MatrixXd (*g)(MatrixXd));
        Optimizer(MatrixXd (*g)(MatrixXd), const int &n, const double &e, const double &a, const string &type);

        // SET METHODS:
        void setStepSize(const double &a);
        void setMaxIter(const int &n);
        void setObjectiveFunction(MatrixXd (*g)(MatrixXd, MatrixXd));
        void setStepMethod(const string &type);

        // MEMBER FUNCTIONS:
        MatrixXd step(MatrixXd x, MatrixXd dg);
        MatrixXd solve(MatrixXd x0);
    };
}

#endif