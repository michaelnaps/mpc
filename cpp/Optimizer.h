
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
        MatrixXd (*cost)(const MatrixXd &);

        // CONSTRUCTORS:
        Cost(MatrixXd (*g)(const MatrixXd &));
        Cost(MatrixXd (*g)(const MatrixXd &), const double& h);

        // SET METHODS:
        void setStepSize(const double &h);

        // MEMBER FUNCTIONS:
        MatrixXd gradient(const MatrixXd &x);
        MatrixXd hessian(const MatrixXd &x);
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
        Optimizer(MatrixXd (*g)(const MatrixXd &));
        Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n);
        Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e);
        Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e, const double &a);
        Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e, const double &a, const string &type);

        // SET METHODS:
        void setObjectiveFunction(MatrixXd (*g)(const MatrixXd &, const MatrixXd &));
        void setMaxIter(const int &n);
        void setTolerance(const double &e);
        void setStepSize(const double &a);
        void setStepMethod(const string &type);

        // MEMBER FUNCTIONS:
        MatrixXd step(const MatrixXd &x, const MatrixXd &dg);
        MatrixXd solve(const MatrixXd &xinit);
    };
}

#endif