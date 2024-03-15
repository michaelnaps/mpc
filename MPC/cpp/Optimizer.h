
#include <iostream>
#include <Eigen/Dense>
using Eigen::MatrixXd;
using namespace std;

#ifndef MPC_OPTIMIZER
#define MPC_OPTIMIZER

namespace nap {
    class Cost {
    public:
        // VARIABLES:
        int max_iter;
        double alpha;
        double step_size;
        double epsilon;

        // VARIABLES:
        MatrixXd (*cost)(const MatrixXd &);

        // CONSTRUCTORS:
        Cost(MatrixXd (*g)(const MatrixXd &));
        Cost(MatrixXd (*g)(const MatrixXd &), const int &n);
        Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a);
        Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a, const double &h);
        Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a, const double &h, const double &e);

        // TODO: SET METHODS:

        // MEMBER FUNCTIONS:
        MatrixXd gradient(const MatrixXd &x);
        // TODO: MatrixXd hessian(const MatrixXd &x);
        MatrixXd step(const MatrixXd &x, const MatrixXd &dg);
        MatrixXd solve(const MatrixXd &xinit);
    };

    class ModelCost
    {
    private:
        // VARIABLES:

    protected:
        // PROTECTED VARIABLES:
        Cost costx;
        Cost costu;

    public:
        // CONSTRUCTORS:
        ModelCost(const Cost &gx, const Cost &gu);
        ModelCost(MatrixXd (*gx)(const MatrixXd &), MatrixXd (*gu)(const MatrixXd &));

        // MEMBER FUNCTIONS:
        MatrixXd cost(const MatrixXd &x, const MatrixXd &u);
        MatrixXd xgradient(const MatrixXd &u);
        MatrixXd ugradient(const MatrixXd &x);
        MatrixXd gradient(const MatrixXd &x, const MatrixXd &u);
    };
}

#endif