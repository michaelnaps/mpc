
#include <iostream>
#include <Eigen/Dense>
#include "Plant.cpp"

using Eigen::MatrixXd;

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
        Plant mvar;
        Cost costx;
        Cost costu;

    public:
        // CONSTRUCTORS:
        ModelCost(const Plant &f, const Cost &gx, const Cost &gu);
        ModelCost(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*gx)(const MatrixXd &), MatrixXd (*gu)(const MatrixXd &));

        // MEMBER FUNCTIONS:
        MatrixXd cost(const MatrixXd &x, const MatrixXd &u);
        MatrixXd xgradient(const MatrixXd &x);
        MatrixXd ugradient(const MatrixXd &u);
        MatrixXd gradient(const MatrixXd &x, const MatrixXd &u);
    };

    class PredictiveCost: public ModelCost
    {
    private:
        // VARIABLES:
        int horz_length;
        int knot_length;

    protected:
    public:
        // CONSTRUCTORS:
        PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu);
        PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P);
        PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P, const int &k);

        // MEMBER FUNCTIONS:
        MatrixXd statePrediction(const MatrixXd &xinit, const MatrixXd &ulist);
        MatrixXd costPrediction(const MatrixXd &xinit, const MatrixXd &ulist);
    };
}

#endif