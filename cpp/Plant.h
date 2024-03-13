
#include <iostream>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

#ifndef MPC_PLANT
#define MPC_PLANT

namespace nap {
    MatrixXd TaylorMethod(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd x, MatrixXd u, double dt);

    class Plant
    {
    private:
        // VARIABLES:
        std::string model_type;
        double time_step;

    protected:
        //VARIABLES:
        MatrixXd (*model)(const MatrixXd &, const MatrixXd &);

        // PRIVATE MEMBER FUNCTIONS:
        MatrixXd templateModel(const MatrixXd &x, const MatrixXd &u);

    public:
        // VARIABLES:
        MatrixXd (*prop)(const MatrixXd &, const MatrixXd &);

        // CONSTRUCTORS:
        Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &));
        Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), const std::string &type, const double &dt);

        // ACCESSOR FUNCTIONS:
        std::string getModelType();
        double getTimeStep();

        // MEMBER FUNCTIONS:
        MatrixXd cprop(const MatrixXd &x, const MatrixXd &u);
    };
}

#endif