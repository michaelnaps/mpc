
#include "Plant.cpp"
#include "Optimizer.cpp"

#ifndef MPC
#define MPC

namespace nap {

    class ModelPredictiveControl : public Plant, public Optimizer
    {
    private:
        // VARIABLES:
        int horz_length;
        int knot_length;
        MatrixXd xinit;

    protected:
        // PROTECTED MEMBER FUNCTIONS:
        void setInitialCondition(MatrixXd x0);
        MatrixXd costPrediction(MatrixXd ulist);

    public:
        // CONSTRUCTORS:
        ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd));
        ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd), const int &P, const int &k);

        // SET MEMBERS:
        void setHorizonLength(const int &P);
        void setKnotLength(const int &k);

        // MEMBER FUNCTIONS:
        MatrixXd statePrediction(MatrixXd x0, MatrixXd ulist);
        MatrixXd costPrediction(MatrixXd x0, MatrixXd ulist);
        MatrixXd solve(MatrixXd x0, MatrixXd uinit);
    };

}

#endif
