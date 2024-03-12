
#include "Plant.cpp"
#include "Optimizer.cpp"

#ifndef MPC
#define MPC

namespace nap {

    class ModelPredictiveControl
    {
    private:
        // DEFAULT VARIABLES/CONSTANTS:
        const double TOL = 1e-12;
        int horz_length = 10;
        int knot_length = 1;

        // FREE VARIABLES:
        Plant mvar;
        MatrixXd init_cond;

    protected:
        // VARIABLES:
        MatrixXd (*cost)(MatrixXd);

        // PROTECTED MEMBER FUNCTIONS:
        void setInitialConditions(MatrixXd xinit);
        MatrixXd costHorizon(MatrixXd ulist);

    public:
        // CONSTRUCTORS:
        ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd));
        ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd), const int &P, const int &k);

        // SET MEMBERS:
        void setHorizonLength(const int &P);
        void setKnotLength(const int &k);

        // MEMBER FUNCTIONS:
        MatrixXd statePrediction(MatrixXd xinit, MatrixXd ulist);
        MatrixXd costPrediction(MatrixXd xinit, MatrixXd ulist);
        MatrixXd solve(MatrixXd xinit, MatrixXd uinit);
    };

}

#endif
