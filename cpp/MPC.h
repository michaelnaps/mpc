
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
        MatrixXd (*cost)(const MatrixXd &);

        // PROTECTED MEMBER FUNCTIONS:
        void setInitialConditions(const MatrixXd &xinit);
        MatrixXd costHorizon(const MatrixXd &ulist);

    public:
        // CONSTRUCTORS:
        ModelPredictiveControl(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*g)(const MatrixXd &));
        ModelPredictiveControl(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*g)(const MatrixXd &), const int &P, const int &k);

        // SET MEMBERS:
        void setHorizonLength(const int &P);
        void setKnotLength(const int &k);

        // MEMBER FUNCTIONS:
        MatrixXd statePrediction(const MatrixXd &xinit, const MatrixXd &ulist);
        MatrixXd costPrediction(const MatrixXd &xinit, const MatrixXd &ulist);
        MatrixXd solve(const MatrixXd &xinit, const MatrixXd &uinit);
    };

}

#endif
