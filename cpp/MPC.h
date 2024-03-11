
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
        MatrixXd init_cond;

    protected:
        // VARIABLES:
        MatrixXd (*costWindow)(MatrixXd);

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
        MatrixXd solveMPC(MatrixXd xinit, MatrixXd uinit);
    };

}

#endif
