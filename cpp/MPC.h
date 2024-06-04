
#include "Plant.cpp"
#include "Optimizer.cpp"

#ifndef MPC
#define MPC

namespace nap {

    class MPC: private PredictiveCost
    {
    private:
    protected:
    public:
        // CONSTRUCTORS:
        MPC(const Plant &f, const Cost &gx, const Cost &gu);
        MPC(const Plant &f, const Cost &gx, const Cost &gu, const int &P);
        MPC(const Plant &f, const Cost &gx, const Cost &gu, const int &P, const int &k);

        // MEMBER FUNCTIONS:
        MatrixXd solve(const MatrixXd &xinit, const MatrixXd &ulist);
    };

}

#endif
