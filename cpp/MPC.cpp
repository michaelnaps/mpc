
#include "MPC.h"

namespace nap
{
    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd)) : mvar(f), cost(g) {}

    void ModelPredictiveControl::setInitialConditions(MatrixXd xinit)
    {
        // Set initial conditions.
        init_cond = xinit;

        // Return nothing.
        return;
    }

    MatrixXd ModelPredictiveControl::statePrediction(MatrixXd xinit, MatrixXd ulist)
    {
        // Initialize state prediction list.
        const int N = init_cond.rows();
        MatrixXd xlist(N,horz_length);

        // Simulate over horizon using plant class.
        xlist.col(0) = xinit;
        for (int i(0); i < horz_length-1; ++i) {
            xlist.col(i+1) = mvar.prop(xlist.col(i), ulist.col(i));
        }

        // Return prediction list.
        return xlist;
    }

    MatrixXd ModelPredictiveControl::costHorizon(MatrixXd ulist)
    {
        // Check dimensions of ulist: if vector, reshape properly.
        int M = ulist.rows(), P = ulist.cols();
        if ((double) P/horz_length - 1. < TOL) {
            M = round((double) M/horz_length);
            ulist = ulist.reshaped(M,horz_length).eval();
        }

        // Dimensions of prediction horizon.
        const int N = init_cond.rows();

        // Simulate over horizon.
        MatrixXd xlist(N,horz_length);
        xlist = statePrediction(init_cond, ulist);

        // Calculate cost at each horizon window.
        MatrixXd g(1,1); g(0,0) = 0;
        for (int i(0); i < horz_length; ++i) {
            g += cost(xlist.col(i));
        }

        // Return predicted cost.
        return g;
    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd xinit, MatrixXd ulist)
    {
        // Set initial conditions.
        setInitialConditions(xinit);

        // Calculate and return cost of horizon.
        return costHorizon(ulist);
    }

    MatrixXd ModelPredictiveControl::solve(MatrixXd xinit, MatrixXd uinit)
    {
        // Get dimensions of optimization problem.
        const int N = xinit.rows(), M = uinit.rows();

        // Set initial conditions.
        this->setInitialConditions(xinit);

        // Vectorize list of control inputs.
        MatrixXd uvect(N*M,1);  uvect = uinit.reshaped(N*M,1);

        // Solve optimization problem from initial guess.
        Optimizer ovar(costHorizon);
        MatrixXd ufinal(N*M,1);  ufinal = ovar.solve(uvect);

        // Return solution after reshape.
        return ufinal.reshaped(M,horz_length).eval();
    }
}