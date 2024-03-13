
#include "MPC.h"

namespace nap
{
    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*g)(const MatrixXd &)):
    mvar(f), cost(g) {}

    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*g)(const MatrixXd &), const int &P, const int &k):
    mvar(f), cost(g), horz_length(P), knot_length(k) {}

    void ModelPredictiveControl::setHorizonLength(const int &P)
    {
        horz_length = P;
    }

    void ModelPredictiveControl::setKnotLength(const int &k)
    {
        knot_length = k;
    }

    void ModelPredictiveControl::setInitialConditions(const MatrixXd &xinit)
    {
        init_cond = xinit;
    }

    MatrixXd ModelPredictiveControl::statePrediction(const MatrixXd &xinit, const MatrixXd &ulist)
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

    MatrixXd ModelPredictiveControl::costHorizon(const MatrixXd &ulist)
    {
        // Dimensions of state space.
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

    MatrixXd ModelPredictiveControl::costPrediction(const MatrixXd &xinit, const MatrixXd &ulist)
    {
        // Set initial conditions.
        setInitialConditions(xinit);

        // Calculate and return cost of horizon.
        return costHorizon(ulist);
    }

    MatrixXd ModelPredictiveControl::solve(const MatrixXd &xinit, const MatrixXd &uinit)
    {
        // Get dimensions of optimization problem.
        const int N = xinit.rows(), M = uinit.rows();

        // Set initial conditions.
        setInitialConditions(xinit);

        // Vectorize list of control inputs.
        MatrixXd uvect(N*M,1);  uvect = uinit.reshaped(N*M,1);

        // Solve optimization problem from initial guess.
        // Optimizer ovar(costHorizon);
        // MatrixXd ufinal(N*M,1);  ufinal = ovar.solve(uvect);

        // Return solution after reshape.
        // return ufinal.reshaped(M,horz_length).eval();
        return uvect;
    }
}