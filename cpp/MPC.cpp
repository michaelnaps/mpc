
#include "MPC.h"

namespace nap
{
    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd)) : Plant(f), Optimizer(costHorizon), horz_length(10), knot_length(1)
    {
        costWindow = g;
    }

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
        const int N(init_cond.rows());
        MatrixXd xlist(N,horz_length);

        // Simulate over horizon using plant class.
        xlist.col(0) = xinit;
        for (int i(0); i < horz_length-1; ++i) {
            xlist.col(i+1) = prop( xlist.col(i), ulist.col(i) );
        }

        // Return prediction list.
        return xlist;
    }

    MatrixXd ModelPredictiveControl::costHorizon(MatrixXd ulist)
    {
        // Dimensions of prediction horizon.
        const int N(init_cond.rows());
        MatrixXd x(N,horz_length);

        // Simulate over prediction horizon.
        x = statePrediction( init_cond, ulist );

        // Calculate cost at each horizon window.
        MatrixXd g(1,1); g(0,0) = 0;
        for (int i(0); i < horz_length; ++i) {
            // cout << x.col(i).transpose() << endl;
            g += costWindow( x.col(i) );
        }

        // Return predicted cost.
        return g;
    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd xinit, MatrixXd ulist)
    {
        // Set initial conditions.
        setInitialConditions( xinit );

        // Calculate and return cost of horizon.
        return costHorizon( ulist );
    }

    MatrixXd ModelPredictiveControl::solveMPC(MatrixXd xinit, MatrixXd ulist)
    {
        // Get dimensions of optimization problem.
        const int N(xinit.rows()), M(ulist.rows());

        // Set initial conditions.
        setInitialConditions( xinit );

        // Vectorize list of control inputs.
        MatrixXd uvect(N*M,1);  uvect = ulist.reshaped(N*M,1);

        // // Solve optimization problem from initial guess.
        // ufinal = solve()
        return uvect;
    }
}