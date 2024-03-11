
#include "MPC.h"

namespace nap
{
    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd)) : Plant(f), Optimizer(g), horz_length(10), knot_length(1) {}

    void ModelPredictiveControl::setInitialConditions(MatrixXd x0)
    {
        // Set initial conditions.
        xinit = x0;

        // Return nothing.
        return;
    }

    MatrixXd ModelPredictiveControl::statePrediction(MatrixXd x0, MatrixXd ulist)
    {
        // Initialize state prediction list.
        const int N(xinit.rows());
        MatrixXd xlist(N,horz_length);

        // Simulate over horizon using plant class.
        xlist.col(0) = x0;
        for (int i(0); i < horz_length-1; ++i) {
            xlist.col(i+1) = prop( xlist.col(i), ulist.col(i) );
        }

        // Return prediction list.
        return xlist;
    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd ulist)
    {
        // Dimensions of prediction horizon.
        const int N(xinit.rows());
        MatrixXd x(N,horz_length);

        // Simulate over prediction horizon.
        x = statePrediction(xinit, ulist);

        // Calculate cost at each horizon window.
        MatrixXd g(1,1); g(0,0) = 0;
        for (int i(0); i < horz_length; ++i) {
            g += cost( x.col(i) );
        }

        // Return predicted cost.
        return g;
    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd x0, MatrixXd ulist)
    {
        // Set initial conditions.
        setInitialConditions( x0 );

        // Calculate and return cost of horizon.
        return costPrediction( ulist );
    }
}