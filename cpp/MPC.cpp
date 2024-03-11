
#include "MPC.h"

namespace nap
{
    ModelPredictiveControl::ModelPredictiveControl(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd (*g)(MatrixXd)) : Plant(f), Optimizer(g), P(10), k(1)
    {
    }

    MatrixXd ModelPredictiveControl::statePrediction(MatrixXd x0, MatrixXd ulist)
    {

    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd ulist)
    {
        // Dimensions of prediction horizon.
        const int N(xinit.rows());
        MatrixXd x(N,horz_length);

        // Simulate over prediction horizon.
        x = statePrediction(xinit, ulist);

        // Calculate cost at each horizon window.
        MatrixXd g(1,1) = 0;
        for (int i(0); i < horz_length; ++i) {
            g += cost( x.col(i) );
        }

        // Return predicted cost.
        return g;
    }

    MatrixXd ModelPredictiveControl::costPrediction(MatrixXd x0, MatrixXd ulist)
    {
        // Set initial conditions and solve for cost of horizon.
        setInitialConditions( x0 );
        return costPrediction( ulist );
    }
}