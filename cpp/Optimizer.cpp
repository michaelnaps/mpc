
#include "Optimizer.h"

namespace nap
{
    // HELPER FUNCTIONS:
    // TODO: This is a mess.
    MatrixXd offset(MatrixXd x, const double &h)
    {
        const int N(x.rows()), M(x.cols());
        MatrixXd y(N,M);
        for (int i(0); i < N; ++i) {
            for (int j(0); j < M; ++j) {
                y(i,j) = x(i,j) + h;
            }
        }
        return y;
    }

    MatrixXd fdm2c(MatrixXd (*obj)(MatrixXd), MatrixXd x, const double &h)
    {
        // State and cost dimensions.
        const int N = x.rows();
        const int M = obj(x).rows();

        // Calculate derivative at each input.
        MatrixXd g(N,M);
        MatrixXd xn1(N,1), xp1(N,1);
        MatrixXd gn1(M,1), gp1(M,1);
        for (int i(0); i < N; ++i) {
            xn1 = x;  xn1.row(i) = offset(xn1.row(i), -h);
            xp1 = x;  xp1.row(i) = offset(xp1.row(i),  h);

            gn1 = obj( xn1 );
            gp1 = obj( xp1 );

            g.row(i) = (gp1 - gn1)/(2*h);
        }

        // Return gradient of objective function.
        return g;
    }

    Cost::Cost(MatrixXd (*g)(MatrixXd)) : step_size(1e-6)
    {
        cost = g;
    }

    Cost::Cost(MatrixXd (*g)(MatrixXd), const double &h)
    {
        cost = g;
        step_size = h;
    }

    MatrixXd Cost::gradient(MatrixXd x)
    {
        return fdm2c(cost, x, step_size);
    }

    // // TODO: Resolve non-static function error message.
    // MatrixXd Cost::hessian(MatrixXd x)
    // {
    //     return fdm2c(gradient, x, step_size);
    // }
}