
#include <Eigen/Dense>
using Eigen::MatrixXd;

#ifndef MPC_HELPERS
#define MPC_HELPERS

namespace nap
{
    MatrixXd TaylorMethod(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), const MatrixXd &x, const MatrixXd &u, const double &dt)
    {
        // Return linearization of continuous model.
        return x + dt*f( x, u );
    }

    MatrixXd fdm2c(MatrixXd (*obj)(const MatrixXd &), const MatrixXd &x, const double &h)
    {
        // State and cost dimensions.
        const int N = x.rows();
        const int M = obj(x).rows();

        // Calculate derivative at each input.
        MatrixXd g(N,M);
        MatrixXd xn1(N,1), xp1(N,1);
        MatrixXd gn1(M,1), gp1(M,1);
        for (int i(0); i < N; ++i) {
            xn1 = x;  xn1(i,0) = xn1(i,0) - h;
            xp1 = x;  xp1(i,0) = xp1(i,0) + h;

            gn1 = obj( xn1 );
            gp1 = obj( xp1 );

            g.row(i) = (gp1 - gn1)/(2*h);
        }

        // Return gradient of objective function.
        return g;
    }
}

#endif
