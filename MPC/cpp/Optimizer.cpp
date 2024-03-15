
#include "Optimizer.h"

namespace nap
{
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

// Class: Cost()
    Cost::Cost(MatrixXd (*g)(const MatrixXd &)):
        Cost(g, 1000) {}

    Cost::Cost(MatrixXd (*g)(const MatrixXd &), const int &n):
        Cost(g, n, 0.1) {}

    Cost::Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a):
        Cost(g, n, a, 1e-6) {}

    Cost::Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a, const double &h):
        Cost(g, n, a, h, 1e-6) {}

    Cost::Cost(MatrixXd (*g)(const MatrixXd &), const int &n, const double &a, const double &h, const double &e):
        cost(g), max_iter(n), alpha(a), step_size(h), epsilon(e) {}

    MatrixXd Cost::gradient(const MatrixXd &x)
    {
        return fdm2c(cost, x, step_size);
    }

    MatrixXd Cost::step(const MatrixXd &x, const MatrixXd &dg)
    {
        return x - alpha*dg;
    }

    MatrixXd Cost::solve(const MatrixXd &xinit)
    {
        // Initialize iteration variable at x0.
        MatrixXd x = xinit;
        MatrixXd dg = gradient( x );

        // Optimization loop.
        int n = 0;
        while (dg.norm() > epsilon)
        {
            x = step( x, dg );
            dg = gradient( x );

            n += 1;
            if (n == max_iter) {
                std::cout << "WARNING: Iteration break executed." << std::endl;
                break;
            }
        }

        // Return minimum of cost function.
        return x;
    }
}
