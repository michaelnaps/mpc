
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
    Cost::Cost(MatrixXd (*g)(const MatrixXd &)) : step_size(1e-6)
    {
        cost = g;
    }

    Cost::Cost(MatrixXd (*g)(const MatrixXd &), const double &h)
    {
        cost = g;
        step_size = h;
    }

    MatrixXd Cost::gradient(const MatrixXd &x)
    {
        return fdm2c(cost, x, step_size);
    }

// Class: Optimizer()
    Optimizer::Optimizer(MatrixXd (*g)(const MatrixXd &)) : Cost(g), max_iter(1000), epsilon(1e-3), alpha(0.1), method("ngd") {}

    Optimizer::Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n) : Cost(g), max_iter(n), epsilon(1e-3), alpha(0.1), method("ngd") {}

    Optimizer::Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e) : Cost(g), max_iter(n), epsilon(e), alpha(0.1), method("ngd") {}

    Optimizer::Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e, const double &a) : Cost(g), max_iter(n), epsilon(e), alpha(a), method("ngd") {}

    Optimizer::Optimizer(MatrixXd (*g)(const MatrixXd &), const int &n, const double &e, const double &a, const string &type) : Cost(g), max_iter(n), epsilon(e), alpha(a), method(type) {}

    MatrixXd Optimizer::step(const MatrixXd &x, const MatrixXd &dg)
    {
        // TODO: Alternative step methods.
        return x - alpha*dg;  // Nonlinear gradient descent (ngd).
    }

    MatrixXd Optimizer::solve(const MatrixXd &x0)
    {
        // Objective dimensions.
        const int N = x0.rows();
        MatrixXd x(N,1);  x = x0;
        MatrixXd dg(N,1);  dg = gradient( x );

        // Optimization loop.
        int n = 0;
        while (dg.norm() > epsilon) {
            // Calculate new state and gradient.
            x = step( x, dg );
            dg = gradient( x );

            // If maximum number of iterations reached.
            if (n == max_iter) {
                cout << "Iteration break." << endl;
                break;
            }
            n += 1;
        }

        // Return solution.
        return x;
    }
}
