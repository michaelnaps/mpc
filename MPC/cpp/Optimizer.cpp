
#include "Helpers.h"
#include "Optimizer.h"

namespace nap
{
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
        return fdm2c( cost, x, step_size );
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

// Class: ModelCost()
    ModelCost::ModelCost(const Plant &f, const Cost &gx, const Cost &gu):
        mvar(f), costx(gx), costu(gu) {}

    ModelCost::ModelCost(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), MatrixXd (*gx)(const MatrixXd &), MatrixXd (*gu)(const MatrixXd &)):
        mvar(f), costx(gx), costu(gu) {}

    MatrixXd ModelCost::cost(const MatrixXd &x, const MatrixXd &u)
    {
        return costx.cost(x) + costu.cost(u);
    }

    MatrixXd ModelCost::xgradient(const MatrixXd &x)
    {
        return fdm2c(costx.cost, x, costx.step_size);
    }

    MatrixXd ModelCost::ugradient(const MatrixXd &u)
    {
        return fdm2c(costu.cost, u, costu.step_size);
    }

    MatrixXd ModelCost::gradient(const MatrixXd &x, const MatrixXd &u)
    {
        return xgradient( x ) + ugradient( u );
    }

// Class: PredictiveCost()
    PredictiveCost::PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu):
        PredictiveCost(f, gx, gu, 10, 1) {}

    PredictiveCost::PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P):
        PredictiveCost(f, gx, gx, P, 1) {}

    PredictiveCost::PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P, const int &k):
        ModelCost(f, gx, gx),
        horz_length(P),
        knot_length(k) {}

    MatrixXd PredictiveCost::prediction(const MatrixXd &xinit, const MatrixXd &ulist)
    {
        // Dimensions of simulation.
        const int N = xinit.rows();

        // Check that ulist is properly dimensioned.
        if (ulist.cols() != horz_length) {
            std::cout << "ERROR: 'ulist' is not properly dimensioned." << std::endl;
            return MatrixXd::Zero(N, horz_length+1);
        }

        // Simulation set and initial conditions.
        MatrixXd xlist(N,horz_length+1);
        xlist.col(0) = xinit;

        // Simulation loop.
        for (int i(0); i < horz_length; ++i) {
            xlist.col(i+1) = mvar.prop(xlist.col(i), ulist.col(i));
        }

        // Return simulation set.
        return xlist;
    }

    MatrixXd PredictiveCost::cost(const MatrixXd &xinit, const MatrixXd &ulist)
    {
        // Check that ulist is properly dimensioned.
        MatrixXd C(1,1);  C << -1;
        if (ulist.cols() != horz_length) {
            std::cout << "ERROR: 'ulist' is not properly dimensioned." << std::endl;
            return C;
        }

        // Get dimension and simulation set.
        const int N = xinit.rows();
        MatrixXd xlist = prediction(xinit, ulist);

        // Iterate through list summing cost.
        C << 0;
        for (int i(0); i < horz_length; ++i) {
            C += cost(xlist.col(i), ulist.col(i));
        }

        // Cost of final state (assumed zero input).
        C += cost(xlist.col(horz_length), MatrixXd::Zero(N,1));

        // Return cumulative cost.
        return C;
    }

    // TODO: Function is messy. I would like it to be more generalized.
    MatrixXd PredictiveCost::ugradient(const MatrixXd &xinit, const MatrixXd &ulist)
    {
        // Initialization of gradient vector.
        const int M = ulist.rows();

        // Manually calculate gradient vector.
        MatrixXd up(M,horz_length);  MatrixXd Cp(M,horz_length);
        MatrixXd un(M,horz_length);  MatrixXd Cn(M,horz_length);
        for (int i(0); i < M; ++i) {
            for (int j(0); j < horz_length; ++j) {
                up = ulist;  up(i,j) = ulist(i,j) + costu.step_size;
                un = ulist;  un(i,j) = ulist(i,j) - costu.step_size;

                Cp(i,j) = cost(xinit, up)(0,0);
                Cn(i,j) = cost(xinit, un)(0,0);
            }
        }

        // Return gradient (as matrix).
        return (Cp - Cn)/(2*costu.step_size);
    }
}
