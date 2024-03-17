
#include "Plant.cpp"
#include "Optimizer.cpp"
using namespace std;

MatrixXd model(const MatrixXd &x, const MatrixXd &u)
{
    MatrixXd M(3,3);
    M << 1, 1, 2,
         2, 1, 0,
         0.5, 1, 0;
    return 0.1*M*x + 0.05*u;
}

MatrixXd cost(const MatrixXd &x)
{
    return x.transpose()*x;
}

int main()
{
    nap::Plant mvar( model );
    MatrixXd x( 3, 1 );  x << 100, 55, 243;
    cout << x.transpose() << " -> " << mvar.prop( x, x ).transpose() << endl;

    nap::Cost cvar( cost );
    cout << cvar.solve( x ).transpose() << endl;

    MatrixXd ulist = MatrixXd::Random(3,10);
    nap::PredictiveCost pcvar( model, cost, cost, 10 );
    cout << pcvar.costPrediction(x, ulist) << endl;
}