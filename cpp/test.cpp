
#include "Plant.cpp"
#include "Optimizer.cpp"

MatrixXd model(const MatrixXd &x, const MatrixXd &u)
{
    MatrixXd M(3,3);
    M << 1, 1, 2,
         2, 1, 0,
         0.5, 1, 0;
    return 0.1*M*x + 0.005*u;
}

MatrixXd cost(const MatrixXd &x)
{
    return x.transpose()*x;
}

int main()
{
    nap::Plant mvar( model );
    nap::Optimizer ovar( cost, 1000, 1e-12, 0.1, "discrete" );
    MatrixXd x( 3, 1 );  x << 100, 55, 243;
    cout << x.transpose() << '\n' << mvar.prop( x, x ).transpose() << endl;
    cout << ovar.solve( x ).transpose() << endl;
    // MatrixXd uinit( 3,10 );  uinit << Eigen::ArrayXXd::Zero(3,10);
    // cout << mpcvar.solve( x, uinit ) << endl;
}