
#include "Plant.cpp"
#include "Optimizer.cpp"

MatrixXd cost(const MatrixXd &x)
{
    return x.transpose()*x;
}

MatrixXd model(const MatrixXd &x, const MatrixXd &u)
{
    MatrixXd M(3,3);
    M << 1, 1, 2,
         2, 1, 0,
         0.5, 1, 0;
    return 0.1*M*x + 0*u;
}

int main()
{
    nap::Plant mvar( model );
    nap::Optimizer ovar( cost );
    MatrixXd x( 3, 1 );  x << 100, 55, 243;
    cout << x.transpose() << '\n' << mvar.prop( x, x ).transpose() << endl;
    cout << ovar.solve( x ) << endl;
    // MatrixXd uinit( 3,10 );  uinit << Eigen::ArrayXXd::Zero(3,10);
    // cout << mpcvar.solve( x, uinit ) << endl;
}