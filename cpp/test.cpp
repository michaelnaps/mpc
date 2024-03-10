#include <iostream>
#include <eigen3/Eigen/Dense>
#include "Plant.cpp"

using Eigen::MatrixXd;
using namespace std;

MatrixXd model(MatrixXd x, MatrixXd u)
{
    MatrixXd M(3,3);
    M << 1, 1, 2,
         2, 1, 0,
         0.5, 1, 0;
    return 0.1*M*x;
}

int main()
{
    nap::Plant P( model );
    MatrixXd x0( 3, 1 ); x0 << 2, 2, 0.75;
    MatrixXd uX( 3, 1 );

    // Simulation length.
    double T = 10;  double dt = 1;
    int Nt = T/dt;

    MatrixXd x(3,1);  x = x0;
    for(int i(0); i < Nt; ++i)
    {
        x = P.prop( x, uX );
        cout << x.transpose() << endl;
    }
}