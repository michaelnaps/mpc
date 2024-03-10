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
    return M*x;
}

int main()
{
    nap::Plant P( model );
    MatrixXd x0( 3, 1 ); x0 << 2, 2, 0.75;
    MatrixXd uX( 3, 1 );
    cout << P.prop( x0, uX ) << endl;
}