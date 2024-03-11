
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "Plant.cpp"
#include "Optimizer.cpp"

using Eigen::MatrixXd;
using namespace std;

MatrixXd cost(MatrixXd x)
{
    return x.transpose()*x;
}

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
    nap::Cost g( cost );
    MatrixXd x( 3, 1 ); x << 2, 2, 0.75;
    cout << g.gradient( x ) << endl;
}