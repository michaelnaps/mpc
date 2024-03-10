#include <iostream>
#include <eigen3/Eigen/Dense>
#include "Plant.cpp"

using Eigen::MatrixXd;
using namespace std;

MatrixXd squared_addition(
    MatrixXd x,
    MatrixXd y,
    MatrixXd (*add)(MatrixXd, MatrixXd))
{
    return add(x.transpose()*x, y.transpose()*y);
}

MatrixXd addition(MatrixXd x, MatrixXd y)
{
    return x + y;
}

int main()
{
    MatrixXd M(2,2);
    MatrixXd x(2,1);
    nap::Plant P;

    // Matrix init.
    M(0,0) = 3;
    M(1,0) = 2.5;
    M(0,1) = -1;
    M(1,1) = M(1,0) + M(0,1);

    // Vector init.
    x << 1, 2.5;

    // Output.
    cout << M << endl;
    cout << x << endl;
    cout << M*x << endl;
    cout << "------" << endl;

    // Testing function pass.
    MatrixXd y(2,1);
    y << 1, 2;
    cout << squared_addition(x, y, addition) << endl;
}