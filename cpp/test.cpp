#include <iostream>
#include <eigen3/Eigen/Dense>

using Eigen::MatrixXd;
using namespace std;

int main()
{
    MatrixXd M(2,2);
    MatrixXd x(2,1);

    // Matrix init.
    M(0,0) = 3;
    M(1,0) = 2.5;
    M(0,1) = -1;
    M(1,1) = M(1,0) + M(0,1);

    // Vector init.
    x(0,0) = 1;
    x(1,0) = 2;

    // Output.
    cout << M << endl;
    cout << x << endl;
    cout << M*x << endl;
}