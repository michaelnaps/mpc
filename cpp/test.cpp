
#include "MPC.cpp"

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
    return 0.1*M*x + u;
}

int main()
{
    nap::Optimizer g( cost );
    MatrixXd x( 3, 1 );  x << 100, 55, 243;
    cout << g.solve( x ) << endl;
}