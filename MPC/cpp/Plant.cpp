
// #include <iostream>
#include "Helpers.h"
#include "Plant.h"
// #include <Eigen/Dense>
using Eigen::MatrixXd;

// CONSTRUCTORS:
// Input(s):
//      f: Model function.
// Default variables:
//      time_step = 1e-3
//      model_type = "discrete"
Plant::Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &)):
    model(f),
    model_type("discrete"),
    time_step(1e-3),
    mt('d') {}

// Input(s):
//      f: Model function.
//      model_type: Either continuous or discrete.
//      time_step: Length of time-step.
Plant::Plant(MatrixXd (*f)(const MatrixXd &, const MatrixXd &), const std::string &type, const double &dt):
    model(f),
    model_type(type),
    time_step(dt),
    mt(type[0]) {}

// ACCESSOR FUNCTIONS:
// Function: Plant.getModelType()
// Output: Model type (continuous/discrete).
std::string Plant::getModelType()
{
    return model_type;
}

// Function: Plant.getTimeStep()
// Output: Time step of plant model (double).
double Plant::getTimeStep()
{
    return time_step;
}

// MEMBER FUNCTIONS:
// Function: Plant.prop()
// Input(s):
//      x: State of plant at time, t.
//      u: User-input (control term).
// Output(s): Next step of plant simulation.
MatrixXd Plant::prop(const MatrixXd &x, const MatrixXd &u)
{
    if (mt == 'c') {
        return TaylorMethod(model, x, u, time_step);
    }
    return model(x, u);
}
}
