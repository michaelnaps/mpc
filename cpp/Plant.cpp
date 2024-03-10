
#include "Plant.h"

namespace nap {
    MatrixXd TaylorMethod(MatrixXd (*F)(MatrixXd, MatrixXd), MatrixXd x, MatrixXd u, double dt)
    {
        return x + dt*F( x, u );
    }

    // CONSTRUCTORS:

    // Input(s):
    //      F: Model function.
    // Default variables:
    //      time_step = 1e-3
    //      model_type = "discrete"
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd)) : model_type("discrete"), time_step(1e-3)
    {
        model = F;
    }

    // Input(s):
    //      F: Model function.
    //      model_type: Either continuous or discrete.
    //      time_step: Length of time-step.
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const std::string& type, const double& dt)
    {
        model = F;
        model_type = type;
        time_step = dt;
    }

    // MEMBER FUNCTIONS:
    MatrixXd Plant::prop(MatrixXd x, MatrixXd u)
    {
        return model( x, u );
    }
}
