
#include "Plant.h"

namespace nap {
// HELPER FUNCTIONS:
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
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd)) : prop(F), model_type("discrete"), time_step(1e-3)
    {
        model = F;
        prop = model;
    }

    // Input(s):
    //      F: Model function.
    //      model_type: Either continuous or discrete.
    //      time_step: Length of time-step.
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const std::string &type, const double &dt) : prop(F)
    {
        model = F;
        model_type = type;
        time_step = dt;
    }

// ACCESSOR FUNCTIONS:
    // Function: Plant.getModelType()
    // Output: Model type (string).
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
    // Function: Plant.cprop()
    // Stand-in till a better method for discrete/continuous functions.
    // Input(s):
    //      x: State of plant at time, t.
    //      u: User-input (control term).
    // Output(s): Next step of plant simulation.
    MatrixXd Plant::cprop(MatrixXd x, MatrixXd u)
    {
        return TaylorMethod( model, x, u, time_step );
    }
}
