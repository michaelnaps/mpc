
#include "Plant.h"

namespace nap {
// HELPER FUNCTIONS:
    MatrixXd TaylorMethod(MatrixXd (*f)(MatrixXd, MatrixXd), MatrixXd x, MatrixXd u, double dt)
    {
        return x + dt*f( x, u );
    }

// CONSTRUCTORS:
    // Input(s):
    //      f: Model function.
    // Default variables:
    //      time_step = 1e-3
    //      model_type = "discrete"
    Plant::Plant(MatrixXd (*f)(MatrixXd, MatrixXd)) : prop(f), model_type("discrete"), time_step(1e-3)
    {
        model = f;
        prop = model;
    }

    // Input(s):
    //      f: Model function.
    //      model_type: Either continuous or discrete.
    //      time_step: Length of time-step.
    Plant::Plant(MatrixXd (*f)(MatrixXd, MatrixXd), const std::string &type, const double &dt) : prop(f)
    {
        model = f;
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
