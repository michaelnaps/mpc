
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
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd)) : model_type("discrete"), time_step(1e-3)
    {
        model = F;
        prop = model;
    }

    // Input(s):
    //      F: Model function.
    //      model_type: Either continuous or discrete.
    //      time_step: Length of time-step.
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const std::string &type, const double &dt)
    {
        model = F;
        model_type = type;
        time_step = dt;
        if (model_type == "continuous")
        {
            prop = &templateModel;
        }
        else if (model_type == "discrete")
        {
            prop = model;
        }
    }

// ACCESSOR FUNCTIONS:
    std::string Plant::getModelType()
    {
        return model_type;
    }
    double Plant::getTimeStep()
    {
        return time_step;
    }

// MEMBER FUNCTIONS:
    MatrixXd Plant::templateModel(MatrixXd x, MatrixXd u)
    {
        return TaylorMethod( model, x, u, time_step );
    }
}
