
#include "Plant.h"

namespace nap {
    MatrixXd TaylorMethod(MatrixXd (*F)(MatrixXd, MatrixXd), MatrixXd x, MatrixXd u, double dt)
    {
        return x + dt*F( x, u );
    }

    // CONSTRUCTORS:
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd)) : time_step(1e-3), model_type("continuous")
    {
        model = F;
    }
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt) : model_type("continuous")
    {
        model = F;
        time_step = dt;
    }
    Plant::Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt, const std::string& type)
    {
        model = F;
        time_step = dt;
        model_type = type;
    }

    // MEMBER FUNCTIONS:
    MatrixXd Plant::prop(MatrixXd x, MatrixXd u)
    {
        return model( x, u );
    }
}
