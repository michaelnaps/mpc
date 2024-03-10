
#include "Plant.h"

namespace nap {
    MatrixXd TaylorMethod(
        MatrixXd (*F)(MatrixXd, MatrixXd),
        MatrixXd x, MatrixXd u, double dt)
        {
            return x + dt*F( x, u );
        }

    class Plant
    {
    private:
        double dt;
        std::string model_type;

    protected:
        MatrixXd (*model)(MatrixXd, MatrixXd);

    public:
        // CONSTRUCTORS:
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd));
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt);
        Plant(MatrixXd (*F)(MatrixXd, MatrixXd), const double& dt, const std::string& model_type);

        // ACCESSOR FUNCTIONS:
        double getTimeStep();
        std::string getModelType();

        // MEMBER FUNCTIONS:
        Plant::prop(MatrixXd x, MatrixXd u)
        {
            return model( x, u )
        }
    };
}

#endif