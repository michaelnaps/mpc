
#include "Plant.cpp"
#include "Optimizer.cpp"

#ifndef MPC
#define MPC

namespace nap {

    class MPC: private PredictiveCost
    {
    private:
    protected:
    public:
        // CONSTRUCTORS:
        MPC(const Plant &f, const Cost &gx, const Cost &gu);
        PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P);
        PredictiveCost(const Plant &f, const Cost &gx, const Cost &gu, const int &P, const int &k);
    }
}

#endif
