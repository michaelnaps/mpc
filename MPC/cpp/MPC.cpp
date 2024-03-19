
#include "MPC.h"

namespace nap
{
    MPC::MPC(const Plant &f, const Cost &gx, const Cost &gu)
        PredictiveCost(f, gx, gu) {}
}