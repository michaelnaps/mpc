import sys
sys.path.insert(0, '../.');

import numpy as np
from mpc import *

def model(q, u, _):
    l = 1;
    m = 1;
    c = 1;
    g = 9.81;

    dq = [
        q[1],
        u[0] - g/l*np.cos(q[0]) - c*q[1]
    ];

    return dq;

def cost(mpc_var, q, u, params):
    qd = [np.pi/2, 0];
    C = (q[2][0] - qd[0])**2
    return C;

if __name__ == "__main__":
    PH_length = 5;
    mpc_var = ModelPredictiveControl('nno', cost, model, 'no params', 1, 1, PH_length);

    q0 = [0, 0];
    uinit = [0 for i in range(PH_length)];

    uList = mpc_var.solve(q0, uinit, output=1);

    print(mpc_var.simulate(q0, uList));
