import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

import numpy as np
import matplotlib.pyplot as plt

from mpc import *

def model(q, u, _):
    l = 1;
    m = 1;
    c = 1;
    g = 9.81;
    # dt = 0.025;

    dq = [
        q[1],
        (u[0] - g/l*np.cos(q[0]) - c*q[1])
    ];

    return dq;

def cost(mpc_var, qlist, ulist, params):
    qd = [np.pi/2, 0];
    kq = 1;
    kdq = 0.1;
    ku = 0;

    C = 0;
    for i in range(mpc_var.PH):
        C += kq*(qlist[i][0] - qd[0])**2;
        C += kdq*(qlist[i][1] - qd[1])**2;
        C += ku*ulist[i]**2;
    C += kq*(qlist[-1][0] - qd[0])**2 + kdq*(qlist[-1][1] - qd[1])**2;

    return C;

def pendulum(T, q):
    fig, axes = plt.subplots();
    axes.plot(T, q);

if __name__ == "__main__":
    params = None;
    PH_length = 10;

    mpc_var = ModelPredictiveControl('nno', model, cost, params, 1,
        num_ssvar=2, PH_length=PH_length, model_type='continuous');
    mpc_var.setMinTimeStep(1);

    q0 = [0, 0];
    uinit = [0 for i in range(PH_length)];

    sim_time = 10;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit, output=1)[0:3];

    pendulum(T, qlist);
    plt.show();
