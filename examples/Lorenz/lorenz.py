import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d

from mpc import *


def model(q, u=None, params=None):
    x = q[0];  sig = 28;
    y = q[1];  rho = 10;
    z = q[2];  bet = 8/3;

    dq = [
        sig*(y - x),
        x*(rho - z) - y,
        x*y - bet*z
    ];

    return dq;

def cost(mpc_var, qlist, ulist):
    return 0;

def plot(T, q, params=None):
    fig = plt.figure();
    ax = plt3d.Axes3D(fig);

    npq = np.array(q);
    ax.plot(npq[:,0], npq[:,1], npq[:,2])

    return fig, ax;

def callback(mpc_var, t, q, ulist):
    pass;


if __name__ == "__main__":
    # MPC class variables
    params=None;
    num_inputs = 3;
    PH_length = 10;
    num_ssvar = 3;
    time_step = 0.001;
    model_type = 'continuous';

    mpc_var = ModelPredictiveControl('nno', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=1,
        time_step=0.001, model_type=model_type);

    # state initialization
    q0 = [0.01, 10, -4];
    uinit = [0 for i in range(num_inputs*PH_length)];

    # simulate lorenz
    sim_time = 100;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit, output=0)[0:3];

    plot(T, qlist);
    plt.show();
