import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d

from mpc import *


class Parameters:
    def __init__(self, fig=None, axes=None):
        if fig is None or axes is None:
            self.fig = plt.figure();
            self.axs = plt.axes(projection='3d');
        else:
            self.fig = fig;
            self.axs = axes;

        self.pause = 0.001;


def model(q, u=None, params=None):
    x = q[0];  sig = 10;
    y = q[1];  rho = 28;
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
    params.axs.set_xlim3d(-20, 20);
    params.axs.set_ylim3d(-25, 25);
    params.axs.set_zlim3d(0, 50);
    params.fig.tight_layout();

    if q is not None:
        npq = np.array(q);
        params.axs.plot(npq[:,0], npq[:,1], npq[:,2]);
        plt.show();

    return params;

def callback(mpc_var, t, q, ulist):
    mpc_var.params.axs.plot([q[0]], [q[1]], [q[2]],
        color='b', marker='.', markersize=2);
    mpc_var.params.axs.set_title("time: %.3f" % t);

    plt.show(block=0);
    plt.pause(mpc_var.params.pause);

    return Parameters(mpc_var.params.fig, mpc_var.params.axs);


if __name__ == "__main__":
    # MPC class variables
    params = Parameters();
    params = plot(None,None,params);

    num_inputs = 3;
    PH_length = 10;
    num_ssvar = 3;
    time_step = 0.01;
    model_type = 'continuous';

    mpc_var = ModelPredictiveControl('nno', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=1,
        time_step=time_step, model_type=model_type);

    # state initialization
    q0 = [1, 1, 1];
    uinit = [0 for i in range(num_inputs*PH_length)];

    # simulate lorenz
    sim_time = 30;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit,
        callback=callback, output=0)[0:3];

    # plot(T, qlist, params);
    # plt.show();
