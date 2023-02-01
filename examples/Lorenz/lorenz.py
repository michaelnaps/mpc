import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

import numpy as np

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as plt3d
import matplotlib.patches as patch
import matplotlib.path as path


from mpc import *


class Parameters:
    def __init__(self, fig=None, axes=None, trail=None, event=None):
        if fig is None:
            self.fig = plt.figure();
        else:
            self.fig = fig;

        if axes is None:
            self.axs = plt.axes(projection='3d');
        else:
            self.axs = axes;

        if trail is None:
            self.trail = None;
        else:
            self.trail = trail;

        if event is None:
            self.event = None;
        else:
            self.event = event;

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

    if T is not None:
        npq = np.array(q);
        params.axs.plot(npq[:,0], npq[:,1], npq[:,2]);
        plt.show();
    else:  # if T=None then q=q0
        q0list = np.kron(np.ones( (25,1) ), q);

        params.trail = (patch.PathPatch(path.Path(q0list[:,0:2])), q0list[:,2], q0list);
        params.axs.add_patch(params.trail[0]);
        plt3d.art3d.pathpatch_2d_to_3d(params.trail[0], z=params.trail[1]);

        params.event = q;

    return params;

def callback(mpc_var, t, q, ulist):
    # mpc_var.params.axs.plot([q[0]], [q[1]], [q[2]],
    #     color='b', marker='.', markersize=2);
    mpc_var.params.axs.set_title("time: %.3f" % t);

    mpc_var.params.trail[0].remove();

    trailxyz = mpc_var.params.trail[2];
    trailxyz[:-1] = trailxyz[1:];  trailxyz[-1] = q;

    mpc_var.params.trail = (
        patch.PathPatch(path.Path(trailxyz[:,:2])), trailxyz[:,2], trailxyz
    );

    mpc_var.params.axs.add_patch(mpc_var.params.trail[0]);
    plt3d.art3d.pathpatch_2d_to_3d(mpc_var.params.trail[0], z=mpc_var.params.trail[1]);

    plt.show(block=0);
    plt.pause(mpc_var.params.pause);

    return Parameters(
        mpc_var.params.fig, mpc_var.params.axs, mpc_var.params.trail
    );


if __name__ == "__main__":
    # state initialization
    q0 = [1, 1, 1];

    # MPC class variables
    params = Parameters();
    params = plot(None, q0, params);

    num_inputs = 3;
    PH_length = 10;
    num_ssvar = 3;
    time_step = 0.05;
    model_type = 'continuous';

    mpc_var = ModelPredictiveControl('nno', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=1,
        time_step=time_step, model_type=model_type);

    # simulate lorenz
    uinit = [0 for i in range(num_inputs*PH_length)];

    sim_time = 30;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit,
        callback=callback, output=0)[0:3];

    # plot(T, qlist, params);
    plt.show();
