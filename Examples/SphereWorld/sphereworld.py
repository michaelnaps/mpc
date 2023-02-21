import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

from mpc import *
from Sphere import *

import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.path as path


class Parameters:
    def __init__(self, x0, fig=None, axs=None, buffer_length=10, pause=1e-3, color='k'):
        self.pause = pause;

        if axs is None and fig is None:
            self.fig, self.axs = plt.subplots();
        else:
            self.fig = fig;
            self.axs = axs;

        # initialize buffer (trail)
        self.color = color;

        self.buffer = np.kron( np.ones( (buffer_length, 1) ), x0[:][:2]);
        self.trail_patch = patch.PathPatch(path.Path(self.buffer), color=self.color);

        self.axs.add_patch(self.trail_patch);

        # system specific
        self.xd = [5, 7, 0, 0];
        self.axs.plot(self.xd[0], self.xd[1], color='g', marker='*', markersize=5)

        self.sphereworld = init_sphereworld();
        for sphere in self.sphereworld:
            sphere.plot(self.axs);

        self.axs.set_xlim(-10,10);
        self.axs.set_ylim(-10,10);
        self.axs.axis('equal');

    def update(self, t, x):
        self.trail_patch.remove();

        self.buffer[:-1] = self.buffer[1:];
        self.buffer[-1] = x[:2];

        self.trail_patch = patch.PathPatch(path.Path(self.buffer), fill=0);
        self.axs.add_patch(self.trail_patch);

        plt.show(block=0);
        plt.pause(self.pause);

        return self;

def init_sphereworld():
    wall = Sphere([0,0], -10);
    obs1 = Sphere([2,5], 2.5);
    obs2 = Sphere([-5,1], 4.);
    obs3 = Sphere([0,-7], 3.);
    # obs4 = Sphere([0,-2.5], 3.)

    return (wall, obs1, obs2, obs3);


def model(x, u, _):
    dt = 0.025;
    c = 0.1;

    # discrete steps
    xn = [
        x[0] + dt*x[2],
        x[1] + dt*x[3],
        x[2] + dt*u[0] - c*x[2],
        x[3] + dt*u[1] - c*x[3]
    ];

    return xn;

def cost(mpc_var, xlist, ulist):
    sphereworld = mpc_var.params.sphereworld;
    xd = mpc_var.params.xd;
    Nu = mpc_var.u_num;
    PH = mpc_var.PH;

    kx = 1000;
    kdx = 20;
    ku = 1;
    ko = 1000;

    C = 0;
    k = 0;
    for i in range(PH):
        C += kx*((xlist[i][0] - xd[0])**2 + (xlist[i][1] - xd[1])**2);
        C += kdx*(xlist[i][2]**2 + xlist[i][3]**2);
        C += ku*(ulist[k]**2 + ulist[k+1]**2);
        k += Nu;

        for sphere in sphereworld:
            C += ko/sphere.distance(xlist[i]);

    C += kx*((xlist[-1][0] - xd[0])**2 + (xlist[-1][1] - xd[1])**2);

    return C;

def callback(mvar, T, x, u):
    return mvar.params.update(T, x);


if __name__ == "__main__":
    num_inputs = 2;
    PH_length = 5;
    num_ssvar = 4;
    model_type = 'discrete';

    x0 = [-1.68, -9.6, 0, 0];
    uinit = [0 for i in range(num_inputs*PH_length)];

    params = Parameters(x0, buffer_length=25);

    mpc_var = ModelPredictiveControl('nno', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=5,
        model_type=model_type);
    mpc_var.setMinTimeStep(1);  # arbitrarily large

    sim_time = 20;
    T, xlist, ulist = mpc_var.sim_root(sim_time, x0, uinit,
        callback=callback, output=1)[0:3];

    plot(T, xlist, params);
    plt.show();
