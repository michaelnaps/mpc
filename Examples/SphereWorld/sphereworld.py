import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

from mpc import *
from Sphere import *

import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.path as path


# hyper parameter
dt = 0.025;


class Parameters:
    def __init__(self, x0, xd, PH,
                 fig=None, axs=None,
                 buffer_length=10, pause=1e-3,
                 color='k', record=0):
        if axs is None and fig is None:
            self.fig, self.axs = plt.subplots();
        else:
            self.fig = fig;
            self.axs = axs;

        # initialize world
        self.sphereworld = init_sphereworld();
        self.xd = xd;
        self.axs.plot(self.xd[0], self.xd[1], color='g', marker='*', markersize=5)

        self.sphereworld = init_sphereworld();
        for sphere in self.sphereworld:
            sphere.plot(self.axs);

        # figure scaling
        self.axs.set_xlim(-10,10);
        self.axs.set_ylim(-10,10);
        self.axs.axis('equal');
        self.fig.tight_layout();

        # initialize buffer (trail)
        self.PH = PH;
        self.color = color;

        self.buffer = [x0[:2] for i in range(buffer_length)];
        self.trail_patch = patch.PathPatch(path.Path(self.buffer), color=self.color);

        self.prediction = [x0 for i in range(self.PH+1)];
        self.future_patch = patch.PathPatch(path.Path(self.buffer), color='C1');

        self.axs.add_patch(self.trail_patch);
        self.axs.add_patch(self.future_patch);

        self.pause = pause;

        if record:
            plt.show(block=0);
            input("Press enter when ready...");

    def update(self, t, x, xPH):
        self.trail_patch.remove();
        self.future_patch.remove();

        self.buffer[:-1] = self.buffer[1:];
        self.buffer[-1] = x[:2];

        self.trail_patch = patch.PathPatch(path.Path(self.buffer),
            color=self.color, fill=0);

        self.prediction = [xPH[i][:2] for i in range(self.PH+1)];
        self.future_patch = patch.PathPatch(path.Path(self.prediction),
            color='C1', fill=0);

        self.axs.add_patch(self.trail_patch);
        self.axs.add_patch(self.future_patch);

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

    kx = 150;
    kdx = 4;
    ku = 0.1;
    ko = 50;

    C = 0;
    k = 0;
    for i, x in enumerate(xlist):
        C += kx*((x[0] - xd[0])**2 + (x[1] - xd[1])**2);
        C += kdx*(x[2]**2 + x[3]**2);

        if (i != PH):
            C += ku*(ulist[k]**2 + ulist[k+1]**2);
            k += Nu;

        for sphere in sphereworld:
            C += ko/sphere.distance(x);

    return C;

def callback(mpc_var, T, x, u):
    xPH = mpc_var.simulate(x, u);
    return mpc_var.params.update(T, x, xPH);


if __name__ == "__main__":
    num_inputs = 2;
    num_ssvar = 4;
    PH_length = 10;
    knot_length = 1;
    max_iter = 10;
    model_type = 'discrete';

    x0 = [-1.68, -9.6, 0, 0];
    xd = [5, 7, 0, 0];
    uinit = [0 for i in range(num_inputs*PH_length)];

    params = Parameters(x0, xd, PH_length, buffer_length=25, record=0);

    mpc_var = ModelPredictiveControl('ngd', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=knot_length,
        time_step=dt, max_iter=max_iter, model_type=model_type);
    mpc_var.setAlpha(0.1);
    mpc_var.setMinTimeStep(1);  # arbitrarily large

    sim_time = 10;
    sim_results = mpc_var.sim_root(sim_time, x0, uinit,
        callback=callback, output=1, saveflow=0);
    plt.close('all');

    T = sim_results[0];
    xlist = sim_results[1];
    ulist = sim_results[2];
    tlist = sim_results[6];

    # # plot calculation speed
    # fig, axs = plt.subplots();
    # axs.plot(T, tlist);
    # axs.set_xlim(-1, max(T)+1)
    # plt.show();
