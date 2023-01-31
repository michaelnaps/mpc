import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

from mpc import *
from Sphere import *

class Parameters:
    def __init__(self, fig=None, axes=None):
        if fig is None or axes is None:
            self.fig, self.axes = plt.subplots();
        else:
            self.fig = fig;
            self.axes = axes;
        self.sphereworld = init_sphereworld();
        self.qd = [4, 8, 0, 0];
        self.pause = 1e-6;

def init_sphereworld():
    wall = Sphere([0,0], -10);
    obs1 = Sphere([2,5], 2.5);
    obs2 = Sphere([-5,1], 4.);
    obs3 = Sphere([0,-7], 3.);
    # obs4 = Sphere([0,-2.5], 3.)

    return (wall, obs1, obs2, obs3);

def model(q, u, _):
    dt = 0.025;
    c = 0.1;

    # discrete steps
    qn = [
        q[0] + dt*q[2],
        q[1] + dt*q[3],
        q[2] + dt*u[0] - c*q[2],
        q[3] + dt*u[1] - c*q[3]
    ];

    return qn;

def cost(mpc_var, qlist, ulist):
    sphereworld = mpc_var.params.sphereworld;
    qd = mpc_var.params.qd;
    Nu = mpc_var.u_num;
    PH = mpc_var.PH;

    kq = 1;
    kdq = 0.1;
    ku = 0.001;
    ko = 1;

    C = 0;
    k = 0;
    for i in range(PH):
        C += kq*((qlist[i][0] - qd[0])**2 + (qlist[i][1] - qd[1])**2);
        C += kdq*(qlist[i][2]**2 + qlist[i][3]**2);
        C += ku*(ulist[k]**2 + ulist[k+1]**2);
        k += Nu;

        for sphere in sphereworld:
            C += ko*1/sphere.distance(qlist[i]);

    C += kq*((qlist[-1][0] - qd[0])**2 + (qlist[-1][1] - qd[1])**2);

    return C;

def plot(T, q, params, u=None):
    sphereworld = params.sphereworld;
    qd = params.qd;

    fig = params.fig;
    axes = params.axes;

    fig.tight_layout();

    for sphere in sphereworld:
        sphere.plot(axes);

    if q is not None:
        qlist = np.transpose(q);
        axes.plot(qlist[0], qlist[1])

    axes.plot(qd[0], qd[1], color='yellowgreen', marker='*');
    axes.set_aspect('equal');

def callback(mpc_var, t, q, ulist):
    axes = mpc_var.params.axes;

    axes.plot(q[0], q[1], color='b', marker='.', markersize=2)
    plt.show(block=0);

    plt.pause(mpc_var.params.pause);

    return Parameters(mpc_var.params.fig, mpc_var.params.axes);

if __name__ == "__main__":
    num_inputs = 2;
    PH_length = 5;
    num_ssvar = 4;
    model_type = 'discrete';

    params = Parameters();
    plot(None, None, params)

    mpc_var = ModelPredictiveControl('nno', model, cost, params, num_inputs,
        num_ssvar=num_ssvar, PH_length=PH_length, knot_length=4,
        model_type=model_type);
    mpc_var.setMinTimeStep(1);  # arbitrarily large

    q0 = [-1.68, -9.6, 0, 0];
    uinit = [0 for i in range(num_inputs*PH_length)];

    sim_time = 20;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit,
        callback=callback, output=1)[0:3];

    plot(T, qlist, params);
    plt.show();
