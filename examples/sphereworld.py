import sys
sys.path.insert(0, '../.');

from mpc import *
from Sphere import *

def init_sphereworld():
    wall = Sphere([0,0], -10);
    obs1 = Sphere([2,5], 2.5);
    obs2 = Sphere([-5,1], 4.);
    obs3 = Sphere([0,-7], 3.);

    return (wall, obs1, obs2, obs3);

def model(q, u, _):
    dt = 0.025;

    # discrete
    qn = [
        q[0] + dt*u[0],
        q[1] + dt*u[1]
    ];

    return qn;

def cost(mpc_var, qlist, ulist, sphereworld):
    qd = [4, 8];
    Nu = mpc_var.u_num;
    PH = mpc_var.PH;

    kq = 1;
    ku = 0.01;
    ko = 10;

    C = 0;
    k = 0;
    for i in range(PH):
        C += kq*((qlist[i][0] - qd[0])**2 + (qlist[i][1] - qd[1])**2);
        C += ku*(ulist[k]**2 + ulist[k+1]**2);
        k += Nu;

        for sphere in sphereworld:
            C += ko*abs(1/sphere.distance(qlist[i]));

    C += kq*((qlist[-1][0] - qd[0])**2 + (qlist[-1][1] - qd[1])**2);

    return C;

def plot(T, q, sphereworld):
    qlist = np.transpose(q);

    fig, axes = plt.subplots();

    for sphere in sphereworld:
        sphere.plot(axes);

    axes.plot(qlist[0], qlist[1]);
    axes.set_aspect('equal');

if __name__ == "__main__":
    num_inputs = 2;
    PH_length = 10;
    model_type = 'discrete';
    sphereworld = init_sphereworld();

    mpc_var = ModelPredictiveControl('nno', cost, model, sphereworld, num_inputs,
            PH_length=PH_length, model_type=model_type);

    q0 = [-4, -4];
    uinit = [0 for i in range(num_inputs*PH_length)];

    sim_time = 0.1;
    T, qlist, ulist = mpc_var.sim_root(sim_time, q0, uinit, output=1)[0:3];

    plot(T, qlist, sphereworld);
    plt.show();
