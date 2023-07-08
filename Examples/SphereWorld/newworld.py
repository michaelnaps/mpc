import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')
sys.path.insert(0, expanduser('~')+'/prog/geom')

from Geometry.Circle import *
from Helpers.Vehicle2D import *

import Helpers.Plant as plant
import Helpers.Optimizer as opt


# hyper parameter
xd = np.array( [[5],[7],[0],[0]] );
dt = 0.025;
P = 5;
k = 2;
Nx = 4;
Nu = 2;
max_iter = 25;
model_type = 'discrete';

def init_sphereworld():
    wall = Circle( np.array( [[0],[0]] ), -10 );
    obs1 = Circle( np.array( [[2],[5]] ), 2.5 );
    obs2 = Circle( np.array( [[-5],[1]] ), 4. );
    obs3 = Circle( np.array( [[0],[-7]] ), 3. );

    return (wall, obs1, obs2, obs3);

sphereworld = init_sphereworld();

def model(x, u):
    c = 0.1;

    # discrete steps
    xn = np.array( [
        x[0] + dt*x[2],
        x[1] + dt*x[3],
        (1 - c)*x[2] + dt*u[0],
        (1 - c)*x[3] + dt*u[1]
    ] );

    return xn;

def cost(x, u):
    kx = 100;
    C = 0*(x[0] - xd[0])**2 + kx*(x[1] - xd[1])**2;

    kdx = 2;
    C = C + kdx*(x[2]**2 + x[3]**2);

    ku = 1;
    C += ku*(u[0]**2 + u[1]**2);

    ka = 0;
    kb = 0;
    for sphere in sphereworld:
        C += ka/sphere.distance(x[:2,None])**2;

    return C;

if __name__ == "__main__":
    x0 = np.array( [[-1.68],[-9.6],[0],[0]] );
    uinit = np.zeros( (Nu, P) );

    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nu=Nu, Nx=Nx, dt=dt, model_type=model_type );
    mpc_var.setStepSize( 0.1 );
    mpc_var.setMaxIter( max_iter );

    mvar = plant.Model( model, dt=dt );

    fig, axs = plt.subplots();
    for sphere in init_sphereworld():
        sphere.plot( fig=fig, axs=axs );
    vhc = Vehicle2D( model, x0[:2], radius=0.2,
        fig=fig, axs=axs, tail_length=10 );
    plt.show( block=0 );

    T = 25;  Nt = round( T/dt ) + 1;
    tList = np.array( [[i*dt for i in range( Nt )]] );

    uList = np.zeros( (Nu*P, Nt) );
    xList = np.empty( (Nx, Nt) );
    xList[:,0] = x0[:,0];
    for i in range( Nt-1 ):
        x = xList[:,i,None];
        uList[:,i+1] = mpc_var.solve( x, uList[:,i], verbose=0 )[:,0];
        xList[:,i+1] = mvar.prop( x, uList[:,i+1,None] )[:,0];
        if i % 1 == 0:
            print( 'time =', tList[0][i] );
            # print( uList[:,i+1] );
        # vhc.update( xList[:2,i+1,None] );
        # vhc.draw();

    fig, axs = plt.subplots();
    axs.plot( tList.T, xList.T );
    plt.show();