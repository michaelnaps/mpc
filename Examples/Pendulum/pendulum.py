import sys
sys.path.insert(0, '/home/michaelnaps/prog/mpc');

import numpy as np
import matplotlib.pyplot as plt

import Helpers.Plant as plant
import Helpers.Optimizer as opt

def model(x, u):
    l = 1;
    m = 1;
    c = 10;
    g = 9.81;

    dx = np.array( [
        x[1],
        (u[0] - g/l*np.cos(x[0]) - c*x[1])
    ] );

    return dx;

def cost(x, u):
    xd = np.array( [[np.pi/2],[0]] );
    kx = 100;  kd = 0;
    C = kx*(x[0] - xd[0])**2 + kd*(x[1] - xd[1])**2;
    return C;

def pendulum(tList, xList, uList):
    fig, axs = plt.subplots(2,1);
    axs[0].plot(tList.T, xList.T);
    axs[0].legend( ('$x_1$', '$x_2$') );
    axs[1].plot(tList[:,:-1].T, uList.T);
    axs[1].legend( ('$u_1$',) );

if __name__ == "__main__":
    dt = 0.025;

    mvar = plant.Model( model, model_type='continuous' );
    mpcvar = opt.ModelPredictiveControl(model, cost,
        P=3, Nu=1, Nx=2, dt=dt, model_type='continuous');
    mpcvar.setStepSize( 0.1 );
    mpcvar.setMaxIter( 100 );

    x0 = np.array( [[0],[-0.1]] );
    uinit = np.zeros( (mpcvar.Nu, mpcvar.P) );

    T = 100;  Nt = round( T/mpcvar.dt ) + 1;
    tList = np.array( [ [i*mpcvar.dt for i in range( Nt )] ] );

    uList = np.zeros( (mpcvar.Nx, Nt-1) );
    xList = np.empty( (mpcvar.Nx, Nt) );
    xList[:,0] = x0[:,0];
    for i in range( Nt-1 ):
        uList[:,i] = mpcvar.solve( xList[:,i,None], uinit, verbose=0 )[:,0];
        xList[:,i+1] = mvar.prop( xList[:,i,None], uList[:,i,None] )[:,0];
        if i % 10 == 0:
            print( 'time =', tList[0][i] );
            print( uList[:,i] );


    pendulum( tList, xList, uList[0,None] );
    plt.show();