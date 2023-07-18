import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')

import numpy as np
import matplotlib.pyplot as plt

import MPC.Plant as plant
import MPC.Optimizer as opt

def model(x, u):
    l = 1
    m = 1
    c = 10
    g = 9.81

    dx = np.array( [
        x[1],
        (u[0] - g/l*np.cos(x[0]) - c*x[1])
    ] )

    return dx

def cost(x, u):
    kx = 500
    kd = 0.1
    xd = np.array( [[np.pi/2],[0]] )
    C = kx*(x[0] - xd[0])**2 + kd*(x[1] - xd[1])**2
    return C

def pendulum(tList, xList, uList):
    fig, axs = plt.subplots( 2,1 )

    axs[0].plot( tList.T, xList.T )
    axs[1].plot( tList.T, np.fliplr( uList.T ) )

    axs[0].legend( ('$x_1$', '$x_2$') )
    axs[1].legend( ('$u_1$',) )

    return fig, axs

if __name__ == "__main__":
    dt = 0.025

    mvar = plant.Model( model, dt=dt, model_type='continuous' )
    mpcvar = opt.ModelPredictiveControl(model, cost,
        P=10, Nu=1, Nx=2, dt=dt, model_type='continuous')
    mpcvar.setStepSize( 0.1 )
    mpcvar.setMaxIter( 10 )

    x0 = np.array( [[0],[-0.1]] )

    T = 5;  Nt = round( T/mpcvar.dt ) + 1
    tList = np.array( [ [i*mpcvar.dt for i in range( Nt )] ] )

    uList = np.zeros( (mpcvar.Nu*mpcvar.P, Nt) )
    xList = np.empty( (mpcvar.Nx, Nt) )
    xList[:,0] = x0[:,0]
    for i in range( Nt-1 ):
        x = xList[:,i,None]
        uList[:,i+1] = mpcvar.solve( x, uList[:,i], verbose=0 )[:,0]
        xList[:,i+1] = mvar.prop( x, uList[:,i+1,None] )[:,0]
        if i % 10 == 0:
            print( 'time =', tList[0][i] )
            print( uList[:,i+1] )

    pendulum( tList, xList, uList )
    plt.show()