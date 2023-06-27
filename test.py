import numpy as np
import matplotlib.pyplot as plt

from Helpers.Plant import *
from Helpers.Optimizer import *
from Helpers.Vehicle2D import *

# hyper parameter(s)
verbose = 0;

# model an autonomous system
def VanDerPol(x, u=None):
    mu = 2.0;
    dx = np.array( [
        x[1],
        mu*(1 - x[0]**2)*x[1] - x[0]
    ] );
    return dx;

def VanDerPolCost(x, u=None):
    C = x[0]**2 + x[1]**2;
    return C;

if __name__ == '__main__':
    # model variable
    mvar = Model( VanDerPol, dt=1e-2, model_type='continuous' );
    cvar = Cost( VanDerPolCost );
    ovar = Optimizer( VanDerPolCost );

    # initial conditions
    Nx = 2;
    x = 2*np.random.rand(Nx,1)-1;  # unnecessary, but...

    # time variables
    T = 10;  Nt = round( T/mvar.dt ) + 1;
    tList = [ [i*mvar.dt for i in range( Nt )] ];

    # vehicle initialization
    vhc = Vehicle2D( mvar, x );

    # step for length of sim
    xList = np.empty( (Nx,Nt) );
    cList = np.empty( (1,Nt) );
    gList = np.empty( (Nx,Nt) );
    xList[:,0] = x[:,0];
    cList[:,0] = cvar.cost( x );
    gList[:,0] = cvar.grad( x )[:,0];
    for i in range( Nt-1 ):
        # instantaneous variables (unnecessary)
        x = mvar.prop( x );
        c = cvar.cost( x );
        g = cvar.grad( x );

        # update vehicle
        vhc.update( x );
        vhc.draw();

        # save step
        xList[:,i+1] = x[:,0];
        cList[:,i+1] = c;
        gList[:,i+1] = g[:,0];

        if verbose:
            print( x );
            print( c );
            print( g );
            print( ovar.grad( x ) );
            print( ovar.solve( x ) );
            print( '------------' );

    # plot results
    Np = 3;
    fig, axs = plt.subplots(1,Np);

    axs[0].plot( xList[0], xList[1] );
    axs[0].set_title('Model');

    axs[1].plot( tList[0], cList[0] );
    axs[1].set_title('Cost');

    axs[2].plot( gList[0], gList[1] );
    axs[2].set_title('Gradient');

    for i in range( Np ):
        axs[i].grid(1);
        axs[i].axis('equal');

    plt.show();