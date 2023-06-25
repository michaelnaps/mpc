import numpy as np
import matplotlib.pyplot as plt

from Helpers.Plant import *

# model an autonomous system
def VanDerPol(x0, u=None):
    mu = 2.0;
    dx = np.array( [
        x[1],
        mu*(1-x[0]**2)*x[1] - x[0]
    ] );
    return dx;

def VanDerPolCost(x, u=None):
    C = 0;
    for xi in x[:,0]:
        C += xi**2;
    return C;

if __name__ == '__main__':
    # model variable
    mvar = Model( VanDerPol, model_type='continuous' );
    cvar = Cost( VanDerPolCost );

    # initial conditions
    Nx = 2;
    x = np.array( [[1],[0]] );  # unnecessary, but...

    # time variables
    T = 100;  Nt = round( T/mvar.dt ) + 1;
    tList = [ [i*mvar.dt for i in range( Nt )] ];

    # step for length of sim
    xList = np.empty( (Nx,Nt) );
    cList = np.empty( (1,Nt) );
    xList[:,0] = x[:,0];
    cList[:,0] = cvar.cost( x );
    for i in range( Nt-1 ):
        x = mvar.step( x );
        c = cvar.cost( x );
        xList[:,i+1] = x[:,0];  # save step
        cList[:,i+1] = c;

    # plot results
    fig, axs = plt.subplots(1,2);
    axs[0].plot( xList[0], xList[1] );
    axs[0].set_title('Model');
    axs[1].plot( tList[0], cList[0] );
    axs[1].set_title('Cost');
    plt.show();