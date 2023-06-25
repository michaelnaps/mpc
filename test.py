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

if __name__ == '__main__':
    # model variable
    mvar = Model( VanDerPol, model_type='continuous' );

    # initial conditions
    Nx = 2;
    x = np.array( [[1],[0]] );  # unnecessary, but...

    # time variables
    T = 10;  Nt = round( T/mvar.dt ) + 1;
    tList = [ [i*mvar.dt for i in range( Nt )] ];

    # step for length of sim
    xList = np.empty( (Nx, Nt+1) );
    xList[:,0] = x[:,0];
    for i in range( Nt ):
        x = mvar.step( x );
        xList[:,i+1] = x[:,0];  # save step

    # plot results
    fig, axs = plt.subplots();
    axs.plot( xList[0], xList[1] );
    plt.show();