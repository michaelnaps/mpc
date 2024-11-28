
import numpy as np
import matplotlib.pyplot as plt

from MPC.Plant import *
from MPC.Optimizer import *

dt = 1e-3

def modelref(x, u):
    k = -0.3
    c = -1.21

    dx = np.array( [
        x[1] + np.random.normal( 0, 1/2 ),
        k*x[0] + c*x[1]
    ] )

    return dx

def modellrn(x, params):
    k = -params
    c = -1.21

    dx = np.array( [
        x[1],
        k*x[0] + c*x[1]
    ] )

    return dx

def cost():
    return 1

if __name__ == '__main__':
    # Reference model.
    mrefvar = Model( modelref, dt=dt,
        model_type='continuous' )

    # Generate training data.
    Nt = 5
    T = 10;  x0 = np.array( [1, 0] )
    xtrainlist = [mrefvar.simulate( T, x0 ) for _ in range( Nt )]

    # Generate validation data.
    Nv = 10
    xvalidlist = [mrefvar.simulate( T, x0 ) for _ in range( Nv )]

    # Train the fitted model for parameter k.

    # Plot training and validation data.
    fig, axs = plt.subplots( 2,1 )
    for xtrain in xtrainlist:
        axs[0].plot( xtrain[0], xtrain[1][0] )
    for xvalid in xvalidlist:
        axs[1].plot( xvalid[0], xvalid[1][0] )
    plt.show()
