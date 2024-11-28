
import numpy as np
import matplotlib.pyplot as plt

from MPC.Plant import *
from MPC.Optimizer import *

dt = 1e-3

def modelref(mvar, x, u):
    c = 1.21
    k = 0.3

    dx = np.array( [
        x[1],
        c*x[1] + k*x[0]
    ] )

    return x + dt*dx + np.random.normal( 0, 0.01 )

if __name__ == '__main__':
    # Reference model.
    mrefvar = Model( modelref, dt=dt )

    # Generate training data.
    T = 10;  x0 = np.array( [1, 0] )
    xtrainlist = [mrefvar.simulate( T, x0 )]

    # Generate validation data.
