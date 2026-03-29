
import numpy as np
import matplotlib.pyplot as plt

from MPC.Plant import *
from MPC.Optimizer import *

dt = 1e-3

def modelref(x, u):
    k = -0.3
    c = -1.21

    noise = np.random.normal( 0, 1/2 )

    sig = 3/4
    dx = np.array( [
        x[1] + noise,
        k*x[0] + c*(x[1] + noise)
    ] )

    return dx

def cost(k, params):
    mu = params[0]
    sig = params[1]

    # Temporary model function.
    def tempmodel(x, u):
        c = -1.21

        dx = np.array( [
            x[1],
            k[0][0]*x[0] + c*x[1]
        ] )
        # print( dx )

        return dx

    # Initialize model variable.
    mvar = Model( tempmodel, dt=dt, model_type='continuous' )

    # Generate trajectories.
    Nl = 1;  x0 = np.array( [1,0] )
    xlearnlist = [mvar.simulate( 10, x0 )[1] for _ in range( Nl )]

    # Compute mean and variance.
    mulrn = np.mean( [np.mean( xlearn[0] ) for xlearn in xlearnlist] )
    siglrn = np.std( [np.std( xlearn[0] ) for xlearn in xlearnlist] )
    return np.array( [(mu - mulrn)**2 + (sig - siglrn)**2] )

if __name__ == '__main__':
    # Reference model.
    mrefvar = Model( modelref, dt=dt, model_type='continuous' )

    # Generate training data.
    Nt = 5
    T = 10;  x0 = np.array( [1, 0] )
    xtrainlist = [mrefvar.simulate( T, x0 ) for _ in range( Nt )]

    # Generate validation data.
    Nv = 10
    xvalidlist = [mrefvar.simulate( T, x0 ) for _ in range( Nv )]

    # Train the fitted model for parameter k.
    params = [
        np.mean( [np.mean( xtrain[1][0] ) for xtrain in xtrainlist] ),
        np.std( [np.std( xtrain[1][0] ) for xtrain in xtrainlist] )
    ]
    # print( params )

    # Initialize training variable.
    ovar = Optimizer( cost, params=params )
    k0 = np.array( [[-1.21]] )
    print( ovar.solve( k0, verbose=1 ) )

    # # Plot training and validation data.
    # fig, axs = plt.subplots( 2,1 )
    # for xtrain in xtrainlist:
    #     axs[0].plot( xtrain[0], xtrain[1][0] )
    # for xvalid in xvalidlist:
    #     axs[1].plot( xvalid[0], xvalid[1][0] )
    # plt.show()
