# File: Plant.py
# Classes: Model()

import numpy as np

def TaylorMethod(F, x, u=[None], dt=1e-3):
    return x + dt*F( x, u )

class Model:
    def __init__(self, F, params=None, dt=1e-3,
            model_type='discrete'):
        # default to TaylorMethod when given model is continuous
        self.N = None
        if model_type == 'continuous':
            self.model = lambda x,u=[None]: TaylorMethod( F, x, u, dt )
        else:
            self.model = F

        # set time-step for reference
        self.dt = dt
        self.params = params

    def prop(self, x, u=[None]):
        # print( self.model( x, 1, 1 ) )
        return self.model( x, u )

    def simulate(self, T, x0, params=None, verbose=0):
        # Set class parameter variable.
        self.params = self.params if params is None else params

        # State space dimenions.
        self.N = x0.shape[0] if self.N is None else self.N

        # Simulation time frame.
        Nt = round( T/self.dt ) + 1
        tlist = self.dt*np.array( [t for t in range( Nt )] )

        # Initial conditions.
        xlist = np.empty( (Nt,) + x0.shape )
        xlist[0] = x0

        # Simulation loop.
        for t in range( Nt-1 ):
            xlist[t+1] = self.prop( xlist[t], xlist[:t] )
            if verbose: print(
                '%.3f:'%(self.dt*t), xlist[t], '->', xlist[t+1] )

        # Return simulation results.
        xlist = xlist if len( xlist.shape ) > 2 else xlist.T
        return tlist, xlist