# File: System.py
# Classes: Model(), Cost()

import numpy as np

def TaylorMethod(F, x0, u=[None], dt=1e-3):
    return x0 + dt*F( x0, u );

class Model:
    def __init__(self, F, dt=1e-3,
            model_type='discrete'):
        # default to TaylorMethod when given model is continuous
        if model_type == 'continuous':
            self.model = lambda x0,u=[None]: TaylorMethod( F, x0, u, dt );
        else:
            self.model = F;

        # set time-step for reference
        self.dt = dt;

    def prop(self, x0, u=[None]):
        return self.model( x0, u );
