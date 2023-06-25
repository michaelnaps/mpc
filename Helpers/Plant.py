# File: System.py
# Classes: Model(), Cost()

import numpy as np

def fdm2c(g, x, h=1e-6):
    # initialize parameters
    Nx = len( x );        # dimension of input
    Ng = len( g(x) );     # dimension of output

    # calculate derivative at each input
    grad = np.empty( (Nx,Ng) );
    for i in range( Nx ):
        xn1 = x.copy();  xn1[i] = xn1[i] - h;
        xp1 = x.copy();  xp1[i] = xp1[i] + h;

        gn1 = g( xn1 );
        gp1 = g( xp1 );

        grad[i,:] = (gp1 - gn1).reshape(Ng,)/(2*h);

    return grad;


def TaylorMethod(F, x0, u=None, dt=1e-3):
    return x0 + dt*F( x0, u );

class Cost:
    def __init__(self, g,
            gradient=None, hessian=None):
        # set cost function
        self.cost = g;

        # set gradient function
        if gradient is None:
            self.grad = lambda x: fdm2c( g, x );
        else:
            self.grad = gradient;

        # set hessian function
        if hessian is None:
            self.hess = lambda x: fdm2c( self.grad, x );
        else:
            self.hess = hessian;

class Model:
    def __init__(self, F, dt=1e-3,
            model_type='discrete'):
        # default to TaylorMethod when given model is continuous
        if model_type == 'continuous':
            self.model = lambda x0,u=None: TaylorMethod( F, x0, u, dt );
        else:
            self.model = F;

        # set time-step for reference
        self.dt = dt;

    def step(self, x0, u=None):
        return self.model( x0, u );
