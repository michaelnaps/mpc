import numpy as np
from Helpers.Plant import *

class Optimizer( Cost ):
    def __init__(self, g, eps=1e-6, alpha=1e-3,
            gradient=None, hessian=None,):
        Cost.__init__( self, g,
                gradient=gradient, hessian=hessian );
        self.eps = eps;  # zero-approximation
        self.alpha = alpha;

    def ngd(self, xinit, verbose=0):
        x = xinit.copy();
        g = self.grad( x );
        gnorm = np.linalg.norm( g );

        while gnorm > self.eps:
            x = x - self.alpha*g;
            g = self.grad( x );
            gnorm = np.linalg.norm( g );

            if output:
                # print("Gradient:  ", g);
                print("|g|:          ", gnorm);
                print("New Cost:     ", self.cost( x ));
                print("New Input:\n\t"  , x);

        return x;

class ModelPredictiveControl( Model, Optimizer ):
    def __init__(self, F, g, dt=1e-3, model_type='discrete'):
        Model.__init__( self, F,
                dt=dt, model_type=model_type );
        Optimizer.__init__(self, g);