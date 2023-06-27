import numpy as np
from Helpers.Plant import *

class Optimizer( Cost ):
    def __init__(self, g, eps=1e-6, solver='ngd'):
        # inherit cost function class parameters
        Cost.__init__( self, g );

        # tolerance for zero and gradient descent step-size
        self.eps = eps;  # zero-approximation
        self.alpha = 1e-3;

        # set solution step function
        self.setStepMethod( solver );

    def setStepMethod(self, solver):
        self.solver = solver;
        if self.solver == 'ngd':
            self.step = lambda x, g: x - self.alpha*g;
        if self.solver == 'nno':
            self.step = None;  # not setup

    def solve(self, xinit, verbose=0):
        x = xinit.copy();  # copy the initial guess
        g = self.grad( x );
        gnorm = np.linalg.norm( g );

        while gnorm > self.eps:
            x = self.step( x, g );
            g = self.grad( x );
            gnorm = np.linalg.norm( g );

            if verbose:
                # print("Gradient:  ", g);
                print("|g|:          ", gnorm);
                print("New Cost:     ", self.cost( x ));
                print("New Input:\n"  , x);

        return x;

class ModelPredictiveControl( Model, Optimizer ):
    def __init__(self, F, g, dt=1e-3, model_type='discrete'):
        Model.__init__( self, F,
                dt=dt, model_type=model_type );
        Optimizer.__init__(self, g);