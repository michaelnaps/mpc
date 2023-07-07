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

    def setObjectiveFunction(self, g):
        self.g = g;

        # Return instance of self.
        return self;

    def setStepMethod(self, solver):
        self.solver = solver;
        if self.solver == 'ngd':
            self.step = lambda x, dg: x - self.alpha*dg;
        if self.solver == 'nno':
            self.step = None;  # not setup

        # Return instance of self.
        return self;

    def solve(self, xinit, verbose=0):
        x = xinit.copy();  # copy the initial guess
        dg = self.grad( x );
        gnorm = np.linalg.norm( dg );

        while gnorm > self.eps:
            x = self.step( x, dg );
            dg = self.grad( x );
            gnorm = np.linalg.norm( dg );

            if verbose:
                # print("Gradient:  ", g);
                print("|g|:          ", gnorm);
                print("New Cost:     ", self.cost( x ));
                print("New Input:\n"  , x);

        # Return solution.
        return x;

# Class: ModelPredictiveControl()
# Assumptions:
#   1). Cost is a function of the state AND input:
#       c = g(x,u)
class ModelPredictiveControl( Model, Optimizer ):
    def __init__(self, F, g, P=10, dt=1e-3, model_type='discrete'):
        # Save model to inherited class.
        Model.__init__( self, F,
                dt=dt, model_type=model_type );
        Cost.__init__(self, g);

        # Dimensions parameters.
        self.P = P;
        self.Nx = None;
        self.Nu = None;

    def statePrediction(self, x0, uList):
        if self.Nx is None:
            self.Nx = len( x0 );
        if self.Nu is None:
            self.Nu = len( uList );

        xList = np.empty( (self.Nx, self.P+1) );
        xList[:,0] = x0[:,0];
        for i in range( self.P ):
            x = xList[:,i,None];
            u = uList[:,i,None];
            xList[:,i+1] = self.prop( x, u )[:,0];

        return xList;

    def predictionCost(self, xList, uList):
        C = [0];

        for i in range( self.P ):
            x = xList[:,i+1,None];
            u = uList[:,i,None];
            C += self.cost( x, u );

        return C[0];

    def solve(self, x0, uinit):
        pass;
