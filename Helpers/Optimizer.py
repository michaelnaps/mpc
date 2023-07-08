import numpy as np
from Helpers.Plant import *

# from time import sleep

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

class Optimizer( Cost ):
    def __init__(self, g, eps=1e-6, solver='ngd'):
        # inherit cost function class parameters
        Cost.__init__( self, g );

        # tolerance for zero and gradient descent step-size
        self.eps = eps;  # zero-approximation
        self.alpha = 0.1;

        # set solution step function
        self.setStepMethod( solver );

    def setStepSize(self, a):
        self.alpha = a;
        # Return instance of self.
        return self;

    def setMaxIter(self, n):
        self.max_iter = n;
        # Return instance of self.
        return self;

    def setObjectiveFunction(self, g):
        self.cost = g;
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

        n = 1;
        while gnorm > self.eps:
            x = self.step( x, dg );
            dg = self.grad( x );
            gnorm = np.linalg.norm( dg );

            if verbose:
                # print("Gradient:  ", g);
                print("\n|g|:       ", gnorm);
                print("New Cost:    ", self.cost( x )[0]);
                print("New Input:\n "  , x);

            if n > self.max_iter-1:
                break;
            n += 1;

        # Return solution.
        return x;

# Class: ModelPredictiveControl()
# Assumptions:
#   1). Cost is a function of the state AND input:
#       c = g(x,u)
class ModelPredictiveControl( Model, Cost ):
    def __init__(self, F, g, P=10, Nu=1, Nx=None,
            dt=1e-3, model_type='discrete'):
        # Save model to inherited class.
        Model.__init__( self, F,
                dt=dt, model_type=model_type );
        Cost.__init__(self, g);

        # Dimensions parameters.
        self.P = P;
        self.Nu = Nu;
        self.Nx = Nx;
        self.alpha = 0.1;

    def setStepSize(self, a):
        self.alpha = a;
        # Return instance of self.
        return self;

    def setMaxIter(self, n):
        self.max_iter = 10;
        # Return instance of self.
        return self;

    def statePrediction(self, x0, uList):
        xList = np.empty( (self.Nx, self.P+1) );
        xList[:,0] = x0[:,0];
        for i in range( self.P ):
            x = xList[:,i,None];
            u = uList[:,i,None];
            xList[:,i+1] = self.prop( x, u )[:,0];
        return xList;

    def costPrediction(self, x0, uList):
        xList = self.statePrediction( x0, uList );
        C = [0];
        for i in range( self.P ):
            x = xList[:,i+1,None];
            u = uList[:,i,None];
            C += self.cost( x, u );
        return C

    def costFunctionGenerator(self, x0):
        # Generate cost function around x0.
        return lambda uList: self.costPrediction( x0, uList.reshape( self.Nu,self.P ) );

    def solve(self, x0, uinit, verbose=0):
        # If necessary set Nx to number of states.
        if self.Nx is None:
            self.Nx = len( x0 );

        # Initialize optimization variable with cost generator.
        ovar = Optimizer( self.costFunctionGenerator( x0 ) );
        ovar.setStepSize( self.alpha );
        ovar.setMaxIter( self.max_iter );

        # Return optimization results.
        uvect = uinit.reshape( self.P*self.Nu, 1 );
        return ovar.solve( uvect, verbose=verbose );