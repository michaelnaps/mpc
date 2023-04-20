# Class: dpa.py
# Created by: Michael Napoli
# Created on: 5/31/2022

import math
import time

class DynamicProgramming:
    def __init__(self, pcost, tcost, model, N, Nx, Nu,
        params=None, max_iter=10,
        appx_zero=1e-6, grad_step=1e-3):
        self.pcost = pcost;
        self.tcost = tcost;
        self.model = model;
        self.params = params;

        self.N = N;
        self.Nx = Nx;
        self.Nu = Nu;

        self.zero = appx_zero;
        self.step = grad_step;
        self.nmax = max_iter;

        self._alpha = 1;

    def setAlpha(self, a):
        self._alpha = a;
        return;

    def forward(self, x0, uinit, N=None, output=0):
        # variable setup
        Nu = self.Nu;
        h = self.step;
        a = self._alpha;
        zero = self.zero;
        nmax = self.nmax;

        if N is None:
            return self.forward(x0, uinit, self.N, output);

        if N == 1:
            # period cost + terminal cost
            cost = lambda x,u: self.pcost(x,u) + self.tcost( self.model(x,u) );

            un = uinit;
            gn = self.fdm(cost, x0, un, h=h);
            gnorm = sum([gn[i]**2 for i in range(N)]);

            count = 0;
            while gnorm > zero:
                un = [un[i] - a*gn[i] for i in range(Nu)]
                gn = self.fdm(cost, x0, un, h=h);
                gnorm = sum([gn[i]**2 for i in range(Nu)]);

                if count > nmax-1:
                    break;

            Jn = cost(x0, un);  # cost-to-go
            return un, Jn, self.model(x0, un);

        return (un, Jn);

    def gradientDescent(self, cost, x0, u, h=1e-3):
        pass;

    def fdm(self, cost, x0, u, h=1e-3):
        Nu = len(u);
        dJ = [0 for i in range(Nu)];

        for i in range(Nu):
            un1 = [u[j] - (i==j)*h for j in range(Nu)];
            up1 = [u[j] + (i==j)*h for j in range(Nu)];

            Jn1 = cost(x0, up1);
            Jp1 = cost(x0, up1);

            dJ[i] = (Jp1 - Jn1)/(2*h);

        return dJ;