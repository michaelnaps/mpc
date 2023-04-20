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

        # if nmax is -1 -> no iteration cap
        self.nmax = max_iter;

        self._alpha = 1;

    def setAlpha(self, a):
        self._alpha = a;
        return;

    def forward(self, x0, uinit, N=None, output=0):
        # if N (prediction horizon) not given
        if N is None:
            # start alg. with class N
            return self.forward(x0, uinit, self.N, output);

        # if N = 1 end recursion by minimizing with terminal cost
        if N == 1:
            Jn = cost(x0, un);  # cost-to-go
            return un, Jn, self.model(x0, un);

        return (un, Jn);

    def minimize(self, cost, uinit, h=1e-3):
        un = [uinit[i] for i in range(self.Nu)];
        gn = self.fdm(cost, un, h=self.step);
        gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

        # gradient descent loop
        count = 1;
        while gnorm > self.zero**2:
            un = [un[i] - self._alpha*gn[i] for i in range(self.Nu)];
            gn = self.fdm(cost, un, h=self.step);
            gnorm = sum([gn[i]**2 for i in range(self.Nu)]);

            count += 1;
            if (self.nmax is not -1) and (count > self.nmax-1):  # iteration limit
                print('WARNING: Iteration break in DynamicProgramming.minimize()');
                break;

        return un, cost(un);

    def fdm(self, cost, u, h=1e-3):
        dJ = [0 for i in range(self.Nu)];

        print(u);

        for i in range(self.Nu):
            un1 = [u[j] - (i==j)*h for j in range(self.Nu)];
            up1 = [u[j] + (i==j)*h for j in range(self.Nu)];

            Jn1 = cost(un1);
            Jp1 = cost(up1);

            print('---');
            print(Jn1, Jp1);

            dJ[i] = (Jp1 - Jn1)/(2*h);

            print(dJ[i]);

        return dJ;