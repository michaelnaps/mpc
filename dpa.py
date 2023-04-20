# Class: dpa.py
# Created by: Michael Napoli
# Created on: 5/31/2022

import math
import time

class DynamicProgramming:
    def __init__(self, pcost, tcost, N):
        self.pcost = pcost;
        self.tcost = tcost;
        self.N = N;

    def fdp(self, tcost, x0, uinit, PH=-1, output=0):
        # variable setup
        Nu = self.u_num;
        h = self.h;
        a = self._alpha;
        eps = self.zero;
        imax = self.n_max;

        if PH == -1:
            return self.fdp(x0, uinit, self.PH, output);

        if PH == 1:
            # period cost + terminal cost
            jcost = lambda x,u: self.cost(x,u) + tcost( self.model(x,u,self) );

            un = uinit;
            gn = ifdm(jcost, x0, un, h=h);
            gnorm = sum([gn[i]**2 for i in range(PH)]);

            count = 0;
            while gnorm > eps:
                un = [un[i] - a*gn[i] for i in range(Nu)]
                gn = ifdm(jcost, x0, un, h=h);
                gnorm = sum([gn[i]**2 for i in range(Nu)]);

                if count > imax-1:
                    break;

            Cn = jcost(x0, un);
            return self.model(x0, un, self), un, Cn;

        return (un, Cn);

    def ifdm(cost, x0, u, h=1e-3):
        Nu = len(u);
        dC = [0 for i in range(Nu)];

        for i in range(Nu):
            un1 = [u[j] - (i==j)*h for j in range(Nu)];
            up1 = [u[j] + (i==j)*h for j in range(Nu)];

            Cn1 = cost(x0, un1);
            Cp1 = cost(x0, up1);

            dC[i] = (Cp1 - Cn1)/(2*h)

        return dC;