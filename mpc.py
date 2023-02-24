# Class: mpc.py
# Created by: Michael Napoli
# Created on: 5/31/2022

import time
import math

class ModelPredictiveControl:
    def __init__(self, solver, modelFunction, costFunction,
            user_params, num_inputs, num_ssvar=1,
            PH_length=1, knot_length=1, time_step=0.025,
            appx_zero=1e-6, step_size=1e-3, max_iter=10,
            model_type='continuous'):
        self.solver = solver;
        self.model  = modelFunction;
        self.cost   = costFunction;
        self.params = user_params;
        self.u_num  = num_inputs;
        self.x_num  = num_ssvar;
        self.PH     = PH_length;
        self.k      = knot_length;
        self.dt     = time_step;
        self.zero   = appx_zero;
        self.h      = step_size;
        self.n_max  = max_iter;
        self.type   = model_type;

        self._dt_min = 1e-3;

        self._alpha = 1;
        self._bkl_shrink = 0.1;
        self._a_method = None;

    def setParams(self, user_params):
        self.params = user_params;
        return 1;

    def setMinTimeStep(self, min_time_step):
        self._dt_min = min_time_step;
        return 1;

    def getMinTimeStep(self):
        return self._dt_min;

    def setAlpha(self, a):
        self._alpha = a;
        return 1;

    def getAlpha(self):
        return self._alpha;

    def setAlphaMethod(self, method, shrink=0.1):
        self._a_method = method;
        self._bkl_shink = shrink;
        return 1;

    def getAlphaMethod(self):
        return self._a_method;

    def solve(self, x0, uinit, output=0):

        if (self.solver == 'nno'):
            print("ERROR: nno algorithm is deprecated for MicroPython applications.");
            return;
        #     t = time.time();
        #     (u, C, n, brk) = self.nno(x0, uinit, output);
        #     elapsed = 1000*(time.time() - t);
        if (self.solver == 'ngd'):
            t = time.time();
            (u, C, n, brk) = self.ngd(x0, uinit, output);
            elapsed = 1000*(time.time() - t);

        return (u, C, n, brk, elapsed);

    def ngd(self, x0, uinit, output=0):
        # MPC constants initialization
        N      = self.u_num;
        P      = self.PH;
        eps    = self.zero;
        imax   = self.n_max;
        params = self.params;

        # step size coefficient choice
        alpha = self._alpha;
        a_method = self._a_method;

        # loop variable setup
        uc = uinit;
        x  = self.simulate(x0, uc);
        Cc = self.cost(self, x, uc);
        un = uc;  Cn = Cc;

        if output:
            print("Opt. Start:");
            print("Initial Cost: ", Cc);

        count = 0;
        brk = -2*math.isnan(Cc);
        # cost function must be positive
        while (Cc > eps):
            # calculate the gradient around the current input
            g = self.gradient(x0, uc);
            gnorm = sum([g[i]**2 for i in range(N)]);

            # check if gradient-norm is an approx. of zero
            if (gnorm < eps**2):
                brk = 1;
                break;

            # calculate the next iteration of the input
            if (a_method == "bkl"):
                (un, Cn, _, _, _) = self.alpha_bkl(g, Cc, x0, uc);
            else:
                un = [uc[i] - alpha*g[i] for i in range(P*N)];
                x  = self.simulate(x0, un);
                Cn = self.cost(self, x, un);

            count += 1;  # iterate the loop counter

            if (math.isnan(Cn)):
                brk = -2;
                break;

            if output:
                # print("Gradient:  ", g);
                print("|g|:          ", gnorm);
                print("New Cost:     ", Cn);
                print("New Input: ");
                for i in range(0, N*P, N):
                    print("              ", [un[i+j] for j in range(N)]);

            # break conditions
            if (count > imax):
                brk = -1;
                break;

            if (math.fabs(Cn - Cc) < eps):
                brk = 2;
                break;

            # update loop variables
            uc = un;  Cc = Cn;

        return (un, Cn, count, brk);

    def gradient(self, x0, u, rownum=1):
        # variable setup
        N = self.u_num*self.PH;
        h = self.h;
        g = [0 for i in range(N)];
        params = self.params;

        for i in range(rownum-1, N):
            un1 = [u[j] - (i==j)*h for j in range(N)];
            up1 = [u[j] + (i==j)*h for j in range(N)];

            xn1 = self.simulate(x0, un1);
            xp1 = self.simulate(x0, up1);

            Cn1 = self.cost(self, xn1, un1);
            Cp1 = self.cost(self, xp1, up1);

            g[i] = (Cp1 - Cn1)/(2*h);

        return g;

    def alpha_bkl(self, g, C, x0, uc):
        P = self.PH;
        N = self.u_num;
        eps = self.zero;
        a = self._alpha;
        w = self._bkl_shrink;
        params = self.params;

        count = 0;
        brk = -1;
        while ((count != 1000) & (a > eps)):
            ubkl = [uc[i] - a*g[i] for i in range(P*N)];
            x  = self.simulate(x0, ubkl);
            Cbkl = self.cost(self, x, ubkl);
            count += 1;

            if (Cbkl < C):
                brk = 0;
                break;

            a *= (1 - w);

        return (ubkl, Cbkl, a, count, brk);

    def simulate(self, x0, u):
        # mpc variables
        N  = self.x_num;
        Nu = self.u_num;
        P  = self.PH;
        params = self.params;

        # reshape input variable
        uc = [u[i:i+Nu] for i in range(0,P*Nu,Nu)];

        # Cost of each input over the designated windows
        # simulate over the prediction horizon and sum cost
        x = [[0 for i in range(N)] for j in range(P+1)];
        x[0] = x0;
        for i in range(P):
            if self.type == 'continuous':
                x[i+1] = self.modeuler(x[i], uc[i])[1][-1];
            elif self.type =='discrete':
                x[i+1] = self.model(x[i], uc[i], params);

        return x;

    def modeuler(self, x0, u, knot_length=0):
        N  = self.x_num;
        P  = self.PH;
        dt = self.dt;
        dt_min = self._dt_min;
        params = self.params;

        if (knot_length == 0):  k = self.k;
        else:  k = knot_length;

        if (dt >= dt_min):  adj = int(dt/dt_min);
        else:  adj = 1;

        km = k*adj;  dtm = dt/adj;
        x  = [[0 for j in range(N)] for i in range(k+1)];
        xm = [[0 for j in range(N)] for i in range(km+1)];

        x[0]  = x0;
        xm[0] = x0;
        for i in range(km):
            dx1 = self.model(xm[i], u, params);
            xeu = [xm[i][j] + dx1[j]*dtm for j in range(N)];
            dx2 = self.model(xeu, u, params);
            xm[i+1] = [xm[i][j] + 1/2*(dx1[j] + dx2[j])*dtm for j in range(N)];

            if ((i+1) % adj == 0):  x[int(i/adj+1)] = xm[i+1];

        T = [i*dt for i in range(k+1)];

        return (T, x);

    def sim_root(self, sim_time, x0, u0, callback=None, output=0):
        # mpc variables
        N  = self.u_num;
        P  = self.PH;
        dt = self.dt;
        params = self.params;

        # simulation time variables
        Nt = int(sim_time/dt+1);
        T = [i*dt for i in range(Nt)];

        # state matrices declarations
        xlist = [0 for i in range(Nt)];
        xlist[0] = x0;

        # return variables
        ulist = [0 for i in range(Nt)];
        Clist = [0 for i in range(Nt)];
        nlist = [0 for i in range(Nt)];
        brklist = [100 for i in range(Nt)];
        tlist = [0 for i in range(Nt)];

        ulist[0] = u0;

        for i in range(1,Nt):
            if output:  print("\nTime: %0.3f" % (T[i]));

            opt_results = self.solve(xlist[i-1], ulist[i-1], output);

            ulist[i]   = opt_results[0];
            Clist[i]   = opt_results[1];
            nlist[i]   = opt_results[2];
            brklist[i] = opt_results[3];
            tlist[i]   = opt_results[4];

            if output:  print("Elapsed Time:\n              ", tlist[i]);

            if self.type == 'continuous':
                xlist[i] = self.modeuler(xlist[i-1], ulist[i][:N])[1][1];
            elif self.type == 'discrete':
                xlist[i] = self.model(xlist[i-1], ulist[i][:N], params);

            if (callback is not None):  self.params = callback(self, T[i], xlist[i], ulist[i]);

        return (T, xlist, ulist, Clist, nlist, brklist, tlist);
