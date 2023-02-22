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
        self.q_num  = num_ssvar;
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

    def solve(self, q0, uinit, output=0):

        if (self.solver == 'nno'):
            print("ERROR: nno algorithm is deprecated for MicroPython applications.");
            return;
        #     t = time.time();
        #     (u, C, n, brk) = self.nno(q0, uinit, output);
        #     elapsed = 1000*(time.time() - t);
        if (self.solver == 'ngd'):
            t = time.time();
            (u, C, n, brk) = self.ngd(q0, uinit, output);
            elapsed = 1000*(time.time() - t);

        return (u, C, n, brk, elapsed);

    def ngd(self, q0, uinit, output=0):
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
        q  = self.simulate(q0, uc);
        Cc = self.cost(self, q, uc);
        un = uc;  Cn = Cc;

        if output:
            print("Opt. Start:");
            print("Initial Cost: ", Cc);

        count = 0;
        brk = -2*math.isnan(Cc);
        while (Cc > eps):
            # calculate the gradient around the current input
            g = self.gradient(q0, uc);
            gnorm = math.sqrt(sum([g[i]**2 for i in range(N)]));

            # check if gradient-norm is an approx. of zero
            if (gnorm < eps):
                brk = 1;
                break;

            # calculate the next iteration of the input
            if (a_method == "bkl"):
                (un, Cn, _, _, _) = self.alpha_bkl(g, Cc, q0, uc);
            else:
                un = [uc[i] - alpha*g[i] for i in range(P*N)];
                q  = self.simulate(q0, un);
                Cn = self.cost(self, q, un);

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

    def gradient(self, q0, u, rownum=1):
        # variable setup
        N = self.u_num*self.PH;
        h = self.h;
        g = [0 for i in range(N)];
        params = self.params;

        for i in range(rownum-1, N):
            un1 = [u[j] - (i==j)*h for j in range(N)];
            up1 = [u[j] + (i==j)*h for j in range(N)];

            qn1 = self.simulate(q0, un1);
            qp1 = self.simulate(q0, up1);

            Cn1 = self.cost(self, qn1, un1);
            Cp1 = self.cost(self, qp1, up1);

            g[i] = (Cp1 - Cn1)/(2*h);

        return g;

    def alpha_bkl(self, g, C, q0, uc):
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
            q  = self.simulate(q0, ubkl);
            Cbkl = self.cost(self, q, ubkl);
            count += 1;

            if (Cbkl < C):
                brk = 0;
                break;

            a *= (1 - w);

        return (ubkl, Cbkl, a, count, brk);

    def simulate(self, q0, u):
        # mpc variables
        N  = self.q_num;
        Nu = self.u_num;
        P  = self.PH;
        params = self.params;

        # reshape input variable
        uc = [u[i:i+Nu] for i in range(0,P*Nu,Nu)];

        # Cost of each input over the designated windows
        # simulate over the prediction horizon and sum cost
        q = [[0 for i in range(N)] for j in range(P+1)];
        q[0] = q0;
        for i in range(P):
            if self.type == 'continuous':
                q[i+1] = self.modeuler(q[i], uc[i])[1][-1];
            elif self.type =='discrete':
                q[i+1] = self.model(q[i], uc[i], params);

        return q;

    def modeuler(self, q0, u, knot_length=0):
        N  = self.q_num;
        P  = self.PH;
        dt = self.dt;
        dt_min = self._dt_min;
        params = self.params;

        if (knot_length == 0):  k = self.k;
        else:  k = knot_length;

        if (dt >= dt_min):  adj = int(dt/dt_min);
        else:  adj = 1;

        km = k*adj;  dtm = dt/adj;
        q  = [[0 for j in range(N)] for i in range(k+1)];
        qm = [[0 for j in range(N)] for i in range(km+1)];

        q[0]  = q0;
        qm[0] = q0;
        for i in range(km):
            dq1 = self.model(qm[i], u, params);
            qeu = [qm[i][j] + dq1[j]*dtm for j in range(N)];
            dq2 = self.model(qeu, u, params);
            qm[i+1] = [qm[i][j] + 1/2*(dq1[j] + dq2[j])*dtm for j in range(N)];

            if ((i+1) % adj == 0):  q[int(i/adj+1)] = qm[i+1];

        T = [i*dt for i in range(k+1)];

        return (T, q);

    def sim_root(self, sim_time, q0, u0, callback=None, output=0):
        # mpc variables
        N  = self.u_num;
        P  = self.PH;
        dt = self.dt;
        params = self.params;

        # simulation time variables
        Nt = int(sim_time/dt+1);
        T = [i*dt for i in range(Nt)];

        # state matrices declarations
        qlist = [0 for i in range(Nt)];
        qlist[0] = q0;

        # return variables
        ulist = [0 for i in range(Nt)];
        Clist = [0 for i in range(Nt)];
        nlist = [0 for i in range(Nt)];
        brklist = [100 for i in range(Nt)];
        tlist = [0 for i in range(Nt)];

        ulist[0] = u0;

        for i in range(1,Nt):
            if output:  print("\nTime: %0.3f" % (T[i]));

            opt_results = self.solve(qlist[i-1], ulist[i-1], output);

            ulist[i]   = opt_results[0];
            Clist[i]   = opt_results[1];
            nlist[i]   = opt_results[2];
            brklist[i] = opt_results[3];
            tlist[i]   = opt_results[4];

            if output:  print("Elapsed Time:\n              ", tlist[i]);

            if self.type == 'continuous':
                qlist[i] = self.modeuler(qlist[i-1], ulist[i][:N])[1][1];
            elif self.type == 'discrete':
                qlist[i] = self.model(qlist[i-1], ulist[i][:N], params);

            if (callback is not None):  self.params = callback(self, T[i], qlist[i], ulist[i]);

        return (T, qlist, ulist, Clist, nlist, brklist, tlist);
