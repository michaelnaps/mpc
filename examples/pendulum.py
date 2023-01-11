import sys
sys.path.insert(0, '../.');

import numpy as np

def pendulum(q, u):
    l = 1.0;
    g = 9.81;

    dq = [
        q[1],
        u + g/l*np.sin(q[0])
    ];

    return dq;

if __name__ == "__main__":
    pass;
