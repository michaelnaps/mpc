import sys
from os.path import expanduser
sys.path.insert(0, expanduser('~')+'/prog/mpc')
sys.path.insert(0, expanduser('~')+'/prog/geom')

from GEOM.Circle import *
from MPC.Vehicle2D import *

import MPC.Plant as plant
import MPC.Optimizer as opt


# Hyper parameter(s).
xd = np.array( [[5],[7],[0],[0]] )
dt = 0.025
P = 10
k = 1
Nx = 4
Nu = 2
max_iter = 10
model_type = 'discrete'

def init_sphereworld(fig, axs):
    wall = Circle( np.array( [[0],[0]] ), -10, fig=fig, axs=axs )
    obs1 = Circle( np.array( [[2],[5]] ), 2.5, fig=fig, axs=axs )
    obs2 = Circle( np.array( [[-5],[1]] ), 4., fig=fig, axs=axs )
    obs3 = Circle( np.array( [[0],[-7]] ), 3., fig=fig, axs=axs )

    return (wall, obs1, obs2, obs3)

fig, axs = plt.subplots()
sphereworld = init_sphereworld( fig, axs )

# Model and cost function initialization.
def model(x, u):
    c = 0.1

    # discrete steps
    xn = np.array( [
        x[0] + dt*x[2],
        x[1] + dt*x[3],
        (1 - c)*x[2] + dt*u[0],
        (1 - c)*x[3] + dt*u[1]
    ] )

    return xn

def cost(x, u):
    # Position cost.
    kx = 150
    C = kx*(x[0] - xd[0])**2 + kx*(x[1] - xd[1])**2

    # Velocity cost.
    kdx = 4
    C = C + kdx*(x[2]**2 + x[3]**2)

    # Control cost.
    ku = 0.1
    C += ku*(u[0]**2 + u[1]**2)

    # Barrier costs.
    ko = 1
    for sphere in sphereworld:
        C += ko/sphere.distance(x[:2])**2

    return C


# Main execution block.
if __name__ == "__main__":
    x0 = np.array( [[-1.68],[-9.6],[0],[0]] )
    uinit = np.zeros( (Nu, P) )

    mpc_var = opt.ModelPredictiveControl( model, cost,
        P=P, k=k, Nu=Nu, Nx=Nx, dt=dt, model_type=model_type )
    mpc_var.setStepSize( 0.1 )
    mpc_var.setMaxIter( max_iter )

    m_var = plant.Model( model, dt=dt )

    axs.plot( xd[0], xd[1], marker='*', color='g' )
    for sphere in init_sphereworld( fig, axs ):
        sphere.draw()
    vhc = Vehicle2D( x0[:2], radius=0.2,
        fig=fig, axs=axs, tail_length=10 ).draw()
    plt.show( block=0 )

    T = 15;  Nt = round( T/dt ) + 1
    tList = np.array( [[i*dt for i in range( Nt )]] )
    print( 'Run simulation for %i steps.' % Nt )

    uList = np.zeros( (Nu*P, Nt) )
    xList = np.empty( (Nx, Nt) )
    xList[:,0] = x0[:,0]
    for i in range( Nt-1 ):
        x = xList[:,i,None]
        u = mpc_var.solve( x, uList[:,i], verbose=0 )
        uList[:,i+1] = u.reshape( Nu*P, )
        xList[:,i+1] = m_var.prop( x, u[:,0] )[:,0]
        if i % 10 == 0:
            print( 'step =', i )
        #     print( 'x:', xList[:,i+1] )
        #     print( uList[:,i+1] )
        vhc.update( xList[:2,i+1,None] )
        # vhc.draw()

    fig2, axs2 = plt.subplots()
    axs2.plot( tList.T, xList.T )
    plt.show()