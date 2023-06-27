import numpy as np

from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib import path

# Class: Vehicle2D
# Assumptions: Model is discrete.
class Vehicle2D:
    def __init__(self, F, x0,
            fig=None, axs=None,
            vehicle_color='yellowgreen',
            draw_tail=1, tail_length=10,
            grid=1, pause=1e-3):
        # create figure if not given
        if fig is None and axs is None:
            self.fig, self.axs = plt.subplots();
        else:
            self.fig = fig;  self.axs = axs;

        # figure scaling, grid, equal axes
        self.axs.set_xlim( -10, 10 );
        self.axs.set_ylim( -10, 10 );
        self.axs.axis( 'equal' );
        self.axs.grid( grid );

        # vehicle parameters
        self.color = vhc_color;
        self.edge_color = 'k';
        self.zorder = 1;  # needed when multiple vhc on one plot
        self.label = None;
        self.radius = 2.5;

        # tail parameters
        self.draw_tail = draw_tail;
        self.Nt = tail_length;

        # simulation pause
        self.pause = pause;

        # draw vehicle and tail
        self.drawVehicle( x0 );
        self.drawTail( np.kron( x0,np.ones( (self.Nt,1) ) ) );
