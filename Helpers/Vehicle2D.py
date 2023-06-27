import numpy as np

from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib import path

# Class: Vehicle2D
# Assumptions: Model is discrete.
class Vehicle2D:
    def __init__(self, F, x0,
            fig=None, axs=None, zorder=10,
            vehicle_color='yellowgreen',
            draw_tail=1, tail_length=100,
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
        # self.fig.tight_layout();

        # vehicle parameters
        self.color = vehicle_color;
        self.edge_color = 'k';
        self.zorder = zorder;  # needed when multiple vhc present
        self.label = None;
        self.radius = 0.5;

        # tail parameters
        self.draw_tail = draw_tail;
        self.Nt = tail_length;
        self.linewidth = 2;
        self.linestyle = None;

        # simulation pause
        self.pause = pause;

        # draw vehicle and tail
        self.drawVehicle( x0 );
        if self.draw_tail:
            self.drawTail( np.kron( x0,np.ones( (1,self.Nt) ) ) );

    def draw(self, block=0):
        # show plot and pause
        plt.show( block=block );
        plt.pause( self.pause );

        # Return instance of self.
        return self;

    def update(self, x):
        # update vehicle location
        self.body.remove();
        self.drawVehicle( x );

        # update tail location if applicable
        if self.draw_tail:
            self.tail_patch.remove();
            self.drawTail( x );

        # Return instance of self.
        return self;

    def drawVehicle(self, x):
        # create vehicle circle
        self.body = patches.Circle( x[:,0], self.radius,
            facecolor=self.color, edgecolor=self.edge_color,
            zorder=self.zorder );

        # add to plot
        self.axs.add_patch( self.body );

        # Return instance of self.
        return self;

    def drawTail(self, x):
        # if x is a data set
        if x.shape[1] > 1:
            self.tail = x;
        else:
            self.tail[:,:-1] = self.tail[:,1:];
            self.tail[:,-1] = x[:,0];

        # create vehicle tail object
        self.tail_patch = patches.PathPatch(path.Path(self.tail.T),
            color=self.color, linewidth=self.linewidth, linestyle=self.linestyle,
            fill=0, zorder=1);
        self.axs.add_patch( self.tail_patch );

        # Return instance of self.
        return self;

    def setLimits(self, xlim=None, ylim=None):
        if xlim is not None:
            self.axs.set_xlim( xlim[0], xlim[1] );
        if ylim is not None:
            self.axs.set_ylim( ylim[0], ylim[1] );
        # Return instance of self.
        return self;

    def setFigureDimensions(self, w=None, h=None):
        if w is not None:
            self.fig.set_figwidth( w );
        if h is not None:
            self.fig.set_figheight( h );
        # Return instance of self.
        return self;
