
import matplotlib.pyplot as plt

class Sphere:
    def __init__(self, center, radius, color='#9C9C9C'):
        self.x = center;
        self.r = radius;
        self.color = color;
        return;

    def distance(self, pt):
        if self.r < 0:
            neg = -1;
        else:
            neg =  1;

        d = ((pt[0] - self.x[0])**2 + (pt[1] - self.x[1])**2)**(1/2);
        return neg*d - self.r;

    def plot(self, axes=None):
        if axes is None:
            _, axes = plt.subplots();

        if self.r < 0:
            edge = self.color;
            face = 'none';
        else:
            edge = 'none';
            face = self.color;

        spherepatch = plt.Circle(tuple(self.x), self.r,
            facecolor=face, edgecolor=edge);

        axes.add_patch(spherepatch);
