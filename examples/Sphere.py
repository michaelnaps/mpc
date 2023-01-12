
import matplotlib.pyplot as plt

class Sphere:
    def __init__(self, center, radius, color='yellowgreen'):
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
        pass;
