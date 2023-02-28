import math
import matplotlib.pyplot as plt 
import numpy as np 


class BezierPathSmoothing:

    def __init__(self, ctr_points, num_waypoints=500):
        self._ctr_points = ctr_points
        self._num_ctr_points = len(ctr_points) 
        self._t = np.linspace(0,1 , num_waypoints)
        self._x_bezier = np.zeros((1, num_waypoints))
        self._y_bezier = np.zeros((1, num_waypoints))
        self._output = None

    def _binomial(self, n, i):
        result = math.factorial(n) / (math.factorial(i) * math.factorial(n-i))
        return result

    def _basis_func(self, n , i, t):
        result = self._binomial(n, i) * (t**i) * (1 - t) **(n-i)
        return result

    def compute_smooth_path(self):

        n = self._num_ctr_points-1

        for i, val in enumerate(self._ctr_points):
            x, y = val[0], val[1]

            self._x_bezier += self._basis_func(n, i, self._t) * x
            self._y_bezier += self._basis_func(n, i, self._t) * y 
        self._output = zip(self._x_bezier, self._y_bezier)
        return self._x_bezier, self._y_bezier

    def plot_path(self):

        self.compute_smooth_path()

        fig = plt.figure()
        ax = fig.add_subplot(111)

        x = [] 
        y = [] 

        for _x, _y in self._ctr_points:

            x.append(_x)
            y.append(_y)

        ax.plot(self._x_bezier[0], self._y_bezier[0])

        ax.scatter(x, y, c='black')
        plt.show()
            

if __name__ == "__main__":
    ctr_points = [(0,0), (20,5), (18, 60)]

    b = BezierPathSmoothing(ctr_points=ctr_points)
    b.compute_smooth_path()