#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import scipy.special

show_animation = True

class Bezier:
    def __init__(self, start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset, t):
        self.start_x = start_x
        self.start_y = start_y
        self.start_yaw = start_yaw   
        self.end_x = end_x  
        self.end_y = end_y  
        self.end_yaw = end_yaw
        self.offset = offset
        self.t = t

    def calc_4points_bezier_path(self):
        dist = np.sqrt((self.start_x - self.end_x) ** 2 + (self.start_y - self.end_y) ** 2) / self.offset
        control_points = np.array(
            [[self.start_x, self.start_y],
            [self.start_x + dist * np.cos(self.start_yaw), self.start_y + dist * np.sin(self.start_yaw)],
            [self.end_x - dist * np.cos(self.end_yaw), self.end_y - dist * np.sin(self.end_yaw)],
            [self.end_x, self.end_y]])

        path = self.calc_bezier_path(control_points, n_points=100)

        return path


    def calc_bezier_path(self, control_points, n_points=100):
        traj = []
        for t in np.linspace(0, 1, n_points):
            traj.append(self.bezier(t, control_points))

        return np.array(traj)


    def bernstein_poly(self, n, i, t):
        return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


    def bezier(self, t, control_points):
        n = len(control_points) - 1
        return np.sum([self.bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)


    def bezier_derivatives_control_points(self, control_points, n_derivatives):
        w = {0: control_points}
        for i in range(n_derivatives):
            n = len(w[i])
            w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
                                for j in range(n - 1)])
        return w


    def curvature(self, dx, dy, ddx, ddy):
        return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)


    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
        if not isinstance(x, float):
            for (ix, iy, iyaw) in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                    fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

    def smooth(self, path, weight_data=0.5, weight_smooth=0.1, tolerance=0.000001):
        newpath = [[0 for row in range(len(path[0]))] for col in range(len(path))]
        for i in range(len(path)):
            for j in range(len(path[0])):
                newpath[i][j] = path[i][j]

        change = tolerance
        while change >= tolerance:
            change = 0.0
            for i in range(1, len(path)-1):
                for j in range(len(path[0])):
                    aux = newpath[i][j]
                    newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                    newpath[i][j] += weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - (2.0 * newpath[i][j]))
                    change += abs(aux - newpath[i][j])
        return newpath
