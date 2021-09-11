import matplotlib.pyplot as plt

class B_spline(object):
    def __init__(self, i_x, i_y):
        self.arg = [[-1, 3, -3, 1], [3, -6, 0, 4], [-3, 3, 3, 1], [1, 0, 0, 0]]
        self.x_new = []
        self.y_new = []
        self.x = i_x
        self.y = i_y

    def Ba(self, t, coefficient):
        return (coefficient[0] * t ** 3 + coefficient[1] * t ** 2 + coefficient[2] * t + coefficient[3]) / 6.

    def creat(self, n):
        for i in range(101):
            t = i / 100.
            self.x_new.append(
                self.x[n + 0] * self.Ba(t, self.arg[0]) + self.x[n + 1] * self.Ba(t, self.arg[1]) + self.x[n + 2] * \
                self.Ba(t, self.arg[2]) + self.x[n + 3] * self.Ba(t, self.arg[3]))
            self.y_new.append(
                self.y[n + 0] * self.Ba(t, self.arg[0]) + self.y[n + 1] * self.Ba(t, self.arg[1]) + self.y[n + 2] * \
                self.Ba(t, self.arg[2]) + self.y[n + 3] * self.Ba(t, self.arg[3]))

    def pre(self, x, y):
        i = 0
        while i < 3:
            x.append(x[-1])
            y.append(y[-1])
            x.insert(0, x[0])
            y.insert(0, y[0])
            i += 1
        return x, y

    def get_curv(self):
        self.x, self.y = self.pre(self.x, self.y)
        n = len(self.x) - 3
        for i in range(n):
            self.creat(i)
        return self.x_new, self.y_new


if __name__ == '__main__':
    # x = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    # y = [12, 2, 78, 12, 34, 23, 67, 87, 98, 10]
    x = [85.1, 77, 66, 64]
    y = [38.1, 77, 77, 30]

    curv = B_spline(x, y)
    xnew, ynew = curv.get_curv()

    plt.plot(xnew, ynew)
    plt.plot(x, y)
    plt.scatter(x, y)
    plt.show()

    

