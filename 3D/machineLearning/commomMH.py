import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt
from helper.ambiente import Pontos
import math
from helper.utils import colidir

class Model:
    def __init__(self, xs, ys, zs, xt, yt, zt, xobs, yobs, zobs, robs, n, xmin, ymin, zmin, xmax, ymax, zmax):
        self.xs = xs
        self.ys = ys
        self.zs = zs
        self.xt = xt
        self.yt = yt
        self.zt = zt
        self.xobs = xobs
        self.yobs = yobs
        self.zobs = zobs
        self.robs = robs
        self.n = n
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.zmin = zmin
        self.zmax = zmax

class Solution1:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Solution2:
    def __init__(self, TS, XS, YS, ZS, tt, xx, yy, zz, dx, dy, dz, L, Violation):
        self.TS = TS
        self.XS = XS
        self.YS = YS
        self.ZS = ZS
        self.tt = tt
        self.xx = xx
        self.yy = yy
        self.zz = zz
        self.dx = dx
        self.dy = dy
        self.dz = dz
        self.L = L
        self.Violation = Violation
        self.IsFeasible = (Violation==0)

def createModel(vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None):
    p = Pontos()
    # start node
    xs = p.xs if startx != None else startx
    ys = p.ys if startx != None else starty
    zs = p.zs if startx != None else startz

    # goal node
    xt = p.xt
    yt = p.yt
    zt = p.zt

    # coordenadas e raio dos obstaculos
    xobs = p.xobs if startx != None else vmx
    yobs = p.yobs if startx != None else vmy
    zobs = p.zobs if startx != None else vmz
    robs = p.robs if startx != None else [3] * len(xobs)

    # quantidade de pontos principais
    n = 5

    # limite que pode andar no x
    xmin = p.xmin
    xmax = p.xmax

    # limite que pode andar no y
    ymin = p.ymin
    ymax = p.ymax

    # limite que pode andar no z
    zmin = p.zmin
    zmax = p.zmax
    
    model = Model(xs, ys, zs, xt, yt, zt, xobs, yobs, zobs, robs, n, xmin, ymin, zmin, xmax, ymax, zmax)
    
    return model

def createRandomSolution(model):
    n = model.n

    xmin = model.xmin
    xmax = model.xmax

    ymin = model.ymin
    ymax = model.ymax

    zmin = model.zmin
    zmax = model.zmax

    sol1 = Solution1(np.random.uniform(xmin, xmax, n), np.random.uniform(ymin, ymax, n), np.random.uniform(zmin, zmax, n))
    return sol1

def parseSolution(sol1x, sol1y, sol1z, model):
    x = sol1x
    y = sol1y
    z = sol1z

    xs = model.xs
    ys = model.ys
    zs = model.zs
    xt = model.xt
    yt = model.yt
    zt = model.zt

    xobs = model.xobs
    yobs = model.yobs
    zobs = model.zobs
    robs = model.robs
    
    XS = np.concatenate((np.asarray([xs]), np.asarray(x), np.asarray([xt])), axis=0)
    YS = np.concatenate((np.asarray([ys]), np.asarray(y), np.asarray([yt])), axis=0)
    ZS = np.concatenate((np.asarray([zs]), np.asarray(z), np.asarray([zt])), axis=0)
    k = XS.size
    TS = np.linspace(0, 1, k)
    
    tt = np.linspace(0, 1, 100)
    tcx = interpolate.splrep(TS, XS)
    tcy = interpolate.splrep(TS, YS)
    tcz = interpolate.splrep(TS, ZS)
    xx = interpolate.splev(tt, tcx)
    yy = interpolate.splev(tt, tcy)
    zz = interpolate.splev(tt, tcz)
    
    dx = np.diff(xx)
    dy = np.diff(yy)
    dz = np.diff(zz)
    
    L = sum(np.sqrt(dx**2 + dy**2 + dz**2))
    
    nobs = len(xobs)
    Violation = 0
    for i in range(nobs):
        # if colidir(xobs, yobs, xx, yy, xt, yt):
        #     Violation = Violation + 1

        d = np.sqrt((xx-xobs[i])**2+(yy-yobs[i])**2+(zz-zobs[i])**2)
        v = max(np.concatenate((1-np.asarray(d)/robs[i], np.asarray([0])), axis=0))
        Violation = Violation + np.mean(math.ceil(v))

    sol2 = Solution2(TS, XS, YS, ZS, tt, xx, yy, zz, dx, dy, dz, L, Violation)
    
    return sol2

def myCost(sol1x, sol1y, sol1z, model):
    sol = parseSolution(sol1x, sol1y, sol1z, model)
    beta = 100
    z = sol.L*(1+beta*sol.Violation)
    return z, sol

def plotSolution(sol, model, show):
    xobs = model.xobs
    yobs = model.yobs
    zobs = model.zobs
    robs = model.robs

    XS = sol.XS
    YS = sol.YS
    ZS = sol.ZS
    xx = sol.xx
    yy = sol.yy
    zz = sol.zz

    theta = np.linspace(0, 2*np.pi, 100)

    # if show:
    #     for i in range(len(xobs)):
    #         plt.fill(xobs[i]+robs[i]*np.cos(theta),yobs[i]+robs[i]*np.sin(theta), "black")
    #     plt.plot(xx, yy, "k", LineWidth=2)
    #     plt.plot(XS, YS, ".r")
    #     plt.plot(xx, yy, MarkerSize=12, MarkerFaceColor="y")
    #     plt.plot(xx, yy, MarkerSize=16, MarkerFaceColor="g")
    #     plt.show()

    return xx, yy, zz

def plotBestCost(bestCost, show):
    if show:
        plt.plot(bestCost, LineWidth=2)
        plt.xlabel("Iteration")
        plt.ylabel("Best Cost")
        plt.show()