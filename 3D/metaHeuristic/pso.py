from metaHeuristic.commom import Model, Solution1, Solution2
from metaHeuristic.commom import createModel, plotBestCost, createRandomSolution, plotSolution, myCost
from helper.utils import distancia_rota3D, diminuir_pontos3D, tirarRepertido3D
import numpy as np
import time
from curves import bSpline
from helper.ambiente import Pontos
from curves.spline3D import generate_curve

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d

class Particle:
    def __init__(self, positionx, positiony, positionz, velocityx, velocityy, velocityz, cost, sol, bestPositionx, bestPositiony, bestPositionz, bestCost, bestSol):
        self.positionx = positionx
        self.positiony = positiony
        self.positionz = positionz
        self.velocityx = velocityx
        self.velocityy = velocityy
        self.velocityz = velocityz
        self.sol = sol
        self.cost = cost
        self.bestPositionx = bestPositionx
        self.bestPositiony = bestPositiony
        self.bestPositionz = bestPositionz
        self.bestCost = bestCost
        self.bestSol = bestSol

def run(show=False, vmx=None, vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    start = time.time()
    model = createModel(vmx, vmy, vmz, startx, starty, startz)
    nVar = model.n
    varSize = [1, nVar]

    varMin, varMax = {}, {}
    varMin["x"] = model.xmin
    varMax["x"] = model.xmax
    varMin["y"] = model.ymin
    varMax["y"] = model.ymax
    varMin["z"] = model.zmin
    varMax["z"] = model.zmax

    # Parametros
    maxIt = 30
    nPop = 30 # 15
    w = 1
    wDamp = 0.98
    c1 = 1.5
    c2 = 1.5

    alpha = 0.1
    velMin, velMax = {}, {}
    velMax["x"] = alpha*(varMax["x"]-varMin["x"])
    velMin["x"] = -velMax["x"]
    velMax["y"] = alpha*(varMax["y"]-varMin["y"])
    velMin["y"] = -velMax["y"]
    velMax["z"] = alpha*(varMax["z"]-varMin["z"])
    velMin["z"] = -velMax["z"]

    particle = Particle([0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop)
    globalBest = {"cost": float("inf"), "sol": [], "positionx": [], "positiony": []}
    
    for i in range(nPop):
        if i > 0:
            aux = createRandomSolution(model)
            particle.positionx[i] = aux.x
            particle.positiony[i] = aux.y
            particle.positionz[i] = aux.z
        else:
            # uma linha reta ate o objetivo
            xx = np.linspace(model.xs, model.xt, model.n+2)
            yy = np.linspace(model.ys, model.yt, model.n+2)
            zz = np.linspace(model.zs, model.zt, model.n+2)
            xx = np.array([3.6, 6.1, 7.1, 9.5, 16.9, 18, 19]) 
            yy = np.array([8.3, 8.5, 2.7, 1.2, 2.5, 7, 9]) 
            zz = np.array([2, 2.2, 0.8, 0.5, 2.5, 2.5, 3])
            particle.positionx[i] = xx[1:-1]
            particle.positiony[i] = yy[1:-1] 
            particle.positionz[i] = zz[1:-1] 

        particle.velocityx[i] = np.zeros(varSize)
        particle.velocityy[i] = np.zeros(varSize)
        particle.velocityz[i] = np.zeros(varSize)

        particle.cost[i], particle.sol[i] = myCost(particle.positionx[i], particle.positiony[i], particle.positionz[i], model)

        particle.bestPositionx[i] = particle.positionx[i]
        particle.bestPositiony[i] = particle.positiony[i]
        particle.bestPositionz[i] = particle.positionz[i]
        particle.bestCost[i] = particle.cost[i]
        particle.bestSol[i] = particle.sol[i]

        if particle.bestCost[i] < globalBest["cost"]:
            globalBest["cost"] = particle.bestCost[i]
            globalBest["sol"] = particle.sol[i]
            globalBest["positionx"] = particle.positionx[i]
            globalBest["positiony"] = particle.positiony[i]
            globalBest["positionz"] = particle.positionz[i]

    bestCost = np.zeros([maxIt, 1])
    

    for it in range(maxIt):
        for i in range(nPop):
            # X
            # define velocidade
            particle.velocityx[i] = w * particle.velocityx[i] + c1 * np.random.uniform(0, 1, nVar) * (particle.bestPositionx[i] - particle.positionx[i]) + c2 * np.random.uniform(0, 1, nVar) * (globalBest["positionx"] - particle.positionx[i])
            
            # checa os extremos da velocidade
            particle.velocityx[i][0] = np.clip(np.asarray(particle.velocityx[i][0]), velMin["x"], velMax["x"])
            
            # atualiza a posicao
            particle.positionx[i] = particle.positionx[i] + particle.velocityx[i]
            
            # checa os extremos da posicao
            particle.positionx[i] = np.clip(np.asarray(particle.positionx[i][0]), varMin["x"], varMax["x"])
            
            # Y
            # define velocidade
            particle.velocityy[i] = w * particle.velocityy[i] + c1 * np.random.uniform(0, 1, nVar) * (particle.bestPositiony[i] - particle.positiony[i]) + c2 * np.random.uniform(0, 1, nVar) * (globalBest["positiony"] - particle.positiony[i])

            # checa os extremos da velocidade
            particle.velocityy[i][0] = np.clip(np.asarray(particle.velocityy[i][0]), velMin["y"], velMax["y"])
             
            # atualiza a posicao
            particle.positiony[i] = particle.positiony[i] + particle.velocityy[i]
  
            # checa os extremos da posicao
            particle.positiony[i] = np.clip(np.asarray(particle.positiony[i][0]), varMin["y"], varMax["y"])

            # Z
            # define velocidade
            particle.velocityz[i] = w * particle.velocityz[i] + c1 * np.random.uniform(0, 1, nVar) * (particle.bestPositionz[i] - particle.positionz[i]) + c2 * np.random.uniform(0, 1, nVar) * (globalBest["positionz"] - particle.positionz[i])

            # checa os extremos da velocidade
            particle.velocityz[i][0] = np.clip(np.asarray(particle.velocityz[i][0]), velMin["z"], velMax["z"])
             
            # atualiza a posicao
            particle.positionz[i] = particle.positionz[i] + particle.velocityz[i]
  
            # checa os extremos da posicao
            particle.positionz[i] = np.clip(np.asarray(particle.positionz[i][0]), varMin["z"], varMax["z"])
            
            particle.cost[i], particle.sol[i] = myCost(particle.positionx[i], particle.positiony[i], particle.positionz[i], model)
            
            if particle.cost[i] < particle.bestCost[i]:
                particle.bestPositionx[i] = particle.positionx[i]
                particle.bestPositiony[i] = particle.positiony[i]
                particle.bestPositionz[i] = particle.positionz[i]
                particle.bestCost[i] = particle.cost[i]
                particle.bestSol[i] = particle.sol[i]

                if particle.bestCost[i] < globalBest["cost"]:
                    globalBest["cost"] = particle.bestCost[i]
                    globalBest["sol"] = particle.sol[i]
                    globalBest["positionx"] = particle.positionx[i]
                    globalBest["positiony"] = particle.positiony[i]
                    globalBest["positionz"] = particle.positionz[i]
        
        
        bestCost[it] = globalBest["cost"]

        w *= wDamp
        # w = (it*(np.log10(1) - np.log10(0.1))/maxIt) - np.log10(1)
        
        if show:
            print("Iteration: " + str(it) + ": Best Cost = " + str(bestCost[it]))
            
        #########################################################################################
        ##### VARIA ##### VARIA ##### VARIA ##### VARIA ##### VARIA ##### VARIA ##### VARIA #####
        #########################################################################################
        if bestCost[it] < 17000:
            xxx, yyy, zzz = plotSolution(globalBest["sol"], model, show)
            break

        if it == maxIt - 1:
            #print("tempo: " + str(time.time()-start))
            xxx, yyy, zzz = plotSolution(globalBest["sol"], model, show)

    p = Pontos()
    xxx1, yyy1, zzz1 = diminuir_pontos3D(xxx, yyy, zzz, p.xobs, p.yobs, p.zobs, apf=True)
    xxx1, yyy1, zzz1 = tirarRepertido3D(xxx1, yyy1, zzz1)
    # print(xxx1)
    # print(yyy1)
    # print(zzz1)
    xxx, yyy, zzz = generate_curve(xxx1, yyy1, zzz1)
    # curv = bSpline.B_spline(new_a, new_b)  
    # xnew, ynew = curv.get_curv()

    if show:
        ax = plt.axes(projection = "3d")
        ax.plot3D(xxx, yyy, zzz, 'y-')
        ax.plot3D(xxx1, yyy1, zzz1, 'r-')
        ax.plot3D(p.xobs, p.yobs, p.zobs, '.k')
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        plt.show()

    # plotBestCost(bestCost, show)
    distance = distancia_rota3D(xxx, yyy, zzz)

    return distance, time.time() - start, xxx, yyy, zzz

if __name__ == "__main__":
    run()