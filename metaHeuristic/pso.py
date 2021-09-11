from metaHeuristic.commom import Model, Solution1, Solution2
from metaHeuristic.commom import createModel, plotBestCost, createRandomSolution, plotSolution, myCost
from helper.utils import distancia_rota, diminuir_pontos
import numpy as np
import time
from curves import bSpline
from helper.ambiente import Pontos
import matplotlib.pyplot as plt

class Particle:
    def __init__(self, positionx, positiony, velocityx, velocityy, cost, sol, bestPositionx, bestPositiony, bestCost, bestSol):
        self.positionx = positionx
        self.positiony = positiony
        self.velocityx = velocityx
        self.velocityy = velocityy
        self.sol = sol
        self.cost = cost
        self.bestPositionx = bestPositionx
        self.bestPositiony = bestPositiony
        self.bestCost = bestCost
        self.bestSol = bestSol

def run(show=False, vmx=None, vmy=None, startx=None, starty=None, p1=None):
    start = time.time()
    model = createModel(vmx, vmy, startx, starty)
    nVar = model.n
    varSize = [1, nVar]

    varMin, varMax = {}, {}
    varMin["x"] = model.xmin
    varMax["x"] = model.xmax
    varMin["y"] = model.ymin
    varMax["y"] = model.ymax

    # Parametros
    maxIt = 30
    nPop = 20 # 15
    w = 1
    wDamp = 0.9
    c1 = 1.5
    c2 = 2.5

    alpha = 0.5
    velMin, velMax = {}, {}
    velMax["x"] = alpha*(varMax["x"]-varMin["x"])
    velMin["x"] = -velMax["x"]
    velMax["y"] = alpha*(varMax["y"]-varMin["y"])
    velMin["y"] = -velMax["y"]

    particle = Particle([0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop)
    globalBest = {"cost": float("inf"), "sol": [], "positionx": [], "positiony": []}
    
    for i in range(nPop):
        if i > 0:
            aux = createRandomSolution(model)
            particle.positionx[i] = aux.x
            particle.positiony[i] = aux.y
        else:
            # uma linha reta ate o objetivo
            xx = np.linspace(model.xs, model.xt, model.n+2)
            yy = np.linspace(model.ys, model.yt, model.n+2)
            particle.positionx[i] = xx[1:-1]
            particle.positiony[i] = yy[1:-1] 

        particle.velocityx[i] = np.zeros(varSize)
        particle.velocityy[i] = np.zeros(varSize)

        particle.cost[i], particle.sol[i] = myCost(particle.positionx[i], particle.positiony[i], model)

        particle.bestPositionx[i] = particle.positionx[i]
        particle.bestPositiony[i] = particle.positiony[i]
        particle.bestCost[i] = particle.cost[i]
        particle.bestSol[i] = particle.sol[i]

        if particle.bestCost[i] < globalBest["cost"]:
            globalBest["cost"] = particle.bestCost[i]
            globalBest["sol"] = particle.sol[i]
            globalBest["positionx"] = particle.positionx[i]
            globalBest["positiony"] = particle.positiony[i]

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
            particle.velocityy[i] = w * particle.velocityy[i] + c1 * np.random.uniform(0, 1, nVar) * (particle.bestPositiony[i] - particle.positiony[i]) + c2 * 2 * np.random.uniform(0, 1, nVar) * (globalBest["positiony"] - particle.positiony[i])

            # checa os extremos da velocidade
            particle.velocityy[i][0] = np.clip(np.asarray(particle.velocityy[i][0]), velMin["y"], velMax["y"])
             
            # atualiza a posicao
            particle.positiony[i] = particle.positiony[i] + particle.velocityy[i]
  
            # checa os extremos da posicao
            particle.positiony[i] = np.clip(np.asarray(particle.positiony[i][0]), varMin["y"], varMax["y"])
            
            particle.cost[i], particle.sol[i] = myCost(particle.positionx[i], particle.positiony[i], model)
            
            if particle.cost[i] < particle.bestCost[i]:
                particle.bestPositionx[i] = particle.positionx[i]
                particle.bestPositiony[i] = particle.positiony[i]
                particle.bestCost[i] = particle.cost[i]
                particle.bestSol[i] = particle.sol[i]

                if particle.bestCost[i] < globalBest["cost"]:
                    globalBest["cost"] = particle.bestCost[i]
                    globalBest["sol"] = particle.sol[i]
                    globalBest["positionx"] = particle.positionx[i]
                    globalBest["positiony"] = particle.positiony[i]
        
        
        bestCost[it] = globalBest["cost"]

        w *= wDamp
        # w = (it*(np.log10(1) - np.log10(0.1))/maxIt) - np.log10(1)
        
        if show:
            print("Iteration: " + str(it) + ": Best Cost = " + str(bestCost[it]))

        if bestCost[it] < 100:
            xxx, yyy = plotSolution(globalBest["sol"], model, show)
            break

        if it == maxIt - 1:
            #print("tempo: " + str(time.time()-start))
            xxx, yyy = plotSolution(globalBest["sol"], model, show)

    p = Pontos()
    new_a, new_b = diminuir_pontos(xxx, yyy, p.xobs, p.yobs, apf=True)
    curv = bSpline.B_spline(new_a, new_b)  
    xnew, ynew = curv.get_curv()

    plotBestCost(bestCost, show)
    distance = distancia_rota(xnew, ynew)

    if show:
        plt.plot(p.xobs, p.yobs, ".k")
        plt.plot(np.clip(xnew, 1, p.limiar[0]-1), np.clip(ynew, 1, p.limiar[1]-1))
        plt.show()
    
    return distance, time.time() - start, np.clip(xnew, 1, p.limiar[0]-1), np.clip(ynew, 1, p.limiar[1]-1)

if __name__ == "__main__":
    run()