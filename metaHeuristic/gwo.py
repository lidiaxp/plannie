from metaHeuristic.commom import Model, Solution1, Solution2
from metaHeuristic.commom import createModel, plotBestCost, createRandomSolution, plotSolution, myCost
from helper.utils import distancia_rota
import numpy as np
import time

class Wolf:
    def __init__(self, positionx, positiony, alfax, alfay, betax, betay, deltax, deltay, cost, sol, bestPositionx, bestPositiony, bestCost, bestSol):
        self.positionx = positionx
        self.positiony = positiony
        self.alfax = alfax
        self.alfay = alfay
        self.betax = betax
        self.betay = betay
        self.deltax = deltax
        self.deltay = deltay
        self.cost = cost
        self.sol = sol
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
    nPop = 30 # 50
    w = 2
    wDamp = 0.98
    c1 = 3.

    lobin = Wolf([0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop,[0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop)
    globalBest = {"alfaCost": float("inf"), "betaCost": float("inf"), "deltaCost": float("inf"), "sol": [], "positionAlfax": [], "positionAlfay": [], "positionBetax": [], "positionBetay": [], "positionDeltax": [], "positionDeltay": []}
    
    for i in range(nPop):
        if i > 0:
            aux = createRandomSolution(model)
            lobin.positionx[i] = aux.x
            lobin.positiony[i] = aux.y
        else:
            # uma linha reta ate o objetivo
            xx = np.linspace(model.xs, model.xt, model.n+2)
            yy = np.linspace(model.ys, model.yt, model.n+2)
            lobin.positionx[i] = xx[1:-1]
            lobin.positiony[i] = yy[1:-1] 

        lobin.alfax[i] = np.zeros(varSize)
        lobin.alfay[i] = np.zeros(varSize)
        lobin.betax[i] = np.zeros(varSize)
        lobin.betay[i] = np.zeros(varSize)
        lobin.deltax[i] = np.zeros(varSize)
        lobin.deltay[i] = np.zeros(varSize)

        lobin.cost[i], lobin.sol[i] = myCost(lobin.positionx[i], lobin.positiony[i], model)

        lobin.bestPositionx[i] = lobin.positionx[i]
        lobin.bestPositiony[i] = lobin.positiony[i]
        lobin.bestCost[i] = lobin.cost[i]
        lobin.bestSol[i] = lobin.sol[i]

        if lobin.bestCost[i] < globalBest["alfaCost"]:
            globalBest["alfaCost"] = lobin.bestCost[i]
            globalBest["sol"] = lobin.sol[i]
            globalBest["positionAlfax"] = lobin.positionx[i]
            globalBest["positionAlfay"] = lobin.positiony[i]

        if lobin.bestCost[i] > globalBest["alfaCost"] and lobin.bestCost[i] < globalBest["betaCost"]:
            globalBest["betaCost"] = lobin.bestCost[i]
            globalBest["positionBetax"] = lobin.positionx[i]
            globalBest["positionBetay"] = lobin.positiony[i]

        if lobin.bestCost[i] > globalBest["alfaCost"] and lobin.bestCost[i] > globalBest["betaCost"] and lobin.bestCost[i] < globalBest["deltaCost"]:
            globalBest["betaCost"] = lobin.bestCost[i]
            globalBest["positionDeltax"] = lobin.positionx[i]
            globalBest["positionDeltay"] = lobin.positiony[i]

    bestCost = np.zeros([maxIt, 1])
    

    for it in range(maxIt):
        for i in range(nPop):
            # X
            # define alfa x
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dAlfax = abs(aux * globalBest["positionAlfax"] - lobin.positionx[i])
            x1 = lobin.alfax[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dAlfax 
            
            # define beta x
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dBetax = abs(aux * globalBest["positionBetax"] - lobin.positionx[i])
            x2 = lobin.alfax[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dBetax 

            # define delta x
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dDeltax = abs(aux * globalBest["positionDeltax"] - lobin.positionx[i])
            x3 = lobin.alfax[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dDeltax 

            # atualiza posicao x
            lobin.positionx[i] = (x1+x2+x3)/3
            lobin.positionx[i] = np.clip(np.asarray(lobin.positionx[i]), varMin["x"], varMax["x"])

            # Y
            # define alfa x
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dAlfay = abs(aux * globalBest["positionAlfay"] - lobin.positiony[i])
            y1 = lobin.alfay[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dAlfay 

            # define beta y
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dBetay = abs(aux * globalBest["positionBetay"] - lobin.positiony[i])
            y2 = lobin.alfay[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dBetay 

            # define delta y
            aux = (c1 * w * np.random.rand()) * np.random.rand() # atualiza valor aleatorio
            dDeltay = abs(aux * globalBest["positionDeltay"] - lobin.positiony[i])
            y3 = lobin.alfay[i] - (c1 * w * np.random.uniform(0, 1, nVar) - w) * dDeltay 

            # atualiza posicao y
            lobin.positiony[i] = (y1+y2+y3)/3
            lobin.positiony[i] = np.clip(np.asarray(lobin.positiony[i]), varMin["y"], varMax["y"])

            lobin.cost[i], lobin.sol[i] = myCost(lobin.positionx[i][0], lobin.positiony[i][0], model)
            
            if lobin.cost[i] < lobin.bestCost[i]:
                lobin.bestPositionx[i] = lobin.positionx[i]
                lobin.bestPositiony[i] = lobin.positiony[i]
                lobin.bestCost[i] = lobin.cost[i]
                lobin.bestSol[i] = lobin.sol[i]

                if lobin.bestCost[i] < globalBest["alfaCost"]:
                    globalBest["alfaCost"] = lobin.bestCost[i]
                    globalBest["sol"] = lobin.sol[i]
                    globalBest["positionAlfax"] = lobin.positionx[i]
                    globalBest["positionAlfay"] = lobin.positiony[i]
        
                if lobin.bestCost[i] > globalBest["alfaCost"] and lobin.bestCost[i] < globalBest["betaCost"]:
                    globalBest["betaCost"] = lobin.bestCost[i]
                    globalBest["positionBetax"] = lobin.positionx[i]
                    globalBest["positionBetay"] = lobin.positiony[i]

                if lobin.bestCost[i] > globalBest["alfaCost"] and lobin.bestCost[i] > globalBest["betaCost"] and lobin.bestCost[i] < globalBest["deltaCost"]:
                    globalBest["deltaCost"] = lobin.bestCost[i]
                    globalBest["positionDeltax"] = lobin.positionx[i]
                    globalBest["positionDeltay"] = lobin.positiony[i]
        
        
        bestCost[it] = globalBest["alfaCost"]

        w *= wDamp
        
        if show:
            print("Iteration: " + str(it) + ": Best Cost = " + str(bestCost[it]))

        if it == maxIt - 1:
            xxx, yyy = plotSolution(globalBest["sol"], model, show)

    plotBestCost(bestCost, show)
    distance = distancia_rota(xxx, yyy)

    return distance, time.time() - start, xxx, yyy

if __name__ == "__main__":
    run()