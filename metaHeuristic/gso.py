from metaHeuristic.commom import Model, Solution1, Solution2
from metaHeuristic.commom import createModel, plotBestCost, createRandomSolution, plotSolution, myCost
from helper.utils import distancia_rota
import numpy as np
import time

class Glowworm:
    def __init__(self, positionx, positiony, rangex, rangey, luciferinx, luciferiny, cost, sol, bestPositionx, bestPositiony, bestCost, bestSol):
        self.positionx = positionx
        self.positiony = positiony
        self.rangex = rangex
        self.rangey = rangey
        self.luciferinx = luciferinx
        self.luciferiny = luciferiny
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

    varMin, varMax = {}, {}
    varMin["x"] = model.xmin
    varMax["x"] = model.xmax
    varMin["y"] = model.ymin
    varMax["y"] = model.ymax

    # Parametros
    maxIt = 30
    nPop = 30 #100

    range_init = 5.0
    range_boundary = 50.2

    luciferin_init = 25
    luciferin_decay = 0.4
    luciferin_enhancement = 0.6

    k_neigh = 20.0
    beta = 0.5

    step_size = 1

    glowworm = Glowworm([0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop, [0]*nPop)
    globalBest = {"cost": float("inf"), "sol": [], "positionx": [], "positiony": []}
    
    for i in range(nPop):
        if i > 0:
            aux = createRandomSolution(model)
            glowworm.positionx[i] = aux.x
            glowworm.positiony[i] = aux.y
        else:
            # uma linha reta ate o objetivo
            xx = np.linspace(model.xs, model.xt, model.n+2)
            yy = np.linspace(model.ys, model.yt, model.n+2)
            glowworm.positionx[i] = xx[1:-1]
            glowworm.positiony[i] = yy[1:-1] 

        glowworm.rangex[i] = [range_init] * nVar
        glowworm.rangey[i] = [range_init] * nVar

        glowworm.luciferinx[i] = [luciferin_init] * nVar
        glowworm.luciferiny[i] = [luciferin_init] * nVar

        vizinhos = {"x": [], "y": []}

        glowworm.cost[i], glowworm.sol[i] = myCost(glowworm.positionx[i], glowworm.positiony[i], model)

        glowworm.bestPositionx[i] = glowworm.positionx[i]
        glowworm.bestPositiony[i] = glowworm.positiony[i]
        glowworm.bestCost[i] = glowworm.cost[i]
        glowworm.bestSol[i] = glowworm.sol[i]

        if glowworm.bestCost[i] < globalBest["cost"]:
            globalBest["cost"] = glowworm.bestCost[i]
            globalBest["sol"] = glowworm.sol[i]
            globalBest["positionx"] = glowworm.positionx[i]
            globalBest["positiony"] = glowworm.positiony[i]

    bestCost = np.zeros([maxIt, 1])
    

    for it in range(maxIt):
        for i in range(nPop):
            # X
            # Atualizar luciferin
            glowworm.luciferinx[i] = (1-luciferin_decay) * np.asarray(glowworm.luciferinx[i]) + luciferin_enhancement * (glowworm.cost[i]/10)

            vizinhos["x"] = []

            # Define os vizinhos
            for k in range(nPop):
                dist = abs(glowworm.positionx[i] - glowworm.positionx[k])
                
                # se it estiver a no range visivel e com brilho maior
                if all(d != 0 for d in dist) and all(dist <= glowworm.rangex[i]) and all(glowworm.luciferinx[i] <= glowworm.luciferinx[k]):
                    vizinhos["x"].append(k)

            if len(vizinhos["x"]) > 0:
                # encontrar o no na direcao que o vagalume tem que seguir
                li = glowworm.luciferinx[i]
                sum_lk = sum(glowworm.luciferinx[i])
                neighbors_index = len(vizinhos["x"])

                # probabilidade que cada vizinho tem de ser seguido
                probs = np.zeros(neighbors_index)
                for j in range(neighbors_index):
                    probs[j] = abs(sum(glowworm.luciferinx[j]) - sum(li))
                probs /= abs(sum_lk - sum(probs.size * li))
                
                # escolher o vizinho
                acc = 0
                wheel = []
                for val in range(probs.size):
                    acc = acc + probs[val]
                    wheel.append(acc)
                wheel[-1] = 1

                rand_val = np.random.rand()
                following = i
                
                for k in range(len(wheel)):
                    if rand_val <= wheel[k]:
                        following = k
                        break
                toward_index = following
                
                # Atualiza a posicao
                glowworms = glowworm.positionx[i]
                toward = glowworm.positionx[toward_index]
                normV = np.linalg.norm(toward - glowworms) 
                
                if normV == 0 or normV == None:
                    normV == 5 # tamanho do passo
                
                new_position = glowworms + step_size * (toward - glowworms) / normV
                glowworm.positionx[i] = new_position

            if len(vizinhos["x"]) == 0:
                glowworm.positionx[i] = glowworm.positionx[i] - step_size if sum(glowworm.positionx[i]) > 0 else glowworm.positionx[i] + step_size
                glowworm.positionx[i] = np.clip(np.asarray(glowworm.positionx[i]), varMin["x"], varMax["x"])

            # atualiza alcance dos vizinhos
            for p in range(nVar):
                glowworm.rangex[i][p] = min(range_boundary, max(0.1, glowworm.rangex[i][p] + (beta * (k_neigh - len(vizinhos["x"])))))
            


            # Y
            # Atualizar luciferin
            glowworm.luciferiny[i] = (1-luciferin_decay) * np.asarray(glowworm.luciferiny[i]) + luciferin_enhancement * (glowworm.cost[i]/10)

            vizinhos["y"] = []

            # Define os vizinhos
            for k in range(nPop):
                dist = abs(glowworm.positiony[i] - glowworm.positiony[k])
                
                # se it estiver a no range visivel e com brilho maior
                if all(d != 0 for d in dist) and all(dist <= glowworm.rangey[i]) and all(glowworm.luciferiny[i] <= glowworm.luciferiny[k]):
                    vizinhos["y"].append(k)

            if len(vizinhos["y"]) > 0:
                # encontrar o no na direcao que o vagalume tem que seguir
                li = glowworm.luciferiny[i]
                sum_lk = sum(glowworm.luciferiny[i])
                neighbors_index = len(vizinhos["y"])

                # probabilidade que cada vizinho tem de ser seguido
                probs = np.zeros(neighbors_index)
                
                for j in range(neighbors_index):
                    probs[j] = abs(sum(glowworm.luciferiny[j]) - sum(li))
                probs /= abs(sum_lk - sum(probs.size * li))
                
                # escolher o vizinho
                acc = 0
                wheel = []
                for val in range(probs.size):
                    acc = acc + probs[val]
                    wheel.append(acc)
                wheel[-1] = 1

                rand_val = np.random.rand()
                following = i
                
                for k in range(len(wheel)):
                    if rand_val <= wheel[k]:
                        following = k
                        break
                toward_index = following
                
                # Atualiza a posicao
                glowworms = glowworm.positiony[i]
                toward = glowworm.positiony[toward_index]
                normV = np.linalg.norm(toward - glowworms) 
                
                if normV == 0 or normV == None:
                    normV == 5 # tamanho do passo
                
                new_position = glowworms + step_size * (toward - glowworms) / normV
                glowworm.positiony[i] = new_position

            if len(vizinhos["y"]) == 0:
                glowworm.positiony[i] = glowworm.positiony[i] - step_size if sum(glowworm.positiony[i]) > 0 else glowworm.positiony[i] + step_size
                glowworm.positiony[i] = np.clip(np.asarray(glowworm.positiony[i]), varMin["y"], varMax["y"])

            # Atualiza o alcance dos vizinhos
            for p in range(nVar):
                glowworm.rangey[i][p] = min(range_boundary, max(0.1, glowworm.rangey[i][p] + (beta * (k_neigh - len(vizinhos["y"])))))


            
            glowworm.cost[i], glowworm.sol[i] = myCost(glowworm.positionx[i], glowworm.positiony[i], model)
            
            if glowworm.cost[i] < glowworm.bestCost[i]:
                glowworm.bestPositionx[i] = glowworm.positionx[i]
                glowworm.bestPositiony[i] = glowworm.positiony[i]
                glowworm.bestCost[i] = glowworm.cost[i]
                glowworm.bestSol[i] = glowworm.sol[i]

                if glowworm.bestCost[i] < globalBest["cost"]:
                    globalBest["cost"] = glowworm.bestCost[i]
                    globalBest["sol"] = glowworm.sol[i]
                    globalBest["positionx"] = glowworm.positionx[i]
                    globalBest["positiony"] = glowworm.positiony[i]
        
        
        bestCost[it] = globalBest["cost"]

        if show:
            print("Iteration: " + str(it) + ": Best Cost = " + str(bestCost[it]))
            
        if it == maxIt - 1:
            #print("tempo: " + str(time.time()-start))
            xxx, yyy = plotSolution(globalBest["sol"], model, show)

    plotBestCost(bestCost, show)
    distance = distancia_rota(xxx, yyy)

    return distance, time.time() - start, xxx, yyy

if __name__ == "__main__":
    run()