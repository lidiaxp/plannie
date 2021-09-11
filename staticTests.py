# -*- coding: utf-8 -*-
from classic import a_star, potential_field, probabilistic_roadmap, rrt, rrtConnect
from metaHeuristic import pso, gwo, gso
from machineLearning import deeprl, rl

from helper import ambiente, dinamico, localPlanner

from pathPlanning import nome

import matplotlib.pyplot as plt
import statistics as stc
import numpy as np
import time
import os

show = False

dirName = "logPathPlanner"
try:
    os.mkdir(dirName)
except:
    pass
nameFile = "logPathPlanning" + str(time.time().replace(".", "Tempo"))
file = open(dirName + "/" + nameFile + ".txt","w")

p = ambiente.Pontos()
ox = p.xobs
oy = p.yobs

astarTempo, apfTempo, prmTempo, rrtTempo, rrtcTempo, psoTempo, gwoTempo, gsoTempo, rlTempo, deeprlTempo = [], [], [], [], [], [], [], [], [], []
astarDistancia, apfDistancia, prmDistancia, rrtDistancia, rrtcDistancia, psoDistancia, gwoDistancia, gsoDistancia, rlDistancia, deeprlDistancia = [], [], [], [], [], [], [], [], [], []

for i in range(1):
    astarT, astarD, astarx, astary = a_star.run(show)
    # apfT, apfD, apfx, apfy = potential_field.run(show)
    # prmT, prmD, prmx, prmy = probabilistic_roadmap.run(show)
    # rrtT, rrtD, rrtx, rrty = rrt.run(show)
    # rrtcT, rrtcD, rrtcx, rrtcy = rrtConnect.run(show)
    # psoT, psoD, psox, psoy = pso.run(show)
    # gwoT, gwoD, gwox, gwoy = gwo.run(show)
    # gsoT, gsoD, gsox, gsoy = gso.run(show) 
    # rlT, rlD, rlx, rly = rl.run(show)
    # deeprlT, deeprlD, deeprlx, deeprly = deeprl.run(show)

    astarTempo.append(astarT)
    astarDistancia.append(astarD)
    # apfTempo.append(apfT)
    # apfDistancia.append(apfD)
    # prmTempo.append(prmT)
    # prmDistancia.append(prmD)
    # rrtTempo.append(rrtT)
    # rrtDistancia.append(rrtD)
    # rrtcTempo.append(rrtcT)
    # rrtcDistancia.append(rrtcD)
    # psoTempo.append(psoT)
    # psoDistancia.append(psoD)
    # gwoTempo.append(gwoT)
    # gwoDistancia.append(gwoD)
    # gsoTempo.append(gsoT)
    # gsoDistancia.append(gsoD)
    # rlTempo.append(rlT)
    # rlDistancia.append(rlD)
    # deeprlTempo.append(deeprlT)
    # deeprlDistancia.append(deeprlD)

# x = [astarx, apfx, prmx, rrtx, rrtcx, psox, gwox, gsox, rlx, deeprlx]
# y = [astary, apfy, prmy, rrty, rrtcy, psoy, gwoy, gsoy, rly, deeprly]
# tempo = [astarTempo, apfTempo, prmTempo, rrtTempo, rrtcTempo, psoTempo, gwoTempo, gsoTempo, rlTempo, deeprlTempo]
# distancia = [astarDistancia, apfDistancia, prmDistancia, rrtDistancia, rrtcDistancia, psoDistancia, gwoDistancia, gsoDistancia, rlDistancia, deeprlDistancia]

x = [astarx]
y = [astary]
tempo = [astarTempo]
distancia = [astarDistancia]

# for i in range(len(nome)):
#     file.write("Todos os tempos do " + nome[i] + ": \n")
#     for j in range(len(tempo[j])):
#         file.write(tempo[i]+"\n")

# for i in range(len(nome)):
#     file.write("Todos as distancias do " + nome[i] + ": \n")
#     for j in range(len(distancia[j])):
        # file.write(distancia[i]+"\n")

# Media
# for i in range(len(nome)):
for i in range(1):
    file.write("Media do tempo do " + nome[i] + ": " + str(np.mean(tempo[i])) + "\n")
    file.write("Media da distancia do " + nome[i] + ": " + str(np.mean(distancia[i])) + "\n")

    # Variancia
    file.write("Variancia do tempo do " + nome[i] + ": " + str(stc.variance(tempo[i])) + "\n")
    file.write("Variancia da distancia do " + nome[i] + ": " + str(stc.variance(distancia[i])) + "\n")

    # Mediana
    file.write("Mediana do tempo do " + nome[i] + ": " + str(stc.median(tempo[i])) + "\n")
    file.write("Mediana da distancia do " + nome[i] + ": " + str(stc.median(distancia[i])) + "\n")

    # Desvio padrao
    file.write("Desvio Padrao do tempo do " + nome[i] + ": " + str(stc.stdev(tempo[i])) + "\n")
    file.write("Desvio Padrao da distancia do " + nome[i] + ": " + str(stc.stdev(distancia[i])) + "\n")

# for i in range(len(x)):
for i in range(len(x)):
    file.write("Caminho X do " + nome[i] + ": " + str(x[i]) + "\n")
    file.write("Caminho Y do " + nome[i] + ": " + str(y[i]) + "\n")
    # file.write("Todos os tempos do " + nome[i] + ": " + str(tempo[i]) + "\n")
    # file.write("Todos as distancias do " + nome[i] + ": " + str(distancia[i]) + "\n")
    plt.axis("equal")
    plt.plot(ox, oy, ".b")
    plt.plot(x[i], y[i], "-r")
    plt.title(nome[i])
    plt.show()
    plt.savefig(dirName + "/" + nome[i] + str(time.time()).replace(".", "tempo") + '.png', format='png')

file.close()