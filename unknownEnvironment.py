# -*- coding: utf-8 -*-
from classic import biastar, biAPF, probabilistic_roadmap, rrt, birrt, a_star
from metaHeuristic import aco, pso, gwo, gso
from machineLearning import rl#, gworl#, deeprl

from helper.unknown import run
from helper.ambiente import Pontos
# from pathPlanning import nome
from helper.utils import memory_usage, diminuir_pontos
import matplotlib.pyplot as plt

nome = ["Bi A-Star", "Artificial Potential Field", "Probabilistic Roadmap", "Rapid Random Tree", "Rapid Random Tree Connect", "Ant Colony Optimization", "Particle Swarm Optimization", "Grey Wolf Optimization", "Glowworm Optimization", "Grey Wolf Optimization with Reinforcement Learning", "Reinforcement Learning", "Deep Reinforcement Learning"]

import numpy as np
import statistics as stc
import psutil
from datetime import datetime
import os

memoriaDoPc = 16000 # MB
processadorDoPc = 2800 # MHz

p = Pontos()

Astar = 1
Apf = 0
Prm = 0
Rrt = 0
RrtC = 0
Aco = False # n esta ajustado para rodar aki
Pso = 0
Gwo = 0
Gso = 0
GwoRl = 0
Rl = 0
DeepRl = False

valid = [Astar, Apf, Prm, Rrt, RrtC, Aco, Pso, Gwo, Gso, GwoRl, Rl, DeepRl]

def define_algorithm(q):
    if q == 0:
        return a_star, "BiAStar"
    elif q == 1:
        return biAPF, "BiAPF"
    elif q == 2:
        return probabilistic_roadmap, "PRM"
    elif q == 3:
        return rrt, "RRT"
    elif q == 4:
        return birrt, "RRTC"
    elif q == 5:
        return aco, "ACO"
    elif q == 6:
        return pso, "PSO"
    elif q == 7:
        return gwo, "GWO"
    elif q == 8:
        return gso, "GSO"
    elif q == 9:
        return gworl, "GWORL"
    elif q == 10:
        return rl, "RL"
    elif q == 11:
        return deeprl, "DRL"
    else:
        return a_star, "AStar" # default
    
if __name__ == "__main__":
    show = 0
    nInterations = 1
    rodarMTSVezes = True if nInterations >= 2 else False
    dt, tt, cc = [], [], []
    ppi, ppf, mmi, mmf, pp, mm = [], [], [], [], [], []
    menorCaminhoX, menorCaminhoY = [], []
    log = 0
    lastFileName = ""
    indexIn = 0
    # ADICIONAR BATERIA <------- ta no movimento ros
    for q in range(len(valid)):
        if valid[q]:
            cpu, memory = psutil.cpu_percent(), memory_usage()
            for i in range(nInterations):
                print(i)
                mmi.append(memory_usage())
                ppi.append(psutil.cpu_percent())
                # cpu, memory = psutil.cpu_count(), psutil.virtual_memory()
                alg, name = define_algorithm(q)
                fileName = "logPathPlanner/semUAV/classic/" + name + str(datetime.now().day) + str(datetime.now().month) + str(datetime.now().hour) + str(datetime.now().minute) + str(datetime.now().second) + ".txt"
                if log: f = open(fileName, "a")
                d, t, mx, my, notConclude = run(valid, q, alg, show=show, rmv=rodarMTSVezes) #completeness
                # plt.plot(p.visX, p.visY, ".k")
                # plt.plot(mx, my)
                # plt.show()
                if not notConclude: print("good")
                cc.append(notConclude)
                if notConclude == False or len(dt) == 0: 
                    dt.append(d)
                    tt = np.concatenate((tt, t), axis=0)
                    ppf.append(psutil.cpu_percent())
                    mmf.append(memory_usage())

                    pp.append(abs(ppf[indexIn] - ppi[indexIn]))
                    mm.append(abs(mmf[indexIn] - mmi[indexIn]))
                    indexIn += 1

                    if (d < min(dt) or len(menorCaminhoX) == 0):
                        menorCaminhoX = mx
                        menorCaminhoY = my
            
                ##################################################################################

            print("\n\n")
            print(nome[q])
            print(fileName)
            print("Iteracoes: " + str(i+1))
            print("Completeness: " + str((len(cc) - sum(cc)) * 100 / len(cc)))
            try: 
                print("Media da distancia: " + str(stc.mean(dt)))
                print("Melhor distancia: " + str(min(dt)))
                print("Pior distancia: " + str(max(dt)))
            except:
                print(dt)

            try: 
                print("Variancia da distancia: " + str(stc.variance(dt)))
                print("Desvio Padrao da distancia: " + str(stc.stdev(dt)))
            except:
                pass
            
            try:
                print("Media do tempo: " + str(stc.mean(tt)))
                print("Melhor Tempo: " + str(min(tt)))
                print("Pior Tempo: " + str(max(tt)))
            except:
                print(tt)
            try: 
                print("Variancia do tempo: " + str(stc.variance(tt)))
                print("Desvio Padrao do tempo: " + str(stc.stdev(tt)))
            except:
                pass

            print("CPU inicial: " + str(cpu))
            print("CPU final: " + str(psutil.cpu_percent()))
            print("Memoria inicial: " + str(memory))
            print("Memoria final: " + str(memory_usage()))

            print("Media da CPU inicial: " + str(stc.mean(ppi)))
            print("Media da CPU final: " + str(stc.mean(ppf)))
            print("Media da memoria inicial: " + str(stc.mean(mmi)))
            print("Media da memoria final: " + str(stc.mean(mmf)))
            print("Media da CPU: " + str(stc.mean(pp)))
            print("Media da memoria: " + str(stc.mean(mm)))
            print("Media da CPU (Valor Real): " + str(stc.mean(pp) * processadorDoPc / 100))
            print("Media da memoria (Valor Real): " + str(stc.mean(mm) * memoriaDoPc / 100))

            try:
                print("Variancia da CPU inicial: " + str(stc.variance(ppi)))
                print("Variancia da CPU final: " + str(stc.variance(ppf)))
                print("Variancia da memoria inicial: " + str(stc.variance(mmi)))
                print("Variancia da memoria final: " + str(stc.variance(mmf)))
                print("Variancia da CPU: " + str(stc.variance(pp)))
                print("Variancia da memoria: " + str(stc.variance(mm)))

                print("Desvio padrao da CPU inicial: " + str(stc.stdev(ppi)))
                print("Desvio padrao da CPU final: " + str(stc.stdev(ppf)))
                print("Desvio padrao da memoria inicial: " + str(stc.stdev(mmi)))
                print("Desvio padrao da memoria final: " + str(stc.stdev(mmf)))
                print("Desvio padrao da CPU: " + str(stc.stdev(pp)))
                print("Desvio padrao da memoria: " + str(stc.stdev(mm)))
                print("Desvio padrao da CPU (Valor Real): " + str(stc.stdev(pp) * processadorDoPc / 100))
                print("Desvio padrao da memoria (Valor Real): " + str(stc.stdev(mm) * memoriaDoPc / 100))
            except:
                pass

            try:
                print("Caminho X: " + str(menorCaminhoX.tolist()))
                print("Caminho Y: " + str(menorCaminhoY.tolist()))
            except:
                print("Caminho X: " + str(menorCaminhoX))
                print("Caminho Y: " + str(menorCaminhoY))

            plt.clf()
            plt.plot(p.visX, p.visY, ".k    ")
            plt.rcParams.update({'font.size': 20})
            plt.plot(menorCaminhoX, menorCaminhoY, "-r")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.savefig("path2D"+str(name)+str(nInterations)+".png")
            plt.show()

            # print("Completeness: " + str(np.sum(cr)*100/len(cr)) + "%")
            
            print("\n")

            ##################################################################################

            if log:
                f.write(nome[q] + "\n")
                f.write("Iteracoes: " + str(i+1))
                f.write("Completeness: " + str((len(cc) - sum(cc)) * 100 / len(cc)))
                f.write("Media da distancia: " + str(stc.mean(dt)) + "\n")
                f.write("Melhor distancia: " + str(min(dt)) + "\n")
                f.write("Pior distancia: " + str(max(dt)) + "\n")
                try: 
                    f.write("Variancia da distancia: " + str(stc.variance(dt)) + "\n")
                    f.write("Desvio Padrao da distancia: " + str(stc.stdev(dt)) + "\n")
                except:
                    pass
                
                f.write("Media do tempo: " + str(stc.mean(tt)) + "\n")
                f.write("Melhor Tempo: " + str(min(tt)) + "\n")
                f.write("Pior Tempo: " + str(max(tt)) + "\n")
                try: 
                    f.write("Variancia do tempo: " + str(stc.variance(tt)) + "\n")
                    f.write("Desvio Padrao do tempo: " + str(stc.stdev(tt)) + "\n")
                except:
                    pass

                f.write("CPU inicial: " + str(cpu) + "\n")
                f.write("CPU final: " + str(psutil.cpu_percent()) + "\n")
                f.write("Memoria inicial: " + str(memory) + "\n")
                f.write("Memoria final: " + str(memory_usage()) + "\n")

                f.write("Media da CPU inicial: " + str(stc.mean(ppi)) + "\n")
                f.write("Media da CPU final: " + str(stc.mean(ppf)) + "\n")
                f.write("Media da memoria inicial: " + str(stc.mean(mmi)) + "\n")
                f.write("Media da memoria final: " + str(stc.mean(mmf)) + "\n")
                f.write("Media da CPU: " + str(stc.mean(pp)) + "\n")
                f.write("Media da memoria: " + str(stc.mean(mm)) + "\n")
                f.write("Media da CPU (Valor Real): " + str(stc.mean(pp) * processadorDoPc / 100))
                f.write("Media da memoria (Valor Real): " + str(stc.mean(mm) * memoriaDoPc / 100))

                try:
                    f.write("Variancia da CPU inicial: " + str(stc.variance(ppi)) + "\n")
                    f.write("Variancia da CPU final: " + str(stc.variance(ppf)) + "\n")
                    f.write("Variancia da memoria inicial: " + str(stc.variance(mmi)) + "\n")
                    f.write("Variancia da memoria final: " + str(stc.variance(mmf)) + "\n")
                    f.write("Variancia da CPU: " + str(stc.variance(pp)) + "\n")
                    f.write("Variancia da memoria: " + str(stc.variance(mm)) + "\n")

                    f.write("Desvio padrao da CPU inicial: " + str(stc.stdev(ppi)) + "\n")
                    f.write("Desvio padrao da CPU final: " + str(stc.stdev(ppf)) + "\n")
                    f.write("Desvio padrao da memoria inicial: " + str(stc.stdev(mmi)) + "\n")
                    f.write("Desvio padrao da memoria final: " + str(stc.stdev(mmf)) + "\n")
                    f.write("Desvio padrao da CPU: " + str(stc.stdev(pp)) + "\n")
                    f.write("Desvio padrao da memoria: " + str(stc.stdev(mm)) + "\n")
                    f.write("Desvio padrao da CPU (Valor Real): " + str(stc.stdev(pp) * processadorDoPc / 100))
                    f.write("Desvio padrao da memoria (Valor Real): " + str(stc.stdev(mm) * memoriaDoPc / 100))
                except:
                    pass

                f.write("Caminho X: " + str(menorCaminhoX) + "\n")
                for value in menorCaminhoX:
                    f.write(str(value))
                    f.write(", ")
                f.write("\n\n")
                f.write("Caminho Y: " + str(menorCaminhoY) + "\n")
                for value in menorCaminhoY:
                    f.write(str(value))
                    f.write(", ")
                f.write("\n")

                # f.write("Completeness: " + str(np.sum(cr)*100/len(cr)) + "%" + "\n")
                
                f.write("\n")

                # print("CPU inicial: " + str(cpu))
                # print("CPU final: " + str(psutil.cpu_count()))
                # print("Memoria inicial: " + str(memory))
                # print("Memoria final: " + str(psutil.virtual_memory())) 
                if len(lastFileName) > 0: 
                    os.remove(lastFileName)
                lastFileName = fileName 

        else:
            f = 0

        if log and f!=0: f.close() 
              