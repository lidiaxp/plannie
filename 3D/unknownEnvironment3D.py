# -*- coding: utf-8 -*-
from classic import rrt_connect_3d, Astar_test, rrt3D
from metaHeuristic import pso
from machineLearning import rl#, gworl#, deeprl
import rlpso3D

from helper.unknown3D import run
from helper.utils import memory_usage, diminuir_pontos

import numpy as np
import statistics as stc
import psutil
from datetime import datetime
import os

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from helper.ambiente import Pontos

memoriaDoPc = 16000 # MB
processadorDoPc = 2800 # MHz

p = Pontos()

Astar = 1
RrtC = 0
Pso = 0
PsoRl = 0
Rl = 0
RRT = 0

nome = ["A-Star", "Rapid Random Tree Connect", "Particle Swarm Optimization", "Particle Swarm Optimization with Reinforcement Learning", "Reinforcement Learning", "Rapid Exploring Random Tree"]

valid = [Astar, RrtC, Pso, PsoRl, Rl, RRT]

def define_algorithm(q):
    if q == 0:
        return Astar_test, "BiAStar"
    elif q == 1:
        return rrt_connect_3d, "RRTC"
    elif q == 2:
        return pso, "PSO"
    elif q == 3:
        return rlpso3D, "PSORL"
    elif q == 4:
        return rl, "RL"
    elif q == 5:
        return rrt3D, "RRT"
    else:
        return Astar_test, "AStar" # default
    
if __name__ == "__main__":

    # ax = plt.axes(projection = "3d")
    # # ax.rcParams.update({'font.size': 20})
    # ax.plot3D(p.xobs, p.yobs, p.zobs, 'k.') 
    # ax.set_xlim(0,20) 
    # ax.set_ylim(0,20) 
    # ax.set_zlim(0,6) 
    # ax.set_xlabel("x (m)")
    # ax.set_ylabel("y (m)")
    # ax.set_zlabel("z (m)")
    # plt.pause(0.01)
    # plt.show()

    show = 0
    nInterations = 1  
    rodarMTSVezes = True if nInterations >= 0 else False
    dt, tt, cc = [], [], []
    ppi, ppf, mmi, mmf, pp, mm = [], [], [], [], [], []
    menorCaminhoX, menorCaminhoY = [], []
    log = 0
    lastFileName = ""
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

                d, t, mx, my, mz, notConclude = run(valid, q, alg, show=show, rmv=rodarMTSVezes) #completeness
                # run(valid, q, alg, show=show, rmv=rodarMTSVezes) #completeness
                
                # d, t, mx, my, mz, notConclude = 0, [0], [0], [0], [0], 0

                if not notConclude: print("good")
                cc.append(notConclude)
                dt.append(d)
                tt = np.concatenate((tt, t), axis=0)
                ppf.append(psutil.cpu_percent())
                mmf.append(memory_usage())

                pp.append(abs(ppf[i] - ppi[i]))
                mm.append(abs(mmf[i] - mmi[i]))

                if (d < min(dt) or len(menorCaminhoX) == 0) and (notConclude == False):
                    menorCaminhoX = mx
                    menorCaminhoY = my
            
            ##################################################################################

            print("\n\n")
            print(nome[q])
            print(fileName)
            print("Iteracoes: " + str(i+1))
            print("Completeness: " + str((len(cc) - sum(cc)) * 100 / len(cc)))
            print("Media da distancia: " + str(stc.mean(dt)))
            print("Melhor distancia: " + str(min(dt)))
            print("Pior distancia: " + str(max(dt)))
            
            try: 
                print("Variancia da distancia: " + str(stc.variance(dt)))
                print("Desvio Padrao da distancia: " + str(stc.stdev(dt)))
            except:
                pass
            
            print("Media do tempo: " + str(stc.mean(tt)))
            print("Melhor Tempo: " + str(min(tt)))
            print("Pior Tempo: " + str(max(tt)))
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

            print(mx.tolist())
            print(my.tolist())
            print(mz.tolist())
            
            # plt.rcParams.update({'font.size': 20})
            ax = plt.axes(projection = "3d")
            # ax.rcParams.update({'font.size': 20})
            ax.plot3D(p.xobs, p.yobs, p.zobs, 'k.') 
            ax.plot3D(mx, my, mz, '-r') 
            ax.set_xlim(0,21) 
            ax.set_ylim(0,24) 
            ax.set_zlim(0,6) 
            ax.set_xlabel("x (m)")
            ax.set_ylabel("y (m)")
            ax.set_zlabel("z (m)")
            plt.pause(0.01)
            plt.savefig("path3D"+str(name)+str(nInterations)+".png")
            plt.show()

            # print("Caminho X: " + str(menorCaminhoX))
            # print("Caminho Y: " + str(menorCaminhoY))
            
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
              