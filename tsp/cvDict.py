#!/usr/bin/env python3
import math
import time

def dist_euclidiana(v1, v2):
	dim, soma = len(v1), 0
	for i in range(dim):
		soma += math.pow(v1[i] - v2[i], 2)
	return math.sqrt(soma)

tempo = time.time()

x = [0.75, 7, 3, 4, 6, 7]
y = [0.5, 0.5, 1, 3, 2, 7]
camFinalX = [0.75]
camFinalY = [0.5]
distancias = []

while len(x) > 1:
    d = float("inf")
    coord, index = [], 0
    for i in range(1, len(x)):
        dist = dist_euclidiana([camFinalX[-1], camFinalY[-1]], [x[i], y[i]])
        if dist < d:
            index = i
            coord = [x[i], y[i]]
            d = dist
    del x[index]
    del y[index]

    distancias.append(d)
    camFinalX.append(coord[0])
    camFinalY.append(coord[1])

print(camFinalX)
print(camFinalY)
print(distancias)
print("Tempo de execucao: " + str(time.time()-tempo))