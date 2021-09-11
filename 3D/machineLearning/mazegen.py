# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import copy
from helper.ambiente import Pontos

def make_maze(mx, my, mz, ox, oy, oz, xs, ys, zs, xt, yt, zt, obs, limiar):
	if max(ox) > 200:
		maze = np.ones((my, mx, mz))
		auxmaze = np.ones((my, mx, mz))
		aux = []
		px, py, pz = [xs], [ys], [zs]
		auxpx, auxpy = [], []
		count = 0
		maze[int(xs)][int(ys)][int(zs)] = 0
		maze[int(xt)][int(yt)][int(zt)] = 0
		while len(px) > 0:
			for i in range(len(px)):
				for j in range(1, int(limiar[0])):
					if [int(px[i]+j), int(py[i])] not in obs:
						if px[i]+j < limiar: maze[int(px[i]+j)][int(py[i])] = 0
					else:
						if [int(px[i]+j-1), int(py[i])] not in aux:
							aux.append([int(px[i]+j-1), int(py[i])])
							auxpx.append(int(px[i]+j-1))
							auxpy.append(int(py[i]))
							break
						else: 
							break

				for j in range(1, int(limiar[1])):
					if [int(px[i]-j), int(py[i])] not in obs:
						if px[i]-j >= 0: maze[int(px[i]-j)][int(py[i])] = 0
					else:
						if [int(px[i]-j+1), int(py[i])] not in aux:
							aux.append([int(px[i]-j+1), int(py[i])])
							auxpx.append(int(px[i]-j+1))
							auxpy.append(int(py[i]))
							break
						else: 
							break

				for j in range(1, limiar):
					if [int(px[i]), int(py[i]+j)] not in obs:
						if py[i]+j < limiar: maze[int(px[i])][int(py[i]+j)] = 0
					else:
						if [int(px[i]), int(py[i]+j-1)] not in aux:
							aux.append([int(px[i]), int(py[i]+j-1)])
							auxpx.append(int(px[i]))
							auxpy.append(int(py[i]+j-1))
							break
						else: 
							break

				for j in range(1, limiar):
					if [int(px[i]), int(py[i]-j)] not in obs:
						if py[i]-j >= 0: maze[int(px[i])][int(py[i]-j)] = 0
					else:
						if [int(px[i]), int(py[i]-j+1)] not in aux:
							aux.append([int(px[i]), int(py[i]-j+1)])
							auxpx.append(int(px[i]))
							auxpy.append(int(py[i]-j+1))
							break
						else: 
							break

			px = auxpx
			py = auxpy
			auxpx = []
			auxpy = []
			auxpz = []

			value = 0
			for q, w in zip(maze, auxmaze):
				if list(q) != list(w):
					value = 1
					break
			if value == 0:
				if count == 0:
					count = 1
					px, py, pz = [xt], [yt], [zt]
				else:
					return maze

			auxmaze = copy.deepcopy(maze)
	else:
		p = Pontos()
		maze = np.zeros((mx+1, my+1, mz+1))

		for i, j, k in zip(ox, oy, oz):
			print(str(i) + " - " + str(j) + " - " + str(k))
			if i >= p.limiar[0]: i = p.limiar[0] - 2
			if j >= p.limiar[1]: j = p.limiar[1] - 2
			if k >= 4: k = 4

			if i <= 0: i = 0
			if j <= 0: j = 0
			if k <= 0: k = 0

			maze[int(i)][int(j)][int(k)] = 1

	return maze