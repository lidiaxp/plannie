# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
import copy

def make_maze(mx, my, ox, oy, xs, ys, xt, yt, obs, limiar):
	if max(ox) > 150:
		maze = np.ones((my, mx))
		auxmaze = np.ones((my, mx))
		aux = []
		px, py = [xs], [ys]
		auxpx, auxpy = [], []
		count = 0
		maze[int(xs)][int(ys)] = 0
		maze[int(xt)][int(yt)] = 0
		while len(px) > 0:
			for i in range(len(px)):
				for j in range(1, limiar):
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

				for j in range(1, limiar):
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

			value = 0
			for q, w in zip(maze, auxmaze):
				if list(q) != list(w):
					value = 1
					break
			if value == 0:
				if count == 0:
					count = 1
					px, py = [xt], [yt]
				else:
					return maze

			auxmaze = copy.deepcopy(maze)
	else:
		maze = np.zeros((my, mx))

		for i, j in zip(ox, oy):
			maze[int(i)][int(j)] = 1

	return maze