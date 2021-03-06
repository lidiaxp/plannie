#!/usr/bin/env python3
#__*__ coding: utf-8 __*__
# # A simple example of RMPflow: goal reaching while avoiding obstacles
# @author Anqi Li
# @date April 8, 2019


from rmp import RMPRoot
from rmp_leaf import CollisionAvoidance, GoalAttractorUni

import numpy as np
from numpy.linalg import norm
from scipy.integrate import solve_ivp

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import time

t = time.time()
# ---------------------------------------------
# build the rmp tree
x_g = np.array([6, 7.5])
x_o = np.array([1, 1]) # 25 18
r_o = 1

r = RMPRoot('root')
leaf1 = CollisionAvoidance('collision_avoidance', r, None, epsilon=0.2)
leaf2 = GoalAttractorUni('goal_attractor', r, x_g)

# ----------------------------------------------

# -----------------------------------------
# possible initial configurations

# x = np.array([-2, -2])
# x_dot = np.array([2.3, 0])

# x = np.array([2, -1])
# x_dot = np.array([1, 0])

x = np.array([-1, -1])
x_dot = np.array([0, 0])

# x = np.array([-10, 0])
# x_dot = np.array([-1, 0])

# x = np.array([2.5, -3.2])
# x_dot = np.array([-1, 1])

# x = np.zeros((2, 1))
# x_dot = np.zeros((2, 1))
# while norm(x) <= 1.1:
#     x = np.array([3, -3]) + np.random.randn(2) * 3
#     x_dot = np.array([-1, 1])

state_0 = np.concatenate((x, x_dot), axis=None)
# --------------------------------------------

# --------------------------------------------
# dynamics
def dynamics(t, state):
    state = state.reshape(2, -1)
    x = state[0]
    x_dot = state[1]
    x_ddot = r.solve(x, x_dot)
    state_dot = np.concatenate((x_dot, x_ddot), axis=None)
    return state_dot
# --------------------------------------------

# ---------------------------------------------
# solve the diff eq
sol = solve_ivp(dynamics, [0, 40], state_0)
# ---------------------------------------------

print(time.time() - t)

# --------------------------------------------
# plot trajectories

paredeX = [22,22,22,22,22,22,27,27,27,27,27,27,22,23,24,25,26,27,22,23,24,25,26,27]
paredeY = [22,23,24,25,26,27,22,23,24,25,26,27,22,22,22,22,22,22,27,27,27,27,27,27]

plt.plot(np.asarray(paredeX)-24, np.asarray(paredeY)-17, ".k")
plt.plot([1,2,3,4,5], [1,1,1,1,1], ".g")
plt.plot(sol.y[0], sol.y[1])
plt.plot(x_g[0], x_g[1], 'go')

# circle = plt.Circle((x_o[0], x_o[1]), r_o, color='k', fill=False)
# plt.gca().add_artist(circle)

plt.axis([-8, 12, -4, 13])
plt.gca().set_aspect('equal', 'box')
plt.show()
# --------------------------------------------
