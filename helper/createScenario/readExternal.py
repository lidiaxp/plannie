import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
import math
import numpy as np
import json

def rotationMatrix3D(psi0, x1, y1, z1, eixo="yaw"):
    if eixo == "yaw": 
        r = [[np.cos(psi0), np.sin(psi0) * -1, 0], [np.sin(psi0), np.cos(psi0), 0], [0, 0, 1]]
    elif eixo == "pitch": 
        r = [[1, 0, 0], [0, np.cos(psi0), -np.sin(psi0)], [0, np.cos(psi0), np.cos(psi0)]]
    elif eixo == "roll":
        r = [[np.cos(psi0), 0, np.sin(psi0)], [0, 1, 0], [-np.sin(psi0), 0, np.cos(psi0)]]
    else:
        print("Axes accepted: roll, pitch, roll")
        return [x1, y1, z1]

    pos_local = np.dot(np.transpose(np.asarray(r)), np.asarray([x1, y1, z1]))
    return pos_local

def simulate_points(x1, x2, y1, y2, juntos=False):
    aux = math.ceil(max(abs(x1 - x2), abs(y1 - y2)))
    aux *= 2
    a1 = np.linspace(x1, x2, int(aux))
    a2 = np.linspace(y1, y2, int(aux))

    if juntos:
        jj = []
        for i in range(len(a1)):
            jj.append([a1[i], a2[i]])

        return jj
    else:
        return a1, a2

def arredondarTraj(x, y, z=0, threeD=0):
    if threeD == 0:
        trajX, trajY = [], []
        trajXY = []
        for i, j in zip(x, y):
            if [round(i), round(j)] not in trajXY: trajXY.append([round(i), round(j)])
        for i in trajXY:
            trajX.append(i[0])   
            trajY.append(i[1])
        return trajX, trajY   
    else:
        trajX, trajY, trajZ = [], [], []
        trajXYZ = []
        for i, j, k in zip(x, y, z):
            if [round(i), round(j), round(k)] not in trajXYZ: trajXYZ.append([round(i), round(j), round(k)])
        for i in trajXYZ:
            trajX.append(i[0])   
            trajY.append(i[1])
            trajZ.append(i[2])
        return trajX, trajY, trajZ

    

def readFromHouseExpo(maxX=17, maxY=10, grid=4, file='658e5214673c7a4e25b458e56bdb6144', plot=False):
    grid = grid
    maxX, maxY = maxX * grid, maxY * grid

    px1, py1 = simulate_points(0,maxX,0,0)
    px2, py2 = simulate_points(0,maxX,maxY,maxY)
    px3, py3 = simulate_points(0,0,0,maxY)
    px4, py4 = simulate_points(maxX,maxX,0,maxY)

    c = 0
    auxX, auxY = 0, 0
    x, y = [], []
    f = open(file + ".json",)
    data = json.load(f)
    for i in data['verts']:
        # c = 1 if c == 0 else 0
        if c == 1:
            # print("para")
            # print(i[0])
            # print(i[1])
            newX, newY = simulate_points(auxX, i[0]*grid, auxY, i[1]*grid)
            x = np.concatenate([x,newX]) 
            y = np.concatenate([y,newY]) 
            c = 0
        elif c == 0:
            # print("de")
            # print(i[0])
            # print(i[1])
            auxX, auxY = i[0]*grid, i[1]*grid
            c = 1
    f.close()

    x = np.concatenate([x, px1, px2, px3, px4])
    y = np.concatenate([y, py1, py2, py3, py4])

    x, y = arredondarTraj(x, y)

    if plot:
        plt.plot(x, y, ".k")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        # plt.xlim(0,20)
        # plt.ylim(0,20)
        plt.show()

    return x, y

# x, y = readFromHouseExpo()
# print(x)
# print(y)

def readFromWarframe(grid=8, path="/home/lidiaxp/Downloads/warframe", file="A1.3dmap", plot=False):
    c = 0
    grid = grid
    x3d, y3d, z3d = [], [], []
    xyz = []
    f = open(path + "/" + file, "r")
    for x in f:
        c += 1
        if c > 1:
            aux = x.split()
            auxX, auxY, auxZ = int((int(aux[0])/grid)), int((int(aux[1])/grid)), int((int(aux[2])/grid))
            if [auxX, auxY, auxZ] not in xyz:
                xyz.append([auxX, auxY, auxZ])
                x3d.append(auxX) 
                y3d.append(auxY) 
                z3d.append(auxZ)

    x3d2, y3d2, z3d2 = rotationMatrix3D(math.radians(15), x3d, y3d, z3d)

    # x3d2, y3d2, z3d2 = arredondarTraj(x3d2, y3d2, z3d2, True)

    if plot:
        ax = plt.axes(projection = "3d")
        # plt.plot(x3d, y3d, z3d, ".r")
        plt.plot(x3d2, y3d2, z3d2, ".k")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        # ax.set_xlim(0,20) 
        ax.set_ylim(0,20) 
        # ax.set_zlim(0,5) 
        # ax.legend()
        plt.pause(0.01)
        plt.show()

    return x3d2, y3d2, z3d2

x, y, z = readFromWarframe(plot=True)
with open('warframeMap.txt', 'w') as f:
    f.write("x = [")
    for i in x:
        f.write(str(i)+',')
    f.write("]" + "\n" + "y = [")
    for i in y:
        f.write(str(i)+',')
    f.write("]" + "\n" + "z = [")
    for i in z:
        f.write(str(i)+',')
    f.write("]")