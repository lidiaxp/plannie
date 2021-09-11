# https://github.com/motion-planning/rrt-algorithms
import numpy as np

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D,art3d
from helper.ambiente import Pontos

from rrt.rrt_connect import RRTConnect
from search_space.search_space import SearchSpace
from utilities.plotting import Plot

from curves.spline3D import generate_curve
from helper.utils import distancia_rota3D, diminuir_pontos3D, tirarRepertido3D
import time

def run(show=False, vmx=[None], vmy=None, vmz=None, startx=None, starty=None, startz=None, p1=None, pseudox=None, pseudoy=None, pseudoz=None):
    # print("RRTC 3D")
    p = Pontos()
    start = time.time()

    x_min = 0
    x_max = p.limiar[0]
    y_min = 0
    y_max = p.limiar[1]
    z_min = 0
    z_max = 5

    X_dimensions = np.array([(x_min, x_max), (y_min, y_max), (z_min, z_max)])  # dimensions of Search Space

    # obstacles
    if vmx[0] == None:
        Obstacles = []
        for x1, y1, z1 in zip(p.xobs, p.yobs, p.zobs):
            # if x1 >= x_min and y1 >= y_min and z1  >= z_min and x1 < x_max and y1  < y_max and z1  < z_max:
            Obstacles.append((x1-0.5, y1-0.5, z1-0.5, x1+0.5, y1+0.5, z1+0.5))
        Obstacles = np.array(Obstacles)
        x_init = (p.xs, p.ys, p.zs)  # starting location
    else:
        Obstacles = []
        # v1,v2,v3 = [],[],[]
        for x1, y1, z1 in zip(vmx, vmy, vmz):
            # if x1 - 1 >= x_min and y1 - 1 >= y_min and z1 - 1 >= z_min and x1 + 1 < x_max and y1 + 1 < y_max and z1 + 1 < z_max:
                # Obstacles.append((x1-1.5, y1-1.5, z1-1.5, x1+1.5, y1+1.5, z1+1.5))
            Obstacles.append((x1-0.5, y1-0.5, z1-0.5, x1+0.5, y1+0.5, z1+2))
            # v1.append(x1)
            # v2.append(y1)
            # v3.append(z1)
        Obstacles = np.array(Obstacles)
        x_init = (startx, starty, startz)
        
        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection='3d')
        # ax.set_xlim(0,21)
        # ax.set_ylim(0,11)
        # ax.set_zlim(0,5)
        # ax.plot3D(v1,v2,v3, ".k")
        # plt.show()
    x_goal = (p.xt, p.yt, p.zt)  # goal location

    

    Q = np.array([2])  # length of tree edges
    r = 0.5  # length of smallest edge to check for intersection with obstacles
    max_samples = 1024  # max number of samples to take before timing out
    prc = 0.1  # probability of checking for a connection to goal

    # create search space
    X = SearchSpace(X_dimensions, Obstacles)


    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlim(0,30)
    ax.set_ylim(0,30)
    ax.set_zlim(0,5)
    ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
    plt.show()
    
    # # create rrt_search
    # print(x_init)
    # print(x_goal)
    rrt_connect = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)

    # plot = Plot("rrt_connect_3d")
    # plot.plot_tree(X, rrt_connect.trees)
    # plot.plot_obstacles(X, Obstacles)
    # plot.plot_start(X, x_init)
    # plot.plot_goal(X, x_goal)
    # plot.draw(auto_open=True)

    path = []
    # print("start search")
    path = rrt_connect.rrt_connect()

    # print("found")
    # plot
    # print("---------")
    x,y,z = [],[],[]
    # print("-----------------")

    # print(path)
    # while path==None:
    #     # print("pass")
    #     pass
    # print("-----------------------")

    # while path==None:
    #     print("error")
    #     return -1, -1, -1, -1, -1

    # for caminho in path:
    #     x.append(caminho[0])
    #     y.append(caminho[1])
    #     z.append(caminho[2])

    if path == None:
        x = pseudox[1:]
        y = pseudoy[1:]
        z = pseudoz[1:]
        # print(pseudox)
        # print(pseudoy)
    else:
        for caminho in path:
            x.append(caminho[0])
            y.append(caminho[1])
            z.append(caminho[2])
    # print("--------------------------------")

    # x = [2.1, 4.054732531524486, 3.3503870168192615, 5.224903634426269, 7.1656897400823425, 9.0129658646246, 10.997051007824336, 12.339158188667259, 11.168360553402092, 12.112525247817688, 11.828559843240154, 11.828460039239966, 11.869131659859598, 9.940637200243454, 9.409646322439249, 8.878655444635044, 8.347664566830838, 6.969269774032971, 8.195193920874111, 10.165049453007747, 12.134904985141382, 13.612453332915534, 15.39820430997492, 16.55586288310836, 14.92424746762356, 14.712573446567747, 15.441515642503475, 17.27075782125174, 19.1]
    # y = [2.1, 2.5094489137225016, 4.290710698216751, 4.981146291929863, 5.45601488708371, 4.721714978688311, 4.583134652487718, 6.0280702718062, 7.038838737027299, 8.748820020291816, 7.888641447643582, 7.886365656774844, 8.8137844240514, 9.328520632895973, 7.4066397805764375, 5.484758928256903, 3.5628780759373675, 3.6129106881918593, 2.181452361941129, 2.171807461141167, 2.162162560341205, 1.5700185936958666, 1.9265185949435892, 2.5094714819081245, 3.6512274483128895, 5.635866065825407, 7.497407815650411, 8.298703907825205, 9.1]
    # z = [2, 2.106641067377146, 2.6819704087243377, 2.779367232333769, 2.8679624321413373, 3.0879046159684593, 2.8776627606583713, 3.210693346572906, 1.9427895619851006, 2.3723440296654337, 0.5892409838041004, 0.584893884466511, 2.3564017871808636, 2.230085697413355, 2.3863632619460025, 2.54264082647865, 2.6989183910112975, 1.2506350855428146, 0.5812793905649662, 0.9270772482955598, 1.2728751060261534, 0.06200087525096554, 0.8890445240639044, 2.4121967028137363, 2.5971952139739227, 2.725272013934196, 2.782766262714924, 2.891383131357462, 3]

    # x = [2.1, 3.9802099660861585, 5.8604199321723165, 7.740629898258475, 9.620839864344633, 11.50104983043079, 13.38125979651695, 15.261469762603108, 17.141679728689265, 17.00942267917493, 15.02802590282476, 15.01996298031218, 17.00659745684579, 17.966803582364207, 17.205709634030747, 19.1]
    # y = [2.1, 2.776513754376638, 3.453027508753276, 4.129541263129914, 4.8060550175065515, 5.48256877188319, 6.159082526259827, 6.835596280636466, 7.512110035013103, 9.332630693561603, 9.589874702149947, 9.590921508389277, 9.371519929591594, 7.808995832952638, 8.458659654540856, 9.1]
    # z = [2, 1.9155037067705722, 1.8310074135411445, 1.7465111203117167, 1.662014827082289, 1.5775185338528614, 1.493022240623434, 1.4085259473940062, 1.3240296541645786, 0.5065869316133894, 0.41774811850471666, 0.41738660561908236, 0.48912504628458564, 1.28694873854969, 3.018616408955758, 3]

    # print("p1")
    xxx1, yyy1, zzz1 = diminuir_pontos3D(x, y, z, p.xobs, p.yobs, p.zobs, value=0.8)
    xxx1, yyy1, zzz1 = tirarRepertido3D(xxx1, yyy1, zzz1)
    # xxx1, yyy1, zzz1 = tirarRepertido3D(x, y, z)
    xx, yy, zz = generate_curve(xxx1, yyy1, zzz1)
    # print("p2")

    if show:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot3D(x,y,z, "-r")
        ax.plot3D(xxx1,yyy1,zzz1, "-y")
        ax.set_xlim(0,20)
        ax.set_ylim(0,20)
        ax.set_zlim(0,5)
        # print(x)
        # print(y)
        # print(z)
        # x = [2.1, 3.961549643595206, 3.4674538597196443, 2.6289775295503364, 3.4620008952715433, 3.127421069567772, 1.764822163567352, 2.73162346709952, 2.7741667002164716, 4.374184875912914, 5.974203051609357, 7.846117643885764, 8.402916386370785, 8.959715128855803, 9.26621423405786, 11.206685091169213, 13.147155948280565, 14.922888729969635, 16.792925134314704, 18.787264161511306, 20.1]
        # y = [2.1, 2.826237406090995, 4.7515804628366105, 6.555503914113756, 7.964635681352902, 9.697172780557171, 8.359406125104162, 6.733830457865752, 6.11050860845828, 5.253004614290588, 4.395500620122896, 4.013202504258615, 4.079699332164094, 4.146196160069577, 6.058262691149793, 6.495524902357552, 6.932787113565309, 7.852853366902198, 8.561851300483442, 8.610122080873419, 10.1]
        # z = [2, 2.0849244041768062, 1.8637378987162676, 1.6568136959797053, 2.8059088768589904, 3.747381946103613, 3.1526512533320643, 3.8028812046346006, 3.55533433456013, 4.394756002988747, 5.234177671417364, 5.825602992002535, 3.905823491058323, 1.9860439901141103, 2.4861038657622245, 2.694369588701057, 2.902635311639889, 2.8867867310909854, 2.9036916594166464, 2.76127755772356, 3]
        ax.plot3D(p.xobs,p.yobs,p.zobs, ".k")
        ax.plot3D(xx,yy,zz, "-b")
        plt.show()

    # plot = Plot("rrt_connect_3d")
    # plot.plot_tree(X, rrt_connect.trees)
    # if path is not None:
    #     plot.plot_path(X, path)
    # plot.plot_obstacles(X, Obstacles)
    # plot.plot_start(X, x_init)
    # plot.plot_goal(X, x_goal)
    # plot.draw(auto_open=True)

    distance = distancia_rota3D(xx, yy, zz)

    return distance, time.time() - start, xx, yy, zz
