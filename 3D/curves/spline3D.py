# import scipy as sp
# import matplotlib.pyplot as plt
# import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# from scipy.interpolate import splprep, splev, interp2d

# def generate_curve(x, y, z):
#     tck, _ = splprep([x,y,z], s=0, k=1)  # Generate function out of provided points, default k = 3
#     unew = np.arange(0, 1.00, 0.005)
#     newPoints = splev(unew, tck)          # Creating spline points

#     return newPoints

# if __name__ == '__main__':
#     x = np.array([1,2,3,4,5])   
#     y = np.array([1,2,3,4,5])   
#     z = np.array([3,3,3,5,5])  

#     # x = np.array([17,10, 5])   
#     # y = np.array([13.5,5, 5])   
#     # z = np.array([4,2, 2])  

#     # x = np.array([48, 42, 41, 41, 40, 34, 33, 33, 32, 32, 31, 31, 30, 27, 26, 26, 25, 25, 24, 17, 10])
#     # y = np.array([48, 40, 39, 39, 38, 32, 31, 31, 30, 30, 29, 29, 28, 25, 24, 24, 23, 23, 22, 9.25, 5])
#     # z = np.array([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 2])

#     newPoints = generate_curve(x, y, z)

#     print(newPoints)

#     ax = plt.axes(projection = "3d")
#     ax.plot3D(x, y, z, 'go')    
#     ax.plot3D(newPoints[:][0], newPoints[:][1], newPoints[:][2], 'r-')   
#     plt.show()

import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from numpy import matrix, average
import scipy.linalg 

# Parameters
pointsInterpolation=False
curveInterpolation=True
'''
    numberOfInterpolation determines the precision of interpolation.
    bigger numberOfInterpolation, more smooth curve
    CUBIC SPLINE INTERPOLATION
'''
numberOfInterpolation = 100


j=0
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
       
def generate_curve(x_axis,y_axis,z_axis, show=0):
    '''
        prepare right-side vector
    '''
    dx=[]
    dy=[]
    dz=[]
    matrix=[]
    n=2
    while n<len(x_axis):
        dx.append(3*(x_axis[n]-2*x_axis[n-1]+x_axis[n-2]))
        dy.append(3*(y_axis[n]-2*y_axis[n-1]+y_axis[n-2]))
        dz.append(3*(z_axis[n]-2*z_axis[n-1]+z_axis[n-2]))
        n=n+1   
    '''
        produce square matrix looks like :
        [[2.0, 0.5, 0.0, 0.0], [0.5, 2.0, 0.5, 0.0], [0.0, 0.5, 2.0, 0.5], [0.0, 0.0, 2.0, 0.5]]
        the classes of the matrix depends on the length of x_axis(number of nodes)
    '''
    matrix.append([float(2), float(0.5)])
    for m in range(len(x_axis)-4):
        matrix[0].append(float(0))                
    n=2
    while n<len(x_axis)-2:
        matrix.append([])
        for m in range(n-2):
            matrix[n-1].append(float(0)) 
              
        matrix[n-1].append(float(0.5))
        matrix[n-1].append(float(2))
        matrix[n-1].append(float(0.5))
        
        for m in range(len(x_axis)-n-3):
            matrix[n-1].append(float(0)) 
        n=n+1
        
    matrix.append([])
    for m in range(n-2):
        matrix[n-1].append(float(0))    
    matrix[n-1].append(float(0.5))    
    matrix[n-1].append(float(2))
    '''
        LU Factorization may not be optimal method to solve this regular matrix. 
        If you guys have better idea to solve the Equation, please contact me.
        As the LU Factorization algorithm cost 2*n^3/3 + O(n^2) (e.g. Doolittle algorithm, Crout algorithm, etc).
        (How about Rx = Q'y using matrix = QR (Schmidt orthogonalization)?)
        If your application field requires interpolating into constant number nodes, 
        It is highly recommended to cache the P,L,U and reuse them to get O(n^2) complexity.
    '''
    P, L, U = doLUFactorization(matrix)
    u=solveEquations(P,L,U,dx)
    v=solveEquations(P,L,U,dy)
    w=solveEquations(P,L,U,dz)
    
    '''
        define gradient of start/end point
    '''
    m=0
    U=[0]
    V=[0]
    W=[0]
    while m<len(u):
        U.append(u[m])
        V.append(v[m])
        W.append(w[m])
        m=m+1
    U.append(0)
    V.append(0)
    W.append(0)
   
    x, y, z = plotCubicSpline(U,V,W,x_axis,y_axis,z_axis, show)
    return x, y, z
'''
    calculate each parameters of location.
'''
def func(x1,x2,t,v1,v2,t1,t2):
    ft=((t2-t)**3*v1+(t-t1)**3*v2)/6+(t-t1)*(x2-v2/6)+(t2-t)*(x1-v1/6)
    return ft

'''
    note: 
    too many interpolate points make your computer slack.
    To interpolate large amount of input parameters,
    please switch to ax.plot().
'''
def plotCubicSpline(U,V,W,x_axis,y_axis,z_axis, show=0):
    m=1
    xLinespace=[]
    yLinespace=[]
    zLinespace=[]
    while m<len(x_axis):
        for t in np.arange(m-1,m,1/float(numberOfInterpolation)):
            xLinespace.append(func(x_axis[m-1],x_axis[m],t,U[m-1],U[m],m-1,m))
            yLinespace.append(func(y_axis[m-1],y_axis[m],t,V[m-1],V[m],m-1,m))
            zLinespace.append(func(z_axis[m-1],z_axis[m],t,W[m-1],W[m],m-1,m))
        m=m+1
    if show:
        if pointsInterpolation:
            ax.scatter(xLinespace, yLinespace,zLinespace,color="red",s=0.01)
        if curveInterpolation:
            ax.plot(xLinespace, yLinespace,zLinespace,color="red")
        '''
        matched group, annotate it if unnecessary
        '''
        ax.plot(x_axis,y_axis,z_axis,color="blue")

    return xLinespace, yLinespace,zLinespace

'''
    P stands for the permutation Matrix
    L stands for the lower-triangle Matrix
    U stands for the upper-triangle Matrix
    matrix·x = y
    P·matrix = L·U
    P·matrix·x = L·U·x = P·y
    L·U·x = y1
    U·x = y2
    x = y3
'''  
def solveEquations(P,L,U,y):
    y1=np.dot(P,y)
    y2=y1
    m=0
    for m in range(0, len(y)):
        for n in range(0, m):
            y2[m] = y2[m] - y2[n] * L[m][n]
        y2[m] = y2[m] / L[m][m]
    y3 = y2
    for m in range(len(y) - 1,-1,-1):
        for n in range(len(y) - 1, m, -1):
            y3[m] = y3[m] - y3[n] * U[m][n]
        y3[m] = y3[m] / U[m][m]
    return y3

'''
    this is the Scipy tool with high complexity.
'''    
def doLUFactorization(matrix):    
    P, L, U=scipy.linalg.lu(matrix)
    return P, L, U   

'''
    input parameters
    each vector contain at least 3 elements
'''
if __name__ == '__main__':
    show = 1

    x = [1, 2, 3, 4]
    y = [2, 3, 4, 5]
    z = [3, 4, 7, 5]

    x = [2, 10, 13.75, 19]
    y = [4, 9, 9, 9]
    z = [1.5, 3, 1, 3]

    generate_curve(x,y,z, show)

    if show:
        plt.show()