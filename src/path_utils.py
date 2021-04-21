import vg, my_utils 
from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np

# TODO: Convert path to include yaw

def calcPathDist(path):
    dist = 0
    for i in range(len(path)-1):
        dist += my_utils.euclideanDist(path[i], path[i+1])
    dist += my_utils.euclideanDist(path[-2], path[-1])
    return dist

def plotPath(paths):
    if len(paths) > 3:
        return

    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    
    color = ['ro-', 'go-', 'yo-']
    ci = 0

    # Plot the paths
    for path in paths:
        for i in range(len(path)-1):
            xs = [path[i][0], path[i+1][0]]
            ys = [path[i][1], path[i+1][1]]
            zs = [path[i][2], path[i+1][2]]
            ax.plot3D(xs, ys, zs, color[ci])
            #ax.text(xs[0], ys[0], zs[0], "{:.2f}".format(nodeNetworkQualCalc((xs[0], ys[0], zs[0]))))
        ci += 1
    
    ax.plot3D(paths[0][0][0], paths[0][0][1], paths[0][0][2], 'bo-')
    ax.plot3D(paths[0][-1][0], paths[0][-1][1], paths[0][-1][2], 'ko-')

    # ei = 0
    # for (x, y, z) in paths[1][1:-1]:
    #     ax.text(x, y, z, "{:.2f}".format(errors[ei]))
    #     ei += 1

    plt.show()


def calcTimesToPos(path):
    timesToPos = [0]
    for i in range(1, len(path)):
        t = (my_utils.euclideanDist(path[i-1], path[i]) / 3) + 0.28
        t = max(0, t)
        timesToPos.append(round(t, 4))
    return timesToPos

def getBSplineInter(path, path2, showPlot=False):
    numPoints = len(path2)
    if len(path) > len(path2):
        numPoints = len(path)

    x_path, y_path, z_path = zip(*path)
    tck, u = interpolate.splprep([x_path, y_path, z_path], s=2)
    u_fine = np.linspace(0, 1, numPoints)
    x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)
    path_inter = list(zip(x_fine, y_fine, z_fine))

    x_path2, y_path2, z_path2 = zip(*path2)
    tck2, u2 = interpolate.splprep([x_path2, y_path2, z_path2], s=2)
    u_fine2 = np.linspace(0, 1, numPoints)
    x_fine2, y_fine2, z_fine2 = interpolate.splev(u_fine2, tck2)
    path2_inter = list(zip(x_fine2, y_fine2, z_fine2))

    if showPlot:
        ax = plt.axes(projection='3d')
        ax.plot(x_path, y_path, z_path, 'bo-')
        ax.plot(x_fine, y_fine, z_fine, 'ro-')
        ax.plot(x_path2, y_path2, z_path2, 'ko-')
        ax.plot(x_fine2, y_fine2, z_fine2, 'yo-')
        plt.show()

    return path_inter, path2_inter

#region Lane-Reisenfield Path Smoothing

# 0 = insert midpoints, 1 = avg midpoints
# TODO: Replace with BSpline Function
def smoothPath(initialPath, seq):
    points = initialPath
    for i in seq:
        if i == 0:
            points = insertMidpoints(points)
        elif i == 1:
            points = avgMidpoints(points)
    return points

def insertMidpoints(points):
    path = []
    for i in range(len(points)-1):
        x3 = 0.5 * points[i][0] + 0.5 * points[i+1][0]
        y3 = 0.5 * points[i][1] + 0.5 * points[i+1][1]
        z3 = 0.5 * points[i][2] + 0.5 * points[i+1][2]
        path.append((points[i][0], points[i][1], points[i][2]))
        path.append((x3, y3, z3))
    path.append(points[-1])
    return path

def avgMidpoints(points):
    path = [points[0]]
    for i in range(len(points)-1):
        x3 = 0.5 * points[i][0] + 0.5 * points[i+1][0]
        y3 = 0.5 * points[i][1] + 0.5 * points[i+1][1]
        z3 = 0.5 * points[i][2] + 0.5 * points[i+1][2]
        path.append((x3, y3, z3))
    path.append(points[-1])
    return path

def getCornerPoints(path):
    cornerPoints = [path[0]]
    for i in range(1, len(path)-1):
        x1 = path[i-1][0]
        y1 = path[i-1][1]
        z1 = path[i-1][2]

        x2 = path[i][0]
        y2 = path[i][1]
        z2 = path[i][2]

        x3 = path[i+1][0]
        y3 = path[i+1][1]
        z3 = path[i+1][2]

        v1 = [(x2 - x1), (y2 - y1), (z2 - z1)]
        v2 = [(x3 - x1), (y3 - y1), (z3 - z1)]

        if not vg.almost_collinear(v1, v2):
            cornerPoints.append((x2, y2, z2))

    cornerPoints.append(path[-1])
    return cornerPoints
#endregion