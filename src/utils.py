import numpy as np
import matplotlib.pyplot as plt
import vg
import itertools
from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches

# [X, Y, Z]
# X = Longitude, Y = Latitude, Z = Altitude
# Sizes in meters

# Area consts
AREA_SIZE = (250, 250, 120)
AREA_MIN = (20, 20)
AREA_MAX = (230, 230)
AREA_GROUND_HEIGHT = 1

# Base Station consts
BS_SIZE = [3, 3, 25]
BS_RANGE = 40
BS_ANT_HEIGHT = 23
BS_POS_LIST = [(-1, -1, 0), (-1, 69, 0)]

def setBaseStations(bsPosList):
    global BS_POS_LIST
    BS_POS_LIST = bsPosList

def cart2sph(coord):
    x, y, z = coord
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(hxy, z) # z-axis down, switch for xy-plane up
    az = np.arctan2(y, x)
    return r, np.rad2deg(az), np.rad2deg(el)    # radius, theta, phi

def distanceCalc(A, B):
    a = np.array((A[0], A[1], A[2]))
    b = np.array((B[0], B[1], B[2]))
    return np.linalg.norm(a-b)

def nodeNetworkQualCalc(node):
    # Store quality for each valid bs
    qualList = []
    for (x, y, _) in BS_POS_LIST:
        a = np.array([node[0], node[1], node[2]])
        b = np.array([x+1, y+1, BS_ANT_HEIGHT]) # add 1 since it is 3x3

        dist, _, ang = cart2sph(a-b)

        if dist <= BS_RANGE+1:
            qdist = 0.03174603 + 0.139914*dist - 0.005735367*dist**2 + 0.00005606192*dist**3
            qdist = max(0, min(1, qdist))

            qang = -2.5 + 0.0722222*ang - 0.00037037*ang**2
            qang = max(0, min(1, qang))

            q = (0.7 * qang) + (0.3 * qdist) # TODO: ADD SIMULATED QUALITY FOR THIS BS
            qualList.append(q)

    if len(qualList) > 0:
        return np.mean(qualList)
    else:
        return 0

    # if len(qualList) >= 2:
    #     maxQ = max(qualList)
    #     qualList.remove(maxQ)
    #     return maxQ * (1 - np.mean(qualList))
    # elif len(qualList) == 1:
    #     return qualList[0]
    # else:
    #     return 0


# dist_samples = np.linspace(0, 40, 200)
# ys = []
# for s in dist_samples:
#     ys.append((0.03174603 + 0.139914*s - 0.005735367*s**2 + 0.00005606192*s**3)*100)
    
# plt.plot(dist_samples, ys, 'k')
# plt.xlabel("Distance (m)")
# plt.ylabel("Network Quality %")
# plt.xlim(0, 40)
# plt.ylim(0, 100)
# plt.show()


# samples = np.linspace(0, 180, 200)
# ys = []
# for s in samples:
#     ys.append((-2.5 + 0.0722222*s - 0.00037037*s**2)*100)
    
# plt.plot(samples, ys, 'k')
# plt.xlabel("Angle (degrees)")
# plt.ylabel("Network Quality %")
# plt.xlim(0, 180)
# plt.ylim(0, 100)
# plt.show()


def threshSmoothPath(initialPath, thresh, smoothness=0):
    seq = itertools.product([0,1], repeat=6)

    bestQual = 0
    bestPath = []

    threshQual = 1
    threshPath = []
    threshSeq = []

    for i in list(seq):  
        if i.count(1) < smoothness:
            continue

        newPath = smoothPath(initialPath, i)

        avgQual = calcPathAvgQual(newPath)

        if avgQual > bestQual:
            bestQual = avgQual
            bestPath = newPath

        if avgQual < threshQual and avgQual >= thresh:
            threshQual = avgQual
            threshPath = newPath
            threshSeq = i
            
    return threshQual, threshPath, threshSeq, bestQual, bestPath

# 0 = insert midpoints, 1 = avg midpoints
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

def calcPathAvgQual(path):
    qual = 0
    for node in path:
        qual += nodeNetworkQualCalc(node)
    return qual/len(path)

def calcPathDist(path):
    dist = 0
    for i in range(len(path)-1):
        dist += distanceCalc(path[i], path[i+1])
    dist += distanceCalc(path[-2], path[-1])
    return dist

def plotPath(paths):
    if len(paths) > 3:
        return

    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # Plot the base station antennas
    for (x, y, _) in BS_POS_LIST:
        ax.plot3D(x+1, y+1, BS_ANT_HEIGHT, 'yo')
    
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

def vizQualityPattern():
    ax = plt.axes(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    for y in range(-41, 122):
        for z in range(-17, 64):
            if y == 0 and z == 23:
                continue
            q = nodeNetworkQualCalc((0, y, z))
            if q > 0.9:
                ax.plot(0, y, z, marker='o', color='#C73E1D', alpha=1)
            elif q > 0.8:
                ax.plot(0, y, z, marker='o', color='#F18F01', alpha=1)
            elif q > 0.7:
                ax.plot(0, y, z, marker='o', color='#D5B942', alpha=1)
            elif q > 0.6:
                ax.plot(0, y, z, marker='o', color='#330F0A', alpha=1)
            elif q > 0.5:
                ax.plot(0, y, z, marker='o', color='#EDAFB8', alpha=1)
            elif q > 0.4:
                ax.plot(0, y, z, marker='o', color='#B1CC74', alpha=1)
            elif q > 0.3:
                ax.plot(0, y, z, marker='o', color='#E8FCC2', alpha=1)
            elif q > 0.2:
                ax.plot(0, y, z, marker='o', color='#D0F4EA', alpha=1)
            elif q > 0.1:
                ax.plot(0, y, z, marker='o', color='#829399', alpha=1)
            elif q > 0:
                ax.plot(0, y, z, marker='o', color='#545F66', alpha=1)

    # ps = []
    # ps.append(mpatches.Patch(color='#C73E1D', label='> 90%'))
    # ps.append(mpatches.Patch(color='#F18F01', label='> 80%'))
    # ps.append(mpatches.Patch(color='#D5B942', label='> 70%'))
    # ps.append(mpatches.Patch(color='#330F0A', label='> 60%'))
    # ps.append(mpatches.Patch(color='#EDAFB8', label='> 50%'))
    # ps.append(mpatches.Patch(color='#B1CC74', label='> 40%'))
    # ps.append(mpatches.Patch(color='#E8FCC2', label='> 30%'))
    # ps.append(mpatches.Patch(color='#D0F4EA', label='> 20%'))
    # ps.append(mpatches.Patch(color='#829399', label='> 10%'))
    # ps.append(mpatches.Patch(color='#545F66', label='> 0%'))
    # plt.legend(handles=ps, bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0.)

    ax.view_init(elev=0, azim=0)
    ax.set_xticks([0])
    plt.show()

#vizQualityPattern()

def isNodeInBS(node):
    # Get all bss in range
    nx, ny, nz = node
    bsInRange = []
    for (x, y, z) in BS_POS_LIST:
        if distanceCalc((nx, ny, nz), (x, y, z)) <= BS_RANGE+1:
            bsInRange.append((x, y, z))
    
    if len(bsInRange) < 1:
        return False

    # Check bss in range
    for (x, y, z) in bsInRange:
        if (nx >= x and nx <= x + (BS_SIZE[0]+1)) and (ny >= y and ny <= y + (BS_SIZE[1]+1)) and (nz >= z and nz <= z + (BS_SIZE[2]+1)):
            return True

    return False


def calcTimesToPos(path):
    timesToPos = [0]
    for i in range(1, len(path)):
        t = (distanceCalc(path[i-1], path[i]) / 3) + 0.28
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

# def angleCalc(A, B):
#     a = np.array((A[0], A[1], A[2]))
#     b = np.array((B[0], B[1], B[2]))
#     if np.array_equal(a, b):
#         return 0
#     c = b - a   # calc difference vector
#     z = np.array(([0, 0, 1])) # unit vector
#     return vg.angle(c, z)

# # Create gif
# ax = plt.axes(projection='3d')
# ax.set_xlabel('x')
# ax.set_ylabel('y')
# ax.set_zlabel('z')

# n = PathPlanner().neighbors((0, 0, 0))
# for (x, y, z) in n:
#     ax.plot([0, x], [0, y], [0, z], 'ro-')

# ax.plot(0, 0, 0, 'bo')

# for ii in range(0,90,5):
#     ax.view_init(elev=20., azim=ii)
#     plt.savefig(".\\movie\\movie%d.png" % ii)

# plt.show()

real = [(24.366851806640625, 22.311378479003906, 17.96080207824707), (24.561643600463867, 22.438770294189453, 18.138086318969727), (24.787094116210938, 22.302982330322266, 18.233774185180664), (24.913341522216797, 21.945131301879883, 18.37265396118164), (24.8582763671875, 21.445552825927734, 18.546497344970703), (24.600378036499023, 20.8958683013916, 18.7473087310791), (24.154579162597656, 20.337356567382812, 18.989295959472656), (23.559232711791992, 19.767419815063477, 19.26854705810547), (22.870847702026367, 19.168716430664062, 19.577316284179688), (22.161640167236328, 18.503597259521484, 19.885761260986328), (21.535634994506836, 17.725839614868164, 20.179431915283203), (21.07954216003418, 16.86357307434082, 20.447399139404297), (20.837909698486328, 15.952054023742676, 20.633037567138672), (20.856122970581055, 15.10944938659668, 20.7512264251709), (21.10243034362793, 14.381291389465332, 20.839038848876953), (21.552974700927734, 13.746105194091797, 20.900697708129883), (22.156152725219727, 13.271810531616211, 20.93118667602539), (22.876331329345703, 12.988239288330078, 20.946901321411133), (23.659196853637695, 12.90844440460205, 20.952056884765625), (24.502038955688477, 13.000016212463379, 20.95233726501465), (25.3989315032959, 13.21687126159668, 20.953359603881836), (26.34085464477539, 13.533267974853516, 20.95687484741211), (27.30837631225586, 13.954524040222168, 20.960256576538086), (28.292743682861328, 14.468534469604492, 20.96173858642578), (29.30381202697754, 15.055487632751465, 20.962387084960938), (30.352331161499023, 15.68600845336914, 20.96376609802246), (31.424123764038086, 16.35303497314453, 20.967628479003906), (32.486106872558594, 17.094114303588867, 20.974647521972656), (33.500267028808594, 17.920738220214844, 20.98067283630371), (34.42859649658203, 18.80215072631836, 20.982656478881836), (35.33056640625, 19.771987915039062, 20.98122787475586), (36.17872619628906, 20.770084381103516, 20.980562210083008), (37.00077819824219, 21.844270706176758, 20.982664108276367), (37.767154693603516, 22.969446182250977, 20.983713150024414), (38.48152542114258, 24.130603790283203, 20.98296356201172), (39.171966552734375, 25.32952117919922, 20.98166847229004), (39.84621047973633, 26.55068016052246, 20.98252296447754), (40.46049880981445, 27.7966365814209, 20.987821578979492), (40.98335647583008, 29.076629638671875, 20.99277687072754), (41.40105056762695, 30.369808197021484, 20.994373321533203), (41.691497802734375, 31.69866180419922, 21.008468627929688), (41.789459228515625, 33.00202178955078, 21.020660400390625), (41.6934814453125, 34.256744384765625, 21.018625259399414), (41.42778778076172, 35.443359375, 21.008424758911133), (41.00014877319336, 36.57913589477539, 20.991374969482422), (40.4409294128418, 37.66313934326172, 20.967426300048828), (39.791866302490234, 38.72789764404297, 20.9388370513916), (39.14305114746094, 39.80309295654297, 20.890392303466797), (38.59492492675781, 40.92634963989258, 20.843753814697266), (38.20806884765625, 42.11265563964844, 20.78667640686035), (38.03369140625, 43.29182052612305, 20.709243774414062), (38.08743667602539, 44.412353515625, 20.597091674804688), (38.34077453613281, 45.44392776489258, 20.46120262145996), (38.767822265625, 46.440670013427734, 20.327905654907227), (39.3084831237793, 47.42338562011719, 20.21895980834961), (39.89291763305664, 48.44337463378906, 20.130369186401367), (40.40542221069336, 49.53557586669922, 20.116241455078125), (40.691837310791016, 50.63924026489258, 20.177762985229492), (40.720550537109375, 51.681331634521484, 20.24110984802246), (40.52287292480469, 52.633304595947266, 20.29391860961914), (40.12337112426758, 53.52544403076172, 20.37105369567871), (39.55493927001953, 54.358951568603516, 20.47945213317871), (38.86577606201172, 55.15837860107422, 20.605173110961914), (38.10431671142578, 55.98349380493164, 20.747920989990234), (37.36466979980469, 56.880374908447266, 20.91887855529785), (36.77830123901367, 57.85337829589844, 21.113269805908203), (36.41428756713867, 58.85075759887695, 21.299694061279297), (36.25776290893555, 59.879024505615234, 21.4757137298584), (36.29256057739258, 60.912899017333984, 21.60594940185547), (36.49845886230469, 61.93742370605469, 21.70028305053711), (36.8342399597168, 62.951927185058594, 21.781667709350586), (37.25458908081055, 64.0040054321289, 21.835399627685547), (37.681522369384766, 65.10691833496094, 21.828609466552734), (38.03482437133789, 66.2725830078125, 21.800418853759766), (38.25859832763672, 67.4702377319336, 21.760520935058594), (38.38078308105469, 68.68083953857422, 21.717876434326172), (38.46601867675781, 69.94910430908203, 21.680740356445312), (38.57649612426758, 71.22051239013672, 21.676599502563477), (38.82115936279297, 72.46886444091797, 21.805749893188477), (39.2514762878418, 73.57608032226562, 21.98469352722168), (39.84616470336914, 74.50421142578125, 22.161609649658203), (40.58198165893555, 75.26110076904297, 22.349035263061523), (41.41652297973633, 75.81840515136719, 22.5808048248291), (42.310516357421875, 76.16584014892578, 22.87255859375), (43.26774978637695, 76.33808898925781, 23.2144832611084), (44.263641357421875, 76.368408203125, 23.61159896850586), (45.293235778808594, 76.25984954833984, 24.078699111938477), (46.4036750793457, 76.00621032714844, 24.785118103027344), (47.55002212524414, 75.73369598388672, 25.498430252075195), (48.78894805908203, 75.53146362304688, 26.296249389648438), (50.02510070800781, 75.31067657470703, 26.705673217773438), (51.10295486450195, 75.08982849121094, 26.85692596435547)]
gen = [(25, 22, 18), (24.953125, 21.953125, 18.046875), (24.625, 21.625, 18.328125), (23.6875, 20.59375, 19.03125), (22.375, 18.671875, 19.96875), (21.765625, 16.28125, 20.671875), (22.890625, 14.265625, 20.953125), (26.03125, 13.75, 21.0), (30.765625, 15.90625, 21.0), (35.828125, 20.640625, 21.0), (39.578125, 26.125, 21.0), (41.125, 30.859375, 21.0), (40.65625, 34.984375, 21.0), (39.390625, 38.96875, 20.953125), (38.96875, 42.671875, 20.71875), (39.671875, 46.0, 20.296875), (40.0, 49.328125, 20.0625), (38.96875, 53.03125, 20.34375), (37.65625, 56.96875, 21.0), (37.328125, 60.671875, 21.65625), (37.515625, 64.046875, 21.9375), (37.65625, 67.5625, 21.75), (38.78125, 71.359375, 21.65625), (41.65625, 74.484375, 22.40625), (45.171875, 75.890625, 24.0), (47.84375, 75.84375, 25.640625), (49.28125, 75.390625, 26.625), (49.84375, 75.109375, 26.953125), (49.984375, 75.015625, 27.0), (50, 75, 27)]

import random

def genBSLocations(numBS, minD, plot=False):
    locs = []
    num_gen = 0

    while num_gen < numBS:
        x = random.randint(AREA_MIN[0], AREA_MAX[0])
        y = random.randint(AREA_MIN[1], AREA_MAX[1])
        z = BS_ANT_HEIGHT

        if len(locs) == 0:
            locs.append((x, y, z))
            num_gen += 1

        isValid = True

        # check if minD away from all
        for l in locs:
            dist = distanceCalc(l, (x, y, z))
            if dist < minD:
                isValid = False
                break

        # check if within at least one
        if isValid:
            locs.append((x, y, z))
            num_gen += 1

    if plot:
        print(locs)
        ax = plt.axes(projection='3d')
        xs, ys, zs = zip(*locs)
        ax.plot(xs, ys, zs, 'ro')
        plt.show()

    return locs
