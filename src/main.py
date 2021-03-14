from air_sim_client import AirSimClient as ASC
from path_planner import PathPlanner
import utils
import matplotlib.pyplot as plt
import vg
import numpy as np

simulate = True
plot = False

# DENSE
MIN_QUALITY_A = 0.9
START_POS_A = (18, 18, 18)
END_POS_A = (195, 210, 27)
BS_POS_LIST_A = [(58, 169, 0), (136, 35, 0), (180, 183, 0), (20, 94, 0), (167, 105, 0), (38, 21, 0), (96, 102, 0), (227, 27, 0)]
SMOOTHNESS_A = 6

# SPARSE
MIN_QUALITY_B = 0.55
START_POS_B = (12, 10, 18)
END_POS_B = (85, 185, 27)
BS_POS_LIST_B = [(50, 194, 0), (86, 20, 0), (174, 21, 0), (165, 199, 0), (36, 94, 0)]
SMOOTHNESS_B = 2

# CUSTOM
MIN_QUALITY_C = 0.1
START_POS_C = (20, 10, 18)
END_POS_C = (105, 75, 27)
BS_POS_LIST_C = [(25, 25, 0), (25, 85, 0), (85, 25, 0), (85, 85, 0)]
SMOOTHNESS_C = 0

def main(bs_pos_list, start_pos, end_pos, min_quality, smoothness):
    utils.setBaseStations(bs_pos_list)

    print("Generating distance based path . . .")
    distPath = PathPlanner(False).search(start_pos, end_pos)
    print("Distance based path average quality: {:.4f}".format(utils.calcPathAvgQual(distPath)))
    print("Distance based path total distance: {:.2f} meters".format(utils.calcPathDist(distPath)))

    # Generate initial optimal-greedy path
    print("Generating initial path . . .")
    initialPath = PathPlanner().search(start_pos, end_pos)
    print("Initial path average quality: {:.4f}".format(utils.calcPathAvgQual(initialPath)))
    print("Initial path total distance: {:.2f} meters".format(utils.calcPathDist(initialPath)))
    
    # Optimize path
    print("Generating smooth path . . .")
    cornerPoints = utils.getCornerPoints(initialPath)
    smoothQual, smoothPath, _, bestQual, _ = utils.threshSmoothPath(cornerPoints, min_quality, smoothness)

    # Check if optimization was possible
    if len(smoothPath) < 2:
        print("Smooth path not found with minimum quality of {}. Maximum quality found is {:.4f}".format(min_quality, bestQual))
        return
    
    print("Smooth path average quality: {:.4f}".format(smoothQual))
    print("Smooth path total distance: {:.2f} meters".format(utils.calcPathDist(smoothPath)))
    
    if plot:
        utils.plotPath([smoothPath])
    
    if simulate:
        print("Simulating smooth path in AirSim . . .")

        # Init air sim client
        asc = ASC()

        # Spawn base stations
        i = 0
        for (x, y, z) in utils.BS_POS_LIST:
            asc.spawnObject(i, utils.BS_SIZE, (x, y, z))
            i += 1

        # Fly path using AirSimClient
        realPos = asc.flyPath(smoothPath)

        print("Real Positions Avg Net Qual: {:.4f}".format(utils.calcPathAvgQual(realPos)))

        # Evaluate path
        smoothBSpline, realBSpline = utils.getBSplineInter(smoothPath, realPos)
        allDsts = []
        for i in range(len(realBSpline)):
            allDsts.append(utils.distanceCalc(realBSpline[i], smoothBSpline[i])) # aka error 

        maxErr = 0
        maxErrI = 0
        minErr = float("inf")
        minErrI = 0
        for i in range(len(allDsts)):
            if allDsts[i] > maxErr:
                maxErr = allDsts[i]
                maxErrI = i
            if allDsts[i] < minErr:
                minErr = allDsts[i]
                minErrI = i

        print("Max Error:", maxErr, "at", maxErrI, "of", len(allDsts))
        print("Min Error:", minErr, "at", minErrI, "of", len(allDsts))

        rmse = np.sqrt((np.array(allDsts) ** 2).mean())
        print("RMSE:", rmse)

        print("Smooth BSpline Avg Net Qual: {:.4f}".format(utils.calcPathAvgQual(smoothBSpline)))
        print("Real BSpline Avg Net Qual: {:.4f}".format(utils.calcPathAvgQual(realBSpline)))

        utils.plotPath([smoothBSpline, realBSpline])

if __name__ == "__main__":
    #main(BS_POS_LIST_A, START_POS_A, END_POS_A, MIN_QUALITY_A, SMOOTHNESS_A)
    #main(BS_POS_LIST_B, START_POS_B, END_POS_B, MIN_QUALITY_B, SMOOTHNESS_B)
    main(BS_POS_LIST_C, START_POS_C, END_POS_C, MIN_QUALITY_C, SMOOTHNESS_C)