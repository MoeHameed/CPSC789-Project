import airsim
import pprint
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt

from airsim.types import ImageRequest

CUBE_MODEL_STR = "mycube"   # Filled in
CUBE2_MODEL_STR = "mycube2" # Transparent

UAV_POS_TRACK_INTERVAL = 0.5
# TODO: Add flying consts

class AirSimClient:
    def __init__(self):
        self.spawnedObjs = []
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        # self.client.enableApiControl(True)
        # self.client.armDisarm(True)
        # print("Taking off . . .")
        # self.client.takeoffAsync().join()
        # print("Airborne!")

    def spawnObject(self, name, size, position):
        """Spawns an object in the connected UE4 Environment through the AirSim client.

        Args:
            name (string): The object name to appear in UE4 and logs
            size (tuple): The size of the object (x, y, z)
            position (tuple): The position of the object (x, y, z)
        """
        objPose = airsim.Pose()
        x = position[0] + (0.5 * (size[0] - 1))
        y = position[1] + (0.5 * (size[1] - 1))
        z = position[2] + (0.5 * (size[2] - 1))
        objPose.position = airsim.Vector3r(x, y, -z)

        objSize = airsim.Vector3r(size[0], size[1], size[2])

        objName = "SpawnedObject_" + str(name)

        self.client.simSpawnObject(objName, CUBE2_MODEL_STR, objPose, objSize)
        self.spawnedObjs.append(objName)
        
        print("Spawned: ", objName)

    def getDepthImg(self):
        self.client.simPause(True)
        raw_img = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, True, False)])[0]
        pose = self.client.getMultirotorState()
        self.client.simPause(False)

        # dpeth_img_meters = airsim.get_pfm_array(raw_img).reshape(raw_img.height, raw_img.width, 1)
        # depth_8bit_lerped = np.interp(dpeth_img_meters, (0, 150), (0, 255)).astype('uint8')

        return airsim.get_pfm_array(raw_img), pose

    def flyToPosAndYaw(self, pos_to_fly, yaw):
        pos = airsim.Vector3r(pos_to_fly[0], pos_to_fly[1], -pos_to_fly[2])
        #self.client.simPlotPoints([pos], [0, 0, 1, 1], 15, 100000, True)

        print("Flying to position and yaw . . .")
        self.client.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val, 3).join()
        self.client.rotateToYawAsync(yaw).join()
        self.client.hoverAsync().join()
        print("Done!")

    def flyPath(self, path_to_fly):
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        path = [airsim.Vector3r(x, y, -z) for (x, y, z) in path_to_fly]

        print("Plotting path points . . .")
        self.client.simPlotPoints([(path[0])], [0, 0, 1, 1], 15, 100000, True)     # Start point
        self.client.simPlotPoints([(path[-1])], [1, 1, 1, 1], 15, 100000, True)    # End point
        #self.client.simPlotPoints(p, [1, 0, 0, 1], 15, 100000, True)           # Path as points
        self.client.simPlotLineStrip(path, [1, 0, 0, 1], 15, 100000, True)         # Path as lines
    
        print("Taking off . . .")
        self.client.takeoffAsync().join()
        print("Airborne!")

        print("Going to start position . . .")
        self.client.moveToPositionAsync(path[0].x_val, path[0].y_val, path[0].z_val, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        print("At start position!")

        print("Starting flight in 5 seconds . . .")
        time.sleep(5)

        print("Flying along path . . .")
        ret = self.client.moveOnPathAsync(path, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 1, 0)

        # Track drone while it is flying
        uavTrackedPosList = []
        while(not ret._set_flag):
            time.sleep(UAV_POS_TRACK_INTERVAL)
            pos = self.client.getMultirotorState().kinematics_estimated.position
            uavTrackedPosList.append((pos.x_val, pos.y_val, pos.z_val))
            print("Last tracked position:", uavTrackedPosList[-1])

        print("Path flown!")

        print("Drone hovering at end position . . . ")
        self.client.moveToPositionAsync(path[-1].x_val, path[-1].y_val, path[-1].z_val, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        self.client.hoverAsync().join()

        # Return actual drone path flown
        print("Done!")
        return [(x, y, -z) for (x, y, z) in uavTrackedPosList]

    def updateObject(self, name, topleft):
        n = "SimObject_" + str(name)
        p = airsim.Pose()
        p.position = airsim.Vector3r(topleft[0], topleft[1], (topleft[2]-1.5)*-1)
        self.client.simSetObjectPose(n, p)
        print("Updated: ", name)