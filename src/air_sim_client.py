import airsim
import pprint
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from airsim.types import ImageRequest
import os

CUBE_MODEL_STR = "mycube"   # Filled in
CUBE2_MODEL_STR = "mycube2" # Transparent

UAV_POS_TRACK_INTERVAL = 0.5
# TODO: Add flying consts

class AirSimClient:
    def __init__(self):
        self.spawnedObjs = []
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        print("Taking off . . .")
        self.client.takeoffAsync().join()
        print("Airborne!\n")

    def genVoxelGrid(self):
        output_path = os.path.join(os.getcwd(), "binvox\map.binvox")
        self.client.simCreateVoxelGrid(airsim.Vector3r(74.5, 74.5, -26), 150, 150, 50, 1, output_path)  # aircraft

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
        #raw_img = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanner, True, False)])[0]
        pose = self.client.getMultirotorState()
        self.client.simPause(False)
        #img = airsim.get_pfm_array(raw_img).reshape(raw_img.height, raw_img.width, 1)
        img = []
        x = pose.kinematics_estimated.position.x_val
        y = pose.kinematics_estimated.position.y_val
        z = -pose.kinematics_estimated.position.z_val
        r = R.from_quat([pose.kinematics_estimated.orientation.x_val, pose.kinematics_estimated.orientation.y_val, pose.kinematics_estimated.orientation.z_val, pose.kinematics_estimated.orientation.w_val])
        #print(x, y, z, r.as_matrix())
        return img, ((x, y, z), (r))

    def flyToPosAndYaw(self, pose):
        pos = airsim.Vector3r(int(pose[0]), int(pose[1]), int(-pose[2]))
        self.client.simPlotPoints([pos], [0, 0, 1, 1], 15, 100000, True)

        print("Flying to position", pose[0], pose[1], pose[2], "and yaw", pose[3], ". . .")
        self.client.rotateToYawAsync(int(pose[3])).join()   # yaw first to slightly reduce jitter
        self.client.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val, 3, timeout_sec=120).join()
        
        if self.client.simGetCollisionInfo().has_collided:
            print("ERROR :: HAS COLLIDED !!!!!!!!!!!!!!!!")

        #self.client.hoverAsync().join()
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