import airsim
import pprint
import time

class AirSimClient:
    def __init__(self):
        self.spawnedObjs = []
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

    def spawnObject(self, name, size, position):
        objPose = airsim.Pose()
        x = position[0] + (0.5 * (size[0] - 1))
        y = position[1] + (0.5 * (size[1] - 1))
        z = position[2] + (0.5 * (size[2] - 1))
        objPose.position = airsim.Vector3r(x, y, -z)

        objSize = airsim.Vector3r(size[0], size[1], size[2])

        objName = "SimObject_" + str(name)

        self.client.simSpawnObject(objName, "mycube", objPose, objSize)
        self.spawnedObjs.append(objName)
        
        print("Spawned: ", objName)

    def flyPath(self, path):
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        p = []
        for (x, y, z) in path:
            p.append(airsim.Vector3r(x, y, -z))

        print("Plotting path points . . .")
        self.client.simPlotPoints([(p[0])], [0, 0, 1, 1], 15, 100000, True)
        self.client.simPlotPoints([(p[-1])], [1, 1, 1, 1], 15, 100000, True)
        #self.client.simPlotPoints(p, [1, 0, 0, 1], 15, 100000, True)
        self.client.simPlotLineStrip(p, [1, 0, 0, 1], 15, 100000, True)
    
        print("Taking off . . .")
        self.client.takeoffAsync().join()
        print("Airborne!")

        print("Going to start position . . .")
        self.client.moveToPositionAsync(p[0].x_val, p[0].y_val, p[0].z_val, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        print("At start position!")

        print("Starting flight in 5 seconds . . .")
        time.sleep(5)

        realPos = []
        print("Flying along path . . .")
        #ret = self.client.moveToPositionAsync(pos.x_val, pos.y_val, pos.z_val, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 5, 0)
        ret = self.client.moveOnPathAsync(p, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), 1, 0)

        while(not ret._set_flag):
            time.sleep(0.5)
            mPos = self.client.getMultirotorState().kinematics_estimated.position
            rPos = (mPos.x_val, mPos.y_val, mPos.z_val)
            realPos.append(rPos)

        print("Done!")

        print("Drone hovering at end position . . . ")
        self.client.moveToPositionAsync(p[-1].x_val, p[-1].y_val, p[-1].z_val, 3, 3e38, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        self.client.hoverAsync().join()

        # Return actual drone path flown
        rp2 = []
        for (x, y, z) in realPos:
            rp2.append((x, y, -z))
        
        return rp2


# def updateObject(name, topleft):
#     n = "SimObject_" + str(name)
#     p = airsim.Pose()
#     p.position = airsim.Vector3r(topleft[0], topleft[1], (topleft[2]-1.5)*-1)
#     client.simSetObjectPose(n, p)
#     print("Updated: ", name)