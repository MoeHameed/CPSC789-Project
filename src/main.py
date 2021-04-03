from air_sim_client import AirSimClient as ASC
import utils
import matplotlib.pyplot as plt
import vg
import numpy as np
from depth_image_processor import depthImageProcessor as DIP

# [X, Y, Z]
# X = Longitude, Y = Latitude, Z = Altitude
# Sizes in meters

# Area consts
# AREA_SIZE = (250, 250, 120)
# AREA_MIN = (20, 20)
# AREA_MAX = (230, 230)
# AREA_GROUND_HEIGHT = 1

INIT_POS = (10, 76, 14)     # (X, Y, Z)
#dark = 10, 25, 20 : 45
INIT_AZ = 0     # Azimuth (phi): [0, 359] : 0 = Forward X-axis 
INIT_VOL = ((1, 0, 0), (150, 150, 45))     # ((pos_x, pos_y, pos_z), (size_x, size_y, size_z)) : (position, size)

def main():
    # Initialize AirSim connection
    asc = ASC()

    # Get start position
    init_pos = INIT_POS
    init_az = INIT_AZ
    
    # Get area/model to explore -> can use tight bounds intially
    init_vol = INIT_VOL

    # Subdivide region into convex areas -> Octree

    # Fly to start position
    #asc.flyToPosAndYaw(init_pos, init_az)

    img, pose = asc.getDepthImg()
    dip = DIP()
    dip.pfm_to_voxel(img, pose)


    # TODO: Return cells lists of free and occupied cells with x, y, z in planning space
    
    # Get initial discovered region

    # Main Loop

        # Update cells data
        # Update frontier data
        # Update sectors
        # Update global trajectory
        # Update local trajectory
        
        
    

    
if __name__ == "__main__":
    main()