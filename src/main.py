from air_sim_client import AirSimClient as ASC
import utils
import matplotlib.pyplot as plt
import vg
import numpy as np
from depth_image_processor import depthImageProcessor as DIP
import open3d as o3d

# [X, Y, Z]
# X = Longitude, Y = Latitude, Z = Altitude
# Sizes in meters

# Area consts
# AREA_SIZE = (250, 250, 120)
# AREA_MIN = (20, 20)
# AREA_MAX = (230, 230)
# AREA_GROUND_HEIGHT = 1

#INIT_POS = (10, 76, 14)     # (X, Y, Z)
#dark = 10, 25, 20 : 45
INIT_POS = (10, 15, 20)
INIT_AZ = 10     # Azimuth (phi): [0, 359] : 0 = Forward X-axis 
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

    asc.flyToPosAndYaw((10, 5, 20), init_az)
    img, pose = asc.getDepthImg()
    dip = DIP()
    geoms = dip.pfm_to_voxel(img, pose)

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(geoms[0])
    # vis.add_geometry(geoms[1])
    # vis.add_geometry(geoms[2])
    # vis.add_geometry(geoms[3])
    # vc = vis.get_view_control()
    # vc.rotate(0, -500)
    # vc.set_zoom(0.5)
    # vis.poll_events()
    # vis.update_renderer()

    # Fly to start position
    for i in range(10, 80, 5):
        asc.flyToPosAndYaw((10, i, 20), init_az)
        time.sleep(5)
        img, pose = asc.getDepthImg()
        geoms = dip.pfm_to_voxel(img, pose)

        # vis.clear_geometries()
        # vis.add_geometry(geoms[0])
        # vis.add_geometry(geoms[1])
        # vis.add_geometry(geoms[2])
        # vis.add_geometry(geoms[3])
        # vc = vis.get_view_control()
        # vc.rotate(0, -500)
        # vc.set_zoom(0.5)
        # vis.poll_events()
        # vis.update_renderer()


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