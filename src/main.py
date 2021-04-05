from air_sim_client import AirSimClient as ASC
import utils
import matplotlib.pyplot as plt
import vg
import numpy as np
import open3d as o3d
import time
from virtual_map import virtual_map as VMAP


INIT_POS = (10, 15, 20)
INIT_AZ = 45     # Azimuth (phi): [0, 359] : 0 = Forward X-axis 
INIT_VOL = ((1, 0, 0), (150, 150, 50))     # ((pos_x, pos_y, pos_z), (size_x, size_y, size_z)) : (position, size)

def main():
    asc = ASC()
    
    vmap = VMAP()

    pts = utils.getCamPts(25, 10, 20)

    
    

    # vis = o3d.visualization.Visualizer()
    # vis.create_window()
    # vis.add_geometry(cam_pcd)
    # vis.add_geometry(sphere)

    for y in range(10, 100, 5):
        asc.flyToPosAndYaw((10, y, 25), 45)
        _, ((pos), (rot)) = asc.getDepthImg()

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(pts))
        pcd.translate((pos[0], pos[1], pos[2]))
        pcd.rotate(rot.as_matrix(), center=[pos[0], pos[1], pos[2]])

        sphere = o3d.geometry.TriangleMesh.create_sphere(1)
        sphere.translate((pos[0], pos[1], pos[2]))

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

        o3d.visualization.draw_geometries([pcd, sphere, axes])

        # vis.update_geometry(cam_pcd)
        # vis.update_geometry(sphere)
        # vis.poll_events()
        # vis.update_renderer()

    
if __name__ == "__main__":
    main()



# [X, Y, Z]
# X = Longitude, Y = Latitude, Z = Altitude
# Sizes in meters

# Area consts
# AREA_SIZE = (250, 250, 120)
# AREA_MIN = (20, 20)
# AREA_MAX = (230, 230)
# AREA_GROUND_HEIGHT = 1


    #     # Initialize AirSim connection
    # asc = ASC()

    # # Get start position
    # init_pos = INIT_POS
    # init_az = INIT_AZ
    
    # # Get area/model to explore -> can use tight bounds intially
    # init_vol = INIT_VOL

    
    # asc.getDepthImg

    # # Subdivide region into convex areas -> Octree


    # # TODO: Return cells lists of free and occupied cells with x, y, z in planning space
    
    # # Get initial discovered region

    # # Main Loop

    #     # Update cells data
    #     # Update frontier data
    #     # Update sectors
    #     # Update global trajectory
    #     # Update local trajectory