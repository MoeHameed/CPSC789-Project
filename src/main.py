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

    init_cam_pts = utils.getCamPts(25, 10, 20)

    for y in range(10, 100, 5):
        asc.flyToPosAndYaw((10, y, 25), 45)
        _, ((pos), (rot)) = asc.getDepthImg()

        # get rotated cam points
        rot_pcd = o3d.geometry.PointCloud()
        rot_pcd.points = o3d.utility.Vector3dVector(np.asarray(init_cam_pts))
        rot_pcd.translate((pos[0], pos[1], pos[2]))
        rot_pcd.rotate(rot.as_matrix(), center=[pos[0], pos[1], pos[2]])

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vmap.get_cam_traversal_pts(pos, rot_pcd.points))

        voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 1)

        bbox = voxels.get_axis_aligned_bounding_box()

        sphere = o3d.geometry.TriangleMesh.create_sphere(1)
        sphere.translate((pos[0], pos[1], pos[2]))

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

        o3d.visualization.draw_geometries([voxels, bbox, sphere, axes])


if __name__ == "__main__":
    main()