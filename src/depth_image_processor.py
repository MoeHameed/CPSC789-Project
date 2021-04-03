import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class depthImageProcessor():
    def __init__(self):
        pass

    def pfm_to_voxel(self, depth_img, cam_pose):
        height = 144
        width = 256
        cx = width / 2
        cy = height / 2
        fx = cx / np.tan(90/2)
        fy = fx

        intrinsic_params = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        extrinsic = np.array([[1, 0., 0., 0.], 
                              [0., 1., 0., 0.], 
                              [0., 0., 1., 0.], 
                              [0., 0., 0., 1.]])

        depth_img = np.where(depth_img > 150, 0, depth_img)
        img = o3d.geometry.Image(depth_img)

        pcd = o3d.geometry.PointCloud.create_from_depth_image(img, intrinsic_params, extrinsic)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd.translate((cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]))
        r = R.from_euler('x', 90, degrees=True)
        pcd.rotate(r.as_matrix(), center=[cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]])
        r2 = R.from_euler('z', 360-cam_pose[1][2], degrees=True)
        pcd.rotate(r2.as_matrix(), center=[cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]])

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=1)

        bbox = voxel_grid.get_axis_aligned_bounding_box()

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

        sphere = o3d.geometry.TriangleMesh.create_sphere(1)
        sphere.translate((cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]))

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=1)

        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.add_geometry(axes)
        vis.add_geometry(sphere)

        for i in range(360):
            r3 = R.from_euler('z', 1, degrees=True)
            pcd.rotate(r3.as_matrix(), center=[cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]])

            vis.update_geometry(pcd)

            vis.poll_events()
            vis.update_renderer()


        o3d.visualization.draw_geometries([voxel_grid, axes, sphere])
