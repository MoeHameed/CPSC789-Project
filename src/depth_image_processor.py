import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

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
        # extrinsic = np.array([[1., 0., 0., 0.], 
        #                       [0., 1., 0., 0.], 
        #                       [0., 0., 1., 0.], 
        #                       [0., 0., 0., 1.]])

        depth_img = np.where(depth_img > 150, 0, depth_img)
        img = o3d.geometry.Image(depth_img)

        pcd = o3d.geometry.PointCloud.create_from_depth_image(img, intrinsic_params)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        bbox = pcd.get_axis_aligned_bounding_box()
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

        o3d.visualization.draw_geometries([pcd, bbox, axes])


        # pcd_cam = o3d.geometry.PointCloud()
        # pcd_cam.points = o3d.utility.Vector3dVector(cam_coords.T[:, :3])
        # pcd_cam.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        # bbox = pcd_cam.get_axis_aligned_bounding_box()
        # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

        # voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd_cam, voxel_size=1)
        # o3d.visualization.draw_geometries([voxel_grid, bbox, axes])

