import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

class depthImageProcessor():
    def __init__(self):
        pass

    def pfm_to_voxel(self, pfm_img, pose):
        height = 144
        width = 256

        # Compute K and K-inv
        cx = width / 2
        cy = height / 2
        fx = cx / np.tan(90/2)
        fy = fx
        K = np.array([[fx, 0, cx, 0.],
                    [0, fy, cy, 0.],
                    [0, 0, 1., 0.],
                    [0., 0., 0., 1.]])
        K_inv = np.linalg.inv(K)

        # Get pixel coordinates
        x = np.linspace(0, width - 1, width).astype(int)
        y = np.linspace(0, height - 1, height).astype(int)
        [x, y] = np.meshgrid(x, y)
        pixel_coords = np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))

        # Apply back-projection: K_inv @ pixels * depth
        cam_coords = K_inv[:3, :3] @ pixel_coords * pfm_img.flatten()

        # Limit points to 150m in the z-direction for visualisation
        cam_coords = cam_coords[:, np.where(cam_coords[2] <= 150)[0]]

        # Visualize
        intrinsic_params = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        # extrinsic = np.array([[1., 0., 0., 0.], 
        #                       [0., 1., 0., 0.], 
        #                       [0., 0., 1., 0.], 
        #                       [0., 0., 0., 1.]])

        img = o3d.geometry.Image(pfm_img)

        pcd = o3d.geometry.PointCloud.create_from_depth_image(img, intrinsic_params, depth_trunc=150.0)
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

