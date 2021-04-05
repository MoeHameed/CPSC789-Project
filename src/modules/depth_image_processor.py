import time
import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

class depthImageProcessor():
    def __init__(self):
        self.total_pcd_pts = np.asarray([[0, 0, 0]])
        
    def pfm_to_voxel(self, depth_img, cam_pose):
        height = 144
        width = 256
        cx = width / 2
        cy = height / 2
        fx = cx / np.tan(90/2)
        fy = fx

        intrinsic_params = o3d.camera.PinholeCameraIntrinsic(width, height, fx, fy, cx, cy)
        # TODO: Figure out extrinsic info
        extrinsic = np.array([[1., 0., 0., 0.], 
                              [0., 1., 0., 0.], 
                              [0., 0., 1., 0.], 
                              [0., 0., 0., 1.]])

        depth_img = np.where(depth_img > 150, 0, depth_img)
        img = o3d.geometry.Image(depth_img)

        pcd = o3d.geometry.PointCloud.create_from_depth_image(img, intrinsic_params, extrinsic, depth_scale=1)
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        pcd.translate((cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]))

        r = R.from_euler('x', 90, degrees=True)
        pcd.rotate(r.as_matrix(), center=[cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]])

        cam_r = cam_pose[1].as_euler('xyz', degrees=True)
        r2 = R.from_euler('xyz', [360-cam_r[0], 360-cam_r[1], 360-cam_r[2]], degrees=True)
        pcd.rotate(r2.as_matrix(), center=[cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]])

        pcd_pts = np.asarray(pcd.points)
        self.total_pcd_pts = np.concatenate((pcd_pts, self.total_pcd_pts), axis=0)
        
        new_pcd = o3d.geometry.PointCloud()
        new_pcd.points = o3d.utility.Vector3dVector(self.total_pcd_pts)

        voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(new_pcd, voxel_size=1)

        bbox = voxel_grid.get_axis_aligned_bounding_box()

        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

        sphere = o3d.geometry.TriangleMesh.create_sphere(1)
        sphere.translate((cam_pose[0][1], cam_pose[0][0], cam_pose[0][2]))

        #o3d.visualization.draw_geometries([bbox, voxel_grid, axes, sphere])