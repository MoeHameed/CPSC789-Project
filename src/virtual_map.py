import modules.binvox_rw as bvox
import utils
import skimage.draw as sk
import open3d as o3d
import numpy as np

class virtual_map():
    def __init__(self):
        self.voxels = bvox.read_as_3d_array(open("binvox/map.binvox", 'rb'))

    def get_cells_along_line(self, start, end): # (x1, y1, z1), (x2, y2, z2)
        raw_pts = sk.line_nd(start, end, endpoint=True, integer=True)
        pts = list(zip(raw_pts[0], raw_pts[1], raw_pts[2]))
        
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(pts)
        # voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 1)
        # axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # o3d.visualization.draw_geometries([voxels, axes])

        return pts

    def traverse_cam_pts(self, origin, cam_pts):
        total_pts = []
        for pt in cam_pts:
            total_pts.append(self.get_cells_along_line(origin, (pt[0], pt[1], pt[2])))
        
        total_pts = np.concatenate(total_pts, axis=0)
        total_pts = np.unique(total_pts, axis=0)
        total_pts = total_pts[1:]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(total_pts)
        voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 1)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame()
        o3d.visualization.draw_geometries([voxels, axes])


virtual_map().traverse_cam_pts((0, 0, 0), utils.getCamPts(25, 15, 30))