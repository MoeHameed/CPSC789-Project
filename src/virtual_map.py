import modules.binvox_rw as bvox
import utils
import skimage.draw as sk
import open3d as o3d
import numpy as np
from pprint import pprint

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

    def get_cam_traversal_pts(self, origin, cam_pts):
        total_pts = []
        for pt in cam_pts:
            total_pts.append(self.get_cells_along_line(origin, (pt[0], pt[1], pt[2])))
        
        total_pts_vis = np.concatenate(total_pts, axis=0)
        total_pts_vis = np.unique(total_pts_vis, axis=0)
        total_pts_vis = total_pts_vis[1:]
        total_pts_vis = [(y, x, z) for (x, y, z) in total_pts_vis]      # flip xy axis for vis

        return total_pts, total_pts_vis

    # True if occupied, False if free
    # TODO: Range check
    def get_voxel_occ(self, cell):
        (x_lim, y_lim, z_lim) = self.voxels.data.shape
        if cell[0] in range(0, x_lim) and cell[1] in range(0, y_lim) and cell[2] in range(0, z_lim):
            return self.voxels.data[cell[1]][cell[0]][cell[2]]
        else:
            return False

    def get_occ_for_ray(self, cells):
        free_cells = []
        occ_cell = []

        for cell in cells:
            # get voxel occupancy at cell
            if self.get_voxel_occ(cell):
                occ_cell.append(cell)
                break
            else:
                free_cells.append(cell)

        return free_cells, occ_cell

    def get_occ_for_rays(self, rays):
        total_occ = []
        total_free = []

        for ray in rays:
            free, occ = self.get_occ_for_ray(ray)
            if len(free) > 0:
                total_free.append(free)
            if len(occ) > 0:
                total_occ.append(occ)

        total_free = np.concatenate(total_free, axis=0)
        total_free = np.unique(total_free, axis=0)
        total_free = total_free[1:]

        total_occ = np.concatenate(total_occ, axis=0)
        total_occ = np.unique(total_occ, axis=0)
        total_occ = total_occ[1:]

        return total_occ, total_free

        