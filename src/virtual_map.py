import modules.binvox_rw as bvox
import utils
import skimage.draw as sk
import open3d as o3d
import numpy as np
from pprint import pprint

class virtual_map():
    def __init__(self):
        self.voxels = bvox.read_as_3d_array(open("binvox/map.binvox", 'rb'))

    # True if occupied, False if free
    def get_voxel_occ(self, cell):
        (x_lim, y_lim, z_lim) = self.voxels.data.shape
        if cell[0] in range(0, x_lim) and cell[1] in range(0, y_lim) and cell[2] in range(0, z_lim):
            return self.voxels.data[cell[1]][cell[0]][cell[2]]
        else:
            return False

    # TODO: Optimize?
    def get_occ_for_ray(self, cells):
        free_cells = []
        occ_cell = []

        for cell in cells:
            # get voxel occupancy at cell - overwrites existing cell values
            if self.get_voxel_occ(cell):
                occ_cell.append(cell)
                utils.all_cells[cell[0]][cell[1]][cell[2]] = utils.OCCUPIED
                break
            else:
                free_cells.append(cell)
                utils.all_cells[cell[0]][cell[1]][cell[2]] = utils.FREE

        return free_cells, occ_cell
    
    # TODO: Optimize?
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

        total_occ = np.concatenate(total_occ, axis=0)
        total_occ = np.unique(total_occ, axis=0)
        total_occ = total_occ[1:]

        return total_occ, total_free

        