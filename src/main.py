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

    init_cam_pts = utils.getCamPts(25, 20, 36)

    all_occ = []
    all_free = []
    all_frontier = np.empty((0, 3), int)

    for y in range(10, 100, 5):
        # Fly to pose and get depth img
        asc.flyToPosAndYaw((55, y, 20), 45)
        #time.sleep(2)  TODO: Find a way to remove jerkiness 
        tic = time.perf_counter()
        _, ((pos), (rot)) = asc.getDepthImg()

        # Get cam points to check based on pose - Takes ~1 sec
        cam_pts = utils.getRtCamPoints(init_cam_pts, pos, rot)

        cam_tic = time.perf_counter()
        cam_traversal_pts = vmap.get_cam_traversal_pts(pos, cam_pts)
        cam_toc = time.perf_counter()

        # Traverse cam pts to get occupancies - TODO: Optimize
        trav_tic = time.perf_counter()
        occ_pts, free_pts = vmap.get_occ_for_rays(cam_traversal_pts)

        all_occ.append(occ_pts)
        vis_occ = np.unique(np.concatenate(all_occ, axis=0), axis=0)

        all_free.append(free_pts)
        vis_free = np.unique(np.concatenate(all_free, axis=0), axis=0)
        trav_toc = time.perf_counter()

        # Add/remove frontier cells
        frontier_tic = time.perf_counter()
        all_frontier = np.append(all_frontier, utils.setNewFrontierCells(free_pts), axis=0)  # only send new free points TODO: Calc bounding box and send that?
        all_frontier = utils.pruneFrontiers(all_frontier)
        frontier_toc = time.perf_counter()

        toc = time.perf_counter()
        print("Cam Time:", cam_toc-cam_tic)
        print("Trav Time:", trav_toc-trav_tic)
        print("Frontier Time:", frontier_toc-frontier_tic)
        print("= Total Time:", toc-tic)

        # Visualize occupancies
        utils.visOccRays(vis_occ, vis_free, all_frontier, pos)


if __name__ == "__main__":
    main()