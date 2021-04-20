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

    all_occ = np.empty((0, 3), int)
    all_free = np.empty((0, 3), int)
    all_frontier = np.empty((0, 3), int)

    all_tic = time.perf_counter()

    for y in range(10, 100, 5):
        # Fly to pose and get depth img
        asc.flyToPosAndYaw((55, y, 20), 45)
        #time.sleep(2)  TODO: Find a way to remove jerkiness 
        tic = time.perf_counter()
        _, ((pos), (rot)) = asc.getDepthImg()

        # Get cam points to check based on pose - Takes ~1 sec - TODO: Optimize
        cam_tic = time.perf_counter()
        cam_pts = utils.getRtCamPoints(init_cam_pts, pos, rot)
        cam_traversal_pts = utils.get_cam_traversal_pts(pos, cam_pts)
        cam_toc = time.perf_counter()

        # Traverse cam pts to get occupancies - Takes ~1 sec - TODO: Optimize
        trav_tic = time.perf_counter()
        occ_pts, free_pts = vmap.get_occ_for_rays(cam_traversal_pts)

        all_occ = np.append(all_occ, occ_pts, axis=0)
        #vis_occ = np.unique(np.concatenate(all_occ, axis=0), axis=0)

        all_free = np.append(all_free, free_pts, axis=0)
        #vis_free = np.unique(np.concatenate(all_free, axis=0), axis=0)
        trav_toc = time.perf_counter()

        # Add/remove frontier cells
        frontier_tic = time.perf_counter()
        all_frontier = np.append(all_frontier, utils.setNewFrontierCells(free_pts), axis=0)  # only send new free points TODO: Calc bounding box and send that?
        frontier_toc = time.perf_counter()

        toc = time.perf_counter()
        print("Cam Time:", cam_toc-cam_tic)
        print("Trav Time:", trav_toc-trav_tic)
        print("Frontier Time:", frontier_toc-frontier_tic)
        print("= Iter Time:", toc-tic)
        print("")
    
    all_toc = time.perf_counter()

    print("== TOTAL TIME:", all_toc-all_tic)
    print("")

    vis_occ = np.unique(np.concatenate(all_occ, axis=0), axis=0)
    vis_free = np.unique(np.concatenate(all_free, axis=0), axis=0)
    all_frontier = utils.pruneFrontiers(all_frontier)

    # Visualize occupancies
    utils.visOccRays(vis_occ, vis_free, all_frontier, pos)

if __name__ == "__main__":
    main()