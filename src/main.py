from air_sim_client import AirSimClient as ASC
import my_utils
import matplotlib.pyplot as plt
import vg
import numpy as np
import open3d as o3d
import time
from virtual_map import virtual_map as VMAP
from collections import deque


INIT_POSE = (55, 10, 20, 45) # X, Y, Z, AZ      # Azimuth (phi): [0, 359] : 0 = Forward X-axis 
INIT_VOL = ((1, 0, 0), (150, 150, 50))     # ((pos_x, pos_y, pos_z), (size_x, size_y, size_z)) : (position, size)

def main():
    total_proc_time = 0

    asc = ASC()
    
    vmap = VMAP()

    init_cam_pts = my_utils.getCamPts(25, 20, 36)

    all_occ = np.empty((0, 3), int)
    all_free = np.empty((0, 3), int)
    all_frontier = np.empty((0, 3), int)


    # Calculate global path


    asc.flyToPosAndYaw(INIT_POSE)

    poses = deque()
    poses.appendleft(INIT_POSE)
    # poses.appendleft((55, 15, 20, 45))
    # poses.appendleft((55, 20, 20, 45))
    # poses.appendleft((55, 25, 20, 45))
    # poses.appendleft((55, 30, 20, 45))
    # poses.appendleft((55, 35, 20, 45))
    # poses.appendleft((55, 40, 20, 45))
    # poses.appendleft((55, 45, 20, 45))
    # poses.appendleft((55, 50, 20, 45))
    # poses.appendleft((55, 55, 20, 45))

    covered_percent = 0

    all_tic = time.perf_counter()

    while covered_percent < 100:
        # Fly to pose and get depth img
        asc.flyToPosAndYaw(poses.pop())

        #time.sleep(2)  TODO: Find a way to remove jerkiness 
        tic = time.perf_counter()
        _, ((pos), (rot)) = asc.getDepthImg()

        # Get cam points to check based on pose - Takes ~1 sec - TODO: Optimize
        cam_tic = time.perf_counter()
        cam_pts = my_utils.getRtCamPoints(init_cam_pts, pos, rot)
        cam_traversal_pts = my_utils.get_cam_traversal_pts(pos, cam_pts)
        cam_toc = time.perf_counter()

        # Traverse cam pts to get occupancies - Takes ~1 sec - TODO: Optimize
        trav_tic = time.perf_counter()
        occ_pts, free_pts = vmap.get_occ_for_rays(cam_traversal_pts)

        if len(occ_pts) > 0:
            all_occ = np.append(all_occ, occ_pts, axis=0)

        if len(free_pts) > 0:
            all_free = np.append(all_free, free_pts, axis=0)
        trav_toc = time.perf_counter()

        # Add/remove frontier cells
        frontier_tic = time.perf_counter()
        new_frontier_pts = my_utils.setNewFrontierCells(free_pts)
        # clean frontiers
        new_frontier = my_utils.clean_frontiers(new_frontier_pts)
        # calc aabb for new frontier cells
        frontier_aabb = my_utils.getCellsAABB(new_frontier)
        all_frontier = np.append(all_frontier, new_frontier, axis=0)  # only send new free points 
        frontier_toc = time.perf_counter()

        # Calculate possible poses to explore for next pose using random within free space within latest frontier
        possible_poses = my_utils.calcPoses(frontier_aabb, occ_pts)

        # Explore poses and pick best one - does raycasting with each pose
        # Best one covers lots of frontiers with a little already occupied
        

        # Add to poses queue
        poses.appendleft(possible_poses[0])

        covered_percent += 10

        toc = time.perf_counter()
        total_proc_time += toc-tic

        print("Cam Time:", cam_toc-cam_tic)
        print("Trav Time:", trav_toc-trav_tic)
        print("Frontier Time:", frontier_toc-frontier_tic)
        print("= Iter Time:", toc-tic)
        print("")

        vis_occ = np.unique(all_occ, axis=0)
        vis_free = np.unique(all_free, axis=0)
        all_frontier = my_utils.pruneFrontiers(all_frontier) # TODO: Calc bounding box and send that?

        # Visualize occupancies
        my_utils.visOccRays(vis_occ, vis_free, all_frontier, pos)
    
    all_toc = time.perf_counter()

    print("== TOTAL PROC TIME:", total_proc_time)
    print("== TOTAL TIME:", all_toc-all_tic)
    print("")

    vis_occ = np.unique(all_occ, axis=0)
    vis_free = np.unique(all_free, axis=0)
    all_frontier = my_utils.pruneFrontiers(all_frontier) # TODO: Calc bounding box and send that?

    # Visualize occupancies
    my_utils.visOccRays(vis_occ, vis_free, all_frontier, pos)

if __name__ == "__main__":
    main()