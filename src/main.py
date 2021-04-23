from air_sim_client import AirSimClient as ASC
import my_utils
import matplotlib.pyplot as plt
import vg
import numpy as np
import open3d as o3d
import time
from virtual_map import virtual_map as VMAP
from collections import deque


INIT_POSE = (5, 5, 5, 0) # X, Y, Z, AZ      # Azimuth (phi): [0, 359] : 0 = Forward X-axis  # used (55, 10, 20, 45)
INIT_VOL = ((1, 0, 0), (150, 150, 50))     # ((pos_x, pos_y, pos_z), (size_x, size_y, size_z)) : (position, size)

def main():
    # TODO: Add print statements and more perf counters
    total_proc_time = 0

    vmap = VMAP()

    init_cam_pts = my_utils.getCamPts(25, 20, 36)

    all_occ = np.empty((0, 3), int)
    all_free = np.empty((0, 3), int)
    all_frontier = np.empty((0, 3), int)

    # Calculate global path -> 6 x 6 x 2 sectors of 25 x 25 x 25 each
    # Create sectors
    print("Calculating sectors and hamiltonian path . . .")
    sectors = my_utils.calcSectors()

    # Calculate inital hamiltonian order between sectors -> list of sector mid points in order to traverse them in, the total euclidean distance 
    sec_order, global_dist = my_utils.calcHamOrder(sectors)

    # Calculate A* path between each sector -> list of (list of coords between sector a and b)
    # include given start point
    print("Calculating A* between sectors . . .")
    init_path = my_utils.calcAStarBetweenPaths(sec_order, (INIT_POSE[0], INIT_POSE[1], INIT_POSE[2]))    # List of (list of points to traverse)
    init_path = np.concatenate(init_path, axis=0)
    init_path = init_path[1:]     # remove first one since we will already be there
    
    init_path_deque = deque()
    for pt in init_path:
        init_path_deque.appendleft(pt)
    
    # print(global_dist, len(np.concatenate(init_path, axis=0)))

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

    print("Starting AirSimClient . . .")

    # start airsim connection
    asc = ASC()

    all_tic = time.perf_counter()

    while covered_percent < 100:    # TODO: replace with all possible sectors explored
        # Fly to pose and get depth img
        asc.flyToPosAndYaw(poses.pop())

        #time.sleep(2)  TODO: Find a way to remove jerkiness 
        print("Getting depth image . . .")
        tic = time.perf_counter()
        _, ((pos), (rot)) = asc.getDepthImg()

        # Get cam points to check based on pose - Takes ~1 sec - TODO: Optimize
        print("Calculating raycast points . . .")
        cam_tic = time.perf_counter()
        cam_pts = my_utils.getRtCamPoints(init_cam_pts, pos, rot)
        cam_traversal_pts = my_utils.get_cam_traversal_pts(pos, cam_pts)
        cam_toc = time.perf_counter()

        # Traverse cam pts to get occupancies - Takes ~1 sec - TODO: Optimize
        print("Traversing rays . . .")
        trav_tic = time.perf_counter()
        occ_pts, free_pts = vmap.get_occ_for_rays(cam_traversal_pts)

        if len(occ_pts) > 0:
            all_occ = np.append(all_occ, occ_pts, axis=0)

        if len(free_pts) > 0:
            all_free = np.append(all_free, free_pts, axis=0)
        trav_toc = time.perf_counter()

        # Add/remove frontier cells
        print("Calculating and cleaning frontiers . . .")
        frontier_tic = time.perf_counter()
        new_frontier_pts = my_utils.setNewFrontierCells(free_pts)
        # clean frontiers
        new_frontier = my_utils.clean_frontiers(new_frontier_pts)
        # calc aabb for new frontier cells
        frontier_aabb = my_utils.getCellsAABB(new_frontier)
        if len(new_frontier) > 0:
            all_frontier = np.append(all_frontier, new_frontier, axis=0)  # only send new free points 
        frontier_toc = time.perf_counter()

        # Calculate possible poses to explore for next pose using randomness within free space within latest frontier
        # pose should be close to global path
        # pose should cover lots of frontiers and a few occupied cells if any
        # Future TODO: Use several possible poses instead of best one
        print("Calculating next poses . . .")
        possible_poses = my_utils.calcPoses(frontier_aabb, occ_pts)

        # Explore poses and pick best one based on distance to next point
        # Add to poses queue
        next_pos_to_be_close_to = init_path_deque.pop()
        selected_pose, _ = my_utils.findPoseClosest(next_pos_to_be_close_to, possible_poses)
        print("Next pose:", selected_pose)
        poses.appendleft(selected_pose)

        covered_percent += 10

        # Perf
        toc = time.perf_counter()
        total_proc_time += toc-tic

        print("Cam Time:", cam_toc-cam_tic)
        print("Trav Time:", trav_toc-trav_tic)
        print("Frontier Time:", frontier_toc-frontier_tic)
        print("= Iter Time:", toc-tic)
        print("")

        # Vis this iteration
        vis_occ = np.unique(all_occ, axis=0)
        vis_free = np.unique(all_free, axis=0)
        all_frontier = my_utils.pruneFrontiers(all_frontier) # TODO: Calc bounding box and send that?

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