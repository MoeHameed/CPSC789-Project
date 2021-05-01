import math
import time
from pprint import pprint

import numpy as np
import open3d as o3d
import skimage.draw as sk
from python_tsp.exact import (solve_tsp_brute_force,
                              solve_tsp_dynamic_programming)
from python_tsp.heuristics import solve_tsp_local_search
from scipy.spatial.transform import Rotation as R

import modules.binvox_rw as bvox
import my_utils
from astar_search import AStarSearch

UNKNOWN = 0
FREE = 1
OCCUPIED = 2
FRONTIER = 3

all_cells = np.zeros((150, 150, 50), dtype=int)

voxels = []

def initVMAP():
    global voxels
    voxels = bvox.read_as_3d_array(open("binvox/map.binvox", 'rb'))


# True if occupied, False if free
def get_voxel_occ(cell):
    global voxels

    (x_lim, y_lim, z_lim) = voxels.data.shape
    if cell[0] in range(0, x_lim) and cell[1] in range(0, y_lim) and cell[2] in range(0, z_lim):
        return voxels.data[cell[1]][cell[0]][cell[2]]
    else:
        return False

# TODO: Optimize?
def get_occ_for_ray(cells):
    free_cells = []
    occ_cell = []

    for cell in cells:
        if my_utils.all_cells[cell[0]][cell[1]][cell[2]] == my_utils.OCCUPIED:
            break
        
        if my_utils.all_cells[cell[0]][cell[1]][cell[2]] == my_utils.FREE:
            continue

        # get voxel occupancy at cell - overwrites existing cell values
        if get_voxel_occ(cell):
            occ_cell.append(cell)
            my_utils.all_cells[cell[0]][cell[1]][cell[2]] = my_utils.OCCUPIED
            break
        else:
            free_cells.append(cell)
            my_utils.all_cells[cell[0]][cell[1]][cell[2]] = my_utils.FREE

    return free_cells, occ_cell

# TODO: Optimize?
def get_occ_for_rays(rays):
    total_occ = []
    total_free = []

    for ray in rays:
        free, occ = get_occ_for_ray(ray)
        if len(free) > 0:
            total_free.append(free)
        if len(occ) > 0:
            total_occ.append(occ)

    if len(total_free) > 0:
        total_free = np.concatenate(total_free, axis=0)
        total_free = np.unique(total_free, axis=0)

    if len(total_occ) > 0:
        total_occ = np.concatenate(total_occ, axis=0)
        total_occ = np.unique(total_occ, axis=0)
        total_occ = total_occ[1:]

    return total_occ, total_free

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return rho, math.degrees(phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y

def get_cell(pos):
    (x, y, z) = pos
    if x >= 0 and x < 150 and y >= 0 and y < 150 and z >= 0 and z < 50:
        return all_cells[x][y][z]
    else:
        return -1

def cart2cyl(coord):
    """Convert cartesian tuple (x, y, z) to cylindrical tuple (radius (r), azimuth (phi), elevation (z)).

    Args:
        coord ([type]): [description]

    Returns:
        [type]: [description]
    """
    x, y, z = coord
    r = np.sqrt(x**2 + y**2)
    az = np.arctan2(y, x)
    return r, np.rad2deg(az), z   # radius, theta, phi

def cyl2cart(rho, phi, z):
    x = rho * math.cos(math.radians(phi))
    y = rho * math.sin(math.radians(phi))
    return(np.rad2deg(x), np.rad2deg(y), z)

def cart2sph(coord):
    """Convert cartesian tuple (x, y, z) to spherical tuple (radius (r), elevation (theta), azimuth (phi)).
    Elevation is z-axis going downwards

    Args:
        coord ([type]): [description]

    Returns:
        [type]: [description]
    """
    x, y, z = coord
    hxy = np.hypot(x, y)
    r = np.hypot(hxy, z)
    el = np.arctan2(hxy, z) # z-axis down, switch for xy-plane up
    az = np.arctan2(y, x)
    return r, np.rad2deg(az), np.rad2deg(el)    # radius, theta, phi

def sph2cart(az, el, r):
    az = np.deg2rad(az)
    el = np.deg2rad(el)
    rcos_theta = r * np.cos(el)
    x = rcos_theta * np.cos(az)
    y = rcos_theta * np.sin(az)
    z = r * np.sin(el)
    return x, y, z

def euclideanDist(A, B):
    """Euclidean distance between two (x, y, z) tuples

    Args:
        A ([type]): [description]
        B ([type]): [description]

    Returns:
        [type]: [description]
    """
    a = np.array((A[0], A[1], A[2]))
    b = np.array((B[0], B[1], B[2]))
    return np.linalg.norm(a-b)

def getCamPts(r, height, width):
    pts = np.empty((0, 3), int)
    for i in range(-width, width):
        for j in range(-height, height):
            pt = np.round(sph2cart(i, j, r)).astype(int)
            pts = np.append(pts, [pt], axis=0)
    return np.unique(pts, axis=0)
    
    # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(np.asarray(pts))

    # pcd.translate((5, 10, 25))
    # r2 = R.from_euler('xyz', [0, 0, 0], degrees=True)       # TODO: replace with position of cam, add translation matrix
    # pcd.rotate(r2.as_matrix(),center=[0, 0, 0])

    # #o3d.visualization.draw_geometries([pcd, axes])

    # return pcd

def getRtCamPoints(cam_pts, pos, rot):
    # get rotated cam points TODO: Find optimized alternative using Rt matrix, but its already really fast ~0.0002
    rot_pcd = o3d.geometry.PointCloud()
    rot_pcd.points = o3d.utility.Vector3dVector(np.asarray(cam_pts))
    rot_pcd.translate((pos[0], pos[1], pos[2]))
    rot_pcd.rotate(rot.as_matrix(), center=[pos[0], pos[1], pos[2]])
    return rot_pcd.points

def visCamTraversalPts(pts, pos):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)

    voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 1)

    bbox = voxels.get_axis_aligned_bounding_box()

    sphere = o3d.geometry.TriangleMesh.create_sphere(1)
    sphere.translate((pos[1], pos[0], pos[2]))

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

    o3d.visualization.draw_geometries([voxels, bbox, sphere, axes]) # pylint: disable=maybe-no-member

def visOccRays(occ_pts, free_pts, frontier_pts, pos):
    # flip axis for vis
    free_pts = [(y, x, z) for (x, y, z) in free_pts]
    occ_pts = [(y, x, z) for (x, y, z) in occ_pts]
    frontier_pts = [(y, x, z) for (x, y, z) in frontier_pts]

    free_pcd = o3d.geometry.PointCloud()
    free_pcd.points = o3d.utility.Vector3dVector(free_pts)
    free_pcd.paint_uniform_color([1, 0.706, 0])

    #free_voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(free_pcd, 1)

    occ_pcd = o3d.geometry.PointCloud()
    occ_pcd.points = o3d.utility.Vector3dVector(occ_pts)
    #occ_pcd.paint_uniform_color([0, 0, 0])

    occ_voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(occ_pcd, 1)

    frontier_pcd = o3d.geometry.PointCloud()
    frontier_pcd.points = o3d.utility.Vector3dVector(frontier_pts)
    frontier_pcd.paint_uniform_color([0.66, 0.66, 0.66])

    frontier_voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(frontier_pcd, 1)

    #bbox = free_voxels.get_axis_aligned_bounding_box()

    sphere = o3d.geometry.TriangleMesh.create_sphere(1)
    sphere.translate((pos[1], pos[0], pos[2]))

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])

    o3d.visualization.draw_geometries([frontier_voxels, occ_voxels, sphere, axes]) # pylint: disable=maybe-no-member


# TODO: Optimize & Use probabilistic logic to reduce noise, can also use a 3x3 kernel based on how many are frontier/free/occ
def setNewFrontierCells(free_pts):
    frontiers = []

    # set frontiers pts
    # Check 6 sides of each given free cell, and add each side that is unkown
    for (x, y, z) in free_pts:
        posx = np.array([x + 1, y, z])
        negx = np.array([x - 1, y, z])

        posy = np.array([x, y + 1, z])
        negy = np.array([x, y - 1, z])

        posz = np.array([x, y, z + 1])
        negz = np.array([x, y, z - 1])

        if get_cell(posx) == UNKNOWN:
            all_cells[posx[0]][posx[1]][posx[2]] = FRONTIER
            frontiers.append(posx)

        if get_cell(negx) == UNKNOWN:
            all_cells[negx[0]][negx[1]][negx[2]] = FRONTIER
            frontiers.append(negx)

        if get_cell(posy) == UNKNOWN:
            all_cells[posy[0]][posy[1]][posy[2]] = FRONTIER
            frontiers.append(posy)

        if get_cell(negy) == UNKNOWN:
            all_cells[negy[0]][negy[1]][negy[2]] = FRONTIER
            frontiers.append(negy)

        if get_cell(posz) == UNKNOWN:
            all_cells[posz[0]][posz[1]][posz[2]] = FRONTIER
            frontiers.append(posz)

        if get_cell(negz) == UNKNOWN:
            all_cells[negz[0]][negz[1]][negz[2]] = FRONTIER
            frontiers.append(negz)

    return frontiers

# TODO: Optimize
# Remove arrays from list of arrays
def removeElemsFromArray(elems, arr):
    for elem in elems:
        for i in range(len(arr)):
            if np.array_equal(elem, arr[i]):
                arr.pop(i)
                break
    return arr

# TODO: Optimize
def pruneFrontiers(all_frontiers):
    frontier = np.empty((0, 3), int)

    for (x, y, z) in all_frontiers:
        if all_cells[x][y][z] == FRONTIER:
            frontier = np.append(frontier, [[x, y, z]], axis=0)

    return frontier

def round_safe(coords):
    if (len(coords) > 1
            and coords[0] % 1 == 0.5
            and coords[1] - coords[0] == 1):
        _round_function = np.floor
    else:
        _round_function = np.round
    return _round_function(coords).astype(int)

total_rays_pts = []

def get_cells_along_line2(start, stop):
    global total_rays_pts
    start = np.asarray(start)
    stop = np.asarray(stop)
    npoints = int(np.ceil(np.max(np.abs(stop - start)))) + 1

    coords = []
    for dim in range(len(start)):
        dimcoords = np.linspace(start[dim], stop[dim], npoints, True)
        dimcoords = round_safe(dimcoords).astype(int)
        coords.append(dimcoords)

    return list(zip(coords[0], coords[1], coords[2]))

# Bresenham's 3D
def get_cells_along_line(start, stop):
    (x1, y1, z1) = round_safe(start).astype(int)
    (x2, y2, z2) = round_safe(stop).astype(int)

    ListOfPoints = []
    ListOfPoints.append((x1, y1, z1))
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    dz = abs(z2 - z1)
    if (x2 > x1):
        xs = 1
    else:
        xs = -1
    if (y2 > y1):
        ys = 1
    else:
        ys = -1
    if (z2 > z1):
        zs = 1
    else:
        zs = -1
  
    # Driving axis is X-axis"
    if (dx >= dy and dx >= dz):        
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        while (x1 != x2):
            x1 += xs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dx
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            if get_cell((x1, y1, z1)) != -1:
                ListOfPoints.append((x1, y1, z1))
  
    # Driving axis is Y-axis"
    elif (dy >= dx and dy >= dz):       
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        while (y1 != y2):
            y1 += ys
            if (p1 >= 0):
                x1 += xs
                p1 -= 2 * dy
            if (p2 >= 0):
                z1 += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            if get_cell((x1, y1, z1)) != -1:
                ListOfPoints.append((x1, y1, z1))
  
    # Driving axis is Z-axis"
    else:        
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        while (z1 != z2):
            z1 += zs
            if (p1 >= 0):
                y1 += ys
                p1 -= 2 * dz
            if (p2 >= 0):
                x1 += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            if get_cell((x1, y1, z1)) != -1:
                ListOfPoints.append((x1, y1, z1))

    return ListOfPoints

def get_cam_traversal_pts(origin, cam_pts):
    rays = []
    for pt in cam_pts:
        rays.append(get_cells_along_line(origin, (pt[0], pt[1], pt[2])))

    return rays

# returns (x1, y1, z1), (x2, y2, z2)
def getCellsAABB(pts):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, 1)
    bbox = voxels.get_axis_aligned_bounding_box()

    #TODO: Check for bounds limits, sometimes less than zero vals and sometimes higher than max volume 
    max_b = np.ceil(bbox.get_max_bound()).astype(int)
    min_b = np.floor(bbox.get_min_bound()).astype(int)

    return min_b, max_b

# sets forntier with more than 4 free neighbours as free cell -> "cleaning/pruning" the frontier points
def clean_frontiers(frontiers):
    new_pts = []
    new_occ = []
    new_free = []

    for (x, y, z) in frontiers:
        pts = [(nx, ny, nz) for nx, ny, nz in [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z), (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]]
        
        num_free = 0
        num_occ = 0
        for pt in pts:
            pt_type = get_cell(pt)
            if pt_type == FREE:
                num_free += 1
            elif pt_type == OCCUPIED:
                num_occ += 1

        if num_occ > num_free:
            all_cells[x][y][z] = OCCUPIED
            new_occ.append((x, y, z))
        elif num_free > 4:
            all_cells[x][y][z] = FREE
            new_free.append((x, y, z))
        else:
            new_pts.append((x, y, z))

    return new_pts, np.asarray(new_occ), np.asarray(new_free)

def minDistToCells(cell_to_chk, cells):
    minDist = math.inf

    for cell in cells:
        d = euclideanDist(cell_to_chk, cell)
        if d < minDist:
            minDist = d
    
    return minDist

def findPoseClosest(pt_to_be_near, pts_to_chk):
    minDist = math.inf
    closest_pt = pts_to_chk[0]

    for (x, y, z, az) in pts_to_chk:
        d = euclideanDist((x, y, z), pt_to_be_near)
        if d < minDist:
            minDist = d
            closest_pt = (x, y, z, az)
    
    return closest_pt, minDist

# randomly sample for poses in free cells around occupied cells
# given frontier bbox, new occ pts, pt to look at if there are no occ pts
# return list of possible poses
# TODO: use multithreading to speed up random search?
def calcPoses(bbox, all_free, all_occ, occ_pts, init_cam_pts, nextGlobalPt, curr_pos):
    possible_poses = []

    minxyz, maxxyz = bbox

    # TODO: Come up with better numbers
    minOccPercent = 0.0
    minFrontPercent = 0.01

    if len(occ_pts) > 0:
        minOccPercent = 0.001

    numChecked = 0

    while len(possible_poses) < 5 and len(all_free) > 0:
        if numChecked > 5:
            break

        numChecked += 1

        # get up to 5 valid possible poses - small for speed
        for _ in range(5):
            idx = np.random.randint(low=0, high=len(all_free))
            (x, y, z) = all_free[idx]

            if get_cell((x, y, z)) == FREE and minDistToCells((x, y, z), all_occ) > 6 and isLoS(curr_pos, (x, y, z)):
                if len(occ_pts) > 5:
                    # check 4 of the poses - small for speed 
                    for _ in range(4):
                        az = np.random.randint(low=0, high=360)
                        if is_good_pose((x, y, z, az), minOccPercent, minFrontPercent, init_cam_pts):
                            possible_poses.append((x, y, z, az))
                else:
                    # calc rotation to look at global point
                    az = math.atan2(nextGlobalPt[0]-curr_pos[0], nextGlobalPt[1]-curr_pos[1])
                    az = math.degrees(az)
                    if az < 0:
                        az += 360 
                    possible_poses.append((x, y, z, az))

    if len(possible_poses) < 1:
        az = np.random.randint(low=0, high=360)
        possible_poses.append((curr_pos[0], curr_pos[1], curr_pos[2], az))

    return possible_poses

def isLoS(curr_pos, new_pos):
    # calc ray points
    pts = get_cells_along_line(curr_pos, new_pos)

    for pt in pts:
        if get_voxel_occ(pt):
            print("NOT IN LOS")
            return False
    
    return True

# Determine if the pose is good
# Based on if min occ percent and min front percent are met by pose 
def is_good_pose(pose, minOccPer, minFrontPer, init_cam_pts):
    # Use raycasting (using all_cells) to determine how many occupied and frontier would be covered 
    pos = (pose[0], pose[1], pose[2])
    rot = R.from_euler('z', pose[3], degrees=True) 

    cam_pts = getRtCamPoints(init_cam_pts, pos, rot)
    rays = get_cam_traversal_pts(pos, cam_pts)
    num_front, num_occ, total_pts = traversePosePts(rays, pos)

    #print(pose, "::", num_front, num_occ, total_pts)

    # Determine percentage of occupied cells and frontier cells 
    if num_front/total_pts >= minFrontPer and num_occ/total_pts >= minOccPer:
        return True

    return False

# TODO: Visualize
# Do raycasting to check if pose is good
def traversePosePts(rays, pos):
    all_front_pts = []
    all_occ_pts = []
    all_other_pts = []

    for ray in rays:
        fPts, oPts, otherPts = traverseRay(ray)
        if len(fPts) > 0:
            all_front_pts.append(fPts)
        if len(oPts) > 0:
            all_occ_pts.append(oPts)
        if len(otherPts) > 0:
            all_other_pts.append(otherPts)

    if len(all_front_pts) > 0:
        all_front_pts = np.unique(np.concatenate(all_front_pts, axis=0), axis=0)
    if len(all_occ_pts) > 0:
        all_occ_pts = np.unique(np.concatenate(all_occ_pts, axis=0), axis=0)
    if len(all_other_pts) > 0:
        all_other_pts = np.unique(np.concatenate(all_other_pts, axis=0), axis=0)

    #visOccRays(all_occ_pts, all_other_pts, all_front_pts, pos)

    num_front = len(all_front_pts)
    num_occ = len(all_occ_pts)
    total_pts = len(all_other_pts) + num_front + num_occ

    return num_front, num_occ, total_pts

# gets first frontier or occupied point
def traverseRay(ray_pts):
    front_pts = []
    occ_pts = []
    other_pts = []

    for pt in ray_pts:
        if get_cell(pt) == FRONTIER:
            front_pts.append(pt)
            break
        if get_cell(pt) == OCCUPIED:
            occ_pts.append(pt)
            break
        other_pts.append(pt)

    return front_pts, occ_pts, other_pts

# Calculate centers of even sectors of 25 x 25 x 25 within volume 
def calcSectors():

    pts = []
    for z in range(0, 50, 25):
        for x in range(0, 150, 25):
            for y in range(0, 150, 25):
                pts.append([x+12, y+12, z+12])

    # lines = []
    # for i in range(len(pts)-1):
    #     lines.append([i, i+1])

    # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=150, origin=[0, 0, 0])
    # line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(pts), lines=o3d.utility.Vector2iVector(lines))
    # o3d.visualization.draw_geometries([line_set, axes])

    return pts

# Calculate hamiltonian path between given pts => uses TSP solver
# returns pts in order to traverse them in
def calcHamOrder(pts):
    # calc distance matrix between each point
    distMat = []
    for pt in pts:
        dists = []
        for pt2 in pts:
            dists.append(np.round(euclideanDist(pt, pt2)).astype(int))
        distMat.append(dists)

    # remove first col since we aren't going back to start
    distMat = np.asarray(distMat)
    distMat[:, 0] = 0

    # solve tsp
    perm, dist = solve_tsp_local_search(distMat)

    # get pts correseponding to each idx
    new_pts = []
    for i in perm:
        new_pts.append(pts[i])
    
    #Viz
    # pts2 = [(y, x, z) for (x, y, z) in new_pts]
    # lines = []
    # for i in range(len(pts2)-1):
    #     lines.append([i, i+1])

    # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    # line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(pts2), lines=o3d.utility.Vector2iVector(lines))
    # o3d.visualization.draw_geometries([line_set, axes]) # pylint: disable=maybe-no-member

    return new_pts, dist

# Calculate a-star path between all pts using distance and checking for obstacles
def calcAStarBetweenPaths(pts, startPt=None):
    pts = [tuple(l) for l in pts]

    if startPt != None:
        pts.insert(0, startPt)

    astar = AStarSearch()

    all_path_pts = []

    for i in range(len(pts)-1):
        path = astar.search(pts[i], pts[i+1])
        all_path_pts.append(path)    # Add to a deque

    # Viz
    path_pts = np.concatenate(all_path_pts, axis=0)
    path_pts2 = [(y, x, z) for (x, y, z) in path_pts]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(path_pts2)

    lines = []
    for i in range(len(path_pts2)-1):
        lines.append([i, i+1])

    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10, origin=[0, 0, 0])
    line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(path_pts2), lines=o3d.utility.Vector2iVector(lines))
    o3d.visualization.draw_geometries([line_set, pcd, axes]) # pylint: disable=maybe-no-member

    return all_path_pts
