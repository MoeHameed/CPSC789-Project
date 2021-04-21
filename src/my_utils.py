import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import math

UNKNOWN = 0
FREE = 1
OCCUPIED = 2
FRONTIER = 3

all_cells = np.zeros((150, 150, 50), dtype=int)

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
            pt = sph2cart(i, j, r) #np.round(sph2cart(i, j, r)).astype(int)
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

    o3d.visualization.draw_geometries([voxels, bbox, sphere, axes])

def visOccRays(occ_pts, free_pts, frontier_pts, pos):
    # flip axis for vis
    free_pts = [(y, x, z) for (x, y, z) in free_pts]
    occ_pts = [(y, x, z) for (x, y, z) in occ_pts]
    frontier_pts = [(y, x, z) for (x, y, z) in frontier_pts]

    free_pcd = o3d.geometry.PointCloud()
    free_pcd.points = o3d.utility.Vector3dVector(free_pts)
    free_pcd.paint_uniform_color([1, 0.706, 0])

    free_voxels = o3d.geometry.VoxelGrid.create_from_point_cloud(free_pcd, 1)

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

    o3d.visualization.draw_geometries([frontier_voxels, occ_voxels, sphere, axes])


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

def clean_frontiers(frontiers):
    new_pts = []

    for (x, y, z) in frontiers:
        pts = [(nx, ny, nz) for nx, ny, nz in [(x + 1, y, z), (x - 1, y, z), (x, y + 1, z), (x, y - 1, z), (x, y, z + 1), (x, y, z - 1)]]
        
        num_free = 0
        for pt in pts:
            if get_cell(pt) == FREE:
                num_free += 1

        if num_free > 4:
            all_cells[x][y][z] = FREE
        else:
            new_pts.append((x, y, z))

    return new_pts

def minDistToCells(cell_to_chk, cells):
    minDist = 0

    for cell in cells:
        d = euclideanDist(cell_to_chk, cell)
        if d < minDist:
            minDist = d
    
    return minDist

# randomly sample for poses in free cells around occupied cells
# given frontier bbox, new occ pts
# return list of possible poses
def calcPoses(bbox, occ_pts):
    possible_poses = []

    minxyz, maxxyz = bbox

    for _ in range(25):
        x = np.random.randint(low=minxyz[0], high=maxxyz[0])
        y = np.random.randint(low=minxyz[1], high=maxxyz[1])
        z = np.random.randint(low=minxyz[2], high=maxxyz[2])

        if all_cells[x][y][z] == FREE and minDistToCells((x, y, z), occ_pts) > 4:
            az = np.random.choice([0, 45, 90, 135, 180, 225, 270, 315])
            possible_poses.append((x, y, z, az))

    return possible_poses