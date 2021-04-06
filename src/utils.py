import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R

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
    pts = []
    for i in range(-width, width):
        for j in range(-height, height):
            pts.append(sph2cart(i, j, r))
    return np.asarray(pts)
    
    # axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=[0, 0, 0])

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(np.asarray(pts))

    # pcd.translate((5, 10, 25))
    # r2 = R.from_euler('xyz', [0, 0, 0], degrees=True)       # TODO: replace with position of cam, add translation matrix
    # pcd.rotate(r2.as_matrix(),center=[0, 0, 0])

    # #o3d.visualization.draw_geometries([pcd, axes])

    # return pcd

def getRtCamPoints(cam_pts, pos, rot):
    # get rotated cam points TODO: Find optimized alternative using Rt matrix
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

# returns all unique cells that are on the edge 
# TODO: Optimize
def getNewFrontierCells(occ_pts, free_pts, frontier_pts):
    # go through all existing frontiers and remove them if they are now in occ or free
    # pts_to_remove = []
    # for pt in frontier_pts:
    #     if (occ_pts == pt).all(1).any() or (free_pts == pt).all(1).any():
    #         pts_to_remove.append(pt)
    
    # frontier_pts = removeElemsFromArray(pts_to_remove, frontier_pts)

    indexes_to_remove = []
    for i in range(len(frontier_pts)):
        if (occ_pts == frontier_pts[i]).all(1).any() or (free_pts == frontier_pts[i]).all(1).any():
            indexes_to_remove.append(i)
    
    frontier_pts = np.delete(frontier_pts, indexes_to_remove, axis=0)

    if len(frontier_pts) == 0:
        frontier_pts = np.empty((0, 3), int)

    # add new frontiers for free pts that are on the edge
    # Check 6 sides of free cell, and add each side that is neither occupied nor free
    for (x, y, z) in free_pts:
        posx = np.array([x + 1, y, z])
        negx = np.array([x - 1, y, z])

        posy = np.array([x, y + 1, z])
        negy = np.array([x, y - 1, z])

        posz = np.array([x, y, z + 1])
        negz = np.array([x, y, z - 1])

        if not((occ_pts == posx).all(1).any() or (free_pts == posx).all(1).any()):
            frontier_pts = np.append(frontier_pts, [posx], axis=0)
        if not((occ_pts == negx).all(1).any() or (free_pts == negx).all(1).any()):
            frontier_pts = np.append(frontier_pts, [negx], axis=0)
        if not((occ_pts == posy).all(1).any() or (free_pts == posy).all(1).any()):
            frontier_pts = np.append(frontier_pts, [posy], axis=0)
        if not((occ_pts == negy).all(1).any() or (free_pts == negy).all(1).any()):
            frontier_pts = np.append(frontier_pts, [negy], axis=0)
        if not((occ_pts == posz).all(1).any() or (free_pts == posz).all(1).any()):
            frontier_pts = np.append(frontier_pts, [posz], axis=0)
        if not((occ_pts == negz).all(1).any() or (free_pts == negz).all(1).any()):
            frontier_pts = np.append(frontier_pts, [negz], axis=0)

    return frontier_pts

# TODO: Optimize
# Remove arrays from list of arrays
def removeElemsFromArray(elems, arr):
    for elem in elems:
        for i in range(len(arr)):
            if np.array_equal(elem, arr[i]):
                arr.pop(i)
                break
    return arr