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
