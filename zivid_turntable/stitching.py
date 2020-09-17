"""Module for combining into a single point cloud"""

from typing import List
import open3d as o3d
import numpy as np


def stitch(point_clouds: List[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    """Combine point clouds into one (must be already transformed into the same frame)

    Arguments:
        point_clouds:   List of Open3D point clouds
    Returns:
        A new combined Open3D point cloud
    """
    xyz_arrays = []
    rgb_arrays = []

    for pcd in point_clouds:
        xyz_arrays.append(np.asarray(pcd.points))
        rgb_arrays.append(np.asarray(pcd.colors))

    xyz = np.vstack(xyz_arrays)
    rgb = np.vstack(rgb_arrays)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    return pcd
