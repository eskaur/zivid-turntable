"""Module for combining into a single point cloud"""

from typing import Tuple, List

import numpy as np
import zivid

from .featurepoints import find_aruco_markers


def _plane_fit_svd(points: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
    """Plane fit with Singular Value Decomposition.

    Arguments:
        points: Array (nx3) of 3D points that lie in a plane
    Returns:
        Array (3) giving unit normal to plane
        Distance from origin to plane
        Array(3) giving a point in the plane
    """
    point_in_plane = points.mean(axis=0)
    xvecs = points - point_in_plane[np.newaxis, :]
    mat = np.dot(xvecs.T, xvecs)
    normal = np.linalg.svd(mat)[0][:, -1]
    unit_normal = normal / np.linalg.norm(normal)
    distance_to_origin = np.dot(point_in_plane, unit_normal)

    return (unit_normal, distance_to_origin, point_in_plane)


def _dist_from_plane(
    points: np.ndarray, unit_normal: np.ndarray, distance_to_origin: float
) -> np.ndarray:
    """Get distance from plane for each given point

    Arguments:
        points:             Array (nx3) of points to get plane-distance from
        unit_normal:        Array (3) giving unit normal of plane
        distance_to_origin: Array (3) giving distance from origin to plane
    Returns
        Array (n) giving signed distance between plane and each point
    """
    return np.matmul(points, unit_normal) - distance_to_origin


def combine(frames: List[zivid.Frame], floor_tolerance: float) -> np.ndarray:
    """Combine frames into a single colored point cloud with floor removed

    Arguments:
        frames:             List of Zivid frames (already transformed)
        floor_tolerance:    Will keep points more than this distance from floor
    Returns:
        Array (nx6) of XYZRGB points
    """

    # Find floor plane based on frame-0
    floor_points = [
        val.center3d for val in find_aruco_markers(frames[0].point_cloud()).values()
    ]
    unit_normal, distance_to_origin, _ = _plane_fit_svd(np.array(floor_points))

    point_clouds = []
    for i, frame in enumerate(frames):
        print(f"Processing frame: {i}")
        xyz = frame.point_cloud().copy_data("xyz")
        rgb = frame.point_cloud().copy_data("rgba")[:, :, 0:3]
        point_cloud = np.dstack([xyz, rgb])
        # Remove NANs
        point_cloud = point_cloud[~np.isnan(point_cloud[:, :, 2])]
        # Flatten
        point_cloud = point_cloud.reshape(-1, 6)
        # Remove floor
        floor_distance = _dist_from_plane(
            point_cloud[:, 0:3], unit_normal, distance_to_origin
        )
        point_cloud = point_cloud[floor_distance < -floor_tolerance]
        # Append to collection
        point_clouds.append(point_cloud)

    return np.vstack(point_clouds)
