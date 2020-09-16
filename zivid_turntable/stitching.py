"""Module for combining into a single point cloud"""

from typing import List

import numpy as np
import zivid


def combine(frames: List[zivid.Frame], floor_tolerance: float) -> np.ndarray:
    """Combine frames into a single colored point cloud with floor removed

    Arguments:
        frames:             List of Zivid frames (already transformed)
        floor_tolerance:    Will keep points more than this distance from floor
    Returns:
        Array (nx6) of XYZRGB points
    """

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
        point_cloud = point_cloud[point_cloud[:, 2] > floor_tolerance]

        # Append to collection
        point_clouds.append(point_cloud)

    return np.vstack(point_clouds)
