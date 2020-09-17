"""Module for detecting feature points"""

from dataclasses import dataclass
from typing import Dict

import cv2
import numpy as np
import zivid


@dataclass
class ArucoMarker:
    """Class for representing the properties of a detected marker"""

    idnum: int
    center2d: np.ndarray
    center3d: np.ndarray


def _corner2d_to_center2d(corner2d: np.ndarray) -> np.ndarray:
    """Get 2D center point from four 2D corner points

    Arguments:
        corner2d: Array (4x2) of 2D corner points
    Returns:
        Array (2) of the 2D center point
    """
    point0 = corner2d[0, :]
    point1 = corner2d[1, :]
    point2 = corner2d[2, :]
    point3 = corner2d[3, :]
    slope1 = (point3[1] - point1[1]) / (point3[0] - point1[0])
    intersect1 = point1[1] - slope1 * point1[0]
    slope2 = (point0[1] - point2[1]) / (point0[0] - point2[0])
    intersect2 = point2[1] - slope2 * point2[0]
    x_center = (intersect2 - intersect1) / (slope1 - slope2)
    y_center = slope1 * x_center + intersect1
    return np.array([x_center, y_center])


def _point2d_to_point3d(point_cloud: zivid.PointCloud, point2d: np.ndarray) -> str:
    """Get 3D point corresponding to 2D point

    Arguments:
        point_cloud:    A Zivid point cloud
        point2d:        Array (2) specifying a 2D point coordinate
    Returns
        Array (3) specifying a 3D point coordinate
    """
    idx = np.round(point2d).astype(np.int)
    xyz = point_cloud.copy_data("xyz")
    return xyz[idx[1], idx[0], :]


def find_aruco_markers(point_cloud: zivid.PointCloud) -> Dict[int, ArucoMarker]:
    """Find Aruco markers in point cloud

    Arguments:
        point_cloud:    A Zivid point cloud
    Returns:
        Dictionary of {id: ArucoMarker}
    """

    # Use OpenCV to find Aruco markers
    rgba = point_cloud.copy_data("rgba")
    grayscale = cv2.cvtColor(rgba, cv2.COLOR_RGB2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
    res = cv2.aruco.detectMarkers(grayscale, aruco_dict)
    coords = res[0]
    idnums = res[1].flatten()

    # Construct an ArucoMarker object for each detected marker
    markers = {}
    for coord, idnum in zip(coords, idnums):
        try:
            corners = coord[0, :, :]
            center2d = _corner2d_to_center2d(corners)
            center3d = _point2d_to_point3d(point_cloud, center2d)
            if not np.any(np.isnan(center3d)):
                markers[idnum] = ArucoMarker(
                    idnum=idnum, center2d=center2d, center3d=center3d
                )
        except Exception as ex:  # pylint: disable=broad-except
            print(f"Skipping marker due to exception: {str(ex)}")

    return markers
