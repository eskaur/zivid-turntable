"""Module for finding transforms between captures"""

from typing import Dict, Tuple, List

import numpy as np
import zivid

from .featurepoints import ArucoMarker, find_aruco_markers


def plane_fit_svd(points: np.ndarray) -> Tuple[np.ndarray, float, np.ndarray]:
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


def dist_from_plane(
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


def _get_common_feature_points(
    marker_set_a: Dict[int, ArucoMarker], marker_set_b: Dict[int, ArucoMarker]
) -> Tuple[np.ndarray, np.ndarray]:
    """Get the common feature points based on two sets of Aruco markers

    Arguments:
        marker_set_a:   First set of detected Aruco markers
        marker_set_b:   Second set of detected Aruco markers
    Returns:
        Array (nx3) of 3D feature points in frame A
        Array (nx3) of 3D feature points in frame B
    """
    common_ids = set.intersection(set(marker_set_a.keys()), set(marker_set_b.keys()))
    features_a = []
    features_b = []
    for idnum in common_ids:
        features_a.append(marker_set_a[idnum].center3d)
        features_b.append(marker_set_b[idnum].center3d)
    return np.array(features_a), np.array(features_b)


def _get_transform(
    marker_set_a: Dict[int, ArucoMarker], marker_set_b: Dict[int, ArucoMarker]
) -> np.ndarray:
    """Get transform that would make feature points B overlap with feature points B

     Arguments:
        marker_set_a:   First set of detected Aruco markers
        marker_set_b:   Second set of detected Aruco markers
    Returns:
        Array (4x4) representing an affine transformation
    """

    features_a, features_b = _get_common_feature_points(marker_set_a, marker_set_b)
    detection_result_a = zivid.calibration.custom_feature_points(features_a)
    detection_result_b = zivid.calibration.custom_feature_points(features_b)
    calib_res = zivid.calibration.calibrate_multi_camera(
        [detection_result_a, detection_result_b]
    )
    return calib_res.transforms()[-1]


def _get_base_transform(marker_set: Dict[int, ArucoMarker]) -> np.ndarray:
    """Get transform to centerpoint of markers

    Arguments:
        marker_set:     Set of detected Aruco markers
    Returns:
        Array (4x4) giving the pose of the Aruco base in the camera frame
    """
    floor_points = np.array([val.center3d for val in marker_set.values()])
    unit_normal, _, point_in_plane = plane_fit_svd(floor_points)

    transform = np.eye(4)
    transform[0:3, -1] = point_in_plane
    wvec = -unit_normal
    uvec = np.array([1.0, 0.0, 0.0])
    uvec = uvec - wvec * np.dot(uvec, wvec)
    vvec = np.cross(wvec, uvec)
    vvec = vvec / np.linalg.norm(vvec)
    transform[0:3, 0] = uvec
    transform[0:3, 1] = vvec
    transform[0:3, 2] = wvec
    return transform


def transform_frames(frames: List[zivid.Frame]) -> None:
    """In-place transform all point clouds to the coordinate system of the first frame

    This is dependent on several Aruco markers being visible in all frames.

    Arguments:
        List of Zivid frames with visible Aruco markers
    """

    # Find and identify all Aruco markers
    marker_sets = [find_aruco_markers(frame.point_cloud()) for frame in frames]

    print("Detected Aruco marker sets:")
    for i, marker_set in enumerate(marker_sets):
        print(f"Frame {i}:")
        for key, val in marker_set.items():
            print(f"{key}: {val.center3d}")

    # Check quality of marker sets
    for i, marker_set in enumerate(marker_sets):
        n_markers = len(marker_set.keys())
        min_markers = 4
        if n_markers < min_markers:
            raise RuntimeError(
                f"Frame {i} contains only {n_markers} well resolved markers. "
                f"At least {min_markers} is required."
            )

    # Get transform for transforming to base-plate
    base_transform = np.linalg.inv(_get_base_transform(marker_sets[0]))

    # Use markers to transform all frames into the coordinate system of the base
    print("-" * 70)
    print("Calculating transforms...")

    transform_to_maincam = np.eye(4)
    # for i in range(1, len(frames)):
    for i, frame in enumerate(frames):

        if i > 0:
            transform_to_prev = _get_transform(marker_sets[i - 1], marker_sets[i])
            transform_to_maincam = transform_to_maincam.dot(transform_to_prev)

        transform = np.dot(base_transform, transform_to_maincam)

        # transform_current = _get_transform(marker_sets[i - 1], marker_sets[i])
        # transform_to_maincam = transform_to_maincam.dot(transform_current)
        # transform_to_base = np.dot(base_transform, transform_to_maincam)
        print(f"Applying transform to frame {i}:")
        print(transform)
        frame.point_cloud().transform(transform)
