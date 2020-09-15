"""Module for finding transforms between captures"""

from typing import Dict, Tuple, List

import numpy as np
import zivid

from .featurepoints import ArucoMarker, find_aruco_markers


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
        print(f"Frame {i}: {list(marker_set.keys())}")

    # Use markers to transform all frames into the coordinate system of the first frame
    print("-" * 70)
    print("Calculating transforms...")
    transform = np.eye(4)
    for i in range(1, len(frames)):
        transform_current = _get_transform(marker_sets[i - 1], marker_sets[i])
        transform = transform.dot(transform_current)
        print(f"Applying transform to frame {i}:")
        print(transform)
        frames[i].point_cloud().transform(transform)
