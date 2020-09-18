"""Module for processing utilities"""

import numpy as np
import open3d as o3d
import zivid


def frame_to_open3d_pointcloud(frame: zivid.Frame) -> o3d.geometry.PointCloud:
    """Convert Zivid frame to Open3D point cloud

    Arguments:
        frame:  A Zivid frame
    Returns:
        An Open3D point cloud
    """

    xyz = frame.point_cloud().copy_data("xyz").reshape(-1, 3)
    rgb = frame.point_cloud().copy_data("rgba")[:, :, 0:3].reshape(-1, 3) / 255.0
    nanmask = np.isnan(xyz[:, 2])

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz[~nanmask])
    pcd.colors = o3d.utility.Vector3dVector(rgb[~nanmask])
    return pcd


def remove_divergent_normals(
    pcd: o3d.geometry.PointCloud, threshold: float
) -> o3d.geometry.PointCloud:
    """Remove points whose normals are not pointing towards the camera (z-axis)

    Arguments:
        pcd:        The point cloud to filter
        threshold:  The removal threshold for the z-component of the normal
    Returns:
        A new filtered point cloud
    """

    pcd.estimate_normals()
    normals_z = np.asarray(pcd.normals)[:, 2]

    mask = normals_z > threshold
    xyz_out = np.asarray(pcd.points)[mask]
    rgb_out = np.asarray(pcd.colors)[mask]

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(xyz_out)
    pcd_out.colors = o3d.utility.Vector3dVector(rgb_out)
    return pcd_out


def adjust_colors_from_normals(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Adjust colors based on inverting Lambert's cosine law

    Arguments:
        pcd:        The point clout to adjust
    Returns:
        New adjusted point cloud
    """
    pcd.estimate_normals()
    xyz = np.asarray(pcd.points)
    rgb = np.asarray(pcd.colors)

    normals_z = np.abs(np.asarray(pcd.normals)[:, 2])
    nz_cols = np.column_stack((normals_z, normals_z, normals_z))
    rgb = np.clip(rgb / nz_cols, 0.0, 1.0)

    pcd_out = o3d.geometry.PointCloud()
    pcd_out.points = o3d.utility.Vector3dVector(xyz)
    pcd_out.colors = o3d.utility.Vector3dVector(rgb)
    return pcd_out


def clean_outlier_blobs(pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Remove points that are not part of the main structure

    Arguments:
        pcd:    The point cloud to filter
    Returns:
        A new filtered point cloud
    """
    pcd, _ = pcd.remove_radius_outlier(nb_points=100, radius=5)
    return pcd


def remove_by_z_threshold(
    pcd: o3d.geometry.PointCloud, z_threshold: float
) -> o3d.geometry.PointCloud:
    """Remove points that have z-value below some threshold

    Arguments:
        pcd:            The point cloud to filter
        z_threshold:    Threshold for z value
    Returns:
        A new filtered point cloud
    """
    xyz = np.asarray(pcd.points)
    rgb = np.asarray(pcd.colors)

    floor_mask = xyz[:, 2] > z_threshold
    xyz = xyz[floor_mask]
    rgb = rgb[floor_mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    pcd.colors = o3d.utility.Vector3dVector(rgb)
    return pcd
