"""Script for processing turntable data"""

import argparse
from pathlib import Path

import zivid
import open3d as o3d
from zivid_turntable.io import make_casedir_name
from zivid_turntable.calibration import get_transforms
from zivid_turntable.processing import (
    frame_to_open3d_pointcloud,
    remove_divergent_normals,
    remove_by_z_threshold,
    clean_outlier_blobs,
    adjust_colors_from_normals,
)
from zivid_turntable.stitching import stitch


def _main() -> None:

    # Get args
    parser = argparse.ArgumentParser(description="Capture turntable data")
    parser.add_argument("label", type=str, help="label for dataset")
    args = parser.parse_args()
    print(args)

    # Load frames from directory
    label = args.label
    datadir = Path(".") / make_casedir_name(label)
    print(f"Loading frames from {datadir}")
    _ = zivid.Application()
    frames = [zivid.Frame(filepath) for filepath in datadir.glob("*.zdf")]
    print(f"Loaded {len(frames)} frames")

    # Get transforms
    print("Detecting markers and calculating transforms for each frame")
    transforms = get_transforms(frames, equalize_hist=True)

    # Convert to Open3D
    print("Converting to Open3D PointCloud")
    point_cloud_originals = [frame_to_open3d_pointcloud(frame) for frame in frames]

    # Preprocessing
    print("Filtering based on normals")
    point_clouds = [
        remove_divergent_normals(pcd, threshold=0.6) for pcd in point_cloud_originals
    ]

    print("Adjusting colors based on normals")
    point_clouds = [adjust_colors_from_normals(pcd) for pcd in point_clouds]

    # Transform to same coordinate system
    print("Applying transforms")
    for pcd, transform in zip(point_clouds, transforms):
        print(transform)
        pcd.transform(transform)

    # Floor removal
    print("Removing floor from every point cloud")
    point_clouds = [remove_by_z_threshold(pcd, z_threshold=5.0) for pcd in point_clouds]

    # Outlier removal
    print("Removing outliers from every point cloud")
    point_clouds = [clean_outlier_blobs(pcd) for pcd in point_clouds]

    # Stitching
    print("Stitching/combining point clouds")
    pcd = stitch(point_clouds)
    outfile_pre_downsample = datadir / "pre_downsample.ply"
    print(f"Saving to {outfile_pre_downsample}")
    o3d.io.write_point_cloud(str(outfile_pre_downsample), pcd)

    # Downsampling
    print("Downsampling")
    pcd = pcd.voxel_down_sample(voxel_size=0.25)
    pcd.estimate_normals()

    # Save to file
    outfile_post_downsample = datadir / "post_downsample.ply"
    print(f"Saving to {outfile_post_downsample}")
    o3d.io.write_point_cloud(str(outfile_post_downsample), pcd)

    # Visualize
    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    _main()
