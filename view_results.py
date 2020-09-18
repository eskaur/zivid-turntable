"""Script for processing turntable data"""

import argparse
from pathlib import Path

import open3d as o3d
from zivid_turntable.io import make_casedir_name


def _main() -> None:

    # Get args
    parser = argparse.ArgumentParser(description="View processing results")
    parser.add_argument("label", type=str, help="label for dataset")
    args = parser.parse_args()
    print(args)

    # Load results from directory
    label = args.label
    datadir = Path(".") / make_casedir_name(label)
    print(f"Loading results from {datadir}")

    pcd = o3d.io.read_point_cloud(str(datadir / "post_downsample.ply"))

    o3d.visualization.draw_geometries([pcd])


if __name__ == "__main__":
    _main()
