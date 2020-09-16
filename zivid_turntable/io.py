"""Module for input/output"""

import struct
from pathlib import Path
import numpy as np


def _make_casedir_name(label: str) -> str:
    """Create case directory name based on a label

    Arguments:
        label   Label for case directory
    Returns:
        Name of case directory
    """
    return f"data_{label}"


def create_casedir(label: str, workdir: Path) -> Path:
    """Create case directory based on label

    Arguments:
        label:      Label to use for directory
        workdir:    Root directory to create case directory in
    Returns:
        Path to newly created case directory
    """
    dirpath = workdir / _make_casedir_name(label)
    if dirpath.is_dir():
        raise RuntimeError(f"Directory already exists: {dirpath}")
    dirpath.mkdir()
    print(f"Created case directory: {dirpath}")
    return dirpath


def write_ply_binary(fname: str, pts: np.ndarray) -> None:
    """Write XYZRGB array as PLY point cloud

    Arguments
        fname:  File name
        pts:    Array (nx6) giving a colored point cloud
    """

    with open(fname, "wb") as file_pointer:
        line = "ply\n"
        file_pointer.write(line.encode("utf-8"))
        line = "format binary_little_endian 1.0\n"
        file_pointer.write(line.encode("utf-8"))
        line = "element vertex %d\n"
        file_pointer.write(line.encode("utf-8") % pts.shape[0])
        line = "property float x\n"
        file_pointer.write(line.encode("utf-8"))
        line = "property float y\n"
        file_pointer.write(line.encode("utf-8"))
        line = "property float z\n"
        file_pointer.write(line.encode("utf-8"))
        line = "property uchar red\n"
        file_pointer.write(line.encode("utf-8"))
        line = "property uchar green\n"
        file_pointer.write(line.encode("utf-8"))
        line = "property uchar blue\n"
        file_pointer.write(line.encode("utf-8"))
        line = "end_header\n"
        file_pointer.write(line.encode("utf-8"))

        for i in range(len(pts)):

            data = struct.pack(
                "<fffBBB",
                pts[i, 0],
                pts[i, 1],
                pts[i, 2],
                np.uint8(pts[i, 3]),
                np.uint8(pts[i, 4]),
                np.uint8(pts[i, 5]),
            )

            file_pointer.write(data)
