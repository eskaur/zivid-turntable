"""Module for input/output"""

from pathlib import Path


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
