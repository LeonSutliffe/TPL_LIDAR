"""Minimal binary PCD (Point Cloud Data) writer.

No extra dependency (open3d/pypcd) is installed in the ros2 env, and the
format is simple enough that hand-rolling the ~10-line header is less
fragile than pulling one in just for this.
"""

from __future__ import annotations

import numpy as np


def write_pcd(path: str, points_xyzi: np.ndarray) -> None:
    """Write an Nx4 (x, y, z, intensity) float array as binary PCD.

    Binary DATA (one bulk ndarray.tofile() write) rather than ASCII --
    np.savetxt formats every point through Python-level string formatting,
    which dominates total wall-clock time on a real multi-million-point
    merge (a full 180 deg step-and-stare run is tens of millions of
    points). Binary is a single contiguous write, no per-point formatting,
    and ~2.4x smaller on disk (16 bytes/point vs ASCII's ~38) -- also
    cheaper on the WSL2 DrvFs (/mnt/*) I/O path some output_dirs use.
    Points are already float32 by the time they reach here
    (_transform_and_accumulate's rfn.structured_to_unstructured), matching
    PCD's SIZE 4 / TYPE F fields exactly, so no precision is lost versus
    the old ASCII "%.4f"/"%.3f" formatting.
    """
    n = points_xyzi.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        np.ascontiguousarray(points_xyzi, dtype=np.float32).tofile(f)
