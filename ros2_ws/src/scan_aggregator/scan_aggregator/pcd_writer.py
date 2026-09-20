"""Binary PCD writer and directory-entry durability helper.

Scans are saved as binary PCD (switched back from E57 on 2026-09-20 --
the E57 writer was far too slow; see HANDOFF.md). e57_writer.read_pcd_points
reads these files back, for the legacy .pcd -> .e57 conversion path.
"""

from __future__ import annotations

import os

import numpy as np

NL = chr(10)


def write_pcd(path, points, field_names, progress_cb=None):
    """Writes an Nx(len(field_names)) float32 array as a binary PCD v0.7
    file: a small text header, then the array's raw bytes -- no per-point
    encoding work, so save time is bounded by disk write speed, not
    Python. Streams in ~8MB chunks (progress_cb gets a real 0.0-1.0
    fraction after each) rather than one giant write, and fsyncs the file
    itself; the caller still needs fsync_durable for the directory entry.
    Layout matches what e57_writer.read_pcd_points reads back (every
    field SIZE 4/TYPE F/COUNT 1)."""
    points = np.ascontiguousarray(points, dtype=np.float32)
    n = points.shape[0]
    k = len(field_names)
    header = "".join([
        "# .PCD v0.7 - Point Cloud Data file format", NL,
        "VERSION 0.7", NL,
        "FIELDS " + " ".join(field_names), NL,
        "SIZE " + " ".join(["4"] * k), NL,
        "TYPE " + " ".join(["F"] * k), NL,
        "COUNT " + " ".join(["1"] * k), NL,
        f"WIDTH {n}", NL,
        "HEIGHT 1", NL,
        "VIEWPOINT 0 0 0 1 0 0 0", NL,
        f"POINTS {n}", NL,
        "DATA binary", NL,
    ]).encode("ascii")
    rows_per_chunk = max(1, (8 * 1024 * 1024) // (4 * k))
    with open(path, "wb") as f:
        f.write(header)
        for start in range(0, n, rows_per_chunk):
            f.write(points[start:start + rows_per_chunk].tobytes())
            if progress_cb is not None:
                progress_cb(min(1.0, (start + rows_per_chunk) / n))
        f.flush()
        os.fsync(f.fileno())


def fsync_durable(path: str) -> None:
    """Fsync a just-written file's containing directory entry.

    A file's own fsync (already done by the caller before this) only
    guarantees its *contents* are durable -- it says nothing about the
    *directory entry* that makes the filename findable at all. That's
    separate metadata, on most filesystems (including exFAT), and needs its
    own fsync on the containing directory's own file descriptor to be
    guaranteed durable too (confirmed this gap is real, not theoretical --
    see this project's own history with this exact class of bug, in
    HANDOFF.md). Shared by every code path in this project that writes a
    file onto potentially slow/removable media: the E57 writer, the
    USB-export copy path, and the project-bundle zip writer among them.
    """
    dir_fd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)
