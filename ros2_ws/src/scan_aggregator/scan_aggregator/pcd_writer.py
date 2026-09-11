"""Minimal binary PCD (Point Cloud Data) writer.

No extra dependency (open3d/pypcd) is installed in the ros2 env, and the
format is simple enough that hand-rolling the ~10-line header is less
fragile than pulling one in just for this.
"""

from __future__ import annotations

import os

import numpy as np


def fsync_durable(path: str) -> None:
    """Fsync a just-written file's containing directory entry.

    A file's own fsync (already done by the caller before this) only
    guarantees its *contents* are durable -- it says nothing about the
    *directory entry* that makes the filename findable at all. That's
    separate metadata, on most filesystems (including exFAT), and needs its
    own fsync on the containing directory's own file descriptor to be
    guaranteed durable too (confirmed this gap is real, not theoretical --
    see write_pcd's and _on_rename_output_request's own history with this
    exact class of bug). Shared by write_pcd and the USB-export copy path,
    which both write files onto the same slow/removable medium.
    """
    dir_fd = os.open(os.path.dirname(path) or ".", os.O_RDONLY)
    try:
        os.fsync(dir_fd)
    finally:
        os.close(dir_fd)


DEFAULT_FIELD_NAMES = ("x", "y", "z", "intensity")


def write_pcd(
    path: str, points: np.ndarray, field_names: tuple[str, ...] = DEFAULT_FIELD_NAMES
) -> None:
    """Write an Nx(len(field_names)) float array as binary PCD, one
    column per name in `field_names`, in order.

    Widened 2026-09-10 from a fixed Nx4 (x, y, z, intensity) signature to
    accept any field list -- node.py now also captures "ring"/"time"
    when the source topic actually carries them (see
    EXTRA_POINT_FIELDS in node.py), for future per-point sweep
    deskewing. Every field is still written as SIZE 4/TYPE F (float32),
    same as before -- ring's real values (0-15) and time's are both
    exactly representable in float32, so reusing one uniform column type
    avoids a mixed-type PCD (a real format complication, matching
    e57_writer.py's own reasoning for making the same
    single-precision-Float-only choice) for no real precision cost.

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
    n = points.shape[0]
    n_fields = len(field_names)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(field_names)}\n"
        f"SIZE {' '.join(['4'] * n_fields)}\n"
        f"TYPE {' '.join(['F'] * n_fields)}\n"
        f"COUNT {' '.join(['1'] * n_fields)}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        np.ascontiguousarray(points, dtype=np.float32).tofile(f)
        # f.close() (via this `with` block) only flushes Python's own
        # buffer into the OS page cache -- it does NOT guarantee the data
        # has actually reached the physical device. For a large write to
        # removable USB storage this matters a lot: without an explicit
        # fsync, the caller (_write_output_in_background) can mark the
        # scan STATE_DONE and the GUI can report "done" while most of the
        # write is still sitting in cache, not yet durable -- a user who
        # reasonably unplugs the drive right after seeing "done" then
        # loses whatever hadn't been flushed yet (confirmed the hard way:
        # a real scan lost ~28% of its points this way, recovered from
        # exFAT's own lost-cluster recovery -- see HANDOFF.md). flush()
        # pushes Python's buffer to the OS; fsync() then blocks until the
        # OS has actually written it through to the device -- only after
        # this returns is "done" true in the sense the GUI implies.
        f.flush()
        os.fsync(f.fileno())
    fsync_durable(path)
