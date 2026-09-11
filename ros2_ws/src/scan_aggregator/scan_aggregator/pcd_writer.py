"""Directory-entry durability helper.

Used to be a PCD (Point Cloud Data) writer too -- every scan is now
saved natively as E57 instead (see e57_writer.py and HANDOFF.md), so
the PCD-writing half of this module was deleted outright once genuinely
unused (confirmed zero remaining callers anywhere in this workspace).
e57_writer.py's own read_pcd_points still reads existing .pcd files, for
converting ones that already existed on disk before that change -- this
module never wrote what that one reads, historically or otherwise, they
just happen to be the two halves of the same format.
"""

from __future__ import annotations

import os


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
