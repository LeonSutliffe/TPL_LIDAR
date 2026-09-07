"""Offline mount-angle calibration for the tilt_link -> velodyne static
transform (mount_roll_deg/mount_pitch_deg/mount_yaw_deg -- see
vlp16_config/node.py's _publish_mount_transform for the live equivalent
of the math replicated here).

No ROS2 dependency -- just numpy (not even scipy: the optimizer below is
a small hand-rolled compass/pattern search, in keeping with this
project's existing preference for a short hand-rolled implementation
over a dependency for one thing -- see pcd_writer.py's own docstring).
Run this anywhere (does NOT need to be on the Pi): copy the .npz
produced by capture_raw_for_mount_calibration.py over and run it there
instead if that's more convenient.

Why this exists: a real double-image artifact was traced to the physical
mount not sitting at exactly the assumed 45-degree angle between the
VLP-16's own spin axis and the external tilt axis (see HANDOFF.md). The
error is invisible in single-coverage regions and only becomes visible
where a scan's range exceeds ~150 degrees (VLP-16's own 30-degree
vertical FOV), so the same physical geometry gets covered twice via two
different (spin azimuth, tilt angle) combinations that should coincide
but don't. That overlap is exactly the data needed to solve for the true
mount angle: for the *correct* angle, a real flat surface visible in the
overlap should reconstruct as a single sharp plane; for the *wrong*
(currently-assumed) angle, it reconstructs as two offset copies -- i.e.
a thick/doubled band around the true plane. This searches over candidate
mount_roll_deg/mount_pitch_deg to find whichever one makes a
user-specified region (--roi, a flat surface caught in the overlap)
reconstruct as tightly as possible against its own best-fit plane
(minimum RMS perpendicular distance).

mount_yaw_deg is deliberately EXCLUDED from the search, not just left
out for simplicity -- it is mathematically unobservable this way, a real
finding confirmed both algebraically and with a synthetic test before
settling on this design, not a limitation of this particular
implementation. The tilt joint only ever rotates about base_link's Z
axis (scanner.urdf.xacro's tilt_axis_xyz="0 0 1"), and yaw is applied
*before* that rotation in the same composition -- so
Rz(tilt) @ Rz(yaw_error) == Rz(tilt + yaw_error) for every single point,
meaning any yaw error is exactly equivalent to adding a constant to
every point's effective tilt angle, i.e. a single rigid rotation of the
*entire* output about the vertical axis. That preserves every internal
geometric relationship perfectly (flatness of any plane, angles between
planes, all of it) -- there is no self-consistency check, however many
non-parallel surfaces you throw at it, that can ever distinguish the
true yaw from a wrong one. This is the exact same limitation as a
magnetometer-free IMU: gravity alone gives you roll/pitch, never
absolute heading. Fortunately this doesn't matter for the reported bug
either way -- a yaw error can never create or explain internal doubling,
only roll/pitch can, and the double-image symptom is fully explained by
those two alone.

Usage (roi in base_link-frame metres, x-min x-max y-min y-max z-min z-max
-- pick a box around a flat wall visible in the overlap/double-image
region, using the CURRENT calibration to eyeball roughly where it is in
whatever viewer you already used to spot the artifact):

    python3 calibrate_mount_angle.py -i mount_calibration_raw.npz \\
        --roi -2 2 -2 2 0 5

Prints the optimized mount_roll_deg/mount_pitch_deg (mount_yaw_deg is
reported back unchanged, see above) and the before/after RMS plane-fit
error so you can judge whether the improvement is real before adopting
the new values.
"""

from __future__ import annotations

import argparse

import numpy as np


def compass_search(
    objective,
    x0: np.ndarray,
    step: np.ndarray,
    min_step: float = 1e-5,
    max_rounds: int = 200,
) -> tuple[np.ndarray, float]:
    """Small dependency-free derivative-free optimizer (a.k.a. pattern
    search): each round, try nudging every parameter +/-step in turn,
    keep whichever single nudge helps most, and only shrink every step
    size (halved) once a full round finds no improving nudge at all.
    Appropriate here because the objective (plane_fit_rms of a batched
    numpy transform) is smooth, cheap to evaluate, and low-dimensional
    (3-6 parameters) -- exactly the regime this class of optimizer
    handles well without needing gradients or a library."""
    x = x0.copy()
    best = objective(x)
    step = step.copy()
    for _ in range(max_rounds):
        improved = False
        for i in range(len(x)):
            for sign in (1.0, -1.0):
                candidate = x.copy()
                candidate[i] += sign * step[i]
                value = objective(candidate)
                if value < best:
                    x, best = candidate, value
                    improved = True
        if not improved:
            step *= 0.5
            if np.all(step < min_step):
                break
    return x, best


def euler_zyx_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Radians in. Matches vlp16_config's _quaternion_from_euler's 'sxyz'
    convention (roll about X, then pitch about Y, then yaw about Z, all
    in the parent/static frame) -- the standard R = Rz @ Ry @ Rx form for
    that convention, not an independent guess."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform_to_base_link(
    points_velodyne: np.ndarray,
    tilt_rad: np.ndarray,
    mount_xyz: np.ndarray,
    mount_rpy_rad: tuple[float, float, float],
) -> np.ndarray:
    """velodyne -> tilt_link (static mount offset) -> base_link (tilt
    joint, a pure rotation about Z per scanner.urdf.xacro's
    tilt_axis_xyz="0 0 1") -- same composition order as the live tf2
    chain, just done here in one batched numpy pass instead of per-cloud
    tf2 lookups."""
    r_mount = euler_zyx_matrix(*mount_rpy_rad)
    p_tilt_link = points_velodyne @ r_mount.T + mount_xyz
    c, s = np.cos(tilt_rad), np.sin(tilt_rad)
    x = p_tilt_link[:, 0] * c - p_tilt_link[:, 1] * s
    y = p_tilt_link[:, 0] * s + p_tilt_link[:, 1] * c
    z = p_tilt_link[:, 2]
    return np.stack([x, y, z], axis=1)


def plane_fit_rms(points: np.ndarray) -> float:
    """RMS perpendicular distance to the best-fit plane (via SVD of the
    centered points) -- the calibration objective. Minimum possible is 0
    (a perfectly flat, single reconstruction); a doubled/offset overlap
    inflates this well above the sensor's own real noise floor."""
    if points.shape[0] < 10:
        return float("inf")
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, s, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    residuals = centered @ normal
    return float(np.sqrt(np.mean(residuals**2)))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-i", "--input", required=True, help="raw capture .npz path")
    parser.add_argument(
        "--roi",
        action="append",
        type=float,
        nargs=6,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="base_link-frame bounding box around a flat surface in the overlap "
        "region, using the CURRENT (pre-calibration) angles below to locate it. "
        "A single ROI is enough -- roll/pitch (the only two parameters actually "
        "searched, see the module docstring on why yaw is excluded) are both "
        "fully recoverable from one flat plane, confirmed via a synthetic test. "
        "Repeat --roi for extra/independent surfaces anyway if convenient (more "
        "data, more robust to sensor noise), just don't expect it to unlock "
        "yaw -- that's not recoverable this way regardless of how many ROIs are "
        "given. Omit entirely to use the whole cloud -- much less reliable, "
        "only for a quick look.",
    )
    parser.add_argument("--mount-x", type=float, default=0.0)
    parser.add_argument("--mount-y", type=float, default=0.0)
    parser.add_argument("--mount-z", type=float, default=0.0)
    parser.add_argument(
        "--initial-roll-deg", type=float, default=90.0, help="current mount_roll_deg"
    )
    parser.add_argument(
        "--initial-pitch-deg", type=float, default=135.0, help="current mount_pitch_deg"
    )
    parser.add_argument(
        "--initial-yaw-deg", type=float, default=0.0, help="current mount_yaw_deg"
    )
    parser.add_argument(
        "--free-translation",
        action="store_true",
        help="also optimize mount_x/y/z (default: keep them fixed at "
        "--mount-x/y/z -- the reported issue is angular, not positional)",
    )
    args = parser.parse_args()

    data = np.load(args.input)
    points_velodyne = data["points_velodyne"]
    tilt_rad = data["tilt_rad"]
    print(f"Loaded {points_velodyne.shape[0]} points from {args.input}")

    mount_xyz = np.array([args.mount_x, args.mount_y, args.mount_z])

    def roi_mask(points: np.ndarray, roi: list[float]) -> np.ndarray:
        xmin, xmax, ymin, ymax, zmin, zmax = roi
        return (
            (points[:, 0] >= xmin)
            & (points[:, 0] <= xmax)
            & (points[:, 1] >= ymin)
            & (points[:, 1] <= ymax)
            & (points[:, 2] >= zmin)
            & (points[:, 2] <= zmax)
        )

    # yaw_deg is fixed at its initial value throughout -- see the module
    # docstring for why searching over it would be meaningless, not just
    # unnecessary. x is [roll_deg, pitch_deg] (+ mount_x/y/z if freed).
    yaw_deg = args.initial_yaw_deg

    def objective(x: np.ndarray) -> float:
        roll_deg, pitch_deg = x[0], x[1]
        xyz = x[2:] if args.free_translation else mount_xyz
        base = transform_to_base_link(
            points_velodyne, tilt_rad, xyz, np.radians([roll_deg, pitch_deg, yaw_deg])
        )
        if not args.roi:
            return plane_fit_rms(base)
        # Sum, not mean, across ROIs -- each one is an independent
        # constraint on the same candidate angle; a mean would let one
        # region's error be diluted by another's, when what's wanted is
        # "every surface must be flat simultaneously."
        total = 0.0
        for roi in args.roi:
            mask = roi_mask(base, roi)
            if mask.sum() < 10:
                return 1e6
            total += plane_fit_rms(base[mask])
        return total

    initial_roll_pitch = np.array([args.initial_roll_deg, args.initial_pitch_deg])
    x0 = (
        np.concatenate([initial_roll_pitch, mount_xyz])
        if args.free_translation
        else initial_roll_pitch
    )
    before = objective(x0)
    print(f"Before calibration: RMS plane-fit error = {before * 1000:.2f} mm "
          f"(roll={args.initial_roll_deg:.3f}, pitch={args.initial_pitch_deg:.3f} deg; "
          f"yaw={yaw_deg:.3f} deg held fixed, not searched -- see module docstring)")
    if not args.roi:
        print("WARNING: no --roi given -- optimizing against the whole cloud, "
              "which likely includes surfaces outside the overlap region and "
              "won't isolate the actual mounting error well. Pass --roi for a "
              "real result.")
    else:
        print(f"Using {len(args.roi)} ROI(s).")

    # Initial step: 2 degrees per angle, 5cm per translation axis (if
    # freed) -- coarse enough to escape the immediate neighborhood of a
    # wrong starting guess, fine enough that a few halvings converge well
    # within the tolerances that matter for a physical mount angle.
    step = np.full(x0.shape, 2.0)
    if args.free_translation:
        step[2:] = 0.05
    best_x, best_value = compass_search(objective, x0, step)

    best_roll_deg, best_pitch_deg = best_x[0], best_x[1]
    best_xyz = best_x[2:] if args.free_translation else mount_xyz

    print()
    print(f"After calibration:  RMS plane-fit error = {best_value * 1000:.2f} mm")
    print(f"  mount_roll_deg  = {best_roll_deg:.4f}  (was {args.initial_roll_deg:.4f})")
    print(f"  mount_pitch_deg = {best_pitch_deg:.4f}  (was {args.initial_pitch_deg:.4f})")
    print(f"  mount_yaw_deg   = {yaw_deg:.4f}  (unchanged -- not searched)")
    if args.free_translation:
        print(f"  mount_x = {best_xyz[0]:.4f}  mount_y = {best_xyz[1]:.4f}  "
              f"mount_z = {best_xyz[2]:.4f}")
    print()
    if before > 0:
        print(f"Improvement: {(1 - best_value / before) * 100:.1f}% reduction in "
              f"RMS plane-fit error.")


if __name__ == "__main__":
    main()
