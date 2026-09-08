"""Automatic roll/pitch mount calibration -- the live, one-button GUI
counterpart to scripts/calibration/calibrate_mount_angle.py.

No rclpy import here on purpose (numpy only) -- this module is pure math,
independently testable without ROS2, and reusable if anything else in this
package ever needs the same forward model. The core forward model
(euler_zyx_matrix/transform_to_base_link/compass_search/plane_fit_rms) is
duplicated from scripts/calibration/calibrate_mount_angle.py rather than
imported -- that tool is a standalone, no-ROS2-dependency script (copyable
and runnable on its own), and this package has no build-time coupling to
scripts/ any more than to any other ROS2 package (see e.g. this package's
own SETTINGS_PATH helpers, duplicated rather than imported from
tilt_axis_bridge for the identical reason). Already proven correct via
that tool's own synthetic verification earlier this session -- not
re-derived here, just carried over.

mount_yaw_deg is not part of this module at all: a yaw error is
mathematically indistinguishable from a rigid rotation of the whole scan
about the tilt axis (confirmed algebraically, see calibrate_mount_angle.py's
own module docstring for the full proof), so it can never be recovered
from scan geometry alone, automatically-found plane or not.
"""

from __future__ import annotations

import numpy as np


def compass_search(
    objective,
    x0: np.ndarray,
    step: np.ndarray,
    min_step: float = 1e-5,
    max_rounds: int = 200,
) -> tuple[np.ndarray, float]:
    """Small dependency-free derivative-free optimizer -- see
    calibrate_mount_angle.py's own copy of this function for the full
    rationale (identical implementation)."""
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
    convention -- see calibrate_mount_angle.py's own copy for the full
    rationale (identical implementation)."""
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
    """velodyne -> tilt_link (static mount offset) -> base_link -- see
    calibrate_mount_angle.py's own copy for the full rationale (identical
    implementation, same composition order as the live tf2 chain)."""
    r_mount = euler_zyx_matrix(*mount_rpy_rad)
    p_tilt_link = points_velodyne @ r_mount.T + mount_xyz
    c, s = np.cos(tilt_rad), np.sin(tilt_rad)
    x = p_tilt_link[:, 0] * c - p_tilt_link[:, 1] * s
    y = p_tilt_link[:, 0] * s + p_tilt_link[:, 1] * c
    z = p_tilt_link[:, 2]
    return np.stack([x, y, z], axis=1)


def plane_fit_rms(points: np.ndarray) -> float:
    """RMS perpendicular distance to the best-fit plane (via SVD of the
    centered points) -- see calibrate_mount_angle.py's own copy for the
    full rationale (identical implementation)."""
    if points.shape[0] < 10:
        return float("inf")
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    residuals = centered @ normal
    return float(np.sqrt(np.mean(residuals**2)))


def _plane_inlier_mask(points: np.ndarray, sample_idx, threshold: float) -> np.ndarray:
    """Perpendicular-distance inlier mask for the plane through the 3
    points at sample_idx -- the per-iteration RANSAC scoring step."""
    p0, p1, p2 = points[sample_idx]
    normal = np.cross(p1 - p0, p2 - p0)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        # Degenerate (near-collinear) sample -- can't define a plane,
        # reject outright rather than dividing by ~0 into a bogus normal.
        return np.zeros(points.shape[0], dtype=bool)
    normal = normal / norm
    distances = np.abs((points - p0) @ normal)
    return distances <= threshold


def _refine_inliers(pts: np.ndarray, mask: np.ndarray, threshold: float, rounds: int = 3) -> np.ndarray:
    """RANSAC cleanup step 1 of 2: a raw 3-point sample's inlier count is a
    good way to *find* a plane but a noisy way to *define* one -- an
    imprecise 3-point sample can wrongly exclude real inliers or admit
    stray ones. Refit via SVD (plane_fit_rms's own centroid+normal
    calculation) on the current inlier set, then re-threshold *every*
    point in `pts` against that refined, least-squares plane instead of
    the original 3-point one. Note this alone does NOT remove points that
    are genuinely, coincidentally within `threshold` of the *true*
    plane (confirmed via this module's own synthetic test: refitting from
    hundreds of points barely moves the plane when contamination is only
    ~0.4%, so the same handful of coincidentally-nearby points pass every
    round) -- that's what _trim_statistical_outliers below is for,
    applied once after this has converged on a stable plane."""
    for _ in range(rounds):
        if mask.sum() < 3:
            return mask
        inliers = pts[mask]
        centroid = inliers.mean(axis=0)
        _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
        normal = vt[-1]
        distances = np.abs((pts - centroid) @ normal)
        new_mask = distances <= threshold
        if new_mask.sum() == mask.sum() and np.array_equal(new_mask, mask):
            break
        mask = new_mask
    return mask


def _trim_statistical_outliers(
    pts: np.ndarray, mask: np.ndarray, mad_multiplier: float = 5.0, floor: float = 0.001
) -> np.ndarray:
    """RANSAC cleanup step 2 of 2: drop points whose distance from the
    inlier set's own best-fit plane is a statistical outlier *relative to
    that set's own noise floor*, rather than relative to a fixed absolute
    tolerance. A fixed inlier_threshold has to be generous enough for real
    sensor noise on an actual wall, which is exactly generous enough for a
    few unrelated points to coincidentally land inside it too (confirmed
    via this module's own synthetic test -- see _refine_inliers) --
    median absolute deviation (MAD, robust to the very outliers it's
    trying to detect, unlike a plain standard deviation) adapts to
    whatever the real noise floor of THIS inlier set actually is: a clean
    synthetic plane's MAD is ~0, so `floor` (not the MAD term) sets the
    cutoff there; a real, noisy wall's MAD reflects its actual sensor
    noise, and 5x that comfortably keeps every genuine point while still
    rejecting anything sitting well outside the set's own scatter."""
    if mask.sum() < 10:
        return mask
    inliers = pts[mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vt[-1]
    distances = np.abs((pts - centroid) @ normal)
    mad = float(np.median(np.abs(distances[mask] - np.median(distances[mask]))))
    cutoff = max(mad_multiplier * mad, floor)
    return mask & (distances <= cutoff)


def find_best_plane(
    points: np.ndarray,
    tilt_rad: np.ndarray,
    iterations: int = 500,
    inlier_threshold: float = 0.03,
    min_inliers: int = 200,
    rounds: int = 4,
    rng_seed: int | None = None,
    max_search_points: int = 20000,
) -> np.ndarray | None:
    """Hand-rolled RANSAC plane finder (no scipy -- same house style as
    compass_search above): repeatedly sample 3 random points, fit the
    plane through them, count inliers within inlier_threshold, keep
    whichever sample had the most. Runs `rounds` independent searches,
    each over the points not already claimed by a previous round's best
    plane, to surface multiple candidate flat surfaces (e.g. a wall and a
    floor might both be visible) rather than just the single largest one.

    The inner search loop (iterations x an O(n) inlier count each) only
    runs against a bounded random subsample (max_search_points), not the
    full cloud -- confirmed necessary against a real capture on this rig's
    actual hardware: a real ~40s sweep at the VLP-16's own point rate is
    millions of points, and iterations x millions of points x rounds this
    was originally written to just run over directly took minutes on the
    Pi's CPU (found live, not a theoretical concern). A random subsample
    is exactly as representative for *finding* the dominant plane's
    parameters as the full cloud would be; only the *final* inlier
    extraction for that round (a single O(n) pass, not iterations of
    them) runs against every point actually remaining, so no completeness
    is lost in what calibrate_roll_pitch ultimately optimizes against.

    Returns the indices (into the original `points`/`tilt_rad` arrays) of
    whichever candidate's inliers span the WIDEST range of tilt_rad among
    those clearing min_inliers -- not simply the plane with the most
    points. A flat surface seen across many different tilt readings is
    what actually constrains roll/pitch (a wrong mount angle distorts
    each tilt reading's contribution differently -- see this project's
    calibrate_mount_angle.py docstring and HANDOFF.md for the full
    reasoning); a huge but narrow-tilt-range plane (e.g. the floor seen at
    one single tilt stop) gives much weaker leverage than a smaller one
    spanning most of the sweep. Returns None if nothing clears
    min_inliers at all.
    """
    rng = np.random.default_rng(rng_seed)
    remaining = np.arange(points.shape[0])
    candidates: list[np.ndarray] = []

    for _ in range(rounds):
        if remaining.shape[0] < min_inliers:
            break
        pts = points[remaining]

        if pts.shape[0] > max_search_points:
            search_idx = rng.choice(pts.shape[0], size=max_search_points, replace=False)
        else:
            search_idx = np.arange(pts.shape[0])
        search_pts = pts[search_idx]

        best_mask = None
        best_count = 0
        for _ in range(iterations):
            sample = rng.choice(search_pts.shape[0], size=3, replace=False)
            mask = _plane_inlier_mask(search_pts, sample, inlier_threshold)
            count = int(mask.sum())
            if count > best_count:
                best_count, best_mask = count, mask
        if best_mask is None or best_count < min_inliers:
            break

        # Refit from the subsample's own winning inliers, then apply that
        # plane to every point actually remaining this round (once, not
        # iterations of times) -- this is the one full-cloud pass per
        # round, unavoidable and cheap by comparison to the search above.
        sub_inliers = search_pts[best_mask]
        centroid = sub_inliers.mean(axis=0)
        _, _, vt = np.linalg.svd(sub_inliers - centroid, full_matrices=False)
        normal = vt[-1]
        full_mask = np.abs((pts - centroid) @ normal) <= inlier_threshold

        full_mask = _refine_inliers(pts, full_mask, inlier_threshold)
        full_mask = _trim_statistical_outliers(pts, full_mask)
        if int(full_mask.sum()) < min_inliers:
            break
        inlier_idx = remaining[full_mask]
        candidates.append(inlier_idx)
        remaining = remaining[~full_mask]

    if not candidates:
        return None

    def tilt_span(idx: np.ndarray) -> float:
        t = tilt_rad[idx]
        return float(t.max() - t.min())

    return max(candidates, key=tilt_span)


def calibrate_roll_pitch(
    points_velodyne: np.ndarray,
    tilt_rad: np.ndarray,
    initial_roll_deg: float,
    initial_pitch_deg: float,
    mount_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    min_inliers: int = 200,
    min_tilt_range_deg: float = 20.0,
    refit_rounds: int = 3,
) -> dict:
    """The single top-level entry point: given a raw capture (still in
    velodyne frame, tagged with the tilt reading at capture time -- same
    shape as scripts/calibration/capture_raw_for_mount_calibration.py
    produces) and the mount's current roll/pitch, automatically finds a
    suitable flat surface and solves for the true roll/pitch. No ROI, no
    markers -- see this module's own docstring for why yaw isn't part of
    this at all.

    Re-segments and re-fits `refit_rounds` times rather than once: while
    the current roll/pitch estimate is still off, a real wall reconstructs
    as visibly bent (see calibrate_mount_angle.py's docstring on the
    "skewed image" symptom), which inflates find_best_plane's own outlier
    statistics enough to let a few unrelated points slip through as
    inliers (confirmed via this module's own synthetic test -- a single
    pass left measurable residual error from a handful of stray points a
    fixed/statistical threshold alone couldn't fully separate from a
    still-bent wall). Each round's improved estimate makes the wall
    reconstruct flatter, which tightens that same statistical threshold
    and squeezes out more of the contamination -- an iterative
    segment-then-fit loop, not a single fixed-threshold pass.
    """
    mount_xyz_arr = np.asarray(mount_xyz, dtype=np.float64)
    roll_deg, pitch_deg = initial_roll_deg, initial_pitch_deg
    before_rms = None
    plane_idx = None
    tilt_range_deg = 0.0

    for _ in range(refit_rounds):
        rpy = np.radians([roll_deg, pitch_deg, 0.0])
        base_link = transform_to_base_link(points_velodyne, tilt_rad, mount_xyz_arr, rpy)

        plane_idx = find_best_plane(base_link, tilt_rad, min_inliers=min_inliers)
        if plane_idx is None:
            return {
                "success": False,
                "error": "no flat surface with enough points found -- point the "
                "scan at a real wall/flat surface and try again",
            }

        tilt_range_deg = float(np.degrees(tilt_rad[plane_idx].max() - tilt_rad[plane_idx].min()))
        if tilt_range_deg < min_tilt_range_deg:
            return {
                "success": False,
                "error": f"best flat surface found only spans {tilt_range_deg:.1f} deg of "
                f"tilt (need >= {min_tilt_range_deg:.0f}) -- too little leverage on "
                "roll/pitch; use a wider sweep range or a surface visible across "
                "more of it",
            }

        # compass_search calls objective() potentially hundreds of times --
        # capped here (a real wall inlier set can be hundreds of thousands
        # of points on this rig's own real captures) so each call stays
        # fast without meaningfully changing the fit: a random subsample
        # this size is already a very well-determined plane fit, and
        # n_points in the final result below still reports the true,
        # complete inlier count, not this capped figure.
        objective_idx = plane_idx
        if plane_idx.shape[0] > 20000:
            objective_idx = np.random.default_rng(0).choice(plane_idx, size=20000, replace=False)
        plane_points_velodyne = points_velodyne[objective_idx]
        plane_tilt_rad = tilt_rad[objective_idx]

        def objective(x: np.ndarray) -> float:
            rpy = np.radians([x[0], x[1], 0.0])
            base = transform_to_base_link(plane_points_velodyne, plane_tilt_rad, mount_xyz_arr, rpy)
            return plane_fit_rms(base)

        x0 = np.array([roll_deg, pitch_deg])
        round_rms = objective(x0)
        if before_rms is None:
            before_rms = round_rms
        best_x, after_rms = compass_search(objective, x0, step=np.full(2, 2.0))
        roll_deg, pitch_deg = float(best_x[0]), float(best_x[1])

    return {
        "success": True,
        "error": None,
        "roll_deg": roll_deg,
        "pitch_deg": pitch_deg,
        "before_rms_mm": before_rms * 1000.0,
        "after_rms_mm": after_rms * 1000.0,
        "n_points": int(plane_idx.shape[0]),
        "tilt_range_deg": tilt_range_deg,
    }
