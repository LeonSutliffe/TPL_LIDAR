"""Standalone rclpy script -- NOT a colcon package, nothing here needs its
own topics/services (same category as scripts/pi/status_display.py).

Captures raw calibration data for calibrate_mount_angle.py: every point
from /velodyne_points (the RAW cloud, still in the "velodyne" sensor
frame -- deliberately subscribed here rather than any already-transformed
topic, since the whole point is to redo that transform offline under
different candidate mount angles), each tagged with the tilt joint angle
that was in effect when its cloud arrived (from
/tilt_axis_bridge/joint_state, same coarse "whatever the latest sample
says" approach scan_aggregator's own sweep-edge-margin check already uses
-- adequate here for the same reason: one representative tilt angle per
~100ms cloud, not per individual point, matches the granularity the real
production pipeline itself uses via its own per-cloud tf2 lookup, so this
capture is calibrating what the real system actually does, not a
hypothetically finer-grained version of it).

Usage:
    python3 capture_raw_for_mount_calibration.py [--duration 60] [-o out.npz]

Run this from the Pi (or anywhere on the same ROS2 network) WHILE a real
scan runs via the GUI/scan_aggregator as normal -- this script only
listens, it doesn't drive anything. For mount-angle calibration to work,
the scan needs genuine overlap: a step-and-stare or sweep range past
~150 degrees (VLP-16's own 30-degree vertical FOV), covering a flat,
feature-rich surface (a wall) somewhere in the overlap region.

Ctrl+C to stop and save; the resulting .npz has two arrays consumed by
calibrate_mount_angle.py:
    points_velodyne  (N, 3) float32 -- raw sensor-frame xyz
    tilt_rad         (N,)   float32 -- tilt joint angle at capture time
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from numpy.lib import recfunctions as rfn
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2


class CaptureNode(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("capture_raw_for_mount_calibration")
        self._out_path = out_path
        self._current_tilt_rad = 0.0
        self._points_chunks: list[np.ndarray] = []
        self._tilt_chunks: list[np.ndarray] = []
        self._cloud_count = 0

        self.create_subscription(
            JointState, "/tilt_axis_bridge/joint_state", self._on_joint_state, 10
        )
        self.create_subscription(
            PointCloud2, "/velodyne_points", self._on_pointcloud, 10
        )
        self.get_logger().info(
            "Listening on /velodyne_points + /tilt_axis_bridge/joint_state -- "
            "run a real overlap scan now. Ctrl+C here when it's done to save."
        )

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            idx = msg.name.index("tilt_axis")
        except ValueError:
            return
        self._current_tilt_rad = msg.position[idx]

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        structured = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        if not structured.size:
            return
        arr = rfn.structured_to_unstructured(structured, dtype=np.float32)
        self._points_chunks.append(arr)
        self._tilt_chunks.append(
            np.full(arr.shape[0], self._current_tilt_rad, dtype=np.float32)
        )
        self._cloud_count += 1
        if self._cloud_count % 20 == 0:
            total = sum(c.shape[0] for c in self._points_chunks)
            self.get_logger().info(
                f"{self._cloud_count} clouds captured, {total} points so far "
                f"(latest tilt: {self._current_tilt_rad:.4f} rad)"
            )

    def save(self) -> None:
        if not self._points_chunks:
            self.get_logger().warning("No clouds captured -- nothing to save.")
            return
        points = np.concatenate(self._points_chunks, axis=0)
        tilt = np.concatenate(self._tilt_chunks, axis=0)
        np.savez_compressed(self._out_path, points_velodyne=points, tilt_rad=tilt)
        self.get_logger().info(
            f"Saved {points.shape[0]} points from {self._cloud_count} clouds -> {self._out_path}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o", "--output", default="mount_calibration_raw.npz", help="output .npz path"
    )
    args = parser.parse_args()

    rclpy.init()
    node = CaptureNode(args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
