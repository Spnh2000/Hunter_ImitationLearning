#!/usr/bin/env python3
"""Offline: replay a recorded rosbag2 (mcap) and produce the same
summary.csv a live metrics_logger run would have written, using the
shared `stats` module. This is the piece that must keep working
unmodified for Phase 9's offline RL evaluation, whose bags may not have
come from a live ROS graph at all.

Usage:
  summarize_run.py <bag_dir> --track-name Monza --controller-name pid --out summary.csv
"""
import argparse
import os
import sys

import rclpy.serialization
import rosbag2_py

from hunter_msgs.msg import Trajectory, TrackingDiagnostics
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stats  # noqa: E402  (installed alongside this script -- see CMakeLists.txt)

_MSG_TYPES = {
    '/trajectory/reference': Trajectory,
    '/tracking/diagnostics': TrackingDiagnostics,
    '/ackermann_like_controller/odometry': Odometry,
}


def read_bag(
    bag_dir: str, track_name: str, controller_name: str, reversal_deadband_rad: float,
) -> 'stats.RunSummary':
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )

    summary = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg_type = _MSG_TYPES.get(topic)
        if msg_type is None:
            continue
        msg = rclpy.serialization.deserialize_message(data, msg_type)

        if topic == '/trajectory/reference':
            if summary is None:
                summary = stats.RunSummary(
                    track_name, controller_name, msg.total_length_m, msg.is_closed_loop,
                    reversal_deadband_rad=reversal_deadband_rad)
        elif topic == '/tracking/diagnostics':
            if summary is not None:
                stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                summary.add_sample(
                    stamp_s, msg.lateral_error_m, msg.heading_error_rad, msg.speed_error_mps,
                    msg.commanded_steering_rad, msg.path_s_m)
        elif topic == '/ackermann_like_controller/odometry':
            if summary is not None:
                summary.add_odometry(msg.pose.pose.position.x, msg.pose.pose.position.y)

    if summary is None:
        raise RuntimeError(
            f"No /trajectory/reference message found in '{bag_dir}' -- was this bag recorded "
            "during a benchmark run?")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir')
    parser.add_argument('--track-name', required=True)
    parser.add_argument('--controller-name', required=True)
    parser.add_argument('--out', default='summary.csv')
    parser.add_argument('--reversal-deadband-rad', type=float, default=0.02)
    args = parser.parse_args()

    summary = read_bag(
        args.bag_dir, args.track_name, args.controller_name, args.reversal_deadband_rad)
    stats.write_summary_csv(args.out, summary.to_row())
    print(f"Wrote {args.out}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
