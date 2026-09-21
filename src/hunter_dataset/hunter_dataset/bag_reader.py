"""Minimal rosbag2 iteration, isolated so convert_dataset.py stays testable.

Everything ROS-specific about *reading* lives here. The converter itself only
sees plain (stamp_ns, fields) tuples, which means its join logic can be unit
tested against synthetic data without a bag or a ROS graph.
"""
from __future__ import annotations

import math

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def _stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _yaw(q) -> float:
    # ZYX yaw from a quaternion; the robot is planar so roll/pitch are ~0.
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def read_bag(bag_dir: str, topics: list[str]) -> dict[str, list]:
    """Return {topic: [(stamp_ns, msg), ...]} sorted by header stamp.

    Header stamps, NOT bag receive times. Receive time carries transport
    jitter that scales with CPU load, and the sim clock is what every node
    stamped against. Sorting is explicit because mcap preserves *receive*
    order, which can differ from stamp order across topics under load.
    """
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_dir, storage_id='mcap'),
                ConverterOptions('', ''))
    type_of = {t.name: t.type for t in reader.get_all_topics_and_types()}
    wanted = set(topics)
    missing = wanted - set(type_of)
    if missing:
        raise RuntimeError(f"bag {bag_dir} is missing topics {sorted(missing)}. "
                           f"Present: {sorted(type_of)}")
    cls = {t: get_message(type_of[t]) for t in wanted}
    out: dict[str, list] = {t: [] for t in wanted}

    while reader.has_next():
        topic, raw, _recv_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(raw, cls[topic])
        out[topic].append((_stamp_ns(msg.header), msg))

    for t in out:
        out[t].sort(key=lambda p: p[0])
    return out


# Field extractors -- keep the converter free of message-type knowledge.
def scan_ranges(msg):
    return msg.ranges


def diag_fields(msg):
    return dict(
        steering=float(msg.commanded_steering_rad),
        speed=float(msg.commanded_speed_mps),
        progress=float(msg.path_s_m),
        lateral_error=float(msg.lateral_error_m),
        controller=str(msg.controller_name),
    )


def odom_pose(msg):
    p = msg.pose.pose.position
    return (float(p.x), float(p.y), _yaw(msg.pose.pose.orientation))


def event_fields(msg):
    return dict(terminal=bool(msg.terminal), trigger=int(msg.trigger),
                min_scan=float(msg.min_scan_m), contacts=int(msg.contact_count))
