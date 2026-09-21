#!/usr/bin/env python3
"""policy_tracker -- the Phase 8 student. Reads /scan, runs a hunter_il
checkpoint, publishes the same command and diagnostics a pid_tracker would.

Interface (identical slot to pid_tracker, so benchmark.launch.py needs only
controller:=policy and the checkpoint parameter):

    subscribes   /scan                                  ONLY input, by design
    publishes    /ackermann_like_controller/reference   TwistStamped, ZERO stamp
                 /tracking/diagnostics                  TrackingDiagnostics
    service      ~/reset_stack   std_srvs/Trigger       clear the K-frame history
                                                         after a respawn

The node subscribes to no pose topic at all. The student never sees where it
is; that has to be true of the process, not just of the tensor.

Observation pipeline is dataset_spec.scan_to_observation -- the SAME function
the converter used to build the training data, imported from the same module.

Pose-derived diagnostics fields (lateral_error_m, heading_error_rad,
path_s_m, speed_error_mps, ...) are published as NaN: the policy cannot know
them, and a number here would be a lie metrics_logger would average.
metrics_logger derives laps and true error from ground truth itself.
"""
import collections
import math
import time

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

from hunter_dataset.dataset_spec import scan_to_observation
from hunter_il import checkpoint
from hunter_msgs.msg import TrackingDiagnostics

NAN = float('nan')


class PolicyTracker(Node):
    def __init__(self):
        super().__init__('policy_tracker')
        ckpt = self.declare_parameter('checkpoint_path', '').value
        arch = self.declare_parameter('arch', '').value
        self.controller_name = self.declare_parameter('controller_name', 'policy').value
        self.wheelbase = float(self.declare_parameter('wheelbase_m', 0.65).value)
        self.max_steer = float(self.declare_parameter('max_steering_rad', 0.461).value)
        self.max_speed = float(self.declare_parameter('max_speed_mps', 2.7).value)
        self.decode_mode = self.declare_parameter('decode_mode', 'argmax').value
        threads = int(self.declare_parameter('torch_threads', 1).value)
        # Tolerated by the tracker slot; the policy is scan-driven, not timer-driven.
        self.declare_parameter('control_frequency_hz', 50.0)
        if not ckpt:
            raise RuntimeError('policy_tracker needs checkpoint_path')
        torch.set_num_threads(threads)
        self.model, meta = checkpoint.load(ckpt)
        if arch and arch != meta['arch']:
            raise RuntimeError(f"arch parameter '{arch}' but checkpoint is '{meta['arch']}' -- refusing "
                               f"to run the wrong head silently")
        self.K = int(meta['K'])
        self.stack = collections.deque(maxlen=self.K)
        self.cmd_pub = self.create_publisher(TwistStamped, '/ackermann_like_controller/reference', 10)
        diag_qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE)
        self.diag_pub = self.create_publisher(TrackingDiagnostics, '/tracking/diagnostics', diag_qos)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_service(Trigger, '~/reset_stack', self.on_reset)
        self.tick_ms = collections.deque(maxlen=400)
        self.n = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"policy_tracker ready: {meta['arch']} K={self.K} from {ckpt} "
            f"(trained {meta.get('saved_utc')}), clamp steer ±{self.max_steer} speed ≤{self.max_speed}")

    def on_reset(self, _req, res):
        self.stack.clear()
        res.success, res.message = True, 'frame stack cleared'
        return res

    def on_scan(self, msg: LaserScan):
        t0 = time.perf_counter()
        obs = scan_to_observation(np.asarray(msg.ranges, dtype=np.float32))
        if not self.stack:                       # episode start: repeat the first scan,
            for _ in range(self.K):              # exactly as training padded it
                self.stack.append(obs)
        else:
            self.stack.append(obs)
        x = torch.from_numpy(np.stack(self.stack)[None])          # [1, K, 720]
        with torch.no_grad():
            if hasattr(self.model, 'logits'):
                a = self.model(x, mode=self.decode_mode)[0]
            else:
                a = self.model(x)[0]
        steer = min(max(float(a[0]), -self.max_steer), self.max_steer)
        speed = min(max(float(a[1]), 0.0), self.max_speed)

        cmd = TwistStamped()
        cmd.header.stamp.sec = 0                 # zero-stamped, like pid_tracker
        cmd.header.stamp.nanosec = 0
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = speed
        cmd.twist.angular.z = speed * math.tan(steer) / self.wheelbase
        self.cmd_pub.publish(cmd)

        d = TrackingDiagnostics()
        d.header.stamp = msg.header.stamp        # the scan's stamp: same clock as the data
        d.header.frame_id = 'odom'
        d.controller_name = self.controller_name
        d.lateral_error_m = NAN
        d.heading_error_rad = NAN
        d.target_speed_mps = NAN
        d.actual_speed_mps = NAN
        d.speed_error_mps = NAN
        d.commanded_steering_rad = steer
        d.commanded_speed_mps = speed
        d.path_s_m = NAN
        d.closest_waypoint_index = 0
        self.diag_pub.publish(d)
        self.tick_ms.append((time.perf_counter() - t0) * 1e3)
        self.n += 1

    def report(self):
        if not self.tick_ms:
            self.get_logger().warn('no /scan received yet')
            return
        t = sorted(self.tick_ms)
        p99 = t[int(len(t) * 0.99) - 1] if len(t) > 1 else t[0]
        msg = f'{self.n} ticks; scan->cmd ms median {t[len(t) // 2]:.2f} p99 {p99:.2f} max {t[-1]:.2f}'
        if p99 > 20.0:
            self.get_logger().warn(msg + '  (p99 above 20 ms: budget is 50)')
        else:
            self.get_logger().info(msg)


def main():
    rclpy.init()
    node = PolicyTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
