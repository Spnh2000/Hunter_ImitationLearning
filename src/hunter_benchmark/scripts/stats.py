#!/usr/bin/env python3
"""Shared incremental statistics for TrackingDiagnostics streams.

Used identically by the live metrics_logger node and the offline
summarize_run.py script, so a bag reprocessed later via summarize_run.py
produces the exact same summary.csv a live run would have written. Pure
Python, no ROS imports, so it works unmodified in both contexts -- and
in a future offline RL evaluation that may not have a live ROS graph at
all (see Phase 3 plan, reuse-risk callout #1).
"""
import csv
import math
import os
import time

CSV_COLUMNS = [
    'track_name', 'controller_name', 'pose_source', 'run_timestamp', 'duration_s',
    'distance_traveled_m', 'laps_completed',
    # Error as the CONTROLLER sees it -- computed against whatever pose estimate
    # it was given. Useful for judging the control loop, useless for judging
    # whether the robot was actually on the track: on the Phase 3 wheel-odometry
    # runs this read 1 mm while ground truth had the robot 2.29 m off-line.
    'mean_lateral_error_m', 'rms_lateral_error_m', 'max_lateral_error_m',
    'mean_heading_error_rad', 'rms_heading_error_rad', 'max_heading_error_rad',
    # Error as REALITY sees it -- the same cross-track quantity recomputed from
    # Gazebo ground-truth pose against the same reference path. This is the
    # column that decides whether a pose source is acceptable.
    'mean_gt_lateral_error_m', 'rms_gt_lateral_error_m', 'max_gt_lateral_error_m',
    # How much the estimator is lying: || estimated pose - ground truth ||.
    # Separates "the controller tracked badly" from "the controller tracked a
    # phantom" -- the distinction the whole phase exists to make.
    'mean_estimator_error_m', 'rms_estimator_error_m', 'max_estimator_error_m',
    # Oscillation metrics. Unchanged from Phase 3 (0.02 rad deadband) so
    # pre-localization numbers stay directly comparable.
    'steering_cmd_variance', 'steering_reversal_rate_hz', 'mean_speed_error_mps',
    'rms_speed_error_mps',
]


class RunningStat:
    """Welford's online mean/variance, plus running max(|x|)."""

    def __init__(self):
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.max_abs = 0.0

    def update(self, value: float) -> None:
        self.n += 1
        delta = value - self.mean
        self.mean += delta / self.n
        delta2 = value - self.mean
        self.m2 += delta * delta2
        self.max_abs = max(self.max_abs, abs(value))

    @property
    def variance(self) -> float:
        return self.m2 / self.n if self.n > 1 else 0.0

    @property
    def rms(self) -> float:
        # E[x^2] = Var(x) + mean(x)^2 -- RMS is sqrt of that, not sqrt(variance).
        return math.sqrt(self.variance + self.mean ** 2)


class RunSummary:
    """Accumulates one benchmark run's stats from a stream of
    TrackingDiagnostics + Odometry samples. controller_name is never
    branched on internally (Phase 3 plan, reuse-risk callout #4) -- it is
    stored purely as a label for the output row."""

    def __init__(
        self, track_name: str, controller_name: str, total_length_m: float, is_closed_loop: bool,
        reversal_deadband_rad: float = 0.02, pose_source: str = 'unknown',
    ):
        self.track_name = track_name
        self.controller_name = controller_name
        # Free-text label, never branched on -- same contract as controller_name.
        self.pose_source = pose_source
        self.total_length_m = total_length_m
        self.is_closed_loop = is_closed_loop
        self.reversal_deadband_rad = reversal_deadband_rad

        self.lateral = RunningStat()
        self.heading = RunningStat()
        self.speed_error = RunningStat()
        # Ground-truth-referenced channels. Populated by add_ground_truth_sample()
        # and left empty (n == 0) when no ground truth is available, e.g. when
        # reprocessing a bag recorded on hardware.
        self.gt_lateral = RunningStat()
        self.estimator_error = RunningStat()

        self.distance_traveled_m = 0.0
        self.laps_completed = 0

        self._start_time_s = None
        self._end_time_s = None
        self._prev_s_m = None
        self._prev_steering = None
        self._reversal_count = 0
        self._steering_n = 0
        self._steering_mean = 0.0
        self._steering_m2 = 0.0
        self._prev_xy = None
        self._laps_from_tracker = False
        self._prev_gt_s_m = None

    def add_sample(
        self, stamp_s: float, lateral_error_m: float, heading_error_rad: float,
        speed_error_mps: float, commanded_steering_rad: float, path_s_m: float,
    ) -> None:
        if self._start_time_s is None:
            self._start_time_s = stamp_s
        self._end_time_s = stamp_s

        self.lateral.update(lateral_error_m)
        self.heading.update(heading_error_rad)
        self.speed_error.update(speed_error_mps)

        self._steering_n += 1
        delta = commanded_steering_rad - self._steering_mean
        self._steering_mean += delta / self._steering_n
        self._steering_m2 += delta * (commanded_steering_rad - self._steering_mean)

        if self._prev_steering is not None:
            prev, cur = self._prev_steering, commanded_steering_rad
            if (
                abs(prev) >= self.reversal_deadband_rad
                and abs(cur) >= self.reversal_deadband_rad
                and (prev > 0) != (cur > 0)
            ):
                self._reversal_count += 1
        self._prev_steering = commanded_steering_rad

        # Lap counting from the tracker's OWN progress only when it reports one.
        # A scan-only policy (Phase 8) publishes path_s_m = NaN; its laps are
        # counted from ground truth in add_ground_truth_sample instead. The two
        # never both count: whichever source is live owns laps_completed.
        if path_s_m == path_s_m:                       # not NaN
            self._laps_from_tracker = True
            if self.is_closed_loop and self.total_length_m > 0 and self._prev_s_m is not None:
                # A large backward jump in arc-length means we wrapped past the finish line.
                if path_s_m < self._prev_s_m - self.total_length_m * 0.5:
                    self.laps_completed += 1
            self._prev_s_m = path_s_m

    def add_odometry(self, x: float, y: float) -> None:
        if self._prev_xy is not None:
            self.distance_traveled_m += math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
        self._prev_xy = (x, y)

    def add_ground_truth_sample(
        self, gt_lateral_error_m: float, estimator_error_m: float = None,
        gt_path_s_m: float = None,
    ) -> None:
        """Record the two reality-referenced channels.

        gt_lateral_error_m is the signed cross-track distance from the TRUE
        pose to the same reference path the controller is tracking.
        estimator_error_m is the Euclidean distance between the pose the
        controller was given and the true pose -- omitted when the pose source
        IS ground truth, where it is identically zero by construction.
        """
        self.gt_lateral.update(gt_lateral_error_m)
        if estimator_error_m is not None:
            self.estimator_error.update(estimator_error_m)
        if gt_path_s_m is not None and not self._laps_from_tracker:
            if self.is_closed_loop and self.total_length_m > 0 and self._prev_gt_s_m is not None:
                if gt_path_s_m < self._prev_gt_s_m - self.total_length_m * 0.5:
                    self.laps_completed += 1
            self._prev_gt_s_m = gt_path_s_m

    def steering_variance(self) -> float:
        return self._steering_m2 / self._steering_n if self._steering_n > 1 else 0.0

    def duration_s(self) -> float:
        if self._start_time_s is None or self._end_time_s is None:
            return 0.0
        return self._end_time_s - self._start_time_s

    def reversal_rate_hz(self) -> float:
        d = self.duration_s()
        return self._reversal_count / d if d > 0 else 0.0

    def to_row(self) -> dict:
        return {
            'track_name': self.track_name,
            'controller_name': self.controller_name,
            'pose_source': self.pose_source,
            'run_timestamp': time.strftime('%Y%m%d_%H%M%S'),
            'duration_s': round(self.duration_s(), 3),
            'distance_traveled_m': round(self.distance_traveled_m, 3),
            'laps_completed': self.laps_completed,
            'mean_lateral_error_m': round(self.lateral.mean, 4),
            'rms_lateral_error_m': round(self.lateral.rms, 4),
            'max_lateral_error_m': round(self.lateral.max_abs, 4),
            'mean_heading_error_rad': round(self.heading.mean, 4),
            'rms_heading_error_rad': round(self.heading.rms, 4),
            'max_heading_error_rad': round(self.heading.max_abs, 4),
            'mean_gt_lateral_error_m': round(self.gt_lateral.mean, 4),
            'rms_gt_lateral_error_m': round(self.gt_lateral.rms, 4),
            'max_gt_lateral_error_m': round(self.gt_lateral.max_abs, 4),
            'mean_estimator_error_m': round(self.estimator_error.mean, 4),
            'rms_estimator_error_m': round(self.estimator_error.rms, 4),
            'max_estimator_error_m': round(self.estimator_error.max_abs, 4),
            'steering_cmd_variance': round(self.steering_variance(), 6),
            'steering_reversal_rate_hz': round(self.reversal_rate_hz(), 4),
            'mean_speed_error_mps': round(self.speed_error.mean, 4),
            'rms_speed_error_mps': round(self.speed_error.rms, 4),
        }


def write_summary_csv(path: str, row: dict) -> None:
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerow(row)
