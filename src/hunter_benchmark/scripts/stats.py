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
    'track_name', 'controller_name', 'run_timestamp', 'duration_s', 'distance_traveled_m',
    'laps_completed', 'mean_lateral_error_m', 'rms_lateral_error_m', 'max_lateral_error_m',
    'mean_heading_error_rad', 'rms_heading_error_rad', 'max_heading_error_rad',
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
        reversal_deadband_rad: float = 0.02,
    ):
        self.track_name = track_name
        self.controller_name = controller_name
        self.total_length_m = total_length_m
        self.is_closed_loop = is_closed_loop
        self.reversal_deadband_rad = reversal_deadband_rad

        self.lateral = RunningStat()
        self.heading = RunningStat()
        self.speed_error = RunningStat()

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

        if self.is_closed_loop and self.total_length_m > 0 and self._prev_s_m is not None:
            # A large backward jump in arc-length means we wrapped past the finish line.
            if path_s_m < self._prev_s_m - self.total_length_m * 0.5:
                self.laps_completed += 1
        self._prev_s_m = path_s_m

    def add_odometry(self, x: float, y: float) -> None:
        if self._prev_xy is not None:
            self.distance_traveled_m += math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
        self._prev_xy = (x, y)

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
