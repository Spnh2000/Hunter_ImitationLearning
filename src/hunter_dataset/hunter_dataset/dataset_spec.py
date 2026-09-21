"""Canonical, version-stamped definition of the Phase 6 imitation dataset.

This module is the ONE source of truth. The recorder, the converter, the
inspector and Phase 8's training code all import from here; nothing anywhere
else may hardcode a beam index, a clip bound, a rate or a field dtype. If a
number appears in two places, they will disagree eventually and the
disagreement will be silent.

DEPENDENCY CONSTRAINT -- enforced by test/test_dataset_spec_is_ros_free.py:
this module imports NOTHING from ROS. Phase 8 trains in a torch/d3rlpy
environment that has no ROS installed, and it must be able to read the schema
that produced its data. An `import rclpy` here would force Phase 8 to keep a
second, drifting copy of these constants. numpy is permitted; it is present in
both worlds.

Storage architecture this schema assumes:

    Gazebo -> rosbag2 (immutable, full 1080 beams, all topics)
                 -> convert_dataset.py (the only downsampler)
                    -> dataset_vN.hdf5

The bag keeps what the schema did not anticipate, so a schema change is a
converter re-run rather than 200 re-driven laps.
"""
from __future__ import annotations

import numpy as np

# --------------------------------------------------------------------------
# Version
# --------------------------------------------------------------------------
# Bump MINOR when a field is added in a backward-compatible way; bump MAJOR
# when the meaning of an existing field changes. The inspector refuses to read
# a file whose MAJOR differs from its own.
SCHEMA_VERSION = "1.0.0"


# --------------------------------------------------------------------------
# Observation: LiDAR downsampling 1080 -> 720
# --------------------------------------------------------------------------
RAW_BEAMS = 1080
OBS_DIM = 720

# Keep every beam whose index is NOT 1 mod 3, i.e. drop the middle beam of
# each consecutive triple. 1080 = 360 triples -> 720 kept.
#
# Chosen over linear interpolation deliberately. The simulated LiDAR carries
# ~1 cm Gaussian range noise for realism; averaging adjacent beams would
# shrink independent noise by ~1/sqrt(2), so the student would train on
# cleaner scans than the sensor can ever produce -- an optimistic gap that
# only shows up at deployment. Index selection preserves each beam's noise
# exactly as measured.
#
# The cost is non-uniform angular spacing (0.25 deg / 0.5 deg alternating).
# That is acceptable: an MLP treats its inputs as fixed arbitrary features and
# never assumes uniform angular support. It would NOT be acceptable for a
# convolutional or geometric model, which is worth knowing if the architecture
# ever changes.
DOWNSAMPLE_IDX: np.ndarray = np.array(
    [i for i in range(RAW_BEAMS) if i % 3 != 1], dtype=np.int32
)
assert DOWNSAMPLE_IDX.shape == (OBS_DIM,), (
    f"downsample must yield exactly {OBS_DIM} beams, got {DOWNSAMPLE_IDX.shape}"
)


# --------------------------------------------------------------------------
# Observation: range sanitising
# --------------------------------------------------------------------------
# A no-return beam arrives as +inf (and occasionally NaN). Left alone, either
# propagates through the first matmul to NaN weights and surfaces much later
# as a mysterious optimiser failure with no obvious cause.
RANGE_MIN = 0.15   # metres, sensor minimum (sensors.xacro)
RANGE_MAX = 30.0   # metres, sensor maximum (sensors.xacro)
INVALID_FILL = 30.0  # what nan / +inf / -inf become: "saw nothing out to max range"

OBS_DTYPE = np.float32


# --------------------------------------------------------------------------
# Rate and pairing
# --------------------------------------------------------------------------
# The dataset is at SCAN rate, not control rate. The student can only act on a
# new observation, so at deployment it runs at 20 Hz with a zero-order hold
# between scans; training at 50 Hz would train a policy that never exists.
#
# Recording at 50 Hz by holding the last scan would also triple the row count
# with correlated duplicates, making the dataset look larger than its true
# information content -- which miscalibrates Phase 8 CQL's conservatism
# penalty, since that penalty is tuned against dataset size.
DATASET_HZ = 20.0
CONTROL_HZ = 50.0

# Each scan is paired with the nearest-in-time expert command. At 50 Hz control
# the true worst case is 10 ms; 15 ms leaves margin for jitter without
# admitting a stale command from the previous control tick.
SYNC_SLOP_NS = 15_000_000  # 15 ms


# --------------------------------------------------------------------------
# Action
# --------------------------------------------------------------------------
ACTION_DIM = 2
ACTION_NAMES = ("steering_rad", "speed_mps")

# PHYSICAL UNITS, never normalised. Collection spans three max_speed_mps
# settings (1.5 / 2.1 / 2.7), so a stored 0.85 would mean a different physical
# speed in each run -- corruption that is silent and unrecoverable after the
# fact. Normalisation belongs at training time, using ACTION_BOUNDS below.
#
# Bounds are vehicle limits (robot_core.xacro), not per-run limits: steering
# +/-0.461 rad, speed 0 to the drivetrain ceiling. A run's own max_speed_mps
# is recorded separately as an attribute.
ACTION_BOUNDS = np.array([[-0.461, 0.461],    # steering_rad
                          [0.0, 2.81]],       # speed_mps (17 rad/s * 0.165 m)
                         dtype=np.float32)
ACTION_DTYPE = np.float32


# --------------------------------------------------------------------------
# Episode termination
# --------------------------------------------------------------------------
# terminals and timeouts are SEPARATE and must stay that way. d3rlpy bootstraps
# the value function through a timeout (the episode was cut short, the world
# continued) but not through a terminal (the world ended). Conflating them
# corrupts CQL value estimates in a way that is extremely hard to trace back.
#
# Two independent crash signals, OR-ed. Contact sensors can miss a glancing
# wall scrape, and this project's history argues for two signals over one.
SCAN_CONTACT_M = 0.4   # secondary trigger: min(scan) below this counts as a hit
# Primary trigger is the Gazebo contact sensor on /bumper_contacts; it has no
# threshold to record. This value IS stored in the file attrs, because it is
# part of what the `terminals` column means.


# --------------------------------------------------------------------------
# HDF5 layout
# --------------------------------------------------------------------------
# D4RL / d3rlpy-native so Phase 8 needs no reshaping.
#
# (name, dtype, per-row shape). Leading dimension N is the row count.
DATASETS = (
    ("observations", np.float32, (OBS_DIM,)),
    ("actions",      np.float32, (ACTION_DIM,)),
    ("rewards",      np.float32, ()),
    ("terminals",    np.bool_,   ()),
    ("timeouts",     np.bool_,   ()),
)

# Teacher-side and bookkeeping data. NEVER part of the student's observation:
# gt_pose in particular is the privilege that makes this "Learning by Cheating"
# rather than ordinary behaviour cloning.
INFOS = (
    ("timestamp_ns",  np.int64,   ()),
    ("gt_pose",       np.float32, (3,)),    # x, y, yaw -- teacher-side only
    ("progress_s_m",  np.float32, ()),
    ("lap_index",     np.int32,   ()),
    ("episode_id",    np.int32,   ()),
    ("prev_action",   np.float32, (ACTION_DIM,)),
    ("dagger_beta",   np.float32, ()),
    ("action_source", np.uint8,   ()),      # 0 = expert executed
                                            # 1 = student executed, expert relabelled
)

ACTION_SOURCE_EXPERT = 0
ACTION_SOURCE_STUDENT_RELABELLED = 1

# File-level attributes. track_sha256 is what makes track_id trustworthy: a
# string is an assertion, a hash is verifiable against the CSV that was driven.
ATTRS = (
    "schema_version",
    "track_id",
    "track_sha256",       # SHA256 of the centerline CSV actually driven
    "track_scale",
    "controller_label",
    "max_speed_mps",
    "action_bounds",
    "downsample_indices",  # stored so the mapping is auditable FROM THE DATA
    "dataset_hz",
    "range_clip",
    "scan_contact_m",
    "git_rev",
)


# --------------------------------------------------------------------------
# The policies, executable rather than merely described
# --------------------------------------------------------------------------
def sanitize_ranges(ranges: np.ndarray) -> np.ndarray:
    """Apply the inf/NaN policy, then clip. Returns float32.

    Order is fixed: replace non-finite values FIRST, then clip. Clipping first
    would leave NaN untouched (np.clip propagates NaN), so a NaN would survive
    into the dataset looking like a valid row.
    """
    out = np.asarray(ranges, dtype=np.float64)
    out = np.where(np.isfinite(out), out, INVALID_FILL)
    np.clip(out, RANGE_MIN, RANGE_MAX, out=out)
    return out.astype(OBS_DTYPE)


def downsample(ranges: np.ndarray) -> np.ndarray:
    """Select the 720 kept beams from a full 1080-beam scan."""
    arr = np.asarray(ranges)
    if arr.shape[-1] != RAW_BEAMS:
        raise ValueError(
            f"expected {RAW_BEAMS} beams, got {arr.shape[-1]}. The bag must hold "
            "raw scans; downsampling happens here and only here."
        )
    return arr[..., DOWNSAMPLE_IDX]


def scan_to_observation(ranges: np.ndarray) -> np.ndarray:
    """Full 1080-beam scan -> the stored 720-dim observation.

    Downsample then sanitise. The two commute (selection and an elementwise
    map), but the order is pinned so any two runs of the converter produce
    byte-identical output.
    """
    return sanitize_ranges(downsample(ranges))
