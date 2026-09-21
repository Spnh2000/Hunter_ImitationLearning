"""HDF5 loading, splits, and episode-aware frame stacking.

Everything here works on the schema in hunter_dataset.dataset_spec and
nothing else. No ROS.

Frame stacking (K > 1) is the one place a silent bug is cheap to write and
expensive to notice: a naive obs[t-K+1:t+1] slice runs backwards across the
boundary into the previous episode -- or, when files are concatenated, into a
different TRACK -- and the training loss looks fine. Stacks here are gathered
through an explicit per-row index table where every history index is clamped
to the start of the row's own episode (first scan repeated), and a test
asserts that every stack at an episode start contains only that episode.
"""
from __future__ import annotations

import glob
import os
from dataclasses import dataclass, field

import h5py
import numpy as np

from hunter_dataset import dataset_spec as spec


@dataclass
class Run:
    path: str
    track_id: str
    max_speed_mps: float
    run_id: str
    obs: np.ndarray            # [N, 720] float32, metres
    act: np.ndarray            # [N, 2]   float32, (rad, m/s)
    episode_id: np.ndarray     # [N] int32
    lap_index: np.ndarray      # [N] int32
    terminals: np.ndarray      # [N] bool
    action_source: np.ndarray  # [N] uint8
    progress_s_m: np.ndarray   # [N] float32
    lateral_error_m: np.ndarray
    gt_pose: np.ndarray        # [N, 3]
    attrs: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.act)


def load_run(path: str) -> Run:
    with h5py.File(path, 'r') as f:
        major = str(f.attrs['schema_version']).split('.')[0]
        if major != spec.SCHEMA_VERSION.split('.')[0]:
            raise ValueError(f'{path}: schema {f.attrs["schema_version"]} vs spec {spec.SCHEMA_VERSION}')
        n = f['observations'].shape[0]
        run = Run(
            path=path,
            track_id=str(f.attrs['track_id']),
            max_speed_mps=float(f.attrs['max_speed_mps']),
            run_id=os.path.basename(os.path.dirname(path)),
            obs=f['observations'][:].astype(np.float32),
            act=f['actions'][:].astype(np.float32),
            episode_id=f['infos/episode_id'][:].astype(np.int32),
            lap_index=f['infos/lap_index'][:].astype(np.int32),
            terminals=f['terminals'][:].astype(bool),
            action_source=f['infos/action_source'][:].astype(np.uint8),
            progress_s_m=f['infos/progress_s_m'][:].astype(np.float32),
            lateral_error_m=(f['infos/lateral_error_m'][:].astype(np.float32)
                             if 'infos/lateral_error_m' in f else np.zeros(n, np.float32)),
            gt_pose=f['infos/gt_pose'][:].astype(np.float32),
            attrs={k: (v.tolist() if hasattr(v, 'tolist') else v) for k, v in f.attrs.items()
                   if k not in ('downsample_indices', 'converter_report')},
        )
    if run.obs.shape[1] != spec.OBS_DIM:
        raise ValueError(f'{path}: obs dim {run.obs.shape[1]} != {spec.OBS_DIM}')
    return run


def find_runs(data_dir: str, pattern: str = '*/dataset.hdf5') -> list[str]:
    return sorted(glob.glob(os.path.join(data_dir, pattern)))


# ---------------------------------------------------------------- splits ----

@dataclass
class Split:
    train: list[tuple[int, np.ndarray]]         # (run index, row indices)
    val_laps: list[tuple[int, np.ndarray]]      # held-out laps on TRAINING tracks
    val_tracks: list[tuple[int, np.ndarray]]    # every row of HELD-OUT tracks

    def describe(self, runs):
        def n(sel): return int(sum(len(r) for _, r in sel))
        def tracks(sel): return sorted({runs[i].track_id for i, _ in sel})
        return (f'train {n(self.train)} rows {tracks(self.train)} | '
                f'val_laps {n(self.val_laps)} | val_tracks {n(self.val_tracks)} {tracks(self.val_tracks)}')


def make_split(runs: list[Run], heldout_tracks: list[str], heldout_lap_rule: str = 'p1_lap1',
               drop_terminal_episodes: bool = True) -> Split:
    """Two-level split.

    heldout_tracks     : entire tracks never seen in training (generalisation)
    heldout_lap_rule   : 'p1_lap1' = on training tracks, lap_index 1 of every
                         pass-p1 run (the second lap from a standing start:
                         steady-state, one per speed) is validation. That
                         measures memorisation of seen geometry.
    drop_terminal_episodes : episodes that end in a crash are excluded from
                         TRAINING when the data is expert-only (a crashed
                         expert is not a teacher); they stay in validation.
    """
    train, val_laps, val_tracks = [], [], []
    for i, r in enumerate(runs):
        rows = np.arange(len(r))
        if r.track_id in heldout_tracks:
            val_tracks.append((i, rows))
            continue
        is_val = np.zeros(len(r), bool)
        if heldout_lap_rule == 'p1_lap1' and r.run_id.endswith('_p1'):
            is_val = r.lap_index == 1
        elif heldout_lap_rule == 'none':
            pass
        else:
            raise ValueError(heldout_lap_rule)
        keep = ~is_val
        if drop_terminal_episodes and r.terminals.any():
            bad_eps = np.unique(r.episode_id[r.terminals])
            expert_rows = r.action_source == spec.ACTION_SOURCE_EXPERT
            keep &= ~(np.isin(r.episode_id, bad_eps) & expert_rows)
        if keep.any():
            train.append((i, rows[keep]))
        if is_val.any():
            val_laps.append((i, rows[is_val]))
    return Split(train, val_laps, val_tracks)


# ------------------------------------------------------------- stacking ----

def episode_start_index(episode_id: np.ndarray) -> np.ndarray:
    """For each row, the index of the first row of its episode. Episodes are
    contiguous in a file by construction (the converter assigns them in time
    order), so this is a running 'last change point'."""
    n = len(episode_id)
    start = np.zeros(n, dtype=np.int64)
    for t in range(1, n):
        start[t] = start[t - 1] if episode_id[t] == episode_id[t - 1] else t
    return start


def stack_indices(episode_id: np.ndarray, K: int) -> np.ndarray:
    """[N, K] row indices such that stack[t] = obs[idx[t]] holds the K most
    recent scans of the SAME episode, oldest first, with the episode's first
    scan repeated where history is missing."""
    n = len(episode_id)
    t = np.arange(n, dtype=np.int64)
    start = episode_start_index(episode_id)
    offsets = np.arange(K - 1, -1, -1, dtype=np.int64)        # K-1 ... 0
    idx = t[:, None] - offsets[None, :]
    return np.maximum(idx, start[:, None])


class StackedArrays:
    """Materialises (obs_stack, act) for a selection of rows across runs.

    Memory: rows x K x 720 x 4 bytes. 500k rows at K=3 is 4.3 GB, so stacks
    are gathered lazily per batch from the flat obs of each run instead of
    being materialised. Only the [N,K] index table is built up front.
    """

    def __init__(self, runs: list[Run], selection: list[tuple[int, np.ndarray]], K: int):
        self.K = K
        self.obs = [r.obs for r in runs]
        self.act = [r.act for r in runs]
        self.cap = np.array([r.max_speed_mps for r in runs], dtype=np.float32)
        self.run_of = []      # flat -> run index
        self.idx = []         # flat -> [K] row indices within that run
        for ri, rows in selection:
            full = stack_indices(runs[ri].episode_id, K)
            self.run_of.append(np.full(len(rows), ri, dtype=np.int32))
            self.idx.append(full[rows])
        self.run_of = np.concatenate(self.run_of) if self.run_of else np.zeros(0, np.int32)
        self.idx = np.concatenate(self.idx) if self.idx else np.zeros((0, K), np.int64)
        self.n = len(self.run_of)

    def __len__(self):
        return self.n

    def batch(self, flat: np.ndarray):
        """obs [B, K, 720], act [B, 2], cap [B] (m/s) for flat indices."""
        B = len(flat)
        out = np.empty((B, self.K, spec.OBS_DIM), dtype=np.float32)
        act = np.empty((B, spec.ACTION_DIM), dtype=np.float32)
        runs = self.run_of[flat]
        rows = self.idx[flat]                        # [B, K]
        for ri in np.unique(runs):
            m = runs == ri
            out[m] = self.obs[ri][rows[m]]           # fancy index [b, K] -> [b, K, 720]
            act[m] = self.act[ri][rows[m][:, -1]]    # action of the CURRENT row
        return out, act, self.cap[runs]

    def all_actions(self) -> np.ndarray:
        parts = [self.act[ri][self.idx[self.run_of == ri][:, -1]] for ri in np.unique(self.run_of)]
        return np.concatenate(parts) if parts else np.zeros((0, 2), np.float32)
