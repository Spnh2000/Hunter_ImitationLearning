"""rosbag2 -> HDF5. The ONE canonical downsampler.

    Gazebo -> rosbag2 (immutable, full 1080)
                 -> THIS SCRIPT
                    -> dataset_vN.hdf5

Three stages, kept separate so each is independently checkable:

  1. read   -- bucket every message by topic, keyed on header stamp
  2. join   -- one row per scan:
                 diagnostics  : nearest-in-time via searchsorted, rejected if
                                |dt| > SYNC_SLOP_NS, and the dt distribution is
                                REPORTED rather than silently dropped
                 ground truth : same, nearest-in-time
                 episode event: EXACT stamp match. episode_monitor stamps its
                                event with the scan's own stamp precisely so
                                this join is exact; a miss is a bug and RAISES.
  3. write  -- allocate [N, ...] up front, fill, attach attrs

searchsorted, not message_filters.ApproximateTimeSynchronizer: that is a
live-stream tool, and reusing it offline buys nothing while hiding the one
thing worth seeing -- how far apart the paired stamps actually were. If the
clocks were offset this prints "0 of 4300 rows survived", not a short lap.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys

import h5py
import numpy as np
import yaml

from hunter_dataset import bag_reader as br
from hunter_dataset import dataset_spec as spec

TOPICS = {
    'scan':  '/scan',
    'diag':  '/tracking/diagnostics',
    'gt':    '/ground_truth/odometry',
    'event': '/episode/event',
}

# prev_action at the first row of an episode: repeat the row's own action, the
# same zero-order-hold-backward convention frame stacking uses for its padding.
# Zeros would read as "was stopped", which is a stronger lie than "was doing
# what it is doing now".


def _nearest(anchor_ns: np.ndarray, other_ns: np.ndarray):
    """For each anchor stamp, index of the nearest other stamp and the signed dt."""
    i = np.searchsorted(other_ns, anchor_ns)
    i = np.clip(i, 1, len(other_ns) - 1)
    left, right = other_ns[i - 1], other_ns[i]
    use_left = (anchor_ns - left) < (right - anchor_ns)
    idx = np.where(use_left, i - 1, i)
    return idx, anchor_ns - other_ns[idx]


def _lap_index(progress: np.ndarray, track_len_m: float) -> np.ndarray:
    """Count wraparounds of path_s_m. A jump backwards of more than half the
    track is a lap boundary; anything smaller is the tracker's nearest-point
    search hopping, which happens and is not a lap."""
    lap = np.zeros(len(progress), dtype=np.int32)
    for k in range(1, len(progress)):
        lap[k] = lap[k - 1] + (1 if (progress[k - 1] - progress[k]) > track_len_m * 0.5 else 0)
    return lap


def convert(bag_dir: str, out_path: str, manifest_path: str | None) -> dict:
    data = br.read_bag(bag_dir, list(TOPICS.values()))
    scans = data[TOPICS['scan']]
    diags = data[TOPICS['diag']]
    gts = data[TOPICS['gt']]
    events = data[TOPICS['event']]
    report: dict = {'bag': bag_dir}

    if not scans or not diags or not gts or not events:
        raise RuntimeError(f"empty stream(s): scans={len(scans)} diags={len(diags)} "
                           f"gt={len(gts)} events={len(events)}")

    scan_ns = np.array([s for s, _ in scans], dtype=np.int64)
    diag_ns = np.array([s for s, _ in diags], dtype=np.int64)
    gt_ns = np.array([s for s, _ in gts], dtype=np.int64)
    event_by_ns = {s: m for s, m in events}

    # --- valid window: when ALL streams were live -------------------------
    # Scans before the tracker or the monitor started are not "rejects", they
    # are pre-run. Trim them, say how many, and apply the strict rules inside.
    t_lo = max(diag_ns[0], min(event_by_ns), gt_ns[0])
    t_hi = min(diag_ns[-1], max(event_by_ns), gt_ns[-1])
    in_win = (scan_ns >= t_lo) & (scan_ns <= t_hi)
    report['scans_total'] = int(len(scan_ns))
    report['scans_pre_post_run_trimmed'] = int((~in_win).sum())
    scan_ns_w = scan_ns[in_win]
    scans_w = [m for (s, m), ok in zip(scans, in_win) if ok]

    # --- join: diagnostics, nearest within slop ----------------------------
    di, dt = _nearest(scan_ns_w, diag_ns)
    dt_abs = np.abs(dt)
    keep = dt_abs <= spec.SYNC_SLOP_NS
    report['join_dt_ms'] = {
        'median': float(np.median(dt_abs)) / 1e6,
        'p95': float(np.percentile(dt_abs, 95)) / 1e6,
        'max': float(dt_abs.max()) / 1e6,
        'slop': spec.SYNC_SLOP_NS / 1e6,
    }
    report['rows_rejected_slop'] = int((~keep).sum())
    report['rows_kept'] = int(keep.sum())
    if keep.sum() == 0:
        raise RuntimeError(
            f"0 of {len(keep)} scans found an expert command within "
            f"{spec.SYNC_SLOP_NS/1e6:.0f} ms (median |dt| = "
            f"{report['join_dt_ms']['median']:.1f} ms). The clocks are offset or "
            f"the tracker was not publishing -- this is not a short lap.")

    scan_ns_k = scan_ns_w[keep]
    scans_k = [m for m, ok in zip(scans_w, keep) if ok]
    di_k = di[keep]

    # --- join: ground truth, nearest ------------------------------------------
    gi, _ = _nearest(scan_ns_k, gt_ns)

    # --- join: episode event, EXACT ------------------------------------------
    # episode_monitor copies the scan's header.stamp onto the event for exactly
    # this reason. A missing key means the monitor dropped a scan or was not
    # running -- a bug in the data, not a row to skip.
    missing = [int(s) for s in scan_ns_k if int(s) not in event_by_ns]
    if missing:
        raise RuntimeError(
            f"{len(missing)} of {len(scan_ns_k)} scans have NO episode event with an "
            f"exactly matching stamp (first: {missing[0]}). episode_monitor is "
            f"supposed to emit one event per scan, stamped with the scan's own "
            f"stamp. This is a data bug, not a join tolerance problem.")
    report['episode_events_matched_exactly'] = int(len(scan_ns_k))

    # --- assemble rows ----------------------------------------------------
    N = len(scans_k)
    obs = np.empty((N, spec.OBS_DIM), dtype=spec.OBS_DTYPE)
    act = np.empty((N, spec.ACTION_DIM), dtype=spec.ACTION_DTYPE)
    prog = np.empty(N, dtype=np.float32)
    laterr = np.empty(N, dtype=np.float32)
    gt = np.empty((N, 3), dtype=np.float32)
    term = np.zeros(N, dtype=np.bool_)
    trig = np.zeros(N, dtype=np.uint8)
    controller = None
    for k, (s_ns, m) in enumerate(zip(scan_ns_k, scans_k)):
        obs[k] = spec.scan_to_observation(np.asarray(br.scan_ranges(m), dtype=np.float64))
        d = br.diag_fields(diags[di_k[k]][1])
        act[k] = (d['steering'], d['speed'])
        prog[k] = d['progress']
        laterr[k] = d['lateral_error']
        controller = controller or d['controller']
        gt[k] = br.odom_pose(gts[gi[k]][1])
        ev = br.event_fields(event_by_ns[int(s_ns)])
        term[k] = ev['terminal']
        trig[k] = ev['trigger']

    # rewards: zeros until a later phase defines them
    rew = np.zeros(N, dtype=np.float32)

    # episodes: a terminal ends one; the recording end truncates the last
    ep = np.zeros(N, dtype=np.int32)
    for k in range(1, N):
        ep[k] = ep[k - 1] + (1 if term[k - 1] else 0)
    timeouts = np.zeros(N, dtype=np.bool_)
    if not term[-1]:
        timeouts[-1] = True           # recording stopped, world did not end

    prev = act.copy()
    for k in range(1, N):
        if ep[k] == ep[k - 1]:
            prev[k] = act[k - 1]
    # first row of every episode keeps its own action (see note at top)

    # --- manifest / attrs -------------------------------------------------
    manifest = {}
    if manifest_path and os.path.exists(manifest_path):
        with open(manifest_path) as f:
            manifest = yaml.safe_load(f) or {}
    track_len = None
    tf = manifest.get('track_file')
    if tf and os.path.exists(tf):
        pts = [tuple(float(v) for v in l.split(',')[:2]) for l in open(tf)
               if l.strip() and not l.startswith('#')]
        track_len = float(sum(np.hypot(pts[(i+1) % len(pts)][0]-pts[i][0],
                                       pts[(i+1) % len(pts)][1]-pts[i][1])
                              for i in range(len(pts))))
    lap = _lap_index(prog, track_len) if track_len else np.zeros(N, dtype=np.int32)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with h5py.File(out_path, 'w') as h:
        h.create_dataset('observations', data=obs, compression='gzip', chunks=(min(N, 1024), spec.OBS_DIM))
        h.create_dataset('actions', data=act)
        h.create_dataset('rewards', data=rew)
        h.create_dataset('terminals', data=term)
        h.create_dataset('timeouts', data=timeouts)
        g = h.create_group('infos')
        g.create_dataset('timestamp_ns', data=scan_ns_k.astype(np.int64))
        g.create_dataset('gt_pose', data=gt)
        g.create_dataset('progress_s_m', data=prog)
        g.create_dataset('lap_index', data=lap)
        g.create_dataset('episode_id', data=ep)
        g.create_dataset('prev_action', data=prev)
        g.create_dataset('dagger_beta', data=np.full(N, 1.0, dtype=np.float32))
        g.create_dataset('action_source', data=np.full(N, spec.ACTION_SOURCE_EXPERT, dtype=np.uint8))
        g.create_dataset('lateral_error_m', data=laterr)        # diagnostic, not a feature
        g.create_dataset('terminal_trigger', data=trig)         # which signal fired

        a = h.attrs
        a['schema_version'] = spec.SCHEMA_VERSION
        a['track_id'] = str(manifest.get('track_id', 'unknown'))
        a['track_sha256'] = str(manifest.get('track_sha256', ''))
        a['track_scale'] = 2.0
        a['controller_label'] = str(controller or manifest.get('controller_label', 'unknown'))
        a['max_speed_mps'] = float(manifest.get('max_speed_mps', float('nan')))
        a['action_bounds'] = spec.ACTION_BOUNDS
        a['downsample_indices'] = spec.DOWNSAMPLE_IDX     # auditable FROM THE DATA
        a['dataset_hz'] = spec.DATASET_HZ
        a['range_clip'] = np.array([spec.RANGE_MIN, spec.RANGE_MAX], dtype=np.float32)
        a['scan_contact_m'] = spec.SCAN_CONTACT_M
        a['git_rev'] = str(manifest.get('git_rev', ''))
        a['git_dirty'] = bool(manifest.get('git_dirty', True))
        a['converter_report'] = json.dumps(report)

    report['rows'] = N
    report['out'] = out_path
    return report


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('bag_dir')
    p.add_argument('out_h5')
    p.add_argument('--manifest', default=None,
                   help='run_manifest.yaml (default: <bag_dir>/../run_manifest.yaml)')
    a = p.parse_args(argv)
    man = a.manifest or os.path.join(os.path.dirname(os.path.abspath(a.bag_dir)), 'run_manifest.yaml')
    rep = convert(a.bag_dir, a.out_h5, man)
    print(json.dumps(rep, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
