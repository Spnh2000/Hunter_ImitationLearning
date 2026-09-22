#!/usr/bin/env python3
"""Part E: closed-loop evaluation in Gazebo on the HELD-OUT tracks.

    evaluate_policies --plan src/hunter_dataset/config/collection_plan.yaml \
                      --out results/eval_v1 --laps 2 \
                      pid checkpoints/bc_mlp_s0.pt checkpoints/bc_mlp_s1.pt ...

Each (policy, track) is one run of record_expert.launch.py with the policy
in the tracker slot, no shadow, `laps` laps or the first terminal, whichever
comes first. `pid` as a policy name is the expert itself -- the reference row.
Ground truth is the yardstick everywhere: metrics_logger's summary.csv
(gt_lateral_error, laps_completed from ground truth for NaN-progress trackers)
plus the bag's episode events for the crash. One row per run in
<out>/eval.csv; seeds are whatever checkpoints you pass (train N seeds, pass
them all); the report aggregates mean and std per (policy, track).

Validation loss is not consulted here. That table comes from il_eval_offline
and is placed BESIDE this one; where they disagree is the finding.
"""
import argparse
import csv
import glob
import json
import os
import shutil
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hunter_dataset.collect_dataset import (REPO, _child_env, _resolve, _spin_for, _teardown, _wait_empty,
                                            _write)
from hunter_msgs.msg import EpisodeEvent, Trajectory
from nav_msgs.msg import Odometry

FIELDS = ['policy', 'arch', 'dataset', 'seed', 'track', 'max_speed_mps', 'status', 'crashed', 'crash_s_m',
          'crash_trigger', 'laps_completed', 'distance_m', 'sim_duration_s', 'mean_speed_mps',
          'rms_gt_lateral_error_m', 'max_gt_lateral_error_m', 'mean_gt_lateral_error_m',
          'steering_cmd_variance', 'steering_reversal_rate_hz', 'wall_s', 'run_dir']


class EvalNode(Node):
    """Progress from GROUND TRUTH projected on the path (the student reports
    none); terminal from episode_monitor."""

    def __init__(self):
        super().__init__('evaluate_policies_runner')
        self.reset()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Trajectory, '/trajectory/reference', self._on_traj, latched)
        self.create_subscription(Odometry, '/ground_truth/odometry', self._on_gt, 10)
        self.create_subscription(EpisodeEvent, '/episode/event', self._on_event, 10)

    def reset(self):
        self.wps = None
        self.s_of = None
        self.track_len = None
        self.idx = 0
        self.dist_m = 0.0
        self.prev_s = None
        self.n_gt = 0
        self.sim_t = None
        self.sim_t0 = None
        self.terminal = None
        self.speeds = []

    def _on_traj(self, m):
        self.wps = [(w.x_m, w.y_m) for w in m.waypoints]
        self.s_of = [float(w.s_m) for w in m.waypoints]
        self.track_len = float(m.total_length_m)

    def _on_gt(self, m):
        self.n_gt += 1
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.sim_t0 is None:
            self.sim_t0 = self.sim_t
        self.speeds.append(abs(m.twist.twist.linear.x))
        if not self.wps:
            return
        x, y = m.pose.pose.position.x, m.pose.pose.position.y
        n = len(self.wps)
        best, bd = self.idx, float('inf')
        for k in range(-150, 151):                     # windowed, like the trackers
            i = (self.idx + k) % n
            d = (self.wps[i][0] - x) ** 2 + (self.wps[i][1] - y) ** 2
            if d < bd:
                bd, best = d, i
        self.idx = best
        s = self.s_of[best]
        if self.prev_s is not None:
            ds = s - self.prev_s
            if ds < -0.5 * self.track_len:
                ds += self.track_len
            if 0 < ds < 5.0:
                self.dist_m += ds
        self.prev_s = s

    def _on_event(self, m):
        if m.terminal and self.terminal is None:
            self.terminal = (self.sim_t, int(m.trigger), float(m.min_scan_m), self.prev_s)


def _policy_meta(name):
    if name == 'pid':
        return {'policy': 'pid', 'arch': 'pid', 'dataset': 'expert', 'seed': 0, 'ckpt': ''}
    import torch
    ck = torch.load(name, map_location='cpu', weights_only=False)
    tc = ck.get('train_config', {})
    tag = os.path.splitext(os.path.basename(name))[0]
    return {'policy': tag, 'arch': ck.get('arch'), 'dataset': tc.get('dataset_tag', tc.get('data_dir', '')),
            'seed': tc.get('seed', 0), 'ckpt': os.path.abspath(name)}


def run(node, meta, track, tcfg, speed, laps, out_root, log, max_wall_s, attempts=2):
    # ros2_control startup race (see collect_dataset.RETRY_STATUSES): one retry.
    for attempt in range(1, attempts + 1):
        r = _run_once(node, meta, track, tcfg, speed, laps, out_root, log, max_wall_s)
        if r.get('status') != 'no_start' or attempt == attempts:
            return r
        log(f'  retry {attempt}/{attempts - 1}: no_start -- fresh launch')
        shutil.rmtree(os.path.join(out_root, f'{track}_{speed}ms_eval_{meta["policy"]}'), ignore_errors=True)
    return r


def _run_once(node, meta, track, tcfg, speed, laps, out_root, log, max_wall_s):
    files = _resolve(track, tcfg)
    run_id = f'eval_{meta["policy"]}'
    run_dir = os.path.join(out_root, f'{track}_{speed}ms_{run_id}')
    rep_path = os.path.join(run_dir, 'eval_report.json')
    if os.path.exists(rep_path):
        prev = json.load(open(rep_path))
        if prev.get('status') in ('ok', 'crashed'):
            log(f'  skip {meta["policy"]} on {track}: {prev["status"]}')
            return prev
        shutil.rmtree(run_dir, ignore_errors=True)
    row = {k: '' for k in FIELDS}
    row.update(policy=meta['policy'], arch=meta['arch'], dataset=meta['dataset'], seed=meta['seed'],
               track=track, max_speed_mps=speed, run_dir=run_dir, status='started')
    ok, nodes, models = _wait_empty(node.get_name(), 5)
    if not ok:
        row['status'] = 'preflight_dirty'
        log(f'  ABORT preflight: nodes={nodes} gz={models}')
        return row
    os.makedirs(run_dir, exist_ok=True)
    cmd = ['ros2', 'launch', 'hunter_dataset', 'record_expert.launch.py',
           f'track_name:={track}', f'track_file:={files["track_file"]}',
           f'params_file:={files["params"]}', f'world:={files["world"]}',
           f'max_speed_mps:={speed}', f'run_id:={run_id}', f'output_dir:={out_root}',
           'headless:=true', 'use_rviz:=false']
    if meta['policy'] != 'pid':
        cmd += ['controller:=policy', f'policy_checkpoint:={meta["ckpt"]}', f'policy_arch:={meta["arch"]}']
    launch_log = open(os.path.join(run_dir, 'launch.log'), 'w')
    # Drain the subscription queues before resetting: the previous run's last
    # messages (up to the QoS depth) can still be sitting in the reader, and
    # delivered after reset() they set sim_t0 to the OLD run's clock -- seen
    # as a negative sim_duration in p1's reports. The HDF5 is unaffected.
    _spin_for(node, 1.0)
    node.reset()
    t0 = time.time()
    proc = subprocess.Popen(cmd, cwd=REPO, env=_child_env(), stdout=launch_log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    pgid = os.getpgid(proc.pid)
    log(f'  {meta["policy"]} on {track} @ {speed}: launch pid {proc.pid}')
    while node.n_gt < 20 or node.wps is None:
        _spin_for(node, 1.0)
        if proc.poll() is not None or time.time() - t0 > 180:
            row['status'] = 'no_start'
            _teardown(proc, pgid, node, log, {})
            _write(rep_path, row)
            return row
    target = laps * node.track_len
    stall = (node.sim_t, node.dist_m)
    last = time.time()
    while node.dist_m < target:
        _spin_for(node, 0.5)
        if node.terminal is not None:
            row['status'] = 'crashed'
            break
        if proc.poll() is not None:
            row['status'] = 'launch_died'
            break
        if node.sim_t - stall[0] > 30.0:
            if node.dist_m - stall[1] < 1.0:
                row['status'] = 'stuck'          # a policy that stops is a failure too
                break
            stall = (node.sim_t, node.dist_m)
        if time.time() - t0 > max_wall_s:
            row['status'] = 'wall_timeout'
            break
        if time.time() - last > 60:
            log(f'    {node.dist_m:6.0f}/{target:.0f} m  sim {node.sim_t - node.sim_t0:.0f}s')
            last = time.time()
    if row['status'] == 'started':
        row['status'] = 'ok'
    _spin_for(node, 1.0)
    row.update(crashed=int(node.terminal is not None),
               crash_s_m=round(node.terminal[3], 1) if node.terminal else '',
               crash_trigger=node.terminal[1] if node.terminal else '',
               distance_m=round(node.dist_m, 1),
               sim_duration_s=round((node.sim_t or 0) - (node.sim_t0 or 0), 1),
               mean_speed_mps=round(sum(node.speeds) / max(len(node.speeds), 1), 3),
               laps_completed=int(node.dist_m // node.track_len) if node.track_len else 0)
    _teardown(proc, pgid, node, log, {})
    row['wall_s'] = round(time.time() - t0, 1)
    # metrics_logger's summary (ground-truth referenced) -- newest results dir
    dirs = sorted(glob.glob(os.path.join(REPO, 'src/hunter_benchmark/results', f'{track}_*')), key=os.path.getmtime)
    if dirs and os.path.exists(os.path.join(dirs[-1], 'summary.csv')):
        s = list(csv.DictReader(open(os.path.join(dirs[-1], 'summary.csv'))))[-1]
        for k in ('rms_gt_lateral_error_m', 'max_gt_lateral_error_m', 'mean_gt_lateral_error_m',
                  'steering_cmd_variance', 'steering_reversal_rate_hz'):
            row[k] = s.get(k, '')
        shutil.copy(os.path.join(dirs[-1], 'summary.csv'), os.path.join(run_dir, 'summary.csv'))
    log(f'  {row["status"]}: {row["distance_m"]} m, laps {row["laps_completed"]}, '
        f'gt rms {row["rms_gt_lateral_error_m"]} max {row["max_gt_lateral_error_m"]}, mean v {row["mean_speed_mps"]}')
    _write(rep_path, row)
    return row


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--laps', type=float, default=2.0)
    ap.add_argument('--speed', type=float, default=2.7, help='max_speed_mps clamp / expert cap')
    ap.add_argument('--tracks', default='', help='default: the plan\'s held-out tracks')
    ap.add_argument('--max-wall-s', type=float, default=3600)
    ap.add_argument('policies', nargs='+', help="'pid' and/or checkpoint paths")
    a = ap.parse_args()
    plan = yaml.safe_load(open(a.plan))
    tracks = (a.tracks.split(',') if a.tracks else
              [t for t, c in plan['tracks'].items() if c['role'] == 'heldout'])
    out_root = os.path.abspath(a.out)
    os.makedirs(out_root, exist_ok=True)
    log_path = os.path.join(out_root, 'eval.log')

    def log(msg):
        line = f'[{time.strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        open(log_path, 'a').write(line + '\n')

    metas = [_policy_meta(p) for p in a.policies]
    log(f'eval: {len(metas)} policies x {tracks}, {a.laps} laps, cap {a.speed} m/s')
    rclpy.init()
    node = EvalNode()
    rows = []
    try:
        for m in metas:
            for t in tracks:
                r = run(node, m, t, plan['tracks'][t], a.speed, a.laps, out_root, log, a.max_wall_s)
                rows.append(r)
                if r.get('status') == 'preflight_dirty':
                    return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    with open(os.path.join(out_root, 'eval.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})
    # aggregate per (arch, dataset, track) over seeds
    import statistics as st
    groups = {}
    for r in rows:
        groups.setdefault((r['arch'], r['dataset'], r['track']), []).append(r)
    log('=== per (arch, dataset, track) over seeds: crash rate, laps, gt rms lateral (mean +- std) ===')
    for (arch, ds, tr), rs in sorted(groups.items()):
        rms = [float(x['rms_gt_lateral_error_m']) for x in rs if x.get('rms_gt_lateral_error_m') not in ('', None)]
        laps = [float(x['laps_completed']) for x in rs if x.get('laps_completed') != '']
        cr = [float(x['crashed']) for x in rs if x.get('crashed') != '']
        f = lambda v: f'{st.mean(v):.3f} +- {st.stdev(v) if len(v) > 1 else 0:.3f}' if v else 'n/a'
        log(f'  {arch:9s} {str(ds)[:12]:12s} {tr:13s} n={len(rs)}  crash {f(cr)}  laps {f(laps)}  gt_rms {f(rms)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
