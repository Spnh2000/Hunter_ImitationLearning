#!/usr/bin/env python3
"""DAgger loop (Phase 8, axis 2). Iteration 0 is behaviour cloning; every
later iteration rolls the CURRENT student out with beta = 0 while pid_shadow
relabels every state, then retrains on everything collected so far.

    dagger_runner --plan src/hunter_dataset/config/collection_plan.yaml \
                  --train-config src/hunter_il/config/train_default.yaml \
                  --arch mlp --iterations 6 --data-dir datasets/raw

Per iteration i >= 1:
    train      il_train on data-dir (BC passes + dagger_it<i runs)
                 -> checkpoints/dagger_<arch>_it<i>.pt
    rollout    for each TRAINING track x speed: record_expert.launch.py with
                 controller:=policy shadow_expert:=true, up to `laps` laps.
                 On a terminal: keep the approach (it is the point), respawn
                 on the line `respawn_back_m` before the crash, re-arm the
                 monitor, clear the student's frame stack, carry on.
    convert    convert_dataset picks DAgger mode up from the bag itself
                 (/expert/relabel present) -> action_source = 1, beta = 0.

Crash states are what DAgger is for. Held-out tracks are never rolled out.

Process ownership, preflight and teardown are collect_dataset's.
"""
import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from hunter_dataset.collect_dataset import (REPO, _child_env, _resolve, _sh, _spin_for, _teardown,
                                            _wait_empty, _write)
from hunter_msgs.msg import EpisodeEvent, TrackingDiagnostics, Trajectory

WORLD = 'track_world'
MODEL = 'hunter'


def _centerline(track_file):
    pts = []
    with open(os.path.join(REPO, track_file)) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('#'):
                v = line.split(',')
                pts.append((float(v[0]), float(v[1])))
    seg = [math.hypot(pts[(i + 1) % len(pts)][0] - pts[i][0], pts[(i + 1) % len(pts)][1] - pts[i][1])
           for i in range(len(pts))]
    cum = [0.0]
    for d in seg:
        cum.append(cum[-1] + d)
    return pts, cum


def _pose_at_s(pts, cum, s):
    L = cum[-1]
    s = s % L
    for i in range(len(pts)):
        if cum[i] <= s < cum[i + 1]:
            a, b = pts[i], pts[(i + 1) % len(pts)]
            t = (s - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
            return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), math.atan2(b[1] - a[1], b[0] - a[0])
    return pts[0][0], pts[0][1], 0.0


def _gz_set_pose(x, y, yaw, z=0.3):
    req = (f'name: "{MODEL}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}')
    out = _sh(f'gz service -s /world/{WORLD}/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean '
              f"--timeout 3000 --req '{req}'", timeout=10)
    return 'true' in out.lower()


class DaggerNode(Node):
    """Progress from the SHADOW (the student reports NaN progress), terminals
    from episode_monitor."""

    def __init__(self):
        super().__init__('dagger_runner')
        self.reset()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Trajectory, '/trajectory/reference', self._on_traj, latched)
        self.create_subscription(TrackingDiagnostics, '/expert/relabel', self._on_relabel, 10)
        self.create_subscription(TrackingDiagnostics, '/tracking/diagnostics', self._on_student, 10)
        self.create_subscription(EpisodeEvent, '/episode/event', self._on_event, 10)
        self.reset_monitor = self.create_client(Trigger, '/episode/reset')
        self.reset_stack = self.create_client(Trigger, '/policy_tracker/reset_stack')

    def reset(self):
        self.track_len = None
        self.prev_s = None
        self.s = None
        self.dist_m = 0.0
        self.laps = 0
        self.n_relabel = 0
        self.n_student = 0
        self.sim_t = None
        self.sim_t0 = None
        self.terminal_pending = None
        self.lat_abs = []
        self.student_speeds = []

    def _on_traj(self, m):
        self.track_len = float(m.total_length_m)

    def _on_relabel(self, m):
        s = float(m.path_s_m)
        self.n_relabel += 1
        self.sim_t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.sim_t0 is None:
            self.sim_t0 = self.sim_t
        if self.prev_s is not None and self.track_len:
            ds = s - self.prev_s
            if ds < -0.5 * self.track_len:
                ds += self.track_len
                self.laps += 1
            if 0 < ds < 5.0:            # a respawn jump is not progress
                self.dist_m += ds
        self.prev_s = self.s = s
        self.lat_abs.append(abs(float(m.lateral_error_m)))

    def _on_student(self, m):
        self.n_student += 1
        self.student_speeds.append(float(m.commanded_speed_mps))

    def _on_event(self, m):
        if m.terminal and self.terminal_pending is None:
            self.terminal_pending = (self.sim_t, int(m.trigger), float(m.min_scan_m), self.s)

    def call(self, client, what, log):
        if not client.wait_for_service(timeout_sec=5.0):
            log(f'    service {what} unavailable')
            return False
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return bool(fut.done() and fut.result() and fut.result().success)


def rollout(node, track, tcfg, speed, it, ckpt, arch, laps, out_root, log, respawn_back_m,
            max_crashes, settle_s, attempts=2):
    # ros2_control startup race (see collect_dataset.RETRY_STATUSES): one retry.
    for attempt in range(1, attempts + 1):
        r = _rollout_once(node, track, tcfg, speed, it, ckpt, arch, laps, out_root, log,
                          respawn_back_m, max_crashes, settle_s)
        if r.get('status') != 'no_start' or attempt == attempts:
            return r
        log(f'  retry {attempt}/{attempts - 1}: no_start -- fresh launch')
        shutil.rmtree(os.path.join(out_root, f'{track}_{speed}ms_dagger_it{it}'), ignore_errors=True)
    return r


def _rollout_once(node, track, tcfg, speed, it, ckpt, arch, laps, out_root, log, respawn_back_m,
                  max_crashes, settle_s):
    files = _resolve(track, tcfg)
    run_id = f'dagger_it{it}'
    run_dir = os.path.join(out_root, f'{track}_{speed}ms_{run_id}')
    report_path = os.path.join(run_dir, 'collection_report.json')
    if os.path.exists(report_path):
        prev = json.load(open(report_path))
        if prev.get('status') == 'ok':
            log(f'  skip {track} @ {speed} it{it}: already ok ({prev["rows"]} rows)')
            return prev
        shutil.rmtree(run_dir, ignore_errors=True)
    pts, cum = _centerline(files['track_file'])
    report = {'track': track, 'speed_mps': speed, 'pass': run_id, 'dagger_iteration': it,
              'checkpoint': ckpt, 'laps_target': laps, 'status': 'started', 'crashes': [],
              'wall_started': time.strftime('%Y-%m-%dT%H:%M:%S')}
    ok, nodes, models = _wait_empty(node.get_name(), 5)
    if not ok:
        report.update(status='preflight_dirty', ros_nodes=nodes, gz_models=models)
        log(f'  ABORT preflight: nodes={nodes} gz={models}')
        return report
    os.makedirs(run_dir, exist_ok=True)
    cmd = ['ros2', 'launch', 'hunter_dataset', 'record_expert.launch.py',
           f'track_name:={track}', f'track_file:={files["track_file"]}',
           f'params_file:={files["params"]}', f'world:={files["world"]}',
           f'max_speed_mps:={speed}', f'run_id:={run_id}', f'output_dir:={out_root}',
           'controller:=policy', 'shadow_expert:=true',
           f'policy_checkpoint:={os.path.abspath(ckpt)}', f'policy_arch:={arch}',
           f'dagger_iteration:={it}', 'headless:=true', 'use_rviz:=false']
    launch_log = open(os.path.join(run_dir, 'launch.log'), 'w')
    # Drain the subscription queues before resetting: the previous run's last
    # messages (up to the QoS depth) can still be sitting in the reader, and
    # delivered after reset() they set sim_t0 to the OLD run's clock -- seen
    # as a negative sim_duration in p1's reports. The HDF5 is unaffected.
    _spin_for(node, 1.0)
    node.reset()
    t_wall0 = time.time()
    proc = subprocess.Popen(cmd, cwd=REPO, env=_child_env(), stdout=launch_log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    pgid = os.getpgid(proc.pid)
    log(f'  launch pid {proc.pid}: {track} @ {speed} m/s it{it}, {laps} laps, student={os.path.basename(ckpt)}')

    t0 = time.time()
    while node.n_relabel < 20 or node.n_student < 5 or node.track_len is None:
        _spin_for(node, 1.0)
        if proc.poll() is not None or time.time() - t0 > 180:
            report.update(status='no_start', relabels=node.n_relabel, student_msgs=node.n_student)
            log(f'  ABORT: shadow={node.n_relabel} student={node.n_student} msgs after startup')
            _teardown(proc, pgid, node, log, report)
            _write(report_path, report)
            return report
    target_m = laps * node.track_len
    # The student must be capped at THIS run's speed. If its commands exceed the
    # cap the parameter never reached the node, and the whole rollout would be
    # collected at the wrong speed with correctly-capped labels -- the exact
    # silent failure that cost the first DAgger run. Checked here, loudly.
    smax = max(node.student_speeds) if node.student_speeds else 0.0
    if smax > speed + 0.05:
        report.update(status='student_ignores_cap', student_max_speed=round(smax, 3))
        log(f'  ABORT: student commanded {smax:.2f} m/s with cap {speed} -- parameter not reaching the node')
        _teardown(proc, pgid, node, log, report)
        _write(report_path, report)
        return report
    log(f'    driving; shadow and student both live; target {target_m:.0f} m; '
        f'student max cmd {smax:.2f} <= cap {speed}')

    status = 'ok'
    stall_ref = (node.sim_t, node.dist_m)
    last_print = time.time()
    while node.dist_m < target_m:
        _spin_for(node, 0.5)
        if proc.poll() is not None:
            status = 'launch_died'
            break
        if node.terminal_pending is not None:
            t_ev, trig, min_scan, s_crash = node.terminal_pending
            crash = {'sim_t': round((t_ev or 0) - (node.sim_t0 or 0), 1), 'trigger': trig,
                     'min_scan_m': round(min_scan, 3), 's_m': round(s_crash or 0, 1),
                     'dist_m': round(node.dist_m, 1)}
            report['crashes'].append(crash)
            log(f'    CRASH #{len(report["crashes"])} at s={crash["s_m"]} m (trigger {trig}, '
                f'min_scan {min_scan:.2f} m) after {node.dist_m:.0f} m')
            if len(report['crashes']) > max_crashes:
                status = 'too_many_crashes'
                break
            # keep the approach: the bag already has it. Now put the robot
            # back on the line before the crash and re-arm.
            _spin_for(node, settle_s)                       # let the last rows land
            x, y, yaw = _pose_at_s(pts, cum, (s_crash or 0) - respawn_back_m)
            if not _gz_set_pose(x, y, yaw):
                log('    set_pose FAILED')
                status = 'respawn_failed'
                break
            _spin_for(node, settle_s)                       # physics settles
            node.call(node.reset_stack, 'policy_tracker/reset_stack', log)
            node.call(node.reset_monitor, 'episode/reset', log)
            node.terminal_pending = None
            node.prev_s = None                              # next ds is the jump
            stall_ref = (node.sim_t, node.dist_m)
            continue
        st, sd = stall_ref
        if node.sim_t is not None and st is not None and node.sim_t - st > 30.0:
            if node.dist_m - sd < 1.0:
                status = 'stuck'
                log(f'    STUCK at s={node.s} (no terminal fired: student stopped?)')
                break
            stall_ref = (node.sim_t, node.dist_m)
        if time.time() - last_print > 60:
            wall = time.time() - t_wall0
            log(f'    {node.dist_m:6.0f}/{target_m:.0f} m  crashes {len(report["crashes"])}  '
                f'|lat| median {sorted(node.lat_abs)[len(node.lat_abs) // 2] if node.lat_abs else 0:.3f} m  '
                f'wall {wall:.0f}s')
            last_print = time.time()
    _spin_for(node, 2.0)
    la = sorted(node.lat_abs)
    report.update(status=status, distance_m=round(node.dist_m, 1), laps_wrapped=node.laps,
                  sim_duration_s=round((node.sim_t or 0) - (node.sim_t0 or 0), 1),
                  lateral_abs_median_m=round(la[len(la) // 2], 4) if la else None,
                  lateral_abs_p95_m=round(la[int(len(la) * 0.95)], 4) if la else None,
                  n_crashes=len(report['crashes']))
    _teardown(proc, pgid, node, log, report)
    report['wall_duration_s'] = round(time.time() - t_wall0, 1)

    bag_dir = os.path.join(run_dir, 'bag')
    if not os.path.exists(os.path.join(bag_dir, 'metadata.yaml')):
        report['status'] = f'{status}+bag_not_finalised'
        _write(report_path, report)
        return report
    h5 = os.path.join(run_dir, 'dataset.hdf5')
    conv = subprocess.run(['ros2', 'run', 'hunter_dataset', 'convert_dataset', bag_dir, h5,
                           '--manifest', os.path.join(run_dir, 'run_manifest.yaml')],
                          cwd=REPO, env=_child_env(), capture_output=True, text=True)
    open(os.path.join(run_dir, 'convert.log'), 'w').write(conv.stdout + conv.stderr)
    if conv.returncode != 0:
        report['status'] = f'{status}+convert_failed'
        log(f'  CONVERT FAILED; see convert.log')
        _write(report_path, report)
        return report
    try:
        rep = json.loads(conv.stdout[conv.stdout.index('{'):])
        report.update(rows=rep.get('rows'), mode=rep.get('mode'), join_dt_ms=rep.get('join_dt_ms'),
                      relabel_dt_ms=rep.get('relabel_dt_ms'))
    except (ValueError, json.JSONDecodeError):
        report['rows'] = None
    insp_cmd = ['ros2', 'run', 'hunter_dataset', 'inspect_dataset', h5]
    if tcfg.get('role') == 'ims':
        insp_cmd += ['--expect-constant', 'speed_mps']
    insp = subprocess.run(insp_cmd, cwd=REPO, env=_child_env(), capture_output=True, text=True)
    open(os.path.join(run_dir, 'inspect.txt'), 'w').write(insp.stdout + insp.stderr)
    report['inspect_exit'] = insp.returncode
    report['inspect_fails'] = [l.strip() for l in insp.stdout.splitlines() if '[FAIL]' in l]
    if insp.returncode != 0 and status == 'ok':
        report['status'] = 'inspect_failed'
    if report.get('mode') != 'dagger' and status == 'ok':
        report['status'] = 'not_dagger_mode'          # the shadow's relabel never made it to the bag
    log(f'  {report["status"]}: {report.get("rows")} rows ({report.get("mode")}), {len(report["crashes"])} crashes, '
        f'|lat| median {report.get("lateral_abs_median_m")} p95 {report.get("lateral_abs_p95_m")}')
    _write(report_path, report)
    return report


def train(cfg_path, arch, data_dir, out, seed, log):
    cmd = ['ros2', 'run', 'hunter_il', 'il_train', '--config', cfg_path, '--arch', arch,
           '--data-dir', data_dir, '--out', out, '--seed', str(seed)]
    log(f'  train: {" ".join(cmd[3:])}')
    r = subprocess.run(cmd, cwd=REPO, env=_child_env(), capture_output=True, text=True)
    open(out + '.train.log', 'w').write(r.stdout + r.stderr)
    if r.returncode != 0:
        raise SystemExit(f'training failed; see {out}.train.log')
    last = [l for l in r.stdout.splitlines() if l.startswith('saved')]
    log('  ' + (last[-1][:200] if last else 'trained'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--train-config', required=True)
    ap.add_argument('--arch', default='mlp')
    ap.add_argument('--iterations', type=int, default=6)
    ap.add_argument('--start-iter', type=int, default=1)
    ap.add_argument('--data-dir', default='datasets/raw')
    ap.add_argument('--ckpt-dir', default='checkpoints')
    ap.add_argument('--laps', type=float, default=1.5)
    ap.add_argument('--respawn-back-m', type=float, default=10.0)
    ap.add_argument('--max-crashes', type=int, default=8)
    ap.add_argument('--settle-s', type=float, default=1.0)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--tracks', default='')
    ap.add_argument('--speeds', default='')
    ap.add_argument('--skip-train', action='store_true', help='reuse existing checkpoint for the iteration')
    a = ap.parse_args()

    plan = yaml.safe_load(open(a.plan))
    tracks = [t for t, c in plan['tracks'].items() if c['role'] in ('train', 'ims')
              and (not a.tracks or t in a.tracks.split(','))]
    speeds = [s for s in plan['speeds_mps'] if not a.speeds or str(s) in a.speeds.split(',')]
    out_root = os.path.abspath(a.data_dir)
    os.makedirs(a.ckpt_dir, exist_ok=True)
    os.makedirs(out_root, exist_ok=True)
    log_path = os.path.join(out_root, f'dagger_{a.arch}.log')

    def log(msg):
        line = f'[{time.strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        open(log_path, 'a').write(line + '\n')

    log(f'DAgger {a.arch}: iterations {a.start_iter}..{a.iterations}, tracks {tracks}, speeds {speeds}')
    rclpy.init()
    node = DaggerNode()
    try:
        for it in range(a.start_iter, a.iterations + 1):
            ckpt = os.path.join(a.ckpt_dir, f'dagger_{a.arch}_it{it}.pt')
            log(f'=== iteration {it}: train on everything so far, then roll out ===')
            if not (a.skip_train and os.path.exists(ckpt)):
                train(a.train_config, a.arch, a.data_dir, ckpt, a.seed, log)
            for t in tracks:
                # IMS is an oval: geometrically repetitive, and the first DAgger
                # run showed the student holding a steady ~0.44 m wide line
                # there for a whole lap, so two thirds of its rows carried the
                # expert's full-lock recovery label with the SAME sign. 66k such
                # rows (13 % of the aggregate) is not what "IMS is capped at
                # 1-2 laps" meant. Half a lap still covers both straights and
                # both turn types.
                laps = a.laps if plan['tracks'][t]['role'] != 'ims' else min(a.laps, 0.5)
                for s in speeds:
                    r = rollout(node, t, plan['tracks'][t], s, it, ckpt, a.arch, laps, out_root, log,
                                a.respawn_back_m, a.max_crashes, a.settle_s)
                    if r.get('status') == 'preflight_dirty':
                        log('stopping: the graph is not ours to clean')
                        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
