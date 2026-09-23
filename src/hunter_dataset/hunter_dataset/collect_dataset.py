#!/usr/bin/env python3
"""Phase 6 collection runner: drives the plan in collection_plan.yaml.

For every (track, speed) in a pass:

    preflight   ROS graph and gz daemon must be EMPTY (no stale robot)
    launch      record_expert.launch.py in its own process group
    drive       count laps from /tracking/diagnostics path_s_m; watchdogs
    stop        SIGINT the recorder and WAIT for it to finalise the mcap,
                then SIGINT the launch and WAIT, then SIGKILL the group
    verify      graph empty again -- a leftover node contaminates the next run
    convert     bag -> HDF5, then inspect_dataset (nonzero exit = FAIL)
    report      <run_dir>/collection_report.json; runs with status ok are
                skipped on re-invocation, so the runner is resumable.

Process ownership is by PID and process group, never by name: a Python
node's comm is "python3", and three metrics_logger zombies from three runs
were once found alive at the same time because a comm-based kill loop never
touched them.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from hunter_msgs.msg import EpisodeEvent, TrackingDiagnostics, Trajectory

REPO = os.environ.get('HUNTER_REPO', os.getcwd())

# --- environment traps (all verified the hard way) --------------------------
# conda's python shadows /usr/bin/python3 and lacks rclpy; SNAP/GTK vars from a
# VS Code terminal crash Gazebo; GZ_IP keeps gz-transport on loopback; the
# plugin path is what lets gz find ros_gz systems.
def _child_env():
    env = dict(os.environ)
    for k in list(env):
        if k.startswith(('SNAP', 'GTK_', 'GDK_')) or k in ('CONDA_PREFIX', 'CONDA_DEFAULT_ENV', 'CONDA_PYTHON_EXE'):
            env.pop(k, None)
    env['PATH'] = '/usr/bin:' + env.get('PATH', '')
    env['GZ_IP'] = '127.0.0.1'
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = env.get('LD_LIBRARY_PATH', '')
    return env


def _sh(cmd, timeout=30):
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                           timeout=timeout, env=_child_env())
        return r.stdout
    except subprocess.TimeoutExpired:
        return ''


def _ros_nodes(exclude):
    # --no-daemon: a fresh discovery instead of the daemon's cache, which can
    # both hold dead nodes and miss live ones for several seconds.
    out = _sh('ros2 node list --no-daemon --spin-time 3', timeout=25)
    return sorted(n for n in out.split() if n and n.lstrip('/') != exclude.lstrip('/'))


def _gz_models():
    return [t for t in _sh('gz topic -l', timeout=15).split() if 'model/hunter' in t]


def _wait_empty(exclude, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        nodes, models = _ros_nodes(exclude), _gz_models()
        if not nodes and not models:
            return True, [], []
        time.sleep(3)
    return False, nodes, models


def _wait_exit(pid, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        # reap if it is our child
        try:
            if os.waitpid(pid, os.WNOHANG)[0] == pid:
                return True
        except ChildProcessError:
            return True
        time.sleep(0.5)
    return False


class Runner(Node):
    def __init__(self):
        super().__init__('collect_dataset_runner')
        self.reset()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Trajectory, '/trajectory/reference', self._on_traj, latched)
        self.create_subscription(TrackingDiagnostics, '/tracking/diagnostics', self._on_diag, 10)
        self.create_subscription(EpisodeEvent, '/episode/event', self._on_event, 10)

    def reset(self):
        self.track_len = None
        self.prev_s = None
        self.dist_m = 0.0
        self.laps = 0
        self.n_diag = 0
        self.sim_t0 = None
        self.sim_t = None
        self.stall_ref = None      # (sim_t, dist) for the stuck watchdog
        self.terminal = None       # (sim_t, trigger, min_scan)

    def _on_traj(self, m):
        self.track_len = float(m.total_length_m)

    def _on_diag(self, m):
        s = float(m.path_s_m)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.n_diag += 1
        self.sim_t = t
        if self.sim_t0 is None:
            self.sim_t0 = t
        if self.prev_s is not None and self.track_len:
            ds = s - self.prev_s
            if ds < -0.5 * self.track_len:      # wrapped past s = 0
                ds += self.track_len
                self.laps += 1
            if ds > 0:
                self.dist_m += ds
        self.prev_s = s

    def _on_event(self, m):
        if m.terminal and self.terminal is None:
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.terminal = (t, int(m.trigger), float(m.min_scan_m))


def _spin_for(node, seconds):
    t_end = time.time() + seconds
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.2)


def _resolve(track, cfg):
    return {
        'world': cfg.get('world', f'{track}.sdf'),
        'track_file': cfg.get('track_file', f'src/{track}/{track}_centerline_2x.csv'),
        'params': cfg.get('params', f'{track.lower()}_defaults.yaml'),
    }


def _stop_process(pid, name, int_wait, term_wait, log):
    """SIGINT, wait; SIGTERM, wait; report which one worked."""
    try:
        os.kill(pid, signal.SIGINT)
    except ProcessLookupError:
        return 'already_gone'
    if _wait_exit(pid, int_wait):
        return 'sigint'
    log(f'    {name} pid {pid} ignored SIGINT for {int_wait}s; SIGTERM')
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return 'sigint'
    if _wait_exit(pid, term_wait):
        return 'sigterm'
    return 'alive'


# Startup statuses worth one more try with a fresh launch. Seen in 2 of 24 p1
# runs: the ackermann_like_controller spawner never reached
# /controller_manager/list_controllers although the joint_state_broadcaster
# spawner had just used it -- a ros2_control discovery race, not a data
# problem. A retry is a new Gazebo, not a poke at the old one.
RETRY_STATUSES = ('no_diagnostics_180s', 'launch_died_early')


def run_one(node, track, tcfg, speed, pass_name, pcfg, laps_target, out_root, log,
            delete_bag_on_pass=False, attempts=2):
    for attempt in range(1, attempts + 1):
        r = _run_once(node, track, tcfg, speed, pass_name, pcfg, laps_target, out_root, log,
                      delete_bag_on_pass)
        if r.get('status') not in RETRY_STATUSES or attempt == attempts:
            return r
        log(f'  retry {attempt}/{attempts - 1}: {r["status"]} -- fresh launch')
        shutil.rmtree(os.path.join(out_root, f'{track}_{speed}ms_{pass_name}'), ignore_errors=True)
    return r


def _run_once(node, track, tcfg, speed, pass_name, pcfg, laps_target, out_root, log,
              delete_bag_on_pass=False):
    files = _resolve(track, tcfg)
    run_dir = os.path.join(out_root, f'{track}_{speed}ms_{pass_name}')
    report_path = os.path.join(run_dir, 'collection_report.json')
    if os.path.exists(report_path):
        with open(report_path) as f:
            prev = json.load(f)
        if prev.get('status') == 'ok':
            log(f'  skip {track} @ {speed} m/s {pass_name}: already ok ({prev["rows"]} rows)')
            return prev
        log(f'  redo {track} @ {speed} m/s {pass_name}: previous status {prev.get("status")}')
        shutil.rmtree(run_dir, ignore_errors=True)
    # Clear ANY leftover run directory that is not a completed run. The
    # earlier version only cleaned up when a report file existed -- but a hard
    # crash (host suspend, reboot, SIGKILL) leaves a directory with a bag and
    # NO report, so the cleanup was skipped and the stale bag/ survived.
    # `ros2 bag record -o <dir>` then refuses to start because the output
    # directory exists, the recorder dies at launch, and the rollout drives a
    # perfect lap that records nothing. Cost one 24-minute Nuerburgring
    # rollout on 2026-09-23; it did not corrupt anything, because the run ends
    # as "ok+bag_not_finalised" rather than "ok" and is redone -- but only
    # after wasting the drive.
    elif os.path.isdir(run_dir):
        shutil.rmtree(run_dir, ignore_errors=True)

    report = {'track': track, 'speed_mps': speed, 'pass': pass_name,
              'spawn_s_frac': pcfg['spawn_s_frac'], 'laps_target': laps_target,
              'status': 'started', 'wall_started': time.strftime('%Y-%m-%dT%H:%M:%S')}

    # --- preflight ----------------------------------------------------------
    ok, nodes, models = _wait_empty(node.get_name(), 5)
    if not ok:
        report.update(status='preflight_dirty', ros_nodes=nodes, gz_models=models)
        log(f'  ABORT preflight: nodes={nodes} gz={models}')
        return report

    # --- launch -------------------------------------------------------------
    os.makedirs(run_dir, exist_ok=True)
    cmd = ['ros2', 'launch', 'hunter_dataset', 'record_expert.launch.py',
           f'track_name:={track}', f'track_file:={files["track_file"]}',
           f'params_file:={files["params"]}', f'world:={files["world"]}',
           f'max_speed_mps:={speed}', f'run_id:={pass_name}', f'output_dir:={out_root}',
           f'spawn_s_frac:={pcfg["spawn_s_frac"]}', 'headless:=true', 'use_rviz:=false']
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
    log(f'  launch pid {proc.pid} pgid {pgid}: {track} @ {speed} m/s, {laps_target} laps, '
        f'spawn {pcfg["spawn_s_frac"]:.2f}')

    # --- wait for the graph to come up ----------------------------------------
    t0 = time.time()
    while node.n_diag < 20 or node.track_len is None:
        _spin_for(node, 1.0)
        if proc.poll() is not None:
            report.update(status='launch_died_early', exit_code=proc.returncode)
            log('  ABORT: launch exited before driving started (see launch.log)')
            _teardown(proc, pgid, node, log, report)
            _write(report_path, report)
            return report
        if time.time() - t0 > 180:
            report.update(status='no_diagnostics_180s')
            log('  ABORT: no /tracking/diagnostics within 180 s')
            _teardown(proc, pgid, node, log, report)
            _write(report_path, report)
            return report
    log(f'    driving; track length {node.track_len:.1f} m, target {laps_target * node.track_len:.0f} m')

    # --- drive --------------------------------------------------------------
    target_m = laps_target * node.track_len
    node.stall_ref = (node.sim_t, node.dist_m)
    last_print = time.time()
    status = 'ok'
    while True:
        _spin_for(node, 1.0)
        if node.dist_m >= target_m:
            break
        if node.terminal is not None:
            status = 'expert_crash'
            log(f'    TERMINAL at sim {node.terminal[0] - node.sim_t0:.1f}s trigger={node.terminal[1]} '
                f'min_scan={node.terminal[2]:.2f} m after {node.dist_m:.0f} m -- stopping')
            break
        if proc.poll() is not None:
            status = 'launch_died'
            log('    launch process died mid-run')
            break
        # stuck watchdog: 30 simulated seconds with < 1 m of progress
        st, sd = node.stall_ref
        if node.sim_t - st > 30.0:
            if node.dist_m - sd < 1.0:
                status = 'stuck'
                log(f'    STUCK: {node.dist_m - sd:.2f} m in 30 sim-s at s={node.prev_s:.1f}')
                break
            node.stall_ref = (node.sim_t, node.dist_m)
        if time.time() - last_print > 60:
            wall = time.time() - t_wall0
            simt = node.sim_t - node.sim_t0
            rate = node.dist_m / max(wall, 1)
            eta = (target_m - node.dist_m) / max(rate, 1e-3)
            log(f'    {node.dist_m:6.0f}/{target_m:.0f} m  laps {node.laps}  sim {simt:.0f}s  '
                f'wall {wall:.0f}s  RTF {simt / max(wall, 1):.2f}  eta {eta / 60:.0f} min')
            last_print = time.time()

    # a little more so the last row is well inside the target
    _spin_for(node, 2.0)
    report.update(status=status, laps_wrapped=node.laps, distance_m=round(node.dist_m, 1),
                  sim_duration_s=round((node.sim_t or 0) - (node.sim_t0 or 0), 1),
                  track_length_m=round(node.track_len, 1),
                  terminal=node.terminal)

    # --- stop ---------------------------------------------------------------
    _teardown(proc, pgid, node, log, report)
    report['wall_duration_s'] = round(time.time() - t_wall0, 1)

    # --- convert + inspect ----------------------------------------------------
    bag_dir = os.path.join(run_dir, 'bag')
    if not os.path.exists(os.path.join(bag_dir, 'metadata.yaml')):
        report['status'] = f'{status}+bag_not_finalised'
        log('  bag has no metadata.yaml -- recorder did not finalise')
        _write(report_path, report)
        return report
    report['bag_mb'] = round(sum(os.path.getsize(os.path.join(bag_dir, f))
                                 for f in os.listdir(bag_dir)) / 1e6, 1)
    h5 = os.path.join(run_dir, 'dataset.hdf5')
    conv = subprocess.run(['ros2', 'run', 'hunter_dataset', 'convert_dataset', bag_dir, h5,
                           '--manifest', os.path.join(run_dir, 'run_manifest.yaml')],
                          cwd=REPO, env=_child_env(), capture_output=True, text=True)
    with open(os.path.join(run_dir, 'convert.log'), 'w') as f:
        f.write(conv.stdout + conv.stderr)
    if conv.returncode != 0:
        report['status'] = f'{status}+convert_failed'
        log(f'  CONVERT FAILED (exit {conv.returncode}); see convert.log')
        _write(report_path, report)
        return report
    try:
        rep = json.loads(conv.stdout[conv.stdout.index('{'):])
        report['rows'] = rep.get('rows')
        report['join_dt_ms'] = rep.get('join_dt_ms')
        report['rows_rejected_slop'] = rep.get('rows_rejected_slop')
    except (ValueError, json.JSONDecodeError):
        report['rows'] = None
    insp_cmd = ['ros2', 'run', 'hunter_dataset', 'inspect_dataset', h5]
    if tcfg.get('role') == 'ims':
        # 0 % curvature binding: a flat speed column is the expected result,
        # and the reason IMS is capped at one lap per speed.
        insp_cmd += ['--expect-constant', 'speed_mps']
    insp = subprocess.run(insp_cmd,
                          cwd=REPO, env=_child_env(), capture_output=True, text=True)
    with open(os.path.join(run_dir, 'inspect.txt'), 'w') as f:
        f.write(insp.stdout + insp.stderr)
    report['inspect_exit'] = insp.returncode
    if insp.returncode != 0 and status == 'ok':
        report['status'] = 'inspect_failed'
    fails = [l.strip() for l in insp.stdout.splitlines() if '[FAIL]' in l]
    report['inspect_fails'] = fails
    log(f'  {report["status"]}: {report.get("rows")} rows, bag {report.get("bag_mb")} MB, '
        f'inspect exit {insp.returncode}' + (f' FAILS: {fails}' if fails else ''))
    if delete_bag_on_pass and report['status'] == 'ok':
        shutil.rmtree(bag_dir, ignore_errors=True)
        report['bag_deleted'] = True
    _write(report_path, report)
    return report


def _teardown(proc, pgid, node, log, report):
    """Recorder first (so the mcap is finalised), then the launch, then the
    whole group, then prove the graph is empty."""
    rec = _sh(f'pgrep -g {pgid} -f "ros2 bag record"').split()
    how = {}
    for pid in rec:
        how[f'recorder_{pid}'] = _stop_process(int(pid), 'recorder', 30, 15, log)
    how['launch'] = _stop_process(proc.pid, 'launch', 40, 15, log)
    try:
        os.killpg(pgid, signal.SIGKILL)
        how['group_sigkill'] = True
    except ProcessLookupError:
        how['group_sigkill'] = False
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    ok, nodes, models = _wait_empty(node.get_name(), 40)
    how['graph_empty'] = ok
    if not ok:
        log(f'    WARNING graph not empty after teardown: nodes={nodes} gz={models}')
        how['leftover_nodes'], how['leftover_gz'] = nodes, models
    report['teardown'] = how


def _write(path, report):
    with open(path, 'w') as f:
        json.dump(report, f, indent=2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--plan', required=True)
    ap.add_argument('--pass', dest='pass_name', required=True)
    ap.add_argument('--output', default='datasets/raw')
    ap.add_argument('--tracks', default='', help='comma-separated subset')
    ap.add_argument('--speeds', default='', help='comma-separated subset')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--delete-bag-on-pass', action='store_true')
    a = ap.parse_args()

    with open(a.plan) as f:
        plan = yaml.safe_load(f)
    pcfg = plan['passes'][a.pass_name]
    tracks = [t for t in plan['tracks'] if not a.tracks or t in a.tracks.split(',')]
    speeds = [s for s in plan['speeds_mps'] if not a.speeds or str(s) in a.speeds.split(',')]
    out_root = os.path.abspath(a.output)
    os.makedirs(out_root, exist_ok=True)
    log_path = os.path.join(out_root, f'collect_{a.pass_name}.log')

    def log(msg):
        line = f'[{time.strftime("%H:%M:%S")}] {msg}'
        print(line, flush=True)
        with open(log_path, 'a') as f:
            f.write(line + '\n')

    jobs = []
    for t in tracks:
        n = pcfg['laps'][plan['tracks'][t]['role']]
        for s in speeds:
            if n > 0:
                jobs.append((t, s, n))
    log(f'pass {a.pass_name}: {len(jobs)} runs, {sum(j[2] for j in jobs)} laps, spawn_s_frac={pcfg["spawn_s_frac"]}')
    for t, s, n in jobs:
        log(f'   {t:13s} {s} m/s  x{n}')
    if a.dry_run:
        return 0

    rclpy.init()
    node = Runner()
    results = []
    try:
        for i, (t, s, n) in enumerate(jobs, 1):
            log(f'[{i}/{len(jobs)}] {t} @ {s} m/s x{n}')
            r = run_one(node, t, plan['tracks'][t], s, a.pass_name, pcfg, n, out_root, log,
                        a.delete_bag_on_pass)
            results.append(r)
            if r.get('status') == 'preflight_dirty':
                log('stopping: the graph is not ours to clean')
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    log('=== summary ===')
    for r in results:
        log(f'  {r["track"]:13s} {r["speed_mps"]} m/s  {r.get("status"):24s} rows={r.get("rows")} '
            f'wall={r.get("wall_duration_s")}s bag={r.get("bag_mb")}MB')
    bad = [r for r in results if r.get('status') != 'ok']
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
