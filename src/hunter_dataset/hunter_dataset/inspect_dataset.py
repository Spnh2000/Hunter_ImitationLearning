"""The Step 3 gate. Reads an HDF5 and says, loudly, whether it is trustworthy.

Pure h5py + numpy + dataset_spec: no ROS, so it runs in the training
environment against any dataset, forever. Exit code is nonzero on any FAIL.

This project's recurring failure is a pipeline that reports success while
producing wrong data. Every check here prints its evidence, not just a verdict,
and a constant action column -- the specific failure that would train a speed
head to emit one number while showing excellent loss -- is called out in a way
that cannot be missed.
"""
from __future__ import annotations

import argparse
import json
import sys

import h5py
import numpy as np

from hunter_dataset import dataset_spec as spec

FAILS: list[str] = []
WARNS: list[str] = []


def _ok(label, cond, detail=''):
    tag = 'PASS' if cond else 'FAIL'
    print(f"  [{tag}] {label}" + (f"  -- {detail}" if detail else ''))
    if not cond:
        FAILS.append(label)
    return cond


def _warn(label, detail=''):
    print(f"  [WARN] {label}" + (f"  -- {detail}" if detail else ''))
    WARNS.append(label)


def inspect(path: str, expected_hz: float = spec.DATASET_HZ) -> int:
    print(f"\n=== {path} ===")
    with h5py.File(path, 'r') as h:
        obs = h['observations'][:]
        act = h['actions'][:]
        rew = h['rewards'][:]
        term = h['terminals'][:]
        tmo = h['timeouts'][:]
        ts = h['infos/timestamp_ns'][:]
        ep = h['infos/episode_id'][:]
        prev = h['infos/prev_action'][:]
        attrs = dict(h.attrs)
        rep = json.loads(attrs.get('converter_report', '{}'))
    N = len(obs)

    # ---------------------------------------------------------------- schema
    print("\nSCHEMA")
    ver = str(attrs.get('schema_version', ''))
    _ok("schema_version major matches", ver.split('.')[0] == spec.SCHEMA_VERSION.split('.')[0],
        f"file={ver} spec={spec.SCHEMA_VERSION}")
    _ok("observations shape [N,720] float32",
        obs.shape == (N, spec.OBS_DIM) and obs.dtype == np.float32, f"{obs.shape} {obs.dtype}")
    _ok("actions shape [N,2] float32",
        act.shape == (N, spec.ACTION_DIM) and act.dtype == np.float32, f"{act.shape} {act.dtype}")
    _ok("rewards/terminals/timeouts length N",
        len(rew) == N and len(term) == N and len(tmo) == N)
    _ok("terminals is bool", term.dtype == np.bool_, str(term.dtype))
    _ok("timeouts is bool", tmo.dtype == np.bool_, str(tmo.dtype))
    _ok("terminals and timeouts never both true on one row", not np.any(term & tmo),
        f"{int(np.sum(term & tmo))} rows conflict")
    _ok("terminals and timeouts are not the same array", not np.array_equal(term, tmo) or not term.any(),
        "identical arrays would mean they were conflated")

    # ---------------------------------------------------------- observations
    print("\nOBSERVATIONS")
    n_inf = int(np.isinf(obs).sum()); n_nan = int(np.isnan(obs).sum())
    _ok("zero inf", n_inf == 0, f"{n_inf} inf values")
    _ok("zero NaN", n_nan == 0, f"{n_nan} NaN values")
    lo, hi = float(np.nanmin(obs)), float(np.nanmax(obs))
    _ok(f"all within [{spec.RANGE_MIN}, {spec.RANGE_MAX}]",
        lo >= spec.RANGE_MIN - 1e-6 and hi <= spec.RANGE_MAX + 1e-6, f"min={lo:.3f} max={hi:.3f}")
    at_max = float((obs >= spec.RANGE_MAX - 1e-3).mean() * 100)
    print(f"         beams at range_max (no-return / clipped): {at_max:.1f}%")
    if at_max > 50:
        _warn("more than half of all beams are at range_max", "sensor may be pointing at nothing")

    # ------------------------------------------------ downsample round-trip
    print("\nDOWNSAMPLE INDEX")
    stored = np.asarray(attrs.get('downsample_indices', []), dtype=np.int64)
    rederived = np.array([i for i in range(spec.RAW_BEAMS) if i % 3 != 1], dtype=np.int64)
    _ok("stored index array present and length 720", stored.shape == (spec.OBS_DIM,), str(stored.shape))
    _ok("stored == dataset_spec.DOWNSAMPLE_IDX", np.array_equal(stored, spec.DOWNSAMPLE_IDX))
    _ok("stored == independent re-derivation (i % 3 != 1)", np.array_equal(stored, rederived),
        "the file must prove its own mapping, not just cite the code")

    # ----------------------------------------------------------------- rate
    print("\nRATE")
    dur_s = float(ts[-1] - ts[0]) / 1e9
    exp_n = expected_hz * dur_s
    _ok(f"N ~= {expected_hz:.0f} x duration", abs(N - exp_n) / max(exp_n, 1) < 0.10,
        f"N={N}  duration={dur_s:.1f}s  expected~{exp_n:.0f}  ratio={N/max(exp_n,1):.3f}")
    gaps = np.diff(ts) / 1e6
    print(f"         inter-row gap ms: median={np.median(gaps):.1f}  p95={np.percentile(gaps,95):.1f}  max={gaps.max():.1f}")
    _ok("timestamps strictly increasing", bool(np.all(np.diff(ts) > 0)))

    # ------------------------------------------------------------ join stats
    print("\nJOIN (from converter report)")
    if rep:
        dt = rep.get('join_dt_ms', {})
        print(f"         |dt| ms: median={dt.get('median',0):.2f}  p95={dt.get('p95',0):.2f}  "
              f"max={dt.get('max',0):.2f}  slop={dt.get('slop',0):.0f}")
        kept, rej = rep.get('rows_kept', 0), rep.get('rows_rejected_slop', 0)
        trimmed = rep.get('scans_pre_post_run_trimmed', 0)
        print(f"         rows kept={kept}  rejected(slop)={rej}  pre/post-run trimmed={trimmed}")
        _ok("survival: >90% of in-window scans paired", kept / max(kept + rej, 1) > 0.90,
            f"{kept} of {kept+rej}")
        _ok("episode events matched EXACTLY for every row",
            rep.get('episode_events_matched_exactly', -1) == N,
            f"{rep.get('episode_events_matched_exactly')} == N={N}")
    else:
        _warn("no converter report in attrs", "cannot audit the join")

    # --------------------------------------------------------------- actions
    print("\nACTIONS  (physical units)")
    bounds = np.asarray(attrs.get('action_bounds', spec.ACTION_BOUNDS))
    for d, name in enumerate(spec.ACTION_NAMES):
        col = act[:, d]
        mn, mx, sd = float(col.min()), float(col.max()), float(col.std())
        uniq = int(np.unique(np.round(col, 4)).size)
        print(f"  {name:13s} min={mn:8.4f}  max={mx:8.4f}  std={sd:8.4f}  unique={uniq:6d}")
        _ok(f"{name} within action_bounds", mn >= bounds[d, 0] - 1e-4 and mx <= bounds[d, 1] + 1e-4,
            f"bounds=[{bounds[d,0]}, {bounds[d,1]}]")
        if sd < 1e-4 or uniq <= 2:
            print("\n" + "!" * 78)
            print(f"!!  {name.upper()} IS (NEAR-)CONSTANT: std={sd:.2e}, {uniq} unique value(s).")
            print( "!!  A policy trained on this column learns to emit one number and will")
            print( "!!  report excellent loss while having learned nothing about it.")
            print( "!!  On IMS this is expected (0% curvature binding). On any twisty track")
            print( "!!  it means the speed profile did not engage -- check the params file.")
            print("!" * 78 + "\n")
            _ok(f"{name} carries signal", False, "constant column")
        else:
            _ok(f"{name} carries signal", True, f"std={sd:.4f}, {uniq} unique")

    # ----------------------------------------------------- episodes / infos
    print("\nEPISODES")
    n_ep = int(ep.max()) + 1
    n_term = int(term.sum()); n_tmo = int(tmo.sum())
    print(f"         episodes={n_ep}  terminals={n_term}  timeouts={n_tmo}")
    _ok("every episode ends in exactly one of terminal/timeout",
        n_term + n_tmo == n_ep, f"{n_term}+{n_tmo} vs {n_ep} episodes")
    # prev_action must be the previous row's action within an episode
    same_ep = ep[1:] == ep[:-1]
    ok_prev = np.allclose(prev[1:][same_ep], act[:-1][same_ep])
    _ok("prev_action == previous row's action within episodes", ok_prev)
    first_rows = np.r_[True, ~same_ep]
    _ok("prev_action at episode start == own action (documented padding)",
        np.allclose(prev[first_rows], act[first_rows]))

    # -------------------------------------------------------------- summary
    print("\nATTRS")
    for k in ('track_id', 'track_sha256', 'max_speed_mps', 'controller_label', 'git_rev', 'git_dirty'):
        v = attrs.get(k, '<<MISSING>>')
        print(f"  {k:16s} {str(v)[:60]}")
    _ok("track_sha256 present", bool(str(attrs.get('track_sha256', ''))))
    if attrs.get('git_dirty', True):
        _warn("git_dirty=True", "git_rev does not fully describe the code that produced this data")

    print("\n" + "=" * 78)
    if FAILS:
        print(f"GATE: FAIL  ({len(FAILS)} failing check(s))")
        for f in FAILS:
            print(f"   - {f}")
    else:
        print(f"GATE: PASS  ({len(WARNS)} warning(s))")
    print("=" * 78)
    return 1 if FAILS else 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('h5')
    p.add_argument('--hz', type=float, default=spec.DATASET_HZ)
    a = p.parse_args(argv)
    return inspect(a.h5, a.hz)


if __name__ == '__main__':
    sys.exit(main())
