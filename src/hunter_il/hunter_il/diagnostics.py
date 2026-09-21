"""Part D: dataset diagnostics to run BEFORE training anything.

    il_diagnostics --data-dir datasets/raw --out diagnostics/

1. Information asymmetry. The expert acts on ground-truth pose; the student
   sees only the scan. If two states look the same to the scan but the expert
   acts differently in them, no scan-only policy can fit both: that is the
   asymmetry, and it is measurable. Observations -> PCA(32); for each row the
   k=20 nearest neighbours in that space; the std of the expert's action among
   those neighbours, divided by the global std of the action.
       median ratio < 0.2   healthy: scan-alike states get alike actions
       median ratio > 0.4   problem: the scan does not determine the action
   The ratio is also written per row with the ground-truth pose so it can be
   coloured onto the track: asymmetry has a location (typically corridor
   sections whose walls look the same from many places).

2. Aggregate speed histogram, per max_speed setting. The fraction of rows
   pinned at the cap is the IMS-style degeneracy measured over the whole set.

3. State coverage, BC vs DAgger: the distribution of lateral offset from the
   line. Expert-only data is a spike at ~0; DAgger must visibly widen it per
   iteration or it is producing expensive behaviour cloning.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np
from scipy.spatial import cKDTree

from hunter_dataset.dataset_spec import ACTION_SOURCE_EXPERT
from hunter_il.data import find_runs, load_run


def pca_fit(x: np.ndarray, n: int, seed: int = 0, fit_rows: int = 60000):
    rng = np.random.default_rng(seed)
    sub = x[rng.choice(len(x), min(fit_rows, len(x)), replace=False)]
    mean = sub.mean(0)
    _, _, vt = np.linalg.svd(sub - mean, full_matrices=False)
    return mean, vt[:n].T                                  # [720, n]


def information_asymmetry(obs, act, k=20, n_pca=32, n_query=30000, seed=0):
    mean, proj = pca_fit(obs, n_pca, seed)
    z = (obs - mean) @ proj
    tree = cKDTree(z)
    rng = np.random.default_rng(seed)
    q = rng.choice(len(z), min(n_query, len(z)), replace=False)
    _, nn = tree.query(z[q], k=k + 1)                      # includes self
    nn = nn[:, 1:]
    local_std = act[nn].std(axis=1)                        # [nq, 2]
    global_std = act.std(axis=0)
    ratio = local_std / np.maximum(global_std, 1e-9)
    return q, ratio, global_std


def _verdict(r):
    return 'healthy' if r < 0.2 else ('borderline' if r < 0.4 else 'PROBLEM')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--k', type=int, default=20)
    ap.add_argument('--pca', type=int, default=32)
    ap.add_argument('--no-plots', action='store_true')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    runs = [load_run(p) for p in find_runs(a.data_dir)]
    if not runs:
        raise SystemExit(f'no runs under {a.data_dir}')
    obs = np.concatenate([r.obs for r in runs])
    act = np.concatenate([r.act for r in runs])
    src = np.concatenate([r.action_source for r in runs])
    lat = np.concatenate([r.lateral_error_m for r in runs])
    pose = np.concatenate([r.gt_pose for r in runs])
    track = np.concatenate([[r.track_id] * len(r) for r in runs])
    cap = np.concatenate([[r.max_speed_mps] * len(r) for r in runs])
    run_id = np.concatenate([[r.run_id] * len(r) for r in runs])
    summary = {'n_rows': int(len(obs)), 'n_runs': len(runs),
               'tracks': sorted(set(track.tolist())), 'caps': sorted(set(cap.tolist()))}
    print(f'{len(runs)} runs, {len(obs)} rows, tracks {summary["tracks"]}')

    # --- 1. information asymmetry -------------------------------------------
    # Expert rows only: the question is whether the EXPERT's action is a
    # function of the scan. Per track, because the neighbour structure of one
    # track's corridor should not be diluted by another's.
    asym = {}
    rows_out = []
    for t in summary['tracks']:
        m = (track == t) & (src == ACTION_SOURCE_EXPERT)
        if m.sum() < 500:
            continue
        q, ratio, gstd = information_asymmetry(obs[m], act[m], a.k, a.pca)
        med = np.median(ratio, axis=0)
        asym[t] = {'median_ratio_steer': float(med[0]), 'median_ratio_speed': float(med[1]),
                   'p90_ratio_steer': float(np.percentile(ratio[:, 0], 90)),
                   'p90_ratio_speed': float(np.percentile(ratio[:, 1], 90)),
                   'global_std_steer': float(gstd[0]), 'global_std_speed': float(gstd[1]),
                   'verdict_steer': _verdict(med[0]), 'verdict_speed': _verdict(med[1]), 'n': int(m.sum())}
        idx = np.flatnonzero(m)[q]
        rows_out.append(np.column_stack([idx, pose[idx, 0], pose[idx, 1], ratio[:, 0], ratio[:, 1],
                                         np.full(len(idx), summary['tracks'].index(t))]))
        print(f'  asymmetry {t:13s} steer {med[0]:.3f} ({asym[t]["verdict_steer"]})  '
              f'speed {med[1]:.3f} ({asym[t]["verdict_speed"]})  n={m.sum()}')
    summary['information_asymmetry'] = asym
    if rows_out:
        np.savetxt(os.path.join(a.out, 'asymmetry_by_row.csv'), np.concatenate(rows_out),
                   delimiter=',', header='row,x,y,ratio_steer,ratio_speed,track_idx', comments='',
                   fmt=['%d', '%.3f', '%.3f', '%.4f', '%.4f', '%d'])

    # --- 2. speed histogram ---------------------------------------------------
    sp = {}
    for c in summary['caps']:
        v = act[cap == c, 1]
        at_cap = float((v >= 0.99 * c).mean())
        sp[str(c)] = {'n': int(len(v)), 'frac_at_cap': at_cap, 'min': float(v.min()),
                      'mean': float(v.mean()), 'std': float(v.std()),
                      'hist': np.histogram(v, bins=np.linspace(0, 2.81, 29))[0].tolist()}
        print(f'  speed cap {c}: {len(v)} rows, {at_cap * 100:.1f}% at cap, min {v.min():.2f}, std {v.std():.3f}')
    summary['speed'] = sp

    # --- 3. coverage: lateral offset by action source / DAgger iteration ------
    cov = {}
    edges = np.linspace(-1.0, 1.0, 41)
    for name, m in [('expert', src == ACTION_SOURCE_EXPERT), ('relabelled', src != ACTION_SOURCE_EXPERT)]:
        if m.any():
            cov[name] = {'n': int(m.sum()), 'abs_median': float(np.median(np.abs(lat[m]))),
                         'abs_p95': float(np.percentile(np.abs(lat[m]), 95)),
                         'hist': np.histogram(lat[m], bins=edges)[0].tolist()}
    for rid in sorted(set(run_id.tolist())):
        if 'dagger' in rid:
            m = run_id == rid
            cov[rid] = {'n': int(m.sum()), 'abs_median': float(np.median(np.abs(lat[m]))),
                        'abs_p95': float(np.percentile(np.abs(lat[m]), 95))}
    summary['coverage_lateral_m'] = cov
    for k, v in cov.items():
        print(f'  coverage {k:20s} n={v["n"]:7d} |lat| median {v["abs_median"]:.3f} p95 {v["abs_p95"]:.3f}')

    with open(os.path.join(a.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)

    if not a.no_plots:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            if rows_out:
                d = np.concatenate(rows_out)
                fig, axes = plt.subplots(1, 2, figsize=(14, 6))
                for ax, col, name in [(axes[0], 3, 'steer'), (axes[1], 4, 'speed')]:
                    sc = ax.scatter(d[:, 1], d[:, 2], c=np.clip(d[:, col], 0, 1), s=1, cmap='viridis')
                    ax.set_aspect('equal'); ax.set_title(f'local/global std of {name} (k={a.k})')
                    plt.colorbar(sc, ax=ax)
                fig.savefig(os.path.join(a.out, 'asymmetry_map.png'), dpi=120); plt.close(fig)
            fig, ax = plt.subplots(figsize=(8, 4))
            for c in summary['caps']:
                ax.hist(act[cap == c, 1], bins=np.linspace(0, 2.81, 57), alpha=0.6, label=f'cap {c}')
            ax.set_xlabel('commanded speed m/s'); ax.legend()
            fig.savefig(os.path.join(a.out, 'speed_hist.png'), dpi=120); plt.close(fig)
            fig, ax = plt.subplots(figsize=(8, 4))
            for name in ('expert', 'relabelled'):
                m = (src == ACTION_SOURCE_EXPERT) if name == 'expert' else (src != ACTION_SOURCE_EXPERT)
                if m.any():
                    ax.hist(lat[m], bins=edges, alpha=0.6, label=name, density=True)
            ax.set_xlabel('lateral offset m'); ax.set_yscale('log'); ax.legend()
            fig.savefig(os.path.join(a.out, 'coverage_lateral.png'), dpi=120); plt.close(fig)
        except Exception as e:      # plots are a convenience; the numbers are the deliverable
            print(f'plots skipped: {e}')
    print(f'wrote {a.out}/summary.json')


if __name__ == '__main__':
    main()
