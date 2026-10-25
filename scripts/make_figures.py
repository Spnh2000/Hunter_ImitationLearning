#!/usr/bin/env python3
"""Regenerate the data figures from the repository's own results.

    python3 scripts/make_figures.py                       # PNGs for the README -> media/
    python3 scripts/make_figures.py --out DIR --format pdf  # vector, for a paper

Needs numpy + matplotlib (both in the Docker image). Every number plotted is
read from tracked files, so a fresh clone reproduces the figures exactly:
  closed-loop results   results/eval_v1/eval_all.csv              (84 runs)
  offline steer error   checkpoints/*_s<seed>.pt.json             (val_tracks)
  DAgger outcomes       datasets/raw/*_dagger_it*/collection_report.json
  track geometry        src/race_tracks/<Track>/<Track>_centerline_2x.csv
"""
import collections
import csv
import glob
import json
import os
import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..'))

plt.rcParams.update({
    'font.family': 'serif', 'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif'],
    'font.size': 8, 'axes.titlesize': 8, 'axes.labelsize': 8, 'legend.fontsize': 7,
    'xtick.labelsize': 7, 'ytick.labelsize': 7, 'pdf.fonttype': 42, 'ps.fonttype': 42,
    'axes.spines.top': False, 'axes.spines.right': False,
})
COL = 3.5      # IEEE column width, inches
FULL = 7.16    # IEEE two-column text width, inches
C_TRAIN, C_IMS, C_HELD = '#2b6cb0', '#7f9cc0', '#d9822b'
C_BC, C_DAG, C_BAL = '#c0392b', '#2471a3', '#1e8449'

TRACKS = [('Shanghai', 'train'), ('Nuerburgring', 'train'), ('Melbourne', 'train'),
          ('Monza', 'train'), ('IMS', 'ims'), ('Silverstone', 'held'),
          ('SaoPaulo', 'held'), ('Montreal', 'held')]
PRETTY = {'Nuerburgring': 'Nürburgring', 'SaoPaulo': 'São Paulo'}


def centerline(track):
    xy = np.loadtxt(os.path.join(ROOT, 'src', 'race_tracks', track, f'{track}_centerline_2x.csv'),
                    delimiter=',', comments='#', usecols=(0, 1))
    closed = np.vstack([xy, xy[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(closed, axis=0).T))])
    return closed, s


OUT, FMT = os.path.join(ROOT, 'media'), 'png'


def save(fig, name):
    path = os.path.join(OUT, os.path.splitext(name)[0] + '.' + FMT)
    fig.savefig(path, bbox_inches='tight', pad_inches=0.02, dpi=220, facecolor='white')
    plt.close(fig)
    print('wrote', os.path.relpath(path, ROOT))


# ---------------------------------------------------------------- tracks
def fig_tracks():
    fig, axes = plt.subplots(2, 4, figsize=(FULL, 2.55))
    for ax, (t, role) in zip(axes.flat, TRACKS):
        xy, s = centerline(t)
        c = {'train': C_TRAIN, 'ims': C_IMS, 'held': C_HELD}[role]
        ax.plot(xy[:, 0], xy[:, 1], color=c, lw=1.4)
        ax.plot(*xy[0], 'o', color='k', ms=2.5)
        ax.set_aspect('equal')
        ax.axis('off')
        tag = {'train': 'training', 'ims': 'training (1 lap/speed)', 'held': 'held out'}[role]
        ax.set_title(f'{PRETTY.get(t, t)}\n{s[-1]:.0f} m, {tag}', color=c, pad=2)
    fig.subplots_adjust(wspace=0.05, hspace=0.35)
    save(fig, 'fig_tracks.pdf')


# ---------------------------------------------------------------- offline vs closed loop
def offline_steer_mrad():
    """Mean held-out-TRACK steering RMSE over 3 seeds, per (dataset, arch), in mrad.

    Read from checkpoints/<dataset>_<arch>_s<seed>.pt.json, the metrics each
    training run saves next to its checkpoint (tracked in git, full precision).
    """
    out = {}
    for ds, arch, _ in ORDER:
        v = []
        for seed in range(3):
            with open(os.path.join(ROOT, 'checkpoints', f'{ds}_{arch}_s{seed}.pt.json')) as f:
                v.append(json.load(f)['val_tracks']['rmse_steer_rad'] * 1000.0)
        out[(ds, arch)] = float(np.mean(v))
    return out


def closed_loop():
    rows = list(csv.DictReader(open(os.path.join(ROOT, 'results', 'eval_v1', 'eval_all.csv'))))
    g = collections.defaultdict(list)
    for r in rows:
        g[(r['dataset'], r['arch'])].append(r)
    out = {}
    for k, v in g.items():
        out[k] = dict(n=len(v),
                      crash=100.0 * np.mean([int(r['crashed']) for r in v]),
                      laps=np.mean([float(r['laps_completed']) for r in v]),
                      rms=np.mean([float(r['rms_gt_lateral_error_m']) for r in v]),
                      rms_sd=np.std([float(r['rms_gt_lateral_error_m']) for r in v], ddof=1))
    return out


ORDER = [('bc', 'mlp', 'BC, MLP'), ('bc', 'stacked', 'BC, stacked'), ('bc', 'discrete', 'BC, discretised'),
         ('dagger', 'mlp', 'DAgger, MLP'), ('dagger', 'stacked', 'DAgger, stacked'),
         ('dagger', 'discrete', 'DAgger, discretised'), ('balanced', 'mlp', 'DAgger 50/50, MLP')]


def fig_headline(off, cl):
    labels = [o[2] for o in ORDER]
    colors = [C_BC if o[0] == 'bc' else C_BAL if o[0] == 'balanced' else C_DAG for o in ORDER]
    y = np.arange(len(ORDER))[::-1]
    fig, (a, b) = plt.subplots(1, 2, figsize=(COL, 2.05), sharey=True)
    xo = [off[(o[0], o[1])] for o in ORDER]
    xc = [cl[(o[0], o[1])]['crash'] for o in ORDER]
    a.barh(y, xo, color=colors, height=0.68)
    b.barh(y, xc, color=colors, height=0.68)
    for yy, v in zip(y, xo):
        a.text(v + 5, yy, f'{v:.0f}', va='center', fontsize=6.5)
    for yy, v in zip(y, xc):
        b.text(v + 2, yy, f'{v:.0f}%', va='center', fontsize=6.5)
    a.set_yticks(y)
    a.set_yticklabels(labels)
    a.set_xlabel('Steering error (mrad)')
    b.set_xlabel('Crash rate (%)')
    a.set_title('(a) offline, on the dataset', pad=3)
    b.set_title('(b) closed loop, in Gazebo', pad=3)
    a.set_xlim(0, 330)
    b.set_xlim(0, 85)
    fig.subplots_adjust(wspace=0.34)
    save(fig, 'fig_headline.pdf')


# ---------------------------------------------------------------- DAgger
def fig_dagger():
    it = collections.defaultdict(lambda: dict(ok=0, bad=0, crashes=0))
    hairpin = []
    for f in glob.glob(os.path.join(ROOT, 'datasets', 'raw', '*_dagger_it*', 'collection_report.json')):
        d = json.load(open(f))
        r = it[d['dagger_iteration']]
        r['ok' if d['status'] == 'ok' else 'bad'] += 1
        r['crashes'] += d.get('n_crashes') or 0
        if d['dagger_iteration'] == 1:
            hairpin += [(d['track'], c['s_m']) for c in d.get('crashes', [])]
    its = sorted(it)
    fig, (a, b) = plt.subplots(1, 2, figsize=(FULL, 2.1), gridspec_kw={'width_ratios': [1.15, 1]})
    ok = [it[i]['ok'] for i in its]
    bad = [it[i]['bad'] for i in its]
    a.bar(its, ok, color=C_DAG, label='completed', width=0.7)
    a.bar([i for i in its if it[i]['bad']], [it[i]['bad'] for i in its if it[i]['bad']],
          bottom=[it[i]['ok'] for i in its if it[i]['bad']], color=C_BC, width=0.7,
          label='failed (ended early)')
    for i in its:
        a.text(i, 15.5, f'{it[i]["crashes"]} crashes', ha='center', fontsize=6.5)
    a.set_xticks(its)
    a.set_yticks([0, 5, 10, 15])
    a.set_xlabel('DAgger round')
    a.set_ylabel('Rollouts')
    a.set_ylim(0, 21)
    a.legend(loc='upper center', ncol=2, frameon=False, bbox_to_anchor=(0.5, 1.02))
    a.set_title('(a) outcome of every DAgger rollout', pad=3)

    xy, s = centerline('Shanghai')
    b.plot(xy[:, 0], xy[:, 1], color=C_TRAIN, lw=1.2)
    assert all(t == 'Shanghai' for t, _ in hairpin), hairpin
    for _, sc in hairpin:
        k = int(np.searchsorted(s, sc))
        b.plot(*xy[min(k, len(xy) - 1)], 'x', color=C_BC, ms=6, mew=1.4)
    k = int(np.searchsorted(s, np.mean([sc for _, sc in hairpin])))
    b.annotate(f'all {len(hairpin)} crashes of round 1\n(s = {min(sc for _, sc in hairpin):.0f}–'
               f'{max(sc for _, sc in hairpin):.0f} m, 1.44 m radius)',
               xy=xy[k], xytext=(0.0, 0.0), textcoords='axes fraction', fontsize=6.5,
               arrowprops=dict(arrowstyle='->', lw=0.7, color='k'))
    b.plot(*xy[0], 'o', color='k', ms=2.5)
    b.set_aspect('equal')
    b.axis('off')
    b.set_title('(b) Shanghai: where round 1 failed', pad=3)
    save(fig, 'fig_dagger.pdf')
    return it


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--format', default=FMT, choices=['png', 'pdf', 'svg'])
    a = ap.parse_args()
    OUT, FMT = os.path.abspath(a.out), a.format
    os.makedirs(OUT, exist_ok=True)
    fig_tracks()
    off, cl = offline_steer_mrad(), closed_loop()
    for o in ORDER:
        k = (o[0], o[1])
        print(f'  {o[2]:22s} offline {off[k]:6.1f} mrad | crash {cl[k]["crash"]:3.0f}% '
              f'laps {cl[k]["laps"]:.2f} rms {cl[k]["rms"]:.3f} +- {cl[k]["rms_sd"]:.3f} (n={cl[k]["n"]})')
    print(f'  PID expert rms {cl[("expert", "pid")]["rms"]:.3f} (n={cl[("expert", "pid")]["n"]})')
    fig_headline(off, cl)
    for i, r in sorted(fig_dagger().items()):
        print(f'  DAgger it{i}: {r["ok"]} kept, {r["bad"]} failed, {r["crashes"]} crashes')
