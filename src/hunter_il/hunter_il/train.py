"""Train one policy. Config-driven, seeded, logs validation on BOTH held-out
laps (seen tracks) and held-out tracks (unseen geometry) every epoch.

    il_train --config src/hunter_il/config/train_default.yaml --arch mlp \
             --data-dir datasets/raw --out checkpoints/bc_mlp_s0.pt --seed 0

Validation loss is reported because the brief asks for it beside the
closed-loop result. It is NOT the selection criterion for anything that
matters -- the gap between the two rankings is the finding.
"""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np
import torch
import yaml

from hunter_il import checkpoint, models
from hunter_il.bins import ActionBins
from hunter_il.data import StackedArrays, find_runs, load_run, make_split


def _batches(n: int, batch: int, rng: np.random.Generator | None):
    order = rng.permutation(n) if rng is not None else np.arange(n)
    for i in range(0, n, batch):
        yield order[i:i + batch]


def _evaluate(model, arrays: StackedArrays, bins, batch=2048) -> dict:
    """Loss on the model's own objective, plus physical-unit RMSE per dim so
    the three architectures are comparable on one scale."""
    if len(arrays) == 0:
        return {'loss': float('nan'), 'rmse_steer_rad': float('nan'), 'rmse_speed_mps': float('nan'), 'n': 0}
    model.eval()
    tot, se = 0.0, np.zeros(2)
    with torch.no_grad():
        for flat in _batches(len(arrays), batch, None):
            obs, act, cap = arrays.batch(flat)
            obs_t, act_t, cap_t = torch.from_numpy(obs), torch.from_numpy(act), torch.from_numpy(cap)
            if bins is not None:
                tot += model.loss(obs_t, torch.from_numpy(bins.encode(act)), cap_t).item() * len(flat)
            else:
                tot += model.loss(obs_t, act_t, cap_t).item() * len(flat)
            pred = model(obs_t, cap_t).numpy()
            se += ((pred - act) ** 2).sum(0)
    n = len(arrays)
    return {'loss': tot / n, 'rmse_steer_rad': float(np.sqrt(se[0] / n)),
            'rmse_speed_mps': float(np.sqrt(se[1] / n)), 'n': n}


def train(cfg: dict, arch: str, data_dir: str, out: str, seed: int, log=print) -> dict:
    torch.manual_seed(seed)
    np.random.seed(seed)
    rng = np.random.default_rng(seed)
    torch.set_num_threads(int(cfg.get('threads', 1)))

    paths = find_runs(data_dir, cfg.get('run_glob', '*/dataset.hdf5'))
    if cfg.get('include_runs'):
        paths = [p for p in paths if any(s in p for s in cfg['include_runs'])]
    if not paths:
        raise SystemExit(f'no runs under {data_dir}')
    runs = [load_run(p) for p in paths]
    log(f'{len(runs)} runs, {sum(len(r) for r in runs)} rows, tracks '
        f'{sorted({r.track_id for r in runs})}')
    split = make_split(runs, cfg['heldout_tracks'], cfg.get('heldout_lap_rule', 'p1_lap1'),
                       cfg.get('drop_terminal_episodes', True))
    log(split.describe(runs))

    K = models.k_for(arch)
    tr = StackedArrays(runs, split.train, K)
    vl = StackedArrays(runs, split.val_laps, K)
    vt = StackedArrays(runs, split.val_tracks, K)

    bins = None
    if arch == 'discrete':
        bins = ActionBins.fit(tr.all_actions(), cfg.get('n_steer_bins', 21), cfg.get('n_speed_bins', 21))
        log(f'bins: {bins.n_steer} steer (width at 0: {bins.resolution_near_zero():.4f} rad), '
            f'{bins.n_speed} speed')
    model = models.build(arch, bins, bool(cfg.get('cap_input', True)))
    log(f'cap_input={model.cap_input} (speed cap as a policy input; see models.py)')
    opt = torch.optim.Adam(model.parameters(), lr=float(cfg.get('lr', 1e-3)),
                           weight_decay=float(cfg.get('weight_decay', 0.0)))
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=int(cfg['epochs']))
    batch = int(cfg.get('batch', 256))

    history, best, best_state, since_best = [], float('inf'), None, 0
    t0 = time.time()
    for epoch in range(int(cfg['epochs'])):
        model.train()
        tot = 0.0
        for flat in _batches(len(tr), batch, rng):
            obs, act, cap = tr.batch(flat)
            obs_t, cap_t = torch.from_numpy(obs), torch.from_numpy(cap)
            if bins is not None:
                loss = model.loss(obs_t, torch.from_numpy(bins.encode(act)), cap_t)
            else:
                loss = model.loss(obs_t, torch.from_numpy(act), cap_t)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += loss.item() * len(flat)
        sched.step()
        ev_l, ev_t = _evaluate(model, vl, bins), _evaluate(model, vt, bins)
        row = {'epoch': epoch, 'train_loss': tot / max(len(tr), 1), 'val_laps': ev_l, 'val_tracks': ev_t,
               'wall_s': round(time.time() - t0, 1)}
        history.append(row)
        log(f'ep {epoch:3d} train {row["train_loss"]:.5f} | laps loss {ev_l["loss"]:.5f} '
            f'steer {ev_l["rmse_steer_rad"]:.4f} speed {ev_l["rmse_speed_mps"]:.3f} | '
            f'tracks loss {ev_t["loss"]:.5f} steer {ev_t["rmse_steer_rad"]:.4f} '
            f'speed {ev_t["rmse_speed_mps"]:.3f} | {row["wall_s"]:.0f}s')
        # model selection on held-out LAPS only: the held-out tracks are the
        # test set and must not steer training decisions.
        crit = ev_l['loss'] if ev_l['n'] > 0 else row['train_loss']
        if crit < best:
            best, since_best = crit, 0
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            since_best += 1
            if since_best >= int(cfg.get('patience', 10)):
                log(f'early stop at epoch {epoch}')
                break
    if best_state is not None:
        model.load_state_dict(best_state)
    final = {'val_laps': _evaluate(model, vl, bins), 'val_tracks': _evaluate(model, vt, bins),
             'best_val_laps_loss': best, 'epochs_run': len(history), 'history': history,
             'n_train': len(tr), 'seed': seed, 'runs': [os.path.relpath(p) for p in paths]}
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    checkpoint.save(out, model, arch, bins, {'config': cfg, 'arch': arch, 'seed': seed,
                                              'data_dir': data_dir,
                                              'dataset_tag': cfg.get('dataset_tag', 'all')}, final)
    with open(out + '.json', 'w') as f:
        json.dump(final, f, indent=1)
    log(f'saved {out}: val_laps {final["val_laps"]} val_tracks {final["val_tracks"]}')
    return final


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--arch', required=True, choices=models.ARCHS)
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--epochs', type=int, default=None)
    ap.add_argument('--include', default='',
                    help="comma-separated substrings; only runs whose path contains one are used "
                         "(e.g. '_p1,_p2,_p3' = BC data only; empty = everything incl. DAgger)")
    ap.add_argument('--tag', default='', help='dataset tag stored in the checkpoint (bc | dagger)')
    a = ap.parse_args()
    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    if a.epochs is not None:
        cfg['epochs'] = a.epochs
    if a.include:
        cfg['include_runs'] = a.include.split(',')
    if a.tag:
        cfg['dataset_tag'] = a.tag
    train(cfg, a.arch, a.data_dir, a.out, a.seed)


if __name__ == '__main__':
    main()
