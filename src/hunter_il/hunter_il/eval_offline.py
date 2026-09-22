"""Validation-loss table for a set of checkpoints, so it can sit beside the
closed-loop table. Reports each model on its own objective AND on physical
RMSE per dimension (comparable across the three architectures).

    il_eval_offline --data-dir datasets/raw --config src/hunter_il/config/train_default.yaml \
                    checkpoints/*.pt
"""
from __future__ import annotations

import argparse
import json

import yaml

from hunter_il import checkpoint, models
from hunter_il.data import StackedArrays, find_runs, load_run, make_split
from hunter_il.train import _evaluate


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--config', required=True)
    ap.add_argument('--out', default='')
    ap.add_argument('ckpts', nargs='+')
    a = ap.parse_args()
    with open(a.config) as f:
        cfg = yaml.safe_load(f)
    runs = [load_run(p) for p in find_runs(a.data_dir)]
    split = make_split(runs, cfg['heldout_tracks'], cfg.get('heldout_lap_rule', 'p1_lap1'), False,
                       tuple(cfg.get('heldout_lap_caps', (2.1, 2.7))))
    rows = []
    print(f'{"checkpoint":40s} {"arch":9s} | {"laps loss":>9s} {"steer":>7s} {"speed":>6s} | '
          f'{"tracks loss":>11s} {"steer":>7s} {"speed":>6s}')
    for p in a.ckpts:
        model, ck = checkpoint.load(p)
        K = models.k_for(ck['arch'])
        from hunter_il.bins import ActionBins
        bins = ActionBins.from_dict(ck['bins']) if ck.get('bins') else None
        el = _evaluate(model, StackedArrays(runs, split.val_laps, K), bins)
        et = _evaluate(model, StackedArrays(runs, split.val_tracks, K), bins)
        rows.append({'checkpoint': p, 'arch': ck['arch'], 'seed': ck['train_config'].get('seed'),
                     'val_laps': el, 'val_tracks': et})
        print(f'{p[-40:]:40s} {ck["arch"]:9s} | {el["loss"]:9.5f} {el["rmse_steer_rad"]:7.4f} '
              f'{el["rmse_speed_mps"]:6.3f} | {et["loss"]:11.5f} {et["rmse_steer_rad"]:7.4f} '
              f'{et["rmse_speed_mps"]:6.3f}')
    if a.out:
        with open(a.out, 'w') as f:
            json.dump(rows, f, indent=1)


if __name__ == '__main__':
    main()
