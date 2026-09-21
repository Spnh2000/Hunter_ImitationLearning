"""The six policies x N seeds, as one command.

    il_train_all --data-dir datasets/raw --seeds 0,1,2 --out checkpoints/

    dataset  bc      runs matching _p1/_p2/_p3     (expert drove)
             dagger  everything, incl. *_dagger_it* (expert relabelled the student)
    arch     mlp | stacked | discrete

Checkpoints are named <dataset>_<arch>_s<seed>.pt and carry their tag, so
evaluate_policies can group them without parsing filenames.
"""
import argparse
import os
import subprocess
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', required=True)
    ap.add_argument('--config', default='src/hunter_il/config/train_default.yaml')
    ap.add_argument('--out', default='checkpoints')
    ap.add_argument('--seeds', default='0,1,2')
    ap.add_argument('--archs', default='mlp,stacked,discrete')
    ap.add_argument('--datasets', default='bc,dagger')
    ap.add_argument('--epochs', type=int, default=None)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    include = {'bc': '_p1,_p2,_p3', 'dagger': ''}
    failed = []
    for ds in a.datasets.split(','):
        for arch in a.archs.split(','):
            for seed in a.seeds.split(','):
                out = os.path.join(a.out, f'{ds}_{arch}_s{seed}.pt')
                if os.path.exists(out + '.json'):
                    print(f'skip {out} (exists)', flush=True)
                    continue
                cmd = [sys.executable, '-m', 'hunter_il.train', '--config', a.config, '--arch', arch,
                       '--data-dir', a.data_dir, '--out', out, '--seed', seed, '--tag', ds]
                if include[ds]:
                    cmd += ['--include', include[ds]]
                if a.epochs:
                    cmd += ['--epochs', str(a.epochs)]
                print('==>', ' '.join(cmd[2:]), flush=True)
                r = subprocess.run(cmd)
                if r.returncode != 0:
                    failed.append(out)
    if failed:
        print('FAILED:', failed)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
