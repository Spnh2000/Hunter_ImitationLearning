"""Scale an F1TENTH racetrack up to a size a full-size robot can actually drive.

F1TENTH circuits are drawn for ~0.3 m-long 1/10-scale cars. The AgileX Hunter
V2 is 0.98 m long with a 1.31 m minimum turning radius (0.65 m wheelbase,
+/-0.461 rad steering), while raw Monza centerline corners are as tight as
0.47 m radius -- roughly 3x tighter than the vehicle can physically turn.
Scaling the whole track uniformly multiplies every corner radius and the
corridor width by the same factor, which is the only change that buys
tracking margin without distorting the circuit's shape.

The map PNG is never modified: scaling is expressed purely as a larger
`resolution` (metres per pixel) in the map YAML, with `origin` scaled to
match so the track stays anchored the same way relative to the world origin.
The CSVs are rewritten with scaled coordinates.

Scaling rules, per quantity:
  x, y, s, track widths   *= factor      (lengths)
  kappa                   /= factor      (1/length)
  psi                     unchanged      (angles are scale-invariant)
  vx, ax                  unchanged      (the speed profile is re-synthesized
                                          by trajectory_server anyway)

Usage:
  python3 scale_track.py Monza 2.0
  python3 scale_track.py Monza 2.0 --track-dir src/Monza --suffix _2x
"""
import argparse
import os


def scale_map_yaml(src, dst, factor):
    with open(src) as f:
        lines = f.readlines()

    out = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('resolution:'):
            value = float(stripped.split(':', 1)[1])
            out.append(f'resolution: {value * factor}\n')
        elif stripped.startswith('origin:'):
            body = stripped.split(':', 1)[1].strip().strip('[]')
            ox, oy, otheta = [float(v) for v in body.split(',')]
            # origin is the world position of the map's lower-left pixel, so it
            # scales with everything else; theta is an angle and does not.
            out.append(f'origin: [{ox * factor},{oy * factor}, {otheta:.6f}]\n')
        else:
            out.append(line)

    with open(dst, 'w') as f:
        f.writelines(out)
    print(f'  wrote {dst}')


def scale_centerline(src, dst, factor):
    """Columns: x_m, y_m, w_tr_right_m, w_tr_left_m -- all lengths."""
    rows = 0
    with open(src) as fin, open(dst, 'w') as fout:
        for line in fin:
            s = line.strip()
            if not s or s.startswith('#'):
                fout.write(line)
                continue
            vals = [float(v) for v in s.split(',')]
            fout.write(', '.join(repr(v * factor) for v in vals) + '\n')
            rows += 1
    print(f'  wrote {dst} ({rows} points)')


def scale_raceline(src, dst, factor):
    """Columns: s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2."""
    rows = 0
    with open(src) as fin, open(dst, 'w') as fout:
        for line in fin:
            s = line.strip()
            if not s or s.startswith('#'):
                fout.write(line)
                continue
            v = [float(t) for t in s.split(';')]
            scaled = [
                v[0] * factor,   # s_m
                v[1] * factor,   # x_m
                v[2] * factor,   # y_m
                v[3],            # psi_rad (angle)
                v[4] / factor,   # kappa_radpm (1/length)
                v[5],            # vx_mps
                v[6],            # ax_mps2
            ]
            fout.write(';'.join(f'{t:.7f}' for t in scaled) + '\n')
            rows += 1
    print(f'  wrote {dst} ({rows} points)')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('track', help='Track basename, e.g. Monza')
    parser.add_argument('factor', type=float, help='Uniform scale factor, e.g. 2.0')
    parser.add_argument('--track-dir', default=None,
                        help='Directory holding <track>_map.yaml etc. (default: <script dir>/<track>)')
    parser.add_argument('--suffix', default=None,
                        help='Output filename suffix (default: _<factor>x, e.g. _2x)')
    args = parser.parse_args()

    track_dir = args.track_dir or os.path.join(os.path.dirname(os.path.abspath(__file__)), args.track)
    suffix = args.suffix or f'_{args.factor:g}x'.replace('.', 'p')

    print(f'Scaling {args.track} by {args.factor}x in {track_dir}')

    scale_map_yaml(
        os.path.join(track_dir, f'{args.track}_map.yaml'),
        os.path.join(track_dir, f'{args.track}_map{suffix}.yaml'),
        args.factor)
    scale_centerline(
        os.path.join(track_dir, f'{args.track}_centerline.csv'),
        os.path.join(track_dir, f'{args.track}_centerline{suffix}.csv'),
        args.factor)

    raceline = os.path.join(track_dir, f'{args.track}_raceline.csv')
    if os.path.exists(raceline):
        scale_raceline(
            raceline,
            os.path.join(track_dir, f'{args.track}_raceline{suffix}.csv'),
            args.factor)

    print('\nThe map PNG is unchanged -- only the YAML resolution/origin scale.\n'
          'Regenerate the world from the scaled YAML, e.g.:\n'
          f'  python3 my_convert_track.py {args.track}/{args.track}_map.png '
          f'{args.track}/{args.track}_map{suffix}.yaml \\\n'
          f'      -o hunter_gazebo/worlds/{args.track}.sdf --smoothness 0.0003 --thickness 0.06')


if __name__ == '__main__':
    main()
