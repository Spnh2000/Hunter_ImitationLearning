#!/bin/bash
# Print the closed-loop results table from the per-run reports.
#
#   scripts/show_results.sh          the summary table
#   scripts/show_results.sh --raw    every run, one line each
cd "$(dirname "$0")/.."
/usr/bin/python3 - "$@" << 'EOF'
import json, glob, sys, statistics as st
runs=[]
for p in sorted(glob.glob('results/eval_v1/*/eval_report.json')):
    r=json.load(open(p))
    if r.get('arch'): runs.append(r)
if not runs:
    sys.exit('no evaluation runs found under results/eval_v1/ -- run evaluate_policies first')

if '--raw' in sys.argv:
    print(f"{'policy':22s} {'track':13s} {'status':9s} {'crash':>5s} {'laps':>5s} {'RMS':>8s}")
    for r in sorted(runs, key=lambda x:(x['dataset'],x['arch'],x['track'])):
        rms=r.get('rms_gt_lateral_error_m') or 0
        print(f"{r['policy']:22s} {r['track']:13s} {r['status']:9s} {int(float(r['crashed'] or 0)):5d} "
              f"{float(r['laps_completed'] or 0):5.1f} {float(rms):7.3f}m")
    sys.exit()

# Offline score: the metrics each training run saved beside its checkpoint.
# Not results/eval_v1/offline*.json -- one of those predates the discrete-bin
# fix, and averaging across files mixed broken and repaired checkpoints.
off={}
for f in glob.glob('checkpoints/*_s[0-9].pt.json'):
    off.setdefault(f.split('/')[-1].rsplit('_s',1)[0],[]).append(json.load(open(f))['val_tracks']['rmse_steer_rad'])
g={}
for r in runs: g.setdefault((r['dataset'],r['arch']),[]).append(r)

print("CLOSED-LOOP RESULTS — 3 held-out circuits (Silverstone, SaoPaulo, Montreal) x 3 seeds")
print("the offline column is reported BESIDE the result, never used to select\n")
print(f"{'policy':26s} {'crash':>6s} {'laps':>6s} {'true RMS':>10s} | {'offline':>11s}")
print("-"*68)
for d,a,tag in [('expert','pid',None),('bc','mlp','bc_mlp'),('bc','stacked','bc_stacked'),('bc','discrete','bc_discrete'),
                ('dagger','mlp','dagger_mlp'),('dagger','stacked','dagger_stacked'),('dagger','discrete','dagger_discrete'),
                ('balanced','mlp','balanced_mlp')]:
    v=g.get((d,a))
    if not v: continue
    cr=st.mean([float(x['crashed'] or 0) for x in v])*100
    lp=st.mean([float(x['laps_completed'] or 0) for x in v])
    rms=st.mean([float(x['rms_gt_lateral_error_m']) for x in v if x.get('rms_gt_lateral_error_m')])
    o=off.get(tag)
    ostr=f"{st.mean(o)*1000:8.1f}mrad" if o else "         --"
    print(f"{d+' + '+a:26s} {cr:5.0f}% {lp:6.2f} {rms:9.3f}m | {ostr}")
print("\nraw per-run detail: scripts/show_results.sh --raw   |   full CSV: results/eval_v1/eval_all.csv")
EOF
