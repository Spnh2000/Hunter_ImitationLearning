# Phase 2 Validation: Monza Track And Gazebo Environment

Date: 2026-07-20
Branch: `dev-saqar`

## Verdict

Phase 2 track/world preparation is **PASS WITH WARNINGS** for the current Monza Gazebo environment.

The packaged world parses as valid SDF, contains collision geometry, loads in Gazebo Sim, aligns with the default spawn/waypoint set well enough for manual driving, and supports the Phase 1 runtime tests. Phase 3 remains gated because the F1TENTH planning dependency is missing from the worktree.

## Commands Executed

```bash
gz sdf -k src/hunter_gazebo/worlds/Monza.sdf
diff -q src/Monza.sdf src/hunter_gazebo/worlds/Monza.sdf
python3 <structured SDF/waypoint inspection>
ros2 launch hunter_gazebo launch_gz_sim.launch.py
```

## World File

Result: **PASS**

Evidence:

```text
gz sdf -k src/hunter_gazebo/worlds/Monza.sdf
Valid.
```

The root `src/Monza.sdf` and packaged `src/hunter_gazebo/worlds/Monza.sdf` are identical.

Structured inspection:

```text
world_models=48
wall_models=47
collisions=48
wall_bounds_x=[-1.195,96.859]
wall_bounds_y=[-37.617,131.031]
wall_length_min_max=[3.463,76.894]
wall_length_total=888.543
```

The runtime launch loaded `track_world` from the packaged Monza SDF and spawned `hunter` successfully.

## Waypoints And Spawn Alignment

Result: **PASS WITH WARNING**

Structured waypoint inspection:

```text
Monza_baseline.csv: n=1159, x=[-0.431,96.176], y=[-37.162,130.119], speed=[0.30,0.80], nearest_spawn=0.142
Monza_baseline_smooth.csv: n=1159, x=[-0.421,96.019], y=[-36.969,129.921], speed=[0.30,0.80], nearest_spawn=0.142
monza_track.csv: n=2197, x=[-1.177,96.416], y=[-36.395,129.776], speed=[0.30,0.80], nearest_spawn=0.656
```

The active controller launch default is `Monza_baseline_smooth.csv`, and the simulation launch default spawn is `(x=0.0, y=0.142, yaw=1.503)`. That is close to the baseline start and was validated dynamically by manual driving.

Warning: `monza_track.csv` contains a duplicate closing segment:

```text
duplicate_segment index=2196->0 p=(-0.656291,0.142149)
```

This is not blocking for Phase 1 because the default controller launch uses `Monza_baseline_smooth.csv`, but Phase 3 benchmark generation should decide whether to keep or remove the closing duplicate before computing path length, curvature, lap completion, or nearest-point metrics.

## Map Assets

Result: **PASS**

Evidence:

```text
Monza_map.png size=(2000,2000), mode=L
Monza_map.yaml resolution=0.09585
Monza_map.yaml origin=[-49.83928924498067,-50.50904922690367,0]
```

The map assets are present and consistent with a metric world scale suitable for future localization or benchmark visualization work.

## Phase 3 Gate

Result: **FAIL / BLOCKED**

Evidence:

```bash
git ls-tree HEAD src/f1tenth_planning
```

```text
160000 commit 70f0b6ccd09a975ee8711e132e75800da6cc61e0 src/f1tenth_planning
```

```bash
git submodule status
```

```text
fatal: no submodule mapping found in .gitmodules for path 'src/f1tenth_planning'
```

```bash
python3 -c "import f1tenth_planning"
```

```text
ModuleNotFoundError: No module named 'f1tenth_planning'
```

```bash
ros2 run hunter_controllers hunter_controller_node --ros-args \
  -p waypoint_file:=/home/thesp/sp_codes/hunter_robot/src/hunter_controllers/waypoints/Monza_baseline_smooth.csv \
  -p controller_type:=pure_pursuit
```

```text
Loaded 1159 waypoints from '.../Monza_baseline_smooth.csv'
ModuleNotFoundError: No module named 'f1tenth_planning'
```

Do not begin Phase 3 until this dependency is restored, replaced, or explicitly removed from the controller architecture. The smallest safe correction is to restore a reproducible `f1tenth_planning` source path, preferably with a valid `.gitmodules` entry or documented vendoring strategy pinned to the recorded commit.
