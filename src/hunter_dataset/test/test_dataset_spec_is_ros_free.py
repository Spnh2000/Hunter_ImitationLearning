"""dataset_spec.py must remain importable without ROS.

Phase 8 trains in a torch/d3rlpy environment that has no ROS installed, and it
must be able to read the schema that produced its data. The moment someone
adds `from sensor_msgs.msg import LaserScan` here "just for a type hint",
Phase 8 can no longer import the schema and ends up with a second, drifting
copy of the constants -- which is the exact failure dataset_spec.py exists to
prevent.

Two checks, because either alone can be fooled:
  * static -- an AST scan catches an import that is guarded or unreached
  * dynamic -- a bare-interpreter import catches a transitive dependency that
    the AST scan cannot see (numpy pulling something in, say)
"""
import ast
import os
import subprocess
import sys

SPEC = os.path.join(os.path.dirname(__file__), '..', 'hunter_dataset', 'dataset_spec.py')

# Anything ROS-shaped. numpy is deliberately permitted: it exists in both the
# ROS container and any training environment.
FORBIDDEN_ROOTS = {
    'rclpy', 'rosbag2_py', 'rosidl_runtime_py', 'ros_gz_interfaces',
    'sensor_msgs', 'nav_msgs', 'geometry_msgs', 'std_msgs', 'builtin_interfaces',
    'hunter_msgs', 'message_filters', 'tf2_ros', 'ament_index_python', 'rosgraph_msgs',
}


def _imported_roots(path):
    with open(path) as f:
        tree = ast.parse(f.read(), filename=path)
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                roots.add(a.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module:
                roots.add(node.module.split('.')[0])
    return roots


def test_no_ros_imports_statically():
    offenders = sorted(_imported_roots(SPEC) & FORBIDDEN_ROOTS)
    assert not offenders, (
        f"dataset_spec.py imports ROS packages {offenders}. Phase 8 must be able "
        "to import this file without ROS present -- move whatever needs those "
        "types into convert_dataset.py instead."
    )


def test_imports_in_a_bare_interpreter():
    """Import it with every ROS path stripped from the environment."""
    env = dict(os.environ)
    for var in ('PYTHONPATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                'LD_LIBRARY_PATH', 'ROS_DISTRO', 'ROS_VERSION'):
        env.pop(var, None)

    spec_dir = os.path.abspath(os.path.dirname(SPEC))
    code = (
        "import sys; sys.path.insert(0, %r)\n"
        "import dataset_spec as s\n"
        "assert s.DOWNSAMPLE_IDX.shape == (s.OBS_DIM,)\n"
        "assert s.SCHEMA_VERSION\n"
        "loaded = [m for m in sys.modules if m.split('.')[0] in %r]\n"
        "assert not loaded, 'ROS modules loaded transitively: %%s' %% loaded\n"
        "print('OK')\n" % (spec_dir, sorted(FORBIDDEN_ROOTS))
    )
    r = subprocess.run([sys.executable, '-c', code], env=env,
                       capture_output=True, text=True)
    assert r.returncode == 0, (
        "dataset_spec.py failed to import without ROS:\n"
        f"stdout: {r.stdout}\nstderr: {r.stderr}"
    )


def test_downsample_is_exactly_720_and_drops_the_middle_of_each_triple():
    sys.path.insert(0, os.path.abspath(os.path.dirname(SPEC)))
    import dataset_spec as s
    assert len(s.DOWNSAMPLE_IDX) == 720
    assert set(s.DOWNSAMPLE_IDX) == {i for i in range(1080) if i % 3 != 1}
    # nothing dropped twice, nothing out of range
    assert len(set(s.DOWNSAMPLE_IDX)) == 720
    assert s.DOWNSAMPLE_IDX.min() == 0 and s.DOWNSAMPLE_IDX.max() == 1079


def test_sanitize_kills_every_non_finite_value():
    sys.path.insert(0, os.path.abspath(os.path.dirname(SPEC)))
    import dataset_spec as s
    import numpy as np
    raw = np.array([np.inf, -np.inf, np.nan, 0.0, 1e9, 5.0], dtype=np.float64)
    out = s.sanitize_ranges(raw)
    assert np.isfinite(out).all(), "non-finite survived sanitising"
    assert out.min() >= s.RANGE_MIN and out.max() <= s.RANGE_MAX
    assert out.dtype == np.float32
    # NaN must become INVALID_FILL, not silently clip to RANGE_MIN
    assert out[2] == np.float32(s.INVALID_FILL)
