import numpy as np
from hunter_il.bins import ActionBins
from hunter_dataset.dataset_spec import ACTION_BOUNDS


def _fake_actions(n=20000, seed=0):
    rng = np.random.default_rng(seed)
    steer = np.clip(rng.normal(0, 0.05, n), -0.461, 0.461)
    speed = np.concatenate([np.full(n // 2, 2.7), rng.uniform(0.9, 2.7, n - n // 2)])
    return np.stack([steer, speed], 1).astype(np.float32)


def test_roundtrip_is_within_bin_width():
    a = _fake_actions()
    b = ActionBins.fit(a, 21, 21)
    d = b.decode(b.encode(a))
    assert np.abs(d[:, 0] - a[:, 0]).max() < np.diff(b.steer_edges).max()
    assert np.abs(d[:, 1] - a[:, 1]).max() < np.diff(b.speed_edges).max() + 1e-6


def test_edges_monotonic_and_pinned_to_bounds():
    b = ActionBins.fit(_fake_actions(), 21, 21)
    assert (np.diff(b.steer_edges) > 0).all() and (np.diff(b.speed_edges) > 0).all()
    assert b.steer_edges[0] == ACTION_BOUNDS[0, 0] and b.steer_edges[-1] == ACTION_BOUNDS[0, 1]
    assert b.speed_edges[0] == ACTION_BOUNDS[1, 0] and b.speed_edges[-1] == ACTION_BOUNDS[1, 1]


def test_finer_near_zero_than_at_limits():
    b = ActionBins.fit(_fake_actions(), 21, 21)
    w = np.diff(b.steer_edges)
    assert b.resolution_near_zero() < w[0] and b.resolution_near_zero() < w[-1]


def test_dict_roundtrip():
    b = ActionBins.fit(_fake_actions(), 21, 21)
    b2 = ActionBins.from_dict(b.to_dict())
    assert np.allclose(b.steer_edges, b2.steer_edges) and np.allclose(b.speed_centres, b2.speed_centres)


def test_no_bin_collapses_on_a_realistically_peaked_distribution():
    """The check the original suite was missing.

    Round-trip-within-one-bin-width and monotonic-pinned-edges are BOTH TRUE
    of a degenerate set of edges, so the old tests passed while the widest
    steering bin was 9,931x the narrowest and the two end bins covered 84 %
    and 89 % of their side of the range. Assert the shape of the partition
    itself, not just that encode/decode agree with it.
    """
    a = _fake_actions(50000)
    b = ActionBins.fit(a, 21, 21)
    for name, edges, span in (('steer', b.steer_edges, 0.922), ('speed', b.speed_edges, 2.81)):
        w = np.diff(edges)
        assert w.min() > 0.01 * span, f'{name}: bin of width {w.min():.5f} is below the actuator resolution'
        assert w.max() / w.min() < 40, f'{name}: width ratio {w.max()/w.min():.0f} -- the partition has collapsed'
        assert w.max() < 0.45 * span, f'{name}: one bin covers {w.max()/span:.0%} of the range'


def test_still_finer_near_zero_than_at_the_limits():
    """The blend must not throw away the property that motivated quantiles."""
    b = ActionBins.fit(_fake_actions(50000), 21, 21)
    w = np.diff(b.steer_edges)
    assert b.resolution_near_zero() < w[0] and b.resolution_near_zero() < w[-1]
