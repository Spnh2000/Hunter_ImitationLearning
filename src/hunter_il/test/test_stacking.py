"""The stacking boundary rule, tested because its failure is silent."""
import numpy as np
from hunter_il.data import stack_indices, episode_start_index


def test_episode_start_index():
    ep = np.array([0, 0, 0, 1, 1, 2, 2, 2, 2])
    assert episode_start_index(ep).tolist() == [0, 0, 0, 3, 3, 5, 5, 5, 5]


def test_stack_never_crosses_episode_boundary():
    rng = np.random.default_rng(0)
    ep = np.repeat(np.arange(7), rng.integers(1, 9, size=7))
    for K in (1, 3, 5):
        idx = stack_indices(ep, K)
        assert idx.shape == (len(ep), K)
        for t in range(len(ep)):
            assert (ep[idx[t]] == ep[t]).all(), f'K={K} t={t}: stack leaks across episodes'
            assert idx[t, -1] == t                          # newest = current row
            assert (np.diff(idx[t]) >= 0).all()             # oldest first


def test_stack_pads_by_repeating_first_scan():
    ep = np.array([0, 0, 0, 0, 1, 1])
    idx = stack_indices(ep, 3)
    assert idx[0].tolist() == [0, 0, 0]      # episode start: first scan x3
    assert idx[1].tolist() == [0, 0, 1]
    assert idx[2].tolist() == [0, 1, 2]
    assert idx[3].tolist() == [1, 2, 3]
    assert idx[4].tolist() == [4, 4, 4]      # new episode: its own first scan, not row 3
    assert idx[5].tolist() == [4, 4, 5]
