"""Discretisation for the classification policy.

Edges are computed ONCE from training actions and then live in the
checkpoint. A model whose outputs cannot be decoded without a separate file is
a model that will be decoded wrong.

Why quantile edges: steering spends ~80 % of its time within +-0.05 rad of
zero on these tracks, and the physical limit is +-0.461. Uniform bins would
put most of the resolution where nothing happens. Quantile edges are fine
near zero and coarse toward the limits, with the outer two edges pinned to
the physical bounds so out-of-distribution values still land in a bin.
Speed is trimodal (three max_speed settings) with corner slowdowns below
each mode; quantiles put bins where the mass is.

Bin centres are the MEAN training value inside each bin, not the midpoint:
for a skewed bin (the outermost ones) the midpoint is systematically off.
"""
from __future__ import annotations

import numpy as np

from hunter_dataset.dataset_spec import ACTION_BOUNDS


def make_edges(values: np.ndarray, n_bins: int, lo: float, hi: float) -> np.ndarray:
    """n_bins + 1 strictly increasing edges in [lo, hi]."""
    v = np.clip(values.astype(np.float64), lo, hi)
    q = np.linspace(0.0, 1.0, n_bins + 1)[1:-1]
    inner = np.quantile(v, q)
    edges = np.concatenate([[lo], inner, [hi]])
    # Duplicate quantiles happen when a value dominates (speed pinned at the
    # cap for most of a lap). Nudge to keep edges strictly increasing so every
    # bin has non-zero width; np.digitize needs monotonic edges.
    eps = (hi - lo) * 1e-6
    for i in range(1, len(edges)):
        if edges[i] <= edges[i - 1]:
            edges[i] = edges[i - 1] + eps
    edges[-1] = max(edges[-1], hi)
    return edges


def encode(values: np.ndarray, edges: np.ndarray) -> np.ndarray:
    """Bin index in [0, n_bins-1]; values at/above the last edge go in the last bin."""
    idx = np.digitize(values, edges[1:-1], right=False)
    return np.clip(idx, 0, len(edges) - 2).astype(np.int64)


def centres_from_data(values: np.ndarray, edges: np.ndarray) -> np.ndarray:
    idx = encode(values, edges)
    n = len(edges) - 1
    c = np.empty(n, dtype=np.float32)
    for b in range(n):
        m = idx == b
        c[b] = values[m].mean() if m.any() else 0.5 * (edges[b] + edges[b + 1])
    return c


def decode(idx: np.ndarray, centres: np.ndarray) -> np.ndarray:
    return centres[np.asarray(idx)]


class ActionBins:
    """Both action dimensions. Serialisable to/from a plain dict."""

    def __init__(self, steer_edges, steer_centres, speed_edges, speed_centres):
        self.steer_edges = np.asarray(steer_edges, np.float64)
        self.steer_centres = np.asarray(steer_centres, np.float32)
        self.speed_edges = np.asarray(speed_edges, np.float64)
        self.speed_centres = np.asarray(speed_centres, np.float32)

    @classmethod
    def fit(cls, actions: np.ndarray, n_steer: int = 21, n_speed: int = 21):
        (slo, shi), (vlo, vhi) = ACTION_BOUNDS
        se = make_edges(actions[:, 0], n_steer, float(slo), float(shi))
        ve = make_edges(actions[:, 1], n_speed, float(vlo), float(vhi))
        return cls(se, centres_from_data(actions[:, 0], se), ve, centres_from_data(actions[:, 1], ve))

    @property
    def n_steer(self): return len(self.steer_centres)

    @property
    def n_speed(self): return len(self.speed_centres)

    def encode(self, actions: np.ndarray) -> np.ndarray:
        return np.stack([encode(actions[:, 0], self.steer_edges),
                         encode(actions[:, 1], self.speed_edges)], axis=1)

    def decode(self, idx: np.ndarray) -> np.ndarray:
        return np.stack([decode(idx[:, 0], self.steer_centres),
                         decode(idx[:, 1], self.speed_centres)], axis=1).astype(np.float32)

    def to_dict(self) -> dict:
        return {'steer_edges': self.steer_edges.tolist(), 'steer_centres': self.steer_centres.tolist(),
                'speed_edges': self.speed_edges.tolist(), 'speed_centres': self.speed_centres.tolist()}

    @classmethod
    def from_dict(cls, d: dict):
        return cls(d['steer_edges'], d['steer_centres'], d['speed_edges'], d['speed_centres'])

    def resolution_near_zero(self) -> float:
        """Width of the steering bin containing 0 rad -- the number to compare
        with the expert's own steering std (~0.05 rad)."""
        b = encode(np.array([0.0]), self.steer_edges)[0]
        return float(self.steer_edges[b + 1] - self.steer_edges[b])
