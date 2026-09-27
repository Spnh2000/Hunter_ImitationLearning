"""Discretisation for the classification policy.

Edges are computed ONCE from training actions and then live in the
checkpoint. A model whose outputs cannot be decoded without a separate file is
a model that will be decoded wrong.

Why NOT pure quantile edges: steering spends ~80 % of its time within
+-0.05 rad of zero, and the physical limit is +-0.461. Uniform bins would put
most of the resolution where nothing happens -- but pure quantile bins
collapse (see make_edges). The edges are a 50/50 blend of the two with a floor
under the bin width, and the outer edges pinned to the physical bounds so
out-of-distribution values still land in a bin.

Bin centres are the MEAN training value inside each bin, not the midpoint:
for a skewed bin (the outermost ones) the midpoint is systematically off.

"""
from __future__ import annotations

import numpy as np

from hunter_dataset.dataset_spec import ACTION_BOUNDS


def make_edges(values: np.ndarray, n_bins: int, lo: float, hi: float,
               quantile_weight: float = 0.5, min_width_frac: float = 0.02) -> np.ndarray:
    """n_bins + 1 strictly increasing edges in [lo, hi].

    Pure quantile edges DEGENERATE on this data and did so in the first run:
    the expert holds almost exactly straight for most of a lap, so the inner
    quantiles pile up and the outer two bins absorb the rest of the range.
    Measured with quantile_weight=1.0 on 186k expert rows: widest bin 9,931x
    the narrowest, eight of 21 steering bins resolving below 6e-4 rad (finer
    than the actuator), and the two end bins covering 84 % and 89 % of their
    side -- so a 0.4 rad corner decoded to 0.106 rad. Speed was worse: 14 of
    21 bins under 1e-3 m/s.

    So blend the quantile edges with UNIFORM ones and put a floor under the
    width. quantile_weight=0.5 keeps the useful property (finer near zero,
    where the driving is) without letting any bin collapse.
    min_width_frac is a fraction of the full range: 0.02 of +-0.461 rad is
    0.018 rad, comfortably below the expert's own steering std (~0.05) and
    comfortably above anything the steering cannot express.
    """
    v = np.clip(values.astype(np.float64), lo, hi)
    q = np.linspace(0.0, 1.0, n_bins + 1)
    quant = np.quantile(v, q)
    unif = np.linspace(lo, hi, n_bins + 1)
    edges = quantile_weight * quant + (1.0 - quantile_weight) * unif
    edges[0], edges[-1] = lo, hi

    # Enforce the floor left-to-right, then rescale so the last edge still
    # lands on hi -- pushing edges right without rescaling would walk the top
    # bin past the physical bound.
    min_w = (hi - lo) * min_width_frac
    for i in range(1, len(edges)):
        edges[i] = max(edges[i], edges[i - 1] + min_w)
    if edges[-1] > hi:
        edges = lo + (edges - lo) * (hi - lo) / (edges[-1] - lo)
    edges[-1] = hi
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
