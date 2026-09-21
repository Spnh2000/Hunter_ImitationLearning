"""The three Phase 8 architectures. One trunk, three input/output configurations.

Every model takes RAW observations in metres, shape [B, K, 720], and returns
actions in physical units (rad, m/s). Normalisation is inside the model so
that the deployment node feeds exactly what the dataset holds and gets
exactly what the controller expects -- there is no second place for a scale
factor to drift.

    mlp       K=1, regression, MSE on bound-normalised actions
    stacked   K=3, same head, input is the K scans concatenated
    discrete  K=1, two classification heads over ActionBins, cross-entropy
"""
from __future__ import annotations

import numpy as np
import torch
import torch.nn as nn

from hunter_dataset.dataset_spec import ACTION_BOUNDS, OBS_DIM, RANGE_MAX

ARCHS = ('mlp', 'stacked', 'discrete')
HIDDEN = (256, 128, 64)


def _trunk(in_dim: int, hidden=HIDDEN) -> nn.Sequential:
    layers, d = [], in_dim
    for h in hidden:
        layers += [nn.Linear(d, h), nn.ReLU()]
        d = h
    return nn.Sequential(*layers)


class _Base(nn.Module):
    def __init__(self, K: int):
        super().__init__()
        self.K = K
        self.register_buffer('obs_scale', torch.tensor(float(RANGE_MAX)))
        self.register_buffer('act_lo', torch.tensor(ACTION_BOUNDS[:, 0]))
        self.register_buffer('act_hi', torch.tensor(ACTION_BOUNDS[:, 1]))

    def _flat(self, obs: torch.Tensor) -> torch.Tensor:
        if obs.dim() == 2:                       # [B, 720] -> [B, 1, 720]
            obs = obs.unsqueeze(1)
        if obs.shape[1] != self.K or obs.shape[2] != OBS_DIM:
            raise ValueError(f'expected [B, {self.K}, {OBS_DIM}], got {tuple(obs.shape)}')
        return (obs / self.obs_scale).reshape(obs.shape[0], -1)

    def norm_act(self, act: torch.Tensor) -> torch.Tensor:
        """Physical -> [-1, 1] per dim, so steering and speed weigh equally in MSE."""
        return (act - self.act_lo) / (self.act_hi - self.act_lo) * 2.0 - 1.0

    def denorm_act(self, z: torch.Tensor) -> torch.Tensor:
        return (z + 1.0) / 2.0 * (self.act_hi - self.act_lo) + self.act_lo

    def clamp(self, act: torch.Tensor) -> torch.Tensor:
        return torch.maximum(torch.minimum(act, self.act_hi), self.act_lo)


class MlpPolicy(_Base):
    """720*K -> 256 -> 128 -> 64 -> 2, regression."""

    def __init__(self, K: int = 1):
        super().__init__(K)
        self.trunk = _trunk(OBS_DIM * K)
        self.head = nn.Linear(HIDDEN[-1], 2)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        """Physical actions, clamped to bounds."""
        return self.clamp(self.denorm_act(self.head(self.trunk(self._flat(obs)))))

    def loss(self, obs: torch.Tensor, act: torch.Tensor) -> torch.Tensor:
        z = self.head(self.trunk(self._flat(obs)))
        return nn.functional.mse_loss(z, self.norm_act(act))


class DiscretePolicy(_Base):
    """Same trunk; two softmax heads over steering and speed bins."""

    def __init__(self, steer_centres, speed_centres, K: int = 1):
        super().__init__(K)
        self.trunk = _trunk(OBS_DIM * K)
        self.register_buffer('steer_centres', torch.as_tensor(np.asarray(steer_centres, np.float32)))
        self.register_buffer('speed_centres', torch.as_tensor(np.asarray(speed_centres, np.float32)))
        self.head_steer = nn.Linear(HIDDEN[-1], len(self.steer_centres))
        self.head_speed = nn.Linear(HIDDEN[-1], len(self.speed_centres))

    def logits(self, obs):
        h = self.trunk(self._flat(obs))
        return self.head_steer(h), self.head_speed(h)

    def forward(self, obs: torch.Tensor, mode: str = 'argmax') -> torch.Tensor:
        ls, lv = self.logits(obs)
        if mode == 'argmax':
            s = self.steer_centres[ls.argmax(-1)]
            v = self.speed_centres[lv.argmax(-1)]
        elif mode == 'expected':
            s = (ls.softmax(-1) * self.steer_centres).sum(-1)
            v = (lv.softmax(-1) * self.speed_centres).sum(-1)
        else:
            raise ValueError(mode)
        return self.clamp(torch.stack([s, v], -1))

    def loss(self, obs: torch.Tensor, target_idx: torch.Tensor) -> torch.Tensor:
        """target_idx [B, 2] int64 from ActionBins.encode."""
        ls, lv = self.logits(obs)
        return (nn.functional.cross_entropy(ls, target_idx[:, 0])
                + nn.functional.cross_entropy(lv, target_idx[:, 1]))


def build(arch: str, bins=None) -> _Base:
    if arch == 'mlp':
        return MlpPolicy(K=1)
    if arch == 'stacked':
        return MlpPolicy(K=3)
    if arch == 'discrete':
        if bins is None:
            raise ValueError('discrete needs ActionBins')
        return DiscretePolicy(bins.steer_centres, bins.speed_centres, K=1)
    raise ValueError(f'arch must be one of {ARCHS}, got {arch}')


def k_for(arch: str) -> int:
    return 3 if arch == 'stacked' else 1
