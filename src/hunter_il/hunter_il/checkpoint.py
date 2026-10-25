"""One checkpoint format for all three architectures.

The file is self-describing: arch, K, bins (for discrete), the schema version
it was trained against, and the split it was trained on. `load()` rebuilds
the exact model; `policy_tracker` calls nothing else.
"""
from __future__ import annotations

import time

import torch

from hunter_dataset.dataset_spec import SCHEMA_VERSION
from hunter_il import models
from hunter_il.bins import ActionBins


def save(path: str, model, arch: str, bins: ActionBins | None, train_config: dict, metrics: dict):
    torch.save({
        'format': 'hunter_il.checkpoint.v1',
        'arch': arch,
        'K': model.K,
        'cap_input': bool(getattr(model, 'cap_input', False)),
        'schema_version': SCHEMA_VERSION,
        'state_dict': model.state_dict(),
        'bins': bins.to_dict() if bins is not None else None,
        'train_config': train_config,
        'metrics': metrics,
        'saved_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
    }, path)


def load(path: str, map_location='cpu'):
    ck = torch.load(path, map_location=map_location, weights_only=False)
    if ck.get('format') != 'hunter_il.checkpoint.v1':
        raise ValueError(f'{path}: not a hunter_il checkpoint')
    if ck['schema_version'].split('.')[0] != SCHEMA_VERSION.split('.')[0]:
        raise ValueError(f'{path}: trained on schema {ck["schema_version"]}, spec is {SCHEMA_VERSION}')
    bins = ActionBins.from_dict(ck['bins']) if ck.get('bins') else None
    model = models.build(ck['arch'], bins, ck.get('cap_input', False))
    model.load_state_dict(ck['state_dict'])
    model.eval()
    return model, ck
