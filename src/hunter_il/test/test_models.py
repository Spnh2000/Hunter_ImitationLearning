import numpy as np, torch
from hunter_il import models, checkpoint
from hunter_il.bins import ActionBins
from hunter_dataset.dataset_spec import ACTION_BOUNDS


def test_outputs_within_bounds_all_archs(tmp_path):
    obs = torch.rand(8, 3, 720) * 30
    a = np.stack([np.random.uniform(-0.4, 0.4, 500), np.random.uniform(0.5, 2.7, 500)], 1).astype(np.float32)
    bins = ActionBins.fit(a, 21, 21)
    for arch in models.ARCHS:
        m = models.build(arch, bins)
        x = obs if arch == 'stacked' else obs[:, :1]
        y = m(x).detach().numpy()
        assert y.shape == (8, 2)
        assert (y[:, 0] >= ACTION_BOUNDS[0, 0]).all() and (y[:, 0] <= ACTION_BOUNDS[0, 1]).all()
        assert (y[:, 1] >= ACTION_BOUNDS[1, 0]).all() and (y[:, 1] <= ACTION_BOUNDS[1, 1]).all()
        # checkpoint round-trip rebuilds the same model
        p = str(tmp_path / f'{arch}.pt')
        checkpoint.save(p, m, arch, bins if arch == 'discrete' else None, {}, {})
        m2, ck = checkpoint.load(p)
        assert np.allclose(m2(x).detach().numpy(), y)
        assert ck['K'] == m.K


def test_wrong_K_raises():
    m = models.build('stacked')
    try:
        m(torch.rand(2, 1, 720)); assert False
    except ValueError:
        pass
