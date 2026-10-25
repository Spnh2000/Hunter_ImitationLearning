import numpy as np, torch
from hunter_il import models, checkpoint
from hunter_il.bins import ActionBins
from hunter_dataset.dataset_spec import ACTION_BOUNDS


def test_outputs_within_bounds_all_archs(tmp_path):
    obs = torch.rand(8, 3, 720) * 30
    a = np.stack([np.random.uniform(-0.4, 0.4, 500), np.random.uniform(0.5, 2.7, 500)], 1).astype(np.float32)
    bins = ActionBins.fit(a, 21, 21)
    cap = torch.full((8,), 2.1)
    for arch in models.ARCHS:
        m = models.build(arch, bins)
        x = obs if arch == 'stacked' else obs[:, :1]
        y = m(x, cap).detach().numpy()
        assert y.shape == (8, 2)
        assert (y[:, 0] >= ACTION_BOUNDS[0, 0]).all() and (y[:, 0] <= ACTION_BOUNDS[0, 1]).all()
        assert (y[:, 1] >= ACTION_BOUNDS[1, 0]).all() and (y[:, 1] <= ACTION_BOUNDS[1, 1]).all()
        # checkpoint round-trip rebuilds the same model
        p = str(tmp_path / f'{arch}.pt')
        checkpoint.save(p, m, arch, bins if arch == 'discrete' else None, {}, {})
        m2, ck = checkpoint.load(p)
        assert np.allclose(m2(x, cap).detach().numpy(), y)
        assert ck['K'] == m.K and ck['cap_input'] is True


def test_wrong_K_raises():
    m = models.build('stacked')
    try:
        m(torch.rand(2, 1, 720), torch.ones(2)); assert False
    except ValueError:
        pass


def test_cap_input_is_required_when_enabled_and_absent_when_disabled():
    m = models.build('mlp', cap_input=True)
    try:
        m(torch.rand(2, 1, 720)); assert False
    except ValueError:
        pass
    m0 = models.build('mlp', cap_input=False)
    assert m0(torch.rand(2, 1, 720)).shape == (2, 2)
    assert m.in_dim() == 721 and m0.in_dim() == 720


def test_cap_changes_the_output():
    torch.manual_seed(0)
    m = models.build('mlp', cap_input=True)
    x = torch.rand(1, 1, 720) * 30
    assert not torch.allclose(m(x, torch.tensor([1.5])), m(x, torch.tensor([2.7])))
