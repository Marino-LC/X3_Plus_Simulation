import numpy as np
import pytest

from vla_pose_eval.noise import DepthNoiseModel


def _model(**kw):
    base = dict(subpixel=0, quantize_mm=False, dropout_prob=0.0, edge_dropout_prob=0.0)
    return DepthNoiseModel(**dict(base, **kw))


def test_config_file_loads(cfg):
    m = DepthNoiseModel.from_config(cfg['noise']['depth'])
    assert m.enabled and m.sigma(1.0) > 0
    with pytest.raises(ValueError):
        DepthNoiseModel.from_config({'sigma_typo': 1.0})


@pytest.mark.parametrize('z', [0.8, 1.5, 3.0])
@pytest.mark.parametrize('corr', [0.0, 0.5, 1.0])
def test_axial_sigma_grows_with_z_squared(z, corr):
    m = _model(correlated_fraction=corr)
    out = m.apply(np.full((480, 640), z), np.random.default_rng(3))
    assert np.nanmean(out) == pytest.approx(z, abs=3 * m.sigma(z) / 50)
    assert np.nanstd(out) == pytest.approx(m.sigma(z), rel=0.1)


def test_correlated_noise_is_spatially_smooth():
    iid = _model(correlated_fraction=0.0).apply(np.full((240, 320), 1.0), np.random.default_rng(0))
    cor = _model(correlated_fraction=1.0).apply(np.full((240, 320), 1.0), np.random.default_rng(0))

    def neighbour_corr(a):
        a = a - a.mean()
        return float((a[:, 1:] * a[:, :-1]).mean() / a.var())

    assert abs(neighbour_corr(iid)) < 0.05
    assert neighbour_corr(cor) > 0.8


def test_disparity_quantization_steps():
    m = _model(sigma_const_m=0.0, sigma_quad_per_m=0.0, subpixel=8)
    z = np.linspace(1.0, 2.0, 2000)[None, :]
    out = m.apply(z, np.random.default_rng(0))[0]
    levels = np.unique(out)
    k = m.disparity_fx_px * m.baseline_m
    step_at_2m = 2.0 ** 2 / (k * m.subpixel)            # dz = z^2 / (f B s)
    assert np.max(np.diff(levels)) == pytest.approx(step_at_2m, rel=0.05)
    assert np.allclose(k / levels * m.subpixel, np.round(k / levels * m.subpixel), atol=1e-6)


def test_millimetre_quantization_and_invalid_pixels():
    m = _model(sigma_const_m=0.0, sigma_quad_per_m=0.0, quantize_mm=True)
    d = np.full((10, 10), 1.23456)
    d[0, 0], d[0, 1], d[0, 2], d[0, 3] = np.nan, np.inf, 0.0, 0.3     # 0.3 < min_range
    out = m.apply(d, np.random.default_rng(0))
    assert out.dtype == np.float32
    assert np.isnan(out[0, :4]).all()
    assert out[5, 5] == pytest.approx(1.235, abs=1e-6)


def test_edge_shadow_on_far_side():
    m = _model(sigma_const_m=0.0, sigma_quad_per_m=0.0, edge_dropout_prob=1.0, edge_width_px=2)
    d = np.full((20, 40), 2.0)
    d[:, :20] = 1.0                                     # objeto cercano a la izquierda
    out = m.apply(d, np.random.default_rng(0))
    assert np.isnan(out[:, 20:22]).all()                # lado lejano del borde: hueco
    assert np.isfinite(out[:, :19]).all()               # el objeto cercano conserva dato
    assert np.isfinite(out[:, 23:]).all()


def test_random_dropout_rate():
    m = _model(dropout_prob=0.05)
    out = m.apply(np.full((400, 400), 1.5), np.random.default_rng(1))
    assert np.isnan(out).mean() == pytest.approx(0.05, abs=0.005)


def test_reproducible_and_disabled():
    m = DepthNoiseModel()
    d = np.random.default_rng(0).uniform(0.7, 3.0, (60, 80))
    assert np.array_equal(m.apply(d, m.rng_for(10, 5)), m.apply(d, m.rng_for(10, 5)),
                          equal_nan=True)
    assert not np.array_equal(m.apply(d, m.rng_for(10, 5)), m.apply(d, m.rng_for(10, 6)),
                              equal_nan=True)
    off = DepthNoiseModel(enabled=False).apply(d, np.random.default_rng(0))
    assert np.allclose(off, d.astype(np.float32))


def test_baseline_lift_with_noisy_depth():
    """La línea base sigue recuperando el centro con la profundidad ruidosa (a 1.2 m)."""
    from vla_pose_eval import geometry as g
    from vla_pose_eval.estimators.detector_depth import Box2D, DetectorDepthEstimator
    K = g.intrinsics_from_fov(1280, 720, 1.0472)
    center, half = np.array([0.15, 0.10, 1.20]), 0.02
    depth = np.full((720, 1280), 2.5)
    u, v, _ = g.project(K, center)
    r = int(K[0, 0] * half / (center[2] - half))
    depth[int(v) - r:int(v) + r, int(u) - r:int(u) + r] = center[2] - half
    noisy = DepthNoiseModel().apply(depth, np.random.default_rng(0))
    box = Box2D('cubo', 0.9, (u - r - 6, v - r - 6, u + r + 6, v + r + 6))
    p = DetectorDepthEstimator(center_offsets={'cubo': half}).lift(box, noisy, K)
    assert np.linalg.norm(p - center) < 0.01
