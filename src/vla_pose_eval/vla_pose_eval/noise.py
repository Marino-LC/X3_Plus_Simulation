"""Modelo de ruido de profundidad de una cámara de luz estructurada (Astra).

Numpy puro, para usarlo en el nodo ROS `depth_noise` y en pruebas sin ROS.
Ver config/noise.yaml para el significado y el origen de cada parámetro.
"""

from dataclasses import asdict, dataclass, fields

import numpy as np


@dataclass
class DepthNoiseModel:
    enabled: bool = True
    seed: int = 0
    max_rate_hz: float = 10.0
    sigma_const_m: float = 0.0005
    sigma_quad_per_m: float = 0.0016
    correlated_fraction: float = 0.5
    correlation_px: int = 8
    disparity_fx_px: float = 570.0
    baseline_m: float = 0.075
    subpixel: int = 8
    quantize_mm: bool = True
    dropout_prob: float = 0.002
    edge_threshold_m: float = 0.05
    edge_width_px: int = 2
    edge_dropout_prob: float = 0.8
    min_range_m: float = 0.6
    max_range_m: float = 8.0

    @classmethod
    def from_config(cls, d):
        d = d or {}
        names = {f.name for f in fields(cls)}
        unknown = set(d) - names
        if unknown:
            raise ValueError('parámetros de ruido desconocidos: %s' % ', '.join(sorted(unknown)))
        return cls(**d)

    def to_dict(self):
        return asdict(self)

    def sigma(self, z):
        """Desviación axial (m) a la distancia z (m)."""
        return self.sigma_const_m + self.sigma_quad_per_m * np.square(z)

    def rng_for(self, *keys):
        """Generador reproducible para una imagen (p. ej. su sello de tiempo)."""
        return np.random.default_rng([int(self.seed)] + [int(k) for k in keys])

    # ------------------------------------------------------------------------
    def _correlated_field(self, shape, rng):
        """Campo N(0, 1) suave por bloques de `correlation_px` (interpolación bilineal)."""
        h, w = shape
        c = max(1, int(self.correlation_px))
        gh, gw = h // c + 2, w // c + 2
        grid = rng.standard_normal((gh, gw))
        ys = np.arange(h) / c
        xs = np.arange(w) / c
        y0, x0 = np.floor(ys).astype(int), np.floor(xs).astype(int)
        fy, fx = (ys - y0)[:, None], (xs - x0)[None, :]
        a = grid[y0][:, x0]
        b = grid[y0][:, x0 + 1]
        cc = grid[y0 + 1][:, x0]
        d = grid[y0 + 1][:, x0 + 1]
        f = (a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + cc * (1 - fx) * fy + d * fx * fy)
        # La interpolación bilineal reduce la varianza según la posición dentro
        # del bloque; se renormaliza píxel a píxel para que sea exactamente 1.
        w2 = ((1 - fx) ** 2 + fx ** 2) * ((1 - fy) ** 2 + fy ** 2)
        return f / np.sqrt(w2)

    def _edge_mask(self, depth):
        """Píxeles del lado lejano de un salto de profundidad (sombra), ensanchados."""
        h, w = depth.shape
        far = np.zeros((h, w), bool)
        with np.errstate(invalid='ignore'):
            dx = depth[:, 1:] - depth[:, :-1]
            dy = depth[1:, :] - depth[:-1, :]
            far[:, 1:] |= dx > self.edge_threshold_m      # el de la derecha está más lejos
            far[:, :-1] |= -dx > self.edge_threshold_m    # el de la izquierda está más lejos
            far[1:, :] |= dy > self.edge_threshold_m
            far[:-1, :] |= -dy > self.edge_threshold_m
        out = far.copy()
        for k in range(1, max(0, int(self.edge_width_px))):
            out[:, k:] |= far[:, :-k]
            out[:, :-k] |= far[:, k:]
            out[k:, :] |= far[:-k, :]
            out[:-k, :] |= far[k:, :]
        return out

    def apply(self, depth, rng):
        """Profundidad limpia (m, NaN/inf/<=0 = sin dato) -> profundidad con ruido.

        Devuelve float32; los píxeles sin dato quedan en NaN.
        """
        d = np.array(depth, dtype=np.float64, copy=True)
        valid = np.isfinite(d) & (d > 0.0)
        if not self.enabled:
            d[~valid] = np.nan
            return d.astype(np.float32)
        edges = self._edge_mask(np.where(valid, d, np.inf))

        z = d[valid]
        sig = self.sigma(z)
        f = float(np.clip(self.correlated_fraction, 0.0, 1.0))
        noise = np.sqrt(1.0 - f) * rng.standard_normal(z.shape)
        if f > 0.0:
            noise += np.sqrt(f) * self._correlated_field(d.shape, rng)[valid]
        z = z + sig * noise

        if self.subpixel and self.subpixel > 0:
            k = self.disparity_fx_px * self.baseline_m
            disp = np.round(k / np.maximum(z, 1e-6) * self.subpixel) / self.subpixel
            z = np.where(disp > 0.0, k / np.maximum(disp, 1e-12), np.nan)
        if self.quantize_mm:
            z = np.round(z * 1000.0) / 1000.0
        d[valid] = z

        drop = ~valid
        drop |= edges & (rng.random(d.shape) < self.edge_dropout_prob)
        drop |= rng.random(d.shape) < self.dropout_prob
        with np.errstate(invalid='ignore'):
            drop |= (d < self.min_range_m) | (d > self.max_range_m)
        d[drop] = np.nan
        return d.astype(np.float32)
