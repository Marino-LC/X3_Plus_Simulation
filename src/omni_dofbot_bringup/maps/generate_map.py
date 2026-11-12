#!/usr/bin/env python3
"""
generate_map.py — v3: mapa de referencia de la arena, alineado con el mundo.

Genera arena_map.pgm + arena_map.yaml en el marco del mundo de
arena_world.world (map == world), de modo que waypoints.yaml y la pose
inicial de AMCL se expresan en las mismas coordenadas que Gazebo.

Sirve como mapa estático para AMCL y como referencia (verdad de terreno)
para las métricas de calidad de mapa (IoU, error de pared).

Geometría (idéntica a arena_world.world):
  - Paredes de 0.03 m de espesor CENTRADAS en ±1.35 m (cara interior ±1.335).
  - O1, O2 y pedestal de 0.15 × 0.15 m, sin margen artificial.
Una celda se marca ocupada si se solapa con el objeto.

Uso (escribe junto a este script, o en el directorio dado):
  python3 generate_map.py [directorio_salida]
"""

import math
import sys
from pathlib import Path

import numpy as np

RESOLUTION = 0.02
ARENA      = 2.7      # distancia entre centros de paredes opuestas
WALL_T     = 0.03
ORIGIN     = -1.40    # esquina inferior izquierda del mapa (x e y)
SIZE_PX    = 140      # 2.80 m: contiene las paredes completas

FREE, OCCUPIED, UNKNOWN = 254, 0, 205

# (x_centro, y_centro, ancho_x, ancho_y)
OBSTACLES = [
    (0.00,  0.25, 0.15, 0.15),   # O1
    (0.00, -0.30, 0.15, 0.15),   # O2
    (0.00,  0.81, 0.15, 0.15),   # Pedestal
]


def _cells(lo, hi):
    """Índices de las celdas que se solapan con el intervalo [lo, hi]."""
    i0 = math.floor(round((lo - ORIGIN) / RESOLUTION, 6))
    i1 = math.ceil(round((hi - ORIGIN) / RESOLUTION, 6)) - 1
    return max(0, i0), min(SIZE_PX - 1, i1)


def draw_rect(grid, xc, yc, w, h, value):
    c0, c1 = _cells(xc - w / 2, xc + w / 2)
    r0, r1 = _cells(yc - h / 2, yc + h / 2)
    grid[r0:r1 + 1, c0:c1 + 1] = value


def main():
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent
    grid = np.full((SIZE_PX, SIZE_PX), UNKNOWN, dtype=np.uint8)

    half = ARENA / 2.0
    inner = ARENA - WALL_T
    draw_rect(grid, 0, 0, inner, inner, FREE)                  # interior

    wall_len = ARENA + WALL_T                                  # cierra las esquinas
    draw_rect(grid, 0,  half, wall_len, WALL_T, OCCUPIED)      # norte
    draw_rect(grid, 0, -half, wall_len, WALL_T, OCCUPIED)      # sur
    draw_rect(grid,  half, 0, WALL_T, wall_len, OCCUPIED)      # este
    draw_rect(grid, -half, 0, WALL_T, wall_len, OCCUPIED)      # oeste

    for (xc, yc, w, h) in OBSTACLES:
        draw_rect(grid, xc, yc, w, h, OCCUPIED)

    with open(out_dir / 'arena_map.pgm', 'wb') as f:
        f.write(f'P5\n{SIZE_PX} {SIZE_PX}\n255\n'.encode())
        f.write(np.flipud(grid).tobytes())                     # fila 0 del PGM = y máxima

    with open(out_dir / 'arena_map.yaml', 'w') as f:
        f.write(
            'image: arena_map.pgm\n'
            'mode: trinary\n'
            f'resolution: {RESOLUTION}\n'
            f'origin: [{ORIGIN}, {ORIGIN}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n')

    print(f'Mapa {SIZE_PX}x{SIZE_PX} px en {out_dir}  '
          f'libre={np.sum(grid == FREE)} ocupado={np.sum(grid == OCCUPIED)}')


if __name__ == '__main__':
    main()
