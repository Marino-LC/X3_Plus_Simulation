#!/usr/bin/env python3
"""
generate_map.py — v4: mapa de referencia de la arena, generado desde el mundo.

Lee world/arena_world.world y rasteriza a 2 cm las paredes y todos los modelos
estáticos cuyo nombre empieza por "wall_" u "obstacle_" (cajas y cilindros).
El mapa queda en el marco del MUNDO (map == world), así que config/goals.yaml
y la pose inicial de AMCL usan las mismas coordenadas que Gazebo.

Sirve como mapa estático para AMCL, como referencia para la ruta más corta
(SPL) y como verdad de terreno para la calidad del mapa de SLAM (IoU, error de
pared). Una celda se marca ocupada si se solapa con la geometría.

Uso:
  python3 generate_map.py [ruta_world] [directorio_salida]
  (por defecto: ../world/arena_world.world y el directorio de este script)
"""

import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

RESOLUTION = 0.02
ORIGIN     = -1.40    # esquina inferior izquierda del mapa (x e y), m
SIZE_PX    = 140      # 2.80 m: contiene las paredes completas
PREFIXES   = ("wall_", "obstacle_")

FREE, OCCUPIED, UNKNOWN = 254, 0, 205


def _cells(lo, hi):
    """Índices de las celdas que se solapan con el intervalo [lo, hi]."""
    i0 = math.floor(round((lo - ORIGIN) / RESOLUTION, 6))
    i1 = math.ceil(round((hi - ORIGIN) / RESOLUTION, 6)) - 1
    return max(0, i0), min(SIZE_PX - 1, i1)


def draw_rect(grid, xc, yc, w, h, value):
    c0, c1 = _cells(xc - w / 2, xc + w / 2)
    r0, r1 = _cells(yc - h / 2, yc + h / 2)
    grid[r0:r1 + 1, c0:c1 + 1] = value


def draw_circle(grid, xc, yc, r, value):
    """Marca las celdas cuyo cuadrado se solapa con el círculo."""
    c0, c1 = _cells(xc - r, xc + r)
    r0, r1 = _cells(yc - r, yc + r)
    for row in range(r0, r1 + 1):
        y0 = ORIGIN + row * RESOLUTION
        dy = max(y0 - yc, 0.0, yc - (y0 + RESOLUTION))
        for col in range(c0, c1 + 1):
            x0 = ORIGIN + col * RESOLUTION
            dx = max(x0 - xc, 0.0, xc - (x0 + RESOLUTION))
            if dx * dx + dy * dy < r * r - 1e-12:
                grid[row, col] = value


def read_world(path):
    """Devuelve [(nombre, 'box'|'cylinder', x, y, yaw, dims)] de paredes y obstáculos."""
    shapes = []
    root = ET.parse(path).getroot()
    for model in root.iter("model"):
        name = model.get("name", "")
        if not name.startswith(PREFIXES):
            continue
        x, y, _z, _r, _p, yaw = [float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()]
        geom = model.find("link/collision/geometry")
        if geom is None:
            continue
        if geom.find("box") is not None:
            sx, sy, _sz = [float(v) for v in geom.findtext("box/size").split()]
            shapes.append((name, "box", x, y, yaw, (sx, sy)))
        elif geom.find("cylinder") is not None:
            shapes.append((name, "cylinder", x, y, yaw, (float(geom.findtext("cylinder/radius")),)))
        else:
            raise ValueError(f"{name}: geometría no soportada")
    return shapes


def build_grid(shapes):
    grid = np.full((SIZE_PX, SIZE_PX), UNKNOWN, dtype=np.uint8)
    walls = [s for s in shapes if s[0].startswith("wall_")]
    # Interior libre: rectángulo delimitado por las caras interiores de las paredes
    xs = [s[2] for s in walls if s[5][0] < s[5][1]]     # paredes este/oeste
    ys = [s[3] for s in walls if s[5][0] > s[5][1]]     # paredes norte/sur
    t = min(min(s[5]) for s in walls)
    draw_rect(grid, (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2,
              max(xs) - min(xs) - t, max(ys) - min(ys) - t, FREE)
    for name, kind, x, y, yaw, dims in shapes:
        if kind == "cylinder":
            draw_circle(grid, x, y, dims[0], OCCUPIED)
            continue
        w, h = dims
        if name.startswith("wall_"):        # alarga las paredes para cerrar las esquinas
            w, h = (w + t, h) if w > h else (w, h + t)
        if abs(math.sin(yaw)) > 1e-6 and abs(math.cos(yaw)) > 1e-6:
            raise ValueError(f"{name}: solo se soportan cajas alineadas a los ejes")
        if abs(math.sin(yaw)) > 0.5:
            w, h = h, w
        draw_rect(grid, x, y, w, h, OCCUPIED)
    return grid


def main():
    here = Path(__file__).resolve().parent
    world = Path(sys.argv[1]) if len(sys.argv) > 1 else here.parent / "world" / "arena_world.world"
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else here
    shapes = read_world(world)
    grid = build_grid(shapes)

    with open(out_dir / "arena_map.pgm", "wb") as f:
        f.write(f"P5\n{SIZE_PX} {SIZE_PX}\n255\n".encode())
        f.write(np.flipud(grid).tobytes())                     # fila 0 del PGM = y máxima

    with open(out_dir / "arena_map.yaml", "w") as f:
        f.write(
            "image: arena_map.pgm\n"
            "mode: trinary\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{ORIGIN}, {ORIGIN}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.25\n")

    print(f"{len(shapes)} objetos leídos de {world.name}: "
          + ", ".join(s[0] for s in shapes))
    print(f"Mapa {SIZE_PX}x{SIZE_PX} px en {out_dir}  "
          f"libre={np.sum(grid == FREE)} ocupado={np.sum(grid == OCCUPIED)}")


if __name__ == "__main__":
    main()
