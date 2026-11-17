"""
nav_metrics.py — métricas de navegación del Artículo 1 (sin dependencias de ROS).

Lo usa waypoints_nav.py durante las corridas y también funciona fuera de línea
para regenerar resumen y gráficas de una carpeta de resultados:

  python3 -m omni_dofbot_bringup.nav_metrics results/<fecha>

Convenciones:
  - Poses SE(2) como (x, y, yaw). Todo se evalúa en el marco del MUNDO de
    Gazebo; las poses de otros marcos (map, odom) se llevan al mundo con una
    transformación T_world_<marco> guardada en cada episodio.
  - Mapas de ocupación como arreglos numpy con la fila 0 en la y MÍNIMA
    (al revés que el PGM).
"""

import csv
import json
import math
import os
import heapq

import numpy as np
import yaml


# ══════════════════════════════════════════════════════════════════════════════
# SE(2)
# ══════════════════════════════════════════════════════════════════════════════

def wrap(a):
    """Ángulo(s) a (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def se2_compose(a, b):
    """a ∘ b para poses (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    c, s = math.cos(at), math.sin(at)
    return (ax + c * bx - s * by, ay + s * bx + c * by, float(wrap(at + bt)))


def se2_inverse(a):
    x, y, t = a
    c, s = math.cos(t), math.sin(t)
    return (-c * x - s * y, s * x - c * y, float(wrap(-t)))


def se2_apply(T, xy):
    """Aplica T a un arreglo Nx2 de puntos."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    c, s = math.cos(T[2]), math.sin(T[2])
    return np.column_stack((T[0] + c * xy[:, 0] - s * xy[:, 1],
                            T[1] + s * xy[:, 0] + c * xy[:, 1]))


def align_from_pair(pose_world, pose_frame):
    """T_world_frame tal que T ∘ pose_frame = pose_world (alineación por una pose)."""
    return se2_compose(pose_world, se2_inverse(pose_frame))


IDENTITY = (0.0, 0.0, 0.0)


# ══════════════════════════════════════════════════════════════════════════════
# Mapas
# ══════════════════════════════════════════════════════════════════════════════

class GridMap:
    """Mapa de ocupación en el marco del mundo (fila 0 = y mínima)."""

    def __init__(self, occupied, free, resolution, origin):
        self.occupied = occupied          # bool HxW
        self.free = free                  # bool HxW
        self.res = float(resolution)
        self.ox, self.oy = float(origin[0]), float(origin[1])

    @property
    def shape(self):
        return self.occupied.shape

    def cell_of(self, x, y):
        return (int(math.floor((y - self.oy) / self.res)),
                int(math.floor((x - self.ox) / self.res)))

    def center_of(self, row, col):
        return (self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res)

    def inside(self, row, col):
        return 0 <= row < self.shape[0] and 0 <= col < self.shape[1]

    def occupied_centers(self):
        rows, cols = np.nonzero(self.occupied)
        return np.column_stack((self.ox + (cols + 0.5) * self.res,
                                self.oy + (rows + 0.5) * self.res))

    def extent(self):
        h, w = self.shape
        return (self.ox, self.ox + w * self.res, self.oy, self.oy + h * self.res)


def _read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, idx = [], 0
    while len(tokens) < 4:                      # P5, ancho, alto, maxval (con comentarios)
        while data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            idx = data.index(b"\n", idx) + 1
            continue
        end = idx
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[idx:end].decode())
        idx = end
    idx += 1
    if tokens[0] != "P5":
        raise ValueError(f"{path}: solo se soporta PGM binario (P5)")
    w, h, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    img = np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)
    return img.astype(float) / maxval


def load_map_yaml(yaml_path):
    """Carga un mapa en formato map_server (yaml + pgm)."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(yaml_path), meta["image"])
    img = np.flipud(_read_pgm(img_path))
    occ_prob = img if meta.get("negate", 0) else 1.0 - img
    occupied = occ_prob > float(meta["occupied_thresh"])
    free = occ_prob < float(meta["free_thresh"])
    return GridMap(occupied, free, meta["resolution"], meta["origin"][:2])


def dilate(mask, radius_cells):
    """Dilatación binaria con un disco de radio radius_cells."""
    out = mask.copy()
    r = int(math.ceil(radius_cells))
    h, w = mask.shape
    for dr in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dr == 0 and dc == 0 or dr * dr + dc * dc > radius_cells * radius_cells:
                continue
            src = mask[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
            out[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    return out


def boundary_cells(mask):
    """Celdas ocupadas con al menos un vecino-4 libre (la cara que ve un LiDAR)."""
    pad = np.pad(~mask, 1, constant_values=False)
    neigh_free = pad[:-2, 1:-1] | pad[2:, 1:-1] | pad[1:-1, :-2] | pad[1:-1, 2:]
    return mask & neigh_free


# ══════════════════════════════════════════════════════════════════════════════
# Huella y ruta más corta de referencia (SPL)
# ══════════════════════════════════════════════════════════════════════════════

def footprint_clearance(gmap, pose, x_min, x_max, y_half):
    """Distancia mínima (m) entre la huella rectangular en `pose` y las celdas
    ocupadas (cada celda tratada como un cuadrado). Negativa si se solapan."""
    pts = gmap.occupied_centers()
    if len(pts) == 0:
        return float("inf")
    local = se2_apply(se2_inverse(pose), pts)
    dx = np.maximum.reduce([x_min - local[:, 0], np.zeros(len(local)), local[:, 0] - x_max])
    dy = np.maximum(np.abs(local[:, 1]) - y_half, 0.0)
    d = np.hypot(dx, dy) - gmap.res / 2.0
    inside = (dx == 0) & (dy == 0)
    d[inside] = -gmap.res / 2.0
    return float(d.min())


def shortest_path(gmap, start_xy, goal_xy, robot_radius):
    """Ruta más corta (m) entre dos puntos para un robot de radio robot_radius,
    sobre el mapa de referencia. A* 8-conexo y después recorte por línea de
    vista, que la acerca a la geodésica real. Las celdas desconocidas se tratan
    como ocupadas. Devuelve (longitud, Nx2 puntos) o (inf, None)."""
    blocked = dilate(gmap.occupied | ~gmap.free, robot_radius / gmap.res)
    start, goal = gmap.cell_of(*start_xy), gmap.cell_of(*goal_xy)
    for name, cell in (("inicio", start), ("goal", goal)):
        if not gmap.inside(*cell) or blocked[cell]:
            raise ValueError(f"El punto de {name} {cell} está bloqueado en el mapa inflado")

    h, w = blocked.shape
    steps = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1)
             if dr or dc]
    g = np.full((h, w), np.inf)
    parent = {}
    g[start] = 0.0
    heap = [(0.0, start)]
    while heap:
        _, cur = heapq.heappop(heap)
        if cur == goal:
            break
        for dr, dc, cost in steps:
            nr, nc = cur[0] + dr, cur[1] + dc
            if not (0 <= nr < h and 0 <= nc < w) or blocked[nr, nc]:
                continue
            if dr and dc and (blocked[cur[0] + dr, cur[1]] or blocked[cur[0], cur[1] + dc]):
                continue                                   # no cortar esquinas
            ng = g[cur] + cost
            if ng < g[nr, nc]:
                g[nr, nc] = ng
                parent[(nr, nc)] = cur
                heapq.heappush(heap, (ng + math.hypot(goal[0] - nr, goal[1] - nc), (nr, nc)))
    if not np.isfinite(g[goal]):
        return float("inf"), None

    cells = [goal]
    while cells[-1] != start:
        cells.append(parent[cells[-1]])
    cells.reverse()
    pts = [start_xy] + [gmap.center_of(*c) for c in cells[1:-1]] + [goal_xy]

    def visible(a, b):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (gmap.res / 4.0)))
        for t in np.linspace(0.0, 1.0, n):
            r, c = gmap.cell_of(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            if blocked[r, c]:
                return False
        return True

    smooth, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not visible(pts[i], pts[j]):
            j -= 1
        smooth.append(pts[j])
        i = j
    smooth = np.array(smooth)
    return float(np.sum(np.hypot(*np.diff(smooth, axis=0).T))), smooth


# ══════════════════════════════════════════════════════════════════════════════
# Métricas por episodio
# ══════════════════════════════════════════════════════════════════════════════

def path_length(xy):
    xy = np.asarray(xy, dtype=float)
    if len(xy) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(xy, axis=0).T)))


def polyline_projection(points, poly):
    """Distancia de cada punto a la polilínea y rumbo del segmento más cercano."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    poly = np.asarray(poly, dtype=float).reshape(-1, 2)
    if len(poly) == 1:
        d = np.hypot(*(points - poly[0]).T)
        return d, np.full(len(points), np.nan)
    a, b = poly[:-1], poly[1:]
    ab = b - a
    len2 = np.maximum(np.sum(ab * ab, axis=1), 1e-12)
    ap = points[:, None, :] - a[None, :, :]
    t = np.clip(np.sum(ap * ab[None], axis=2) / len2[None], 0.0, 1.0)
    proj = a[None] + t[..., None] * ab[None]
    d = np.hypot(*(points[:, None, :] - proj).transpose(2, 0, 1))
    k = np.argmin(d, axis=1)
    heading = np.arctan2(ab[k, 1], ab[k, 0])
    return d[np.arange(len(points)), k], heading


def rmse(v):
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    return float(np.sqrt(np.mean(v * v))) if len(v) else None


def episode_metrics(traj, plans, goal, T_world_map, T_world_odom, shortest_len,
                    success, heading_skip_radius=0.15, min_speed=0.02):
    """Métricas de trayectoria de un episodio.

    traj   : arreglo Nx10 [t, gt_x, gt_y, gt_yaw, odom_x, odom_y, odom_yaw,
             est_x, est_y, est_yaw]; gt en el mundo, odom en el marco odom,
             est (localización) en el marco map.
    plans  : lista de (t_recepción, Mx2 en el marco map).
    goal   : (x, y, yaw) en el mundo.
    """
    traj = np.asarray(traj, dtype=float).reshape(-1, 10)
    out = {"path_length": None, "spl": 0.0, "ate_localization": None,
           "ate_odometry": None, "yaw_rmse_localization": None,
           "cte_rmse": None, "cte_max": None, "heading_rmse": None,
           "cte_initial_plan_rmse": None, "n_samples": int(len(traj))}
    if len(traj) == 0:
        return out
    t, gt = traj[:, 0], traj[:, 1:4]

    p_len = path_length(gt[:, :2])
    out["path_length"] = p_len
    if success and shortest_len and np.isfinite(shortest_len):
        out["spl"] = float(shortest_len / max(p_len, shortest_len))

    # ATE: pose estimada llevada al mundo vs verdad de terreno
    est = traj[:, 7:10]
    ok = np.all(np.isfinite(est), axis=1)
    if ok.any():
        est_w = se2_apply(T_world_map, est[ok, :2])
        out["ate_localization"] = rmse(np.hypot(*(est_w - gt[ok, :2]).T))
        out["yaw_rmse_localization"] = rmse(wrap(est[ok, 2] + T_world_map[2] - gt[ok, 2]))
    odo = traj[:, 4:7]
    ok = np.all(np.isfinite(odo), axis=1)
    if ok.any():
        odo_w = se2_apply(T_world_odom, odo[ok, :2])
        out["ate_odometry"] = rmse(np.hypot(*(odo_w - gt[ok, :2]).T))

    # Error de seguimiento contra el plan ACTIVO (el último recibido antes de t)
    if plans:
        plans = sorted(plans, key=lambda p: p[0])
        plans_w = [(pt, se2_apply(T_world_map, xy)) for pt, xy in plans if len(xy)]
        cte = np.full(len(t), np.nan)
        hdg = np.full(len(t), np.nan)
        stamps = np.array([p[0] for p in plans_w])
        idx = np.searchsorted(stamps, t, side="right") - 1
        for k in np.unique(idx[idx >= 0]):
            sel = idx == k
            d, h = polyline_projection(gt[sel, :2], plans_w[k][1])
            cte[sel] = d
            hdg[sel] = wrap(gt[sel, 2] - h)

        # Rumbo: se excluyen el giro final sobre el goal y los instantes detenido
        speed = np.zeros(len(t))
        if len(t) > 1:
            v = np.hypot(*np.diff(gt[:, :2], axis=0).T) / np.maximum(np.diff(t), 1e-6)
            speed[1:] = v
        near_goal = np.hypot(gt[:, 0] - goal[0], gt[:, 1] - goal[1]) < heading_skip_radius
        hdg[near_goal | (speed < min_speed)] = np.nan

        out["cte_rmse"] = rmse(cte)
        out["cte_max"] = float(np.nanmax(cte)) if np.isfinite(cte).any() else None
        out["heading_rmse"] = rmse(hdg)
        first = plans_w[0]
        after = t >= first[0]
        if after.any():
            out["cte_initial_plan_rmse"] = rmse(polyline_projection(gt[after, :2], first[1])[0])
    return out


def map_quality(ref, grid_data, width, height, resolution, origin_pose, T_world_map,
                occupied_threshold=65):
    """IoU de celdas ocupadas y error medio de pared (cm) de un OccupancyGrid
    frente al mapa de referencia. La referencia usa solo las celdas ocupadas
    visibles (con un vecino libre), que es lo único que un LiDAR puede mapear."""
    data = np.asarray(grid_data, dtype=np.int16).reshape(height, width)
    rows, cols = np.nonzero(data >= occupied_threshold)
    if len(rows) == 0:
        return {"map_iou": 0.0, "map_iou_tol1": 0.0, "map_wall_error_cm": None,
                "map_occupied_cells": 0}
    local = np.column_stack(((cols + 0.5) * resolution, (rows + 0.5) * resolution))
    world = se2_apply(se2_compose(T_world_map, origin_pose), local)

    ref_b = boundary_cells(ref.occupied)
    rr = np.floor((world[:, 1] - ref.oy) / ref.res).astype(int)
    rc = np.floor((world[:, 0] - ref.ox) / ref.res).astype(int)
    inside = (rr >= 0) & (rr < ref.shape[0]) & (rc >= 0) & (rc < ref.shape[1])
    pred = np.zeros(ref.shape, dtype=bool)
    pred[rr[inside], rc[inside]] = True
    n_outside = len(set(zip(rr[~inside].tolist(), rc[~inside].tolist())))
    inter = np.sum(pred & ref_b)
    union = np.sum(pred | ref_b) + n_outside

    ref_pts = np.column_stack((ref.ox + (np.nonzero(ref_b)[1] + 0.5) * ref.res,
                               ref.oy + (np.nonzero(ref_b)[0] + 0.5) * ref.res))
    dmin = np.full(len(world), np.inf)
    for i in range(0, len(world), 512):
        chunk = world[i:i + 512]
        d = np.hypot(chunk[:, None, 0] - ref_pts[None, :, 0], chunk[:, None, 1] - ref_pts[None, :, 1])
        dmin[i:i + 512] = d.min(axis=1)
    # IoU tolerante: ambos conjuntos dilatados una celda (absorbe 2 cm de desfase)
    pd, rd = dilate(pred, 1.5), dilate(ref_b, 1.5)
    union_t = np.sum(pd | rd)
    return {"map_iou": float(inter / union) if union else 0.0,
            "map_iou_tol1": float(np.sum(pd & rd) / union_t) if union_t else 0.0,
            "map_wall_error_cm": float(np.mean(dmin) * 100.0),
            "map_occupied_cells": int(len(rows))}


# ══════════════════════════════════════════════════════════════════════════════
# Costo computacional (/proc, solo Linux)
# ══════════════════════════════════════════════════════════════════════════════

_CLK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


def gazebo_pids():
    """PIDs de los procesos de Gazebo (gz sim: servidor y, si existe, GUI)."""
    pids = []
    for pid in os.listdir("/proc") if os.path.isdir("/proc") else []:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                args = [a.decode(errors="ignore") for a in f.read().split(b"\0") if a]
        except OSError:
            continue
        names = [os.path.basename(a) for a in args]
        if ("gz" in names and "sim" in args) or any(n.startswith("gz-sim") for n in names):
            pids.append(int(pid))
    return pids


def cpu_snapshot(pids):
    """(segundos de CPU de los procesos dados, segundos de CPU de toda la máquina)."""
    proc = 0.0
    for pid in pids:
        try:
            with open(f"/proc/{pid}/stat") as f:
                fields = f.read().rsplit(")", 1)[1].split()
            proc += (int(fields[11]) + int(fields[12])) / _CLK      # utime + stime
        except (OSError, IndexError, ValueError):
            pass
    total = float("nan")
    try:
        with open("/proc/stat") as f:
            vals = [int(v) for v in f.readline().split()[1:]]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)          # idle + iowait
        total = (sum(vals[:8]) - idle) / _CLK
    except (OSError, ValueError):
        pass
    return proc, total


# ══════════════════════════════════════════════════════════════════════════════
# Agregación
# ══════════════════════════════════════════════════════════════════════════════

def wilson_interval(successes, n, z=1.96):
    """Intervalo de Wilson para una proporción (95 % con z = 1.96)."""
    if n == 0:
        return None
    p = successes / n
    den = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / den
    half = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / den
    return [max(0.0, centre - half), min(1.0, centre + half)]


STAT_KEYS = ("spl", "mission_time", "path_length", "final_pos_error", "final_yaw_error",
             "ate_localization", "ate_odometry", "cte_rmse", "heading_rmse",
             "cte_initial_plan_rmse", "recoveries", "collisions", "replans", "rtf",
             "cpu_gazebo_per_sim_s", "cpu_total_per_sim_s", "map_iou", "map_iou_tol1",
             "map_wall_error_cm")


def _stats(values):
    v = np.array([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if len(v) == 0:
        return None
    return {"mean": float(v.mean()), "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
            "median": float(np.median(v)), "min": float(v.min()), "max": float(v.max()),
            "n": int(len(v))}


def summarize(episodes):
    """Resumen global y por goal. SPL incluye los fallos (valen 0); el resto de
    estadísticas usa todos los episodios en que la métrica está definida."""
    valid = [e for e in episodes if e.get("valid", True)]

    def block(eps):
        n = len(eps)
        k = sum(1 for e in eps if e["success"])
        b = {"n_episodes": n, "n_success": k,
             "success_rate": k / n if n else None,
             "wilson_95": wilson_interval(k, n),
             "total_collisions": sum(e.get("collisions", 0) for e in eps),
             "total_recoveries": sum(e.get("recoveries", 0) for e in eps),
             "fail_reasons": {}}
        for e in eps:
            if not e["success"]:
                r = e.get("fail_reason") or "desconocido"
                b["fail_reasons"][r] = b["fail_reasons"].get(r, 0) + 1
        for key in STAT_KEYS:
            b[key] = _stats([e.get(key) for e in eps])
        return b

    goals = sorted({e["goal_id"] for e in valid})
    return {"overall": block(valid),
            "per_goal": {str(g): block([e for e in valid if e["goal_id"] == g]) for g in goals},
            "invalid_episodes": len(episodes) - len(valid)}


# ══════════════════════════════════════════════════════════════════════════════
# Archivos
# ══════════════════════════════════════════════════════════════════════════════

TRAJ_HEADER = ["t", "gt_x", "gt_y", "gt_yaw", "odom_x", "odom_y", "odom_yaw",
               "est_x", "est_y", "est_yaw"]


def write_traj_csv(path, traj):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(TRAJ_HEADER)
        w.writerows(traj)


def read_traj_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))[1:]
    return np.array([[float(v) for v in r] for r in rows], dtype=float).reshape(-1, 10)


def write_plans_csv(path, plans):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["plan", "t", "x", "y"])
        for i, (t, xy) in enumerate(plans):
            for x, y in xy:
                w.writerow([i, t, x, y])


def read_plans_csv(path):
    plans = {}
    with open(path) as f:
        for r in list(csv.reader(f))[1:]:
            plans.setdefault(int(r[0]), [float(r[1]), []])[1].append((float(r[2]), float(r[3])))
    return [(t, np.array(xy)) for _, (t, xy) in sorted(plans.items())]


EPISODE_CSV_KEYS = ["run", "goal_id", "success", "fail_reason", "nav_status",
                    "mission_time", "wall_time", "rtf", "cpu_gazebo_per_sim_s",
                    "cpu_total_per_sim_s", "path_length", "shortest_path_length", "spl",
                    "final_pos_error", "final_yaw_error", "within_tolerance",
                    "recoveries", "collisions", "replans", "ate_localization",
                    "ate_odometry", "yaw_rmse_localization", "cte_rmse", "cte_max",
                    "heading_rmse", "cte_initial_plan_rmse", "map_iou",
                    "map_iou_tol1", "map_wall_error_cm"]


def write_episodes_csv(path, episodes):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=EPISODE_CSV_KEYS, extrasaction="ignore")
        w.writeheader()
        for e in episodes:
            w.writerow(e)


# ══════════════════════════════════════════════════════════════════════════════
# Gráficas
# ══════════════════════════════════════════════════════════════════════════════

def _arrow(ax, x, y, yaw, color, length=0.15, **kw):
    ax.annotate("", xy=(x + length * math.cos(yaw), y + length * math.sin(yaw)),
                xytext=(x, y), arrowprops=dict(arrowstyle="-|>", color=color, lw=2), **kw)


def _draw_map(ax, ref):
    img = np.where(ref.occupied, 0.15, np.where(ref.free, 1.0, 0.75))
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=ref.extent(),
              interpolation="nearest")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def make_plots(results_dir, ref=None):
    """Una figura por goal (trayectorias de todas las corridas sobre el mapa y
    error transversal en el tiempo) y una vista general. Devuelve las rutas."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(os.path.join(results_dir, "results.json")) as f:
        res = json.load(f)
    cfg, episodes = res["config"], [e for e in res["episodes"] if e.get("valid", True)]
    if ref is None:
        ref = load_map_yaml(os.path.join(results_dir, cfg["reference_map"]))
    plot_dir = os.path.join(results_dir, "plots")
    os.makedirs(plot_dir, exist_ok=True)
    spawn = cfg["spawn"]
    cmap = plt.get_cmap("tab10")
    out = []

    for goal in cfg["goals"]:
        eps = [e for e in episodes if e["goal_id"] == goal["id"]]
        if not eps:
            continue
        stats = res["summary"]["per_goal"].get(str(goal["id"]), {})
        fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 6),
                                      gridspec_kw={"width_ratios": [1.1, 1]})
        _draw_map(ax, ref)
        if goal.get("shortest_path"):
            sp = np.array(goal["shortest_path"])
            ax.plot(sp[:, 0], sp[:, 1], color="k", lw=1, ls=":", label="ruta más corta")
        for i, e in enumerate(eps):
            traj = read_traj_csv(os.path.join(results_dir, e["files"]["trajectory"]))
            plans = read_plans_csv(os.path.join(results_dir, e["files"]["plans"]))
            T_wm = tuple(e["T_world_map"])
            c = cmap(i % 10)
            if plans:
                p = se2_apply(T_wm, plans[0][1])
                ax.plot(p[:, 0], p[:, 1], color=c, lw=0.8, ls="--", alpha=0.6)
            if len(traj):
                ax.plot(traj[:, 1], traj[:, 2], color=c, lw=1.6, alpha=0.9,
                        label=f"corrida {e['run']}" + ("" if e["success"] else " (fallo)"))
                ax.plot(traj[-1, 1], traj[-1, 2], "o", color=c, ms=4)
                if plans:
                    m = episode_cte_series(traj, plans, T_wm)
                    ax2.plot(traj[:, 0] - traj[0, 0], m * 100.0, color=c, lw=1.2)
        tol = plt.Circle((goal["x"], goal["y"]), cfg["pos_tol"], color="g", fill=False, lw=1.5)
        ax.add_patch(tol)
        _arrow(ax, goal["x"], goal["y"], goal["yaw"], "g")
        _arrow(ax, spawn["x"], spawn["y"], spawn["yaw"], "b")
        pad = 0.25
        xs = [spawn["x"], goal["x"]]
        ys = [spawn["y"], goal["y"]]
        ax.set_xlim(max(ref.extent()[0], min(xs) - 0.6 - pad), min(ref.extent()[1], max(xs) + 0.6 + pad))
        ax.set_ylim(max(ref.extent()[2], min(ys) - 0.6 - pad), min(ref.extent()[3], max(ys) + 0.6 + pad))
        ax.legend(fontsize=7, loc="best")
        ax.set_title("Trayectoria real (continua) y primer plan (discontinua)", fontsize=10)

        ax2.set_xlabel("tiempo desde el envío del goal (s)")
        ax2.set_ylabel("error transversal al plan activo (cm)")
        ax2.grid(alpha=0.3)
        ax2.set_title("Seguimiento de la ruta planificada", fontsize=10)

        def fmt(key, scale=1.0, unit=""):
            s = stats.get(key)
            return "—" if not s else f"{s['mean'] * scale:.2f}±{s['std'] * scale:.2f}{unit}"
        sr = stats.get("success_rate")
        wi = stats.get("wilson_95") or [float("nan")] * 2
        fig.suptitle(
            f"Goal {goal['id']} «{goal.get('label', '')}» ({goal['x']:.2f}, {goal['y']:.2f}, "
            f"{goal['yaw']:.2f} rad) — SR {sr if sr is None else f'{sr:.0%}'} "
            f"[{wi[0]:.2f}, {wi[1]:.2f}]  SPL {fmt('spl')}\n"
            f"error final {fmt('final_pos_error', 100, ' cm')} / {fmt('final_yaw_error', 1, ' rad')}   "
            f"CTE RMSE {fmt('cte_rmse', 100, ' cm')}   rumbo RMSE {fmt('heading_rmse', 1, ' rad')}   "
            f"ATE {fmt('ate_localization', 100, ' cm')}", fontsize=10)
        fig.tight_layout()
        path = os.path.join(plot_dir, f"goal_{goal['id']}.png")
        fig.savefig(path, dpi=120)
        plt.close(fig)
        out.append(path)

    # Vista general: todas las trayectorias, un color por goal
    fig, ax = plt.subplots(figsize=(7.5, 7.5))
    _draw_map(ax, ref)
    for gi, goal in enumerate(cfg["goals"]):
        c = cmap(gi % 10)
        for e in [e for e in episodes if e["goal_id"] == goal["id"]]:
            traj = read_traj_csv(os.path.join(results_dir, e["files"]["trajectory"]))
            if len(traj):
                ax.plot(traj[:, 1], traj[:, 2], color=c, lw=1, alpha=0.6,
                        ls="-" if e["success"] else "--")
        _arrow(ax, goal["x"], goal["y"], goal["yaw"], c)
        ax.text(goal["x"], goal["y"] - 0.12, f"G{goal['id']}", color=c, ha="center",
                fontsize=10, weight="bold")
    _arrow(ax, spawn["x"], spawn["y"], spawn["yaw"], "b")
    ax.set_title(f"{len(episodes)} episodios — continua: éxito, discontinua: fallo", fontsize=10)
    fig.tight_layout()
    path = os.path.join(plot_dir, "overview.png")
    fig.savefig(path, dpi=120)
    plt.close(fig)
    out.append(path)
    return out


def episode_cte_series(traj, plans, T_world_map):
    """Error transversal (m) de cada muestra contra el plan activo; NaN antes del primero."""
    traj = np.asarray(traj, dtype=float).reshape(-1, 10)
    plans = sorted(plans, key=lambda p: p[0])
    stamps = np.array([p[0] for p in plans])
    idx = np.searchsorted(stamps, traj[:, 0], side="right") - 1
    cte = np.full(len(traj), np.nan)
    for k in np.unique(idx[idx >= 0]):
        sel = idx == k
        cte[sel] = polyline_projection(traj[sel, 1:3], se2_apply(T_world_map, plans[k][1]))[0]
    return cte


def main(argv=None):
    import sys
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1:
        print("uso: python3 -m omni_dofbot_bringup.nav_metrics <carpeta_de_resultados>")
        return 1
    d = argv[0]
    with open(os.path.join(d, "results.json")) as f:
        res = json.load(f)
    res["summary"] = summarize(res["episodes"])
    with open(os.path.join(d, "results.json"), "w") as f:
        json.dump(res, f, indent=2)
    write_episodes_csv(os.path.join(d, "episodes.csv"), res["episodes"])
    for p in make_plots(d):
        print(p)
    o = res["summary"]["overall"]
    print(f"SR {o['n_success']}/{o['n_episodes']}  Wilson95 {o['wilson_95']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
