"""Genera el MJCF del X3 Plus + Dofbot con rodillos mecanum de junta libre.

Parte de models/x3plus_rodillos_fijos.xml (conversión directa del URDF, base
fija y ruedas de malla) y lo transforma:

  * cuerpo raíz `omni_dofbot` (= base_footprint) con freejoint
  * cada rueda: 9 rodillos (elipsoide + junta de revolución a 45°)
  * colisiones por bits: rodillo<->piso, chasis<->piso, nada más
  * actuadores: velocidad en ruedas, posición en brazo y gripper
  * <option> de la guía y masa total objetivo
  * mundo: plano vacío o arena equivalente a arena_pid_tuning.world
"""
import copy
import math
import os
import xml.etree.ElementTree as ET

from .params import (ModelParams, WHEEL_JOINTS, ARM_JOINTS, GRIP_JOINTS,
                     ROBOT_BODY, BASE_LINK_Z)

PKG_DIR = os.path.dirname(os.path.abspath(__file__))


def models_dir():
    """Carpeta models/ (árbol fuente o share/ instalado por colcon)."""
    src = os.path.normpath(os.path.join(PKG_DIR, "..", "models"))
    if os.path.isdir(src):
        return src
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("omni_dofbot_mujoco"), "models")


def _f(*v):
    return " ".join(f"{x:.6g}" for x in v)


def _add_rollers(wheel, name, chir, p):
    """Rodillos en el marco de la rueda (eje de giro = z local)."""
    c45 = math.sqrt(0.5)
    for i in range(p.n_rollers):
        th = 2 * math.pi * i / p.n_rollers
        er = (math.cos(th), math.sin(th))
        et = (-math.sin(th), math.cos(th))
        axis = (c45 * et[0], c45 * et[1], c45 * chir)
        b = ET.SubElement(wheel, "body", name=f"{name}_roller_{i + 1}",
                          pos=_f(p.roller_center_radius * er[0],
                                 p.roller_center_radius * er[1], 0))
        ET.SubElement(b, "joint", {"name": f"{name}_roller_{i + 1}_joint",
                                   "class": "roller", "axis": _f(*axis)})
        ET.SubElement(b, "geom", {"class": "roller", "zaxis": _f(*axis)})


def _defaults(p):
    d = ET.Element("default")
    ET.SubElement(d, "geom", friction=_f(p.mu, p.mu_torsional, p.mu_rolling))
    vis = ET.SubElement(d, "default", {"class": "visual"})
    ET.SubElement(vis, "geom", contype="0", conaffinity="0", group="1", density="0")
    r = ET.SubElement(d, "default", {"class": "roller"})
    ET.SubElement(r, "joint", type="hinge", damping=_f(p.roller_damping),
                  armature=_f(p.roller_armature),
                  frictionloss=_f(p.roller_frictionloss))
    ET.SubElement(r, "geom", type="ellipsoid",
                  size=_f(p.roller_radius, p.roller_radius, p.roller_half_length),
                  mass=_f(p.roller_mass), contype="2", conaffinity="1",
                  group="3", rgba="0.15 0.15 0.15 1")
    w = ET.SubElement(d, "default", {"class": "wheel"})
    ET.SubElement(w, "joint", damping=_f(p.wheel_damping),
                  armature=_f(p.wheel_armature),
                  frictionloss=_f(p.wheel_frictionloss))
    a = ET.SubElement(d, "default", {"class": "arm"})
    ET.SubElement(a, "joint", armature=_f(p.arm_armature), damping="0.05")
    fg = ET.SubElement(d, "default", {"class": "finger"})
    ET.SubElement(fg, "joint", armature=_f(p.finger_armature),
                  damping=_f(p.finger_damping))
    st = ET.SubElement(d, "default", {"class": "static"})
    ET.SubElement(st, "geom", contype="1", conaffinity="6")
    return d


def _world(wb, p, world):
    ET.SubElement(wb, "light", name="sol", pos="0 0 3", dir="-0.3 0.2 -0.9",
                  directional="true")
    # Piso: plano infinito (equivale a la caja de Gazebo dentro de las paredes).
    ET.SubElement(wb, "geom", {"name": "floor", "class": "static", "type": "plane",
                               "size": "3 3 0.1", "rgba": "0.92 0.87 0.78 1"})
    if world != "arena":
        return
    hx, hy, h, t = p.arena_x / 2, p.arena_y / 2, p.wall_height, p.wall_thickness
    wf = _f(p.mu_wall, p.mu_torsional, p.mu_rolling)
    walls = {"wall_north": ((0, hy + t / 2, h / 2), (hx, t / 2, h / 2)),
             "wall_south": ((0, -hy - t / 2, h / 2), (hx, t / 2, h / 2)),
             "wall_east": ((hx + t / 2, 0, h / 2), (t / 2, hy, h / 2)),
             "wall_west": ((-hx - t / 2, 0, h / 2), (t / 2, hy, h / 2))}
    for n, (pos, size) in walls.items():
        # priority=1: la pared impone su mu=0.5 en vez del máximo con el chasis
        ET.SubElement(wb, "geom", {"name": n, "class": "static", "type": "box",
                                   "pos": _f(*pos), "size": _f(*size),
                                   "friction": wf, "priority": "1",
                                   "rgba": "0.55 0.60 0.65 1"})
    marks = {"marker_0_origin": ((0, 0), "0.1 0.4 0.9 0.8"),
             "marker_1_p1_p3_forward": ((0, 0.80), "0.9 0.5 0.1 0.8"),
             "marker_2_p3_end": ((0.35, 0.80), "0.6 0.1 0.8 0.8")}
    for n, ((x, y), rgba) in marks.items():
        ET.SubElement(wb, "geom", name=n, type="cylinder", size="0.03 0.0005",
                      pos=_f(x, y, 0.0005), contype="0", conaffinity="0", rgba=rgba)


def build_xml(p=None, world="arena", meshdir=None, chassis_mass=1.0):
    """Devuelve el MJCF como string. `world`: 'arena' o 'vacio'."""
    p = p or ModelParams()
    src = ET.parse(os.path.join(models_dir(), "x3plus_rodillos_fijos.xml")).getroot()
    root = ET.Element("mujoco", model=f"omni_dofbot_{world}")
    ET.SubElement(root, "compiler", angle="radian",
                  meshdir=meshdir or "assets", autolimits="true")
    ET.SubElement(root, "option", timestep=_f(p.timestep), integrator=p.integrator,
                  cone=p.cone, impratio=_f(p.impratio), gravity="0 0 -9.81")
    root.append(_defaults(p))

    asset = copy.deepcopy(src.find("asset"))
    for m in asset.iter("mesh"):
        m.set("file", os.path.basename(m.get("file")))
    root.append(asset)

    wb = ET.SubElement(root, "worldbody")
    _world(wb, p, world)

    # --- Robot: base_footprint libre, base_link 0.0815 m arriba -----------
    robot = ET.SubElement(wb, "body", name=ROBOT_BODY, pos="0 0 0")
    ET.SubElement(robot, "freejoint", name="root")
    ET.SubElement(robot, "camera", name="seguimiento", pos="0 -1.2 0.9",
                  xyaxes="1 0 0 0 0.6 0.8", mode="trackcom")
    base = copy.deepcopy(src.find("worldbody").find("frame"))
    base.set("pos", _f(0, 0, BASE_LINK_Z))
    robot.append(base)

    # Geoms: se conservan solo los visuales; las mallas de colisión se quitan.
    for parent in list(base.iter()) + [base]:
        for g in list(parent.findall("geom")):
            if g.get("contype") == "0":
                for k in ("contype", "conaffinity", "group", "density"):
                    g.attrib.pop(k, None)
                g.set("class", "visual")
            else:
                parent.remove(g)

    # Chasis: inercia concentrada + caja de colisión (solo contra piso/paredes)
    bx, by, bz = p.chassis_box
    cx, cy, cz = p.chassis_com
    m = chassis_mass
    inertia = (m * (by**2 + bz**2) / 12, m * (bx**2 + bz**2) / 12,
               m * (bx**2 + by**2) / 12)
    ET.SubElement(base, "inertial", pos=_f(cx, cy, cz), mass=_f(m),
                  diaginertia=_f(*inertia))
    ET.SubElement(base, "geom", name="chassis_col", type="box",
                  pos="-0.02 0 0.005", size="0.15 0.075 0.03",
                  contype="4", conaffinity="0", group="3", mass="0")
    # <frame> no admite <inertial>: base_link pasa a ser un body fijo al raíz.
    base.tag = "body"
    base.set("name", "base_link")

    # --- Ruedas y brazo ---------------------------------------------------
    for body in list(base.iter("body")):   # lista fija: abajo se añaden rodillos
        j = body.find("joint")
        if j is None:
            continue
        jn = j.get("name")
        j.attrib.pop("actuatorfrcrange", None)
        if jn in WHEEL_JOINTS:
            j.set("class", "wheel")
            x, y = (float(v) for v in body.get("pos").split()[:2])
            chir = p.roller_sign * (1 if x * y > 0 else -1)
            _add_rollers(body, jn.replace("_joint", ""), chir, p)
        elif jn in ARM_JOINTS or jn in GRIP_JOINTS:
            j.set("class", "arm" if jn in ARM_JOINTS else "finger")
        else:
            j.set("class", "finger")

    # El brazo y los dedos no colisionan entre sí ni con el chasis.
    root.append(copy.deepcopy(src.find("equality")))

    act = ET.SubElement(root, "actuator")
    for jn in WHEEL_JOINTS:
        ET.SubElement(act, "velocity", name=f"motor_{jn[:-6]}", joint=jn,
                      kv=_f(p.wheel_kv),
                      ctrlrange=_f(-p.max_wheel_vel, p.max_wheel_vel),
                      forcerange=_f(-p.wheel_max_torque, p.wheel_max_torque))
    for jn in ARM_JOINTS:
        ET.SubElement(act, "position", name=f"servo_{jn}", joint=jn,
                      kp=_f(p.arm_kp), kv=_f(p.arm_kv), inheritrange="1")
    ET.SubElement(act, "position", name="servo_grip_joint", joint="grip_joint",
                  kp=_f(p.grip_kp), kv=_f(p.grip_kv), inheritrange="1")

    sens = ET.SubElement(root, "sensor")
    for jn in WHEEL_JOINTS:
        ET.SubElement(sens, "jointvel", name=f"vel_{jn[:-6]}", joint=jn)
    ET.SubElement(sens, "framepos", name="gt_pos", objtype="body", objname=ROBOT_BODY)
    ET.SubElement(sens, "framequat", name="gt_quat", objtype="body", objname=ROBOT_BODY)

    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def build_model(p=None, world="arena"):
    """Compila el modelo en memoria ajustando la masa del chasis al total objetivo."""
    import mujoco
    p = p or ModelParams()
    md = os.path.join(models_dir(), "assets")
    m = mujoco.MjModel.from_xml_string(build_xml(p, world, md, 1.0))
    chassis = p.total_mass - (mujoco.mj_getTotalmass(m) - 1.0)
    if chassis <= 0:
        raise ValueError("total_mass menor que la masa de ruedas + brazo")
    return mujoco.MjModel.from_xml_string(build_xml(p, world, md, chassis)), chassis


def write_models(p=None, out_dir=None):
    """Escribe models/omni_dofbot_{arena,vacio}.xml (rutas de malla relativas)."""
    p = p or ModelParams()
    out_dir = out_dir or models_dir()
    _, chassis = build_model(p, "vacio")
    paths = []
    for world in ("arena", "vacio"):
        path = os.path.join(out_dir, f"omni_dofbot_{world}.xml")
        with open(path, "w") as f:
            f.write("<!-- GENERADO por scripts/build_model.py. No editar a mano. -->\n")
            f.write(build_xml(p, world, None, chassis) + "\n")
        paths.append(path)
    return paths
