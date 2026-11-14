"""Parámetros del modelo MuJoCo. Única fuente de verdad para generador y pruebas.

Los valores por defecto son el PUNTO DE PARTIDA de la guía
(docs/Guia_friccion_mecanum_MuJoCo.md, sección 3). No están identificados
con el robot real: se calibran con el protocolo de la sección 9.
"""
from dataclasses import dataclass


@dataclass
class ModelParams:
    # --- Contacto ---------------------------------------------------------
    mu: float = 0.68               # fricción de deslizamiento rodillo/piso (MDF)
    mu_torsional: float = 0.005
    mu_rolling: float = 0.0001
    mu_wall: float = 0.5
    # --- Rodillos (geometría medida de RODILLO.stl y del ensamble CAD) -----
    n_rollers: int = 9
    roller_center_radius: float = 0.0327   # eje de rueda -> centro del rodillo [m]
    roller_radius: float = 0.0080          # radio máximo del barril [m]
    # Semieje del elipsoide. NO es el largo real (0.0212): se elige para que la
    # envolvente de la rueda sea plana, hl^2 = r^2 + 2*Rc*r. Más largo => dos
    # rodillos tocan a la vez y la rueda se bloquea en lateral; más corto => baches.
    roller_half_length: float = 0.0242
    roller_mass: float = 0.00577           # [kg]
    roller_damping: float = 1e-4           # palanca principal del deslizamiento lateral
    roller_armature: float = 1e-6
    roller_frictionloss: float = 0.0
    roller_sign: int = 1                   # quiralidad; invertir si y sale al revés
    # --- Junta de rueda ---------------------------------------------------
    wheel_damping: float = 0.002
    wheel_armature: float = 5e-4
    wheel_frictionloss: float = 0.0
    wheel_kv: float = 2.0                  # ganancia del servo de velocidad
    max_wheel_vel: float = 25.0            # rad/s (ctrlrange)
    wheel_max_torque: float = 1.0          # N·m por rueda (forcerange; SUPUESTO)
    # --- Brazo ------------------------------------------------------------
    arm_kp: float = 20.0
    arm_kv: float = 1.0
    arm_armature: float = 0.01
    grip_kp: float = 2.0
    grip_kv: float = 0.05
    finger_armature: float = 1e-4
    finger_damping: float = 0.01
    # --- Masa -------------------------------------------------------------
    total_mass: float = 4.8                # objetivo, igual que Gazebo [kg]
    chassis_com: tuple = (0.02, 0.0, 0.0)  # respecto a base_link (SUPUESTO)
    chassis_box: tuple = (0.24, 0.17, 0.10)  # caja equivalente p/ inercia (SUPUESTO)
    # --- Solver -----------------------------------------------------------
    timestep: float = 0.001
    integrator: str = "implicitfast"
    cone: str = "elliptic"
    impratio: float = 10.0
    # --- Arena: 2.42 x 2.42 m. El AG sintoniza con x, y y yaw (P4 recorre
    # 0.80 m en lateral desde el origen), y eso no cabe en 1.22 m de ancho.
    arena_x: float = 2.42
    arena_y: float = 2.42
    wall_height: float = 0.30
    wall_thickness: float = 0.03


# Geometría cinemática (misma que mecanum_kinematic_node.py)
WHEEL_RADIUS = 0.040
LX = 0.110
LY = 0.102
WHEEL_POLARITY = (-1.0, -1.0, -1.0, -1.0)   # [fl, fr, br, bl]

WHEEL_JOINTS = ("front_left_joint", "front_right_joint",
                "back_right_joint", "back_left_joint")
ARM_JOINTS = ("arm_joint_01", "arm_joint_02", "arm_joint_03",
              "arm_joint_04", "arm_joint_05")
GRIP_JOINTS = ("grip_joint",)

ROBOT_BODY = "omni_dofbot"
BASE_LINK_Z = 0.0815                       # base_footprint -> base_link
RESET_POSE = (0.0, 0.0, 0.05, 1.5707963267948966)   # x, y, z, yaw (igual que el AG)
