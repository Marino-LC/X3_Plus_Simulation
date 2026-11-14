"""Núcleo de simulación sin ROS: lo usan las pruebas de aceptación y el puente."""
import math

import mujoco
import numpy as np

from .params import (ModelParams, WHEEL_JOINTS, ARM_JOINTS, GRIP_JOINTS,
                     ROBOT_BODY, RESET_POSE, WHEEL_RADIUS, LX, LY, WHEEL_POLARITY)
from .model_builder import build_model


def twist_to_wheels(vx, vy, wz, r=WHEEL_RADIUS, lx=LX, ly=LY):
    """Cinemática inversa idéntica a mecanum_kinematic_node.py -> [fl, fr, br, bl] rad/s
    ya con wheel_polarity aplicada (lo que llega a /wheel_velocity_controller/commands)."""
    k = lx + ly
    w = np.array([(vx - vy - k * wz) / r, (vx + vy + k * wz) / r,
                  (vx - vy + k * wz) / r, (vx + vy - k * wz) / r])
    return w * np.array(WHEEL_POLARITY)


def wheels_to_twist(w, r=WHEEL_RADIUS, lx=LX, ly=LY):
    """Cinemática directa idéntica a Mecanum_odometry_node.py (entrada: joint_states)."""
    w = np.asarray(w) * np.array(WHEEL_POLARITY)
    k = lx + ly
    return (r / 4 * (w[0] + w[1] + w[2] + w[3]),
            r / 4 * (-w[0] + w[1] - w[2] + w[3]),
            r / (4 * k) * (-w[0] + w[1] + w[2] - w[3]))


class MujocoSim:
    """Envuelve MjModel/MjData con las operaciones que pide la guía (sección 7)."""

    def __init__(self, model=None, params=None, world="arena"):
        if model is None:
            model, _ = build_model(params or ModelParams(), world)
        elif isinstance(model, str):
            model = mujoco.MjModel.from_xml_path(model)
        self.model = m = model
        self.data = mujoco.MjData(m)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
        self.body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, ROBOT_BODY)
        root = jid("root")
        self.root_qadr, self.root_vadr = m.jnt_qposadr[root], m.jnt_dofadr[root]
        self.wheel_act = [aid(f"motor_{j[:-6]}") for j in WHEEL_JOINTS]
        self.pos_act = {j: aid(f"servo_{j}") for j in ARM_JOINTS + GRIP_JOINTS}
        # Juntas publicadas en /joint_states: todas las hinge salvo los rodillos
        self.joint_names, self._qadr, self._vadr = [], [], []
        for j in range(m.njnt):
            n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE and "_roller_" not in n:
                self.joint_names.append(n)
                self._qadr.append(m.jnt_qposadr[j])
                self._vadr.append(m.jnt_dofadr[j])
        self._traj = {}      # junta -> [(t_ini, q_ini, t_fin, q_fin), ...]
        self.reset()

    # ------------------------------------------------------------------ estado
    @property
    def time(self):
        return self.data.time

    def joint_state(self):
        return (self.joint_names, self.data.qpos[self._qadr].copy(),
                self.data.qvel[self._vadr].copy())

    def wheel_velocities(self):
        idx = [self.joint_names.index(j) for j in WHEEL_JOINTS]
        return self.data.qvel[self._vadr][idx]

    def ground_truth(self):
        """Pose (x, y, z, quat wxyz) en el mundo y twist (lin, ang) en el cuerpo."""
        q = self.data.qpos[self.root_qadr:self.root_qadr + 7].copy()
        v = self.data.qvel[self.root_vadr:self.root_vadr + 6].copy()
        R = self.data.xmat[self.body_id].reshape(3, 3)
        return q[:3], q[3:], R.T @ v[:3], v[3:]   # ang. del freejoint ya es local

    def pose2d(self):
        p, q, _, _ = self.ground_truth()
        yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                         1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return p[0], p[1], yaw

    def tilt(self):
        """Inclinación del chasis respecto a la vertical [rad]."""
        return math.acos(max(-1.0, min(1.0, self.data.xmat[self.body_id][8])))

    # ---------------------------------------------------------------- comandos
    def set_wheel_commands(self, w):
        """[fl, fr, br, bl] rad/s, como /wheel_velocity_controller/commands."""
        self.data.ctrl[self.wheel_act] = w

    def set_twist(self, vx, vy, wz):
        self.set_wheel_commands(twist_to_wheels(vx, vy, wz))

    def set_trajectory(self, joint_names, points):
        """points: [(time_from_start [s], [posiciones])]. Interpolación lineal
        desde el objetivo vigente, como un JointTrajectoryController simple."""
        t0 = self.data.time
        for k, j in enumerate(joint_names):
            if j not in self.pos_act:
                continue
            segs, ta, qa = [], t0, self.data.ctrl[self.pos_act[j]]
            for tfs, pos in points:
                segs.append((ta, qa, t0 + tfs, pos[k]))
                ta, qa = t0 + tfs, pos[k]
            self._traj[j] = segs

    def _update_trajectories(self):
        t = self.data.time
        for j, segs in list(self._traj.items()):
            while segs and t >= segs[0][2]:
                self.data.ctrl[self.pos_act[j]] = segs.pop(0)[3]
            if not segs:
                del self._traj[j]
                continue
            ta, qa, tb, qb = segs[0]
            s = (t - ta) / (tb - ta) if tb > ta else 1.0
            self.data.ctrl[self.pos_act[j]] = qa + (qb - qa) * min(max(s, 0.0), 1.0)

    # ------------------------------------------------------------ simulación
    def step(self, n=1):
        for _ in range(n):
            self._update_trajectories()
            mujoco.mj_step(self.model, self.data)

    def run(self, seconds):
        self.step(int(round(seconds / self.model.opt.timestep)))

    def teleport(self, x, y, z, quat_wxyz):
        """Equivalente a SetEntityPose: fija la pose del chasis y anula velocidades.
        Las juntas del brazo conservan su posición."""
        a = self.root_qadr
        self.data.qpos[a:a + 3] = (x, y, z)
        self.data.qpos[a + 3:a + 7] = quat_wxyz
        self.data.qvel[:] = 0
        self.data.qacc_warmstart[:] = 0
        mujoco.mj_forward(self.model, self.data)

    def reset(self, pose=RESET_POSE):
        """Estado inicial completo (como relanzar): t=0, brazo en cero, pose del AG."""
        mujoco.mj_resetData(self.model, self.data)
        self._traj.clear()
        x, y, z, yaw = pose
        self.teleport(x, y, z, (math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
