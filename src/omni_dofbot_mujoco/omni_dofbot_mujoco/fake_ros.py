"""ROS 2 mínimo en proceso, para correr los nodos REALES sin ROS instalado.

Implementa solo la parte de la API de rclpy y de los mensajes que usan
ag_motion_tests.py, mecanum_kinematic_node.py, Mecanum_odometry_node.py y
mujoco_bridge_node.py: nodos, parámetros (incluido el servicio
<nodo>/set_parameters), tópicos, servicios, temporizadores y reloj de
simulación. Todo es síncrono: publicar llama a los suscriptores en el acto y
una llamada a servicio queda resuelta al regresar call_async().

Uso: install(graph) ANTES de importar esos módulos (ver lockstep.py). No es
un sustituto general de rclpy.
"""
import sys
import types

# ═════════════════════════════════════════════════════════════════════════════
# Mensajes
# ═════════════════════════════════════════════════════════════════════════════


def _msg(_cls_name, **fields):
    """Clase de mensaje con valores por defecto. Los callables son fábricas."""
    def __init__(self, **kw):
        for k, v in fields.items():
            setattr(self, k, kw.pop(k) if k in kw else (v() if callable(v) else v))
        if kw:
            raise TypeError(f"{_cls_name}: campos desconocidos {list(kw)}")

    def __repr__(self):
        return f"{_cls_name}({', '.join(f'{k}={getattr(self, k)!r}' for k in fields)})"
    return type(_cls_name, (), {"__init__": __init__, "__repr__": __repr__,
                           "__slots__": tuple(fields)})


TimeMsg = _msg("Time", sec=0, nanosec=0)
Duration = _msg("Duration", sec=0, nanosec=0)
Header = _msg("Header", stamp=TimeMsg, frame_id="")
Vector3 = _msg("Vector3", x=0.0, y=0.0, z=0.0)
Point = _msg("Point", x=0.0, y=0.0, z=0.0)
Quaternion = _msg("Quaternion", x=0.0, y=0.0, z=0.0, w=1.0)
Pose = _msg("Pose", position=Point, orientation=Quaternion)
Twist = _msg("Twist", linear=Vector3, angular=Vector3)
PoseWithCovariance = _msg("PoseWithCovariance", pose=Pose, covariance=lambda: [0.0] * 36)
TwistWithCovariance = _msg("TwistWithCovariance", twist=Twist,
                           covariance=lambda: [0.0] * 36)
Odometry = _msg("Odometry", header=Header, child_frame_id="",
                pose=PoseWithCovariance, twist=TwistWithCovariance)
Transform = _msg("Transform", translation=Vector3, rotation=Quaternion)
TransformStamped = _msg("TransformStamped", header=Header, child_frame_id="",
                        transform=Transform)
JointState = _msg("JointState", header=Header, name=list, position=list,
                  velocity=list, effort=list)
Float64 = _msg("Float64", data=0.0)
Float64MultiArray = _msg("Float64MultiArray", layout=None, data=list)
JointTrajectoryPoint = _msg("JointTrajectoryPoint", positions=list, velocities=list,
                            accelerations=list, effort=list, time_from_start=Duration)
JointTrajectory = _msg("JointTrajectory", header=Header, joint_names=list, points=list)
Clock = _msg("Clock", clock=TimeMsg)
Entity = _msg("Entity", id=0, name="", type=0)
Entity.NONE, Entity.LIGHT, Entity.MODEL, Entity.LINK = 0, 1, 2, 3
ParameterValue = _msg("ParameterValue", type=0, bool_value=False, integer_value=0,
                      double_value=0.0, string_value="")
RclParameter = _msg("Parameter", name="", value=ParameterValue)
SetParametersResult = _msg("SetParametersResult", successful=True, reason="")


class ParameterType:
    PARAMETER_NOT_SET, PARAMETER_BOOL, PARAMETER_INTEGER = 0, 1, 2
    PARAMETER_DOUBLE, PARAMETER_STRING = 3, 4


def _srv(name, request, response):
    return type(name, (), {"Request": request, "Response": response})


Empty = _srv("Empty", _msg("Empty_Request"), _msg("Empty_Response"))
SetEntityPose = _srv("SetEntityPose",
                     _msg("SetEntityPose_Request", entity=Entity, pose=Pose),
                     _msg("SetEntityPose_Response", success=False))
SetParameters = _srv("SetParameters",
                     _msg("SetParameters_Request", parameters=list),
                     _msg("SetParameters_Response", results=list))


def _param_value(pv):
    return {ParameterType.PARAMETER_BOOL: pv.bool_value,
            ParameterType.PARAMETER_INTEGER: pv.integer_value,
            ParameterType.PARAMETER_DOUBLE: pv.double_value,
            ParameterType.PARAMETER_STRING: pv.string_value}.get(pv.type)


# ═════════════════════════════════════════════════════════════════════════════
# rclpy
# ═════════════════════════════════════════════════════════════════════════════
class Time:
    def __init__(self, seconds=0, nanoseconds=0, clock_type=None):
        self.nanoseconds = int(round(seconds * 1e9)) + int(nanoseconds)

    def __sub__(self, other):
        return DurationValue(self.nanoseconds - other.nanoseconds)

    def to_msg(self):
        return TimeMsg(sec=self.nanoseconds // 1_000_000_000,
                       nanosec=self.nanoseconds % 1_000_000_000)

    @classmethod
    def from_msg(cls, msg, clock_type=None):
        return cls(nanoseconds=msg.sec * 1_000_000_000 + msg.nanosec)


class DurationValue:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class Parameter:
    class Type:
        NOT_SET, BOOL, INTEGER, DOUBLE, STRING = range(5)

    def __init__(self, name, type_=None, value=None):
        self.name, self.type_, self.value = name, type_, value


class _Logger:
    def __init__(self, graph, name):
        self._g, self._name = graph, name

    def _log(self, level, msg, **_):
        self._g.log(self._name, level, msg)

    def debug(self, msg, **kw): self._log("DEBUG", msg, **kw)
    def info(self, msg, **kw): self._log("INFO", msg, **kw)
    def warn(self, msg, **kw): self._log("WARN", msg, **kw)
    def warning(self, msg, **kw): self._log("WARN", msg, **kw)
    def error(self, msg, **kw): self._log("ERROR", msg, **kw)


class _Publisher:
    def __init__(self, graph, topic):
        self._g, self.topic = graph, topic

    def publish(self, msg):
        for cb in list(self._g.subs.get(self.topic, ())):
            cb(msg)


class _Future:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _Client:
    def __init__(self, graph, srv_type, name):
        self._g, self._type, self._name = graph, srv_type, name

    def wait_for_service(self, timeout_sec=None):
        return self._name in self._g.services

    def service_is_ready(self):
        return self.wait_for_service()

    def call_async(self, req):
        return _Future(self._g.services[self._name](req, self._type.Response()))


class _Clock:
    def __init__(self, graph):
        self._g = graph

    def now(self):
        return Time(nanoseconds=int(round(self._g.sim_time() * 1e9)))


class Node:
    def __init__(self, node_name, parameter_overrides=None, **_):
        self._g = GRAPH
        self._name = node_name
        over = {p.name: p.value for p in (parameter_overrides or [])}
        over.update(self._g.param_overrides.get(node_name, {}))
        self._over = over
        self._params = {"use_sim_time": over.get("use_sim_time", True)}
        self._param_cbs = []
        self._g.nodes.append(self)
        self._g.services[f"/{node_name}/set_parameters"] = self._set_parameters_srv

    # ── nombres ──────────────────────────────────────────────────────────────
    def _resolve(self, name):
        if name.startswith("~/"):
            return f"/{self._name}/{name[2:]}"
        return name if name.startswith("/") else "/" + name

    def get_name(self):
        return self._name

    # ── parámetros ───────────────────────────────────────────────────────────
    def declare_parameter(self, name, value=None, *_, **__):
        self._params[name] = self._over.get(name, value)
        return Parameter(name, None, self._params[name])

    def get_parameter(self, name):
        return Parameter(name, None, self._params[name])

    def has_parameter(self, name):
        return name in self._params

    def add_on_set_parameters_callback(self, cb):
        self._param_cbs.append(cb)

    def set_parameters(self, params):
        for cb in self._param_cbs:
            if not cb(params).successful:
                return [SetParametersResult(successful=False)] * len(params)
        for p in params:
            self._params[p.name] = p.value
        return [SetParametersResult(successful=True)] * len(params)

    def _set_parameters_srv(self, req, res):
        params = [Parameter(p.name, None, _param_value(p.value)) for p in req.parameters]
        res.results = self.set_parameters(params)
        return res

    # ── comunicación ─────────────────────────────────────────────────────────
    def get_clock(self):
        return _Clock(self._g)

    def get_logger(self):
        return _Logger(self._g, self._name)

    def create_publisher(self, msg_type, topic, qos=None, **_):
        return _Publisher(self._g, self._resolve(topic))

    def create_subscription(self, msg_type, topic, callback, qos=None, **_):
        self._g.subs.setdefault(self._resolve(topic), []).append(callback)

    def create_service(self, srv_type, name, callback, **_):
        self._g.services[self._resolve(name)] = callback

    def create_client(self, srv_type, name, **_):
        return _Client(self._g, srv_type, self._resolve(name))

    def create_timer(self, period, callback, **_):
        self._g.add_timer(period, callback)

    def destroy_node(self):
        pass


class MultiThreadedExecutor:
    def __init__(self, *_, **__): pass
    def add_node(self, node): pass
    def spin(self): pass
    def shutdown(self, *_, **__): pass


class TransformBroadcaster:
    def __init__(self, node, *_, **__): pass
    def sendTransform(self, t): pass


# ═════════════════════════════════════════════════════════════════════════════
# Grafo: tópicos, servicios, temporizadores
# ═════════════════════════════════════════════════════════════════════════════
class Graph:
    """Estado compartido por todos los nodos falsos de un proceso."""

    def __init__(self, sim_time, log=None):
        self.sim_time = sim_time          # callable -> segundos de simulación
        self.subs, self.services, self.nodes = {}, {}, []
        self.param_overrides = {}         # nombre de nodo -> {param: valor}
        self._timers = []                 # [periodo, siguiente, callback]
        self._log = log

    def add_timer(self, period, cb):
        self._timers.append([period, self.sim_time() + period, cb])

    def fire_timers(self):
        t = self.sim_time()
        for tm in self._timers:
            while t >= tm[1] - 1e-9:
                tm[1] += tm[0]
                tm[2]()

    def log(self, node, level, msg):
        if self._log:
            self._log(node, level, msg)


GRAPH = None


def install(graph):
    """Registra módulos falsos en sys.modules (rclpy, mensajes, tf2_ros...)."""
    global GRAPH
    GRAPH = graph

    def mod(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m

    _ok = [True]
    rclpy = mod("rclpy", init=lambda *a, **k: None, ok=lambda: _ok[0],
                shutdown=lambda *a, **k: None, spin=lambda *a, **k: None,
                try_shutdown=lambda *a, **k: None)
    rclpy.node = mod("rclpy.node", Node=Node)
    rclpy.time = mod("rclpy.time", Time=Time)
    rclpy.parameter = mod("rclpy.parameter", Parameter=Parameter)
    rclpy.executors = mod("rclpy.executors", MultiThreadedExecutor=MultiThreadedExecutor,
                          SingleThreadedExecutor=MultiThreadedExecutor)
    rclpy.callback_groups = mod("rclpy.callback_groups",
                                ReentrantCallbackGroup=lambda *a, **k: None,
                                MutuallyExclusiveCallbackGroup=lambda *a, **k: None)
    rclpy.qos = mod("rclpy.qos", qos_profile_sensor_data=None, QoSProfile=lambda *a, **k: None)

    pkgs = {
        "rcl_interfaces": {"msg": dict(Parameter=RclParameter, ParameterValue=ParameterValue,
                                       ParameterType=ParameterType,
                                       SetParametersResult=SetParametersResult),
                           "srv": dict(SetParameters=SetParameters)},
        "builtin_interfaces": {"msg": dict(Duration=Duration, Time=TimeMsg)},
        "trajectory_msgs": {"msg": dict(JointTrajectory=JointTrajectory,
                                        JointTrajectoryPoint=JointTrajectoryPoint)},
        "geometry_msgs": {"msg": dict(Twist=Twist, Pose=Pose, Point=Point,
                                      Quaternion=Quaternion, Vector3=Vector3,
                                      TransformStamped=TransformStamped)},
        "nav_msgs": {"msg": dict(Odometry=Odometry)},
        "std_msgs": {"msg": dict(Float64=Float64, Float64MultiArray=Float64MultiArray,
                                 Header=Header)},
        "std_srvs": {"srv": dict(Empty=Empty)},
        "sensor_msgs": {"msg": dict(JointState=JointState)},
        "rosgraph_msgs": {"msg": dict(Clock=Clock)},
        "ros_gz_interfaces": {"msg": dict(Entity=Entity),
                              "srv": dict(SetEntityPose=SetEntityPose)},
    }
    for pkg, subs in pkgs.items():
        top = mod(pkg)
        for sub, attrs in subs.items():
            setattr(top, sub, mod(f"{pkg}.{sub}", **attrs))
    mod("tf2_ros", TransformBroadcaster=TransformBroadcaster)
    return graph
