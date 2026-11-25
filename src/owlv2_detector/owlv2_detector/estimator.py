"""OWLv2 como estimador de vla_pose_eval (línea base E1 con las dos cámaras).

Corre SOLO sobre las capturas de la pasada de reconocimiento (una por vista,
después de cada teletransporte), no sobre el video continuo:

    # sin simulación, sobre las vistas que guardó recon_pass:
    ros2 run vla_pose_eval vla_pose_eval run-estimator ~/vla_runs/test --estimator owlv2

    # o en línea, durante la pasada:
    ros2 run vla_pose_eval estimator_node --ros-args -p estimator:=owlv2
    ros2 run vla_pose_eval recon_pass --ros-args -p trial_dir:=... -p online:=true

- Clases y consultas: objects.yaml (objects.<clase>.prompts y
  detector.negative_prompts). Las clases son las del evaluador.
- RGB-D: caja 2D + profundidad -> 3D. Descarta cajas cuyo tamaño métrico supera
  `max_size_factor` x la mayor dimensión de los objetos (los pedestales).
- Cámara del Dofbot: segundo voto de clase y estimación monocular en el campo
  cercano (MultiCameraFusion).
- Guarda por vista una imagen anotada de cada cámara en
  <ensayo>/views/owlv2/view_XXX_{rgbd,mono}.png: verde = detección, gris =
  consulta negativa (pedestal, pared…), rojo = descartada por tamaño.

alias `owlv2_rgbd`: lo mismo solo con la Astra (para comparar una y dos cámaras).
"""

from vla_pose_eval.estimators.detector_depth import Box2D
from vla_pose_eval.estimators.multicam import MultiCameraDetectorEstimator
from vla_pose_eval.scene import shape_dims

from .detector import Owlv2Detector, draw, prompts_from_config

_SHARED = {}     # el modelo se carga una vez por proceso (tarda y ocupa GPU)


class Owlv2Estimator(MultiCameraDetectorEstimator):

    def __init__(self, use_mono=True, name=None, max_size_factor=1.6, debug=True,
                 detector=None, prompts=None, negative_prompts=None, **kwargs):
        """
        detector: opciones de Owlv2Detector (threshold, class_thresholds, tiles,
            model_name, device…); se combinan con objects.yaml -> detector.
        prompts / negative_prompts: sustituyen a las de objects.yaml.
        El resto (center_offsets, fusion, shrink…) va a la línea base.
        """
        super().__init__(**kwargs)
        self.use_mono = bool(use_mono)
        self.cameras = ('rgbd', 'mono') if self.use_mono else ('rgbd',)
        self.name = name or ('owlv2' if self.use_mono else 'owlv2_rgbd')
        self.max_size_factor = float(max_size_factor)
        self.debug = bool(debug)
        self.detector_kwargs = dict(detector or {})
        self.prompts_override = prompts
        self.negatives_override = negative_prompts
        self.detector = None
        self._raw = {}

    def setup(self, gt=None, cfg=None):
        super().setup(gt=gt, cfg=cfg)
        objects = (cfg or {}).get('objects') or {}
        classes, negatives, det_cfg = prompts_from_config(objects)
        classes = self.prompts_override or classes
        negatives = self.negatives_override if self.negatives_override is not None else negatives
        dims = [max(shape_dims(s['shape'], s['size']))
                for s in objects.get('objects', {}).values()]
        if dims and self.max_object_size_m is None:
            self.max_object_size_m = self.max_size_factor * max(dims)
        if not self.center_offsets:        # la profundidad mide la cara visible, no el centro
            self.center_offsets = {c: 0.5 * min(shape_dims(s['shape'], s['size'])[:2])
                                   for c, s in objects.get('objects', {}).items()}
        kw = dict(det_cfg, **self.detector_kwargs)
        key = (repr(sorted(classes.items())), tuple(negatives), repr(sorted(kw.items())))
        if key not in _SHARED:
            _SHARED[key] = Owlv2Detector(classes, negatives, **kw)
        self.detector = _SHARED[key]

    def _boxes(self, rgb, cam):
        raw = self.detector.detect_raw(rgb)
        self._raw[cam] = (rgb, raw)
        return [Box2D(b.cls, b.score, b.xyxy) for b in raw if not b.negative]

    def detect(self, rgb, instruction):
        return self._boxes(rgb, 'rgbd')

    def detect_mono(self, rgb, instruction):
        return self._boxes(rgb, 'mono')

    def estimate(self, frame):
        self._raw = {}
        if self.use_mono:
            dets = super().estimate(frame)
        else:
            dets = super(MultiCameraDetectorEstimator, self).estimate(frame)   # solo Astra
        if self.debug:
            rejected = {'rgbd': [b.xyxy for b, _ in self.rejected]}
            self.debug_images = {cam: draw(rgb, raw, rejected.get(cam, ()))
                                 for cam, (rgb, raw) in self._raw.items()}
        return dets
