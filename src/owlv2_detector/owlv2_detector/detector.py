"""Detector OWLv2 por consultas de texto, sin ROS (lo usan el nodo y el estimador).

Problemas que resuelve respecto a consultar ["a red cube", "a cylinder", "a box"]:

* **Consultas por clase** (objects.yaml -> objects.<clase>.prompts): cada
  consulta se asocia a una clase del evaluador (cubo, caja, cilindro) y se
  admiten varias por clase. Sin colores: el conjunto de prueba tiene cubos de
  otros colores.
* **Consultas negativas** (objects.yaml -> detector.negative_prompts): pedestal,
  pared, suelo, robot. OWLv2 asigna a cada caja la consulta de mayor puntaje; si
  gana una negativa, la caja se descarta, y una caja positiva que se solapa con
  una negativa más segura también. Sin ellas, el pedestal (una caja grande)
  gana "cube" o "box".
* **Mosaicos**: OWLv2 rellena la imagen a cuadrado y la reduce a 960 px. Una
  imagen de 1280x720 se reduce a 0.75 y un cubo de 4 cm a 1.5 m queda en ~22 px.
  Con dos mosaicos de 720x720 se amplía a 1.33 (~40 px).
* Supresión de duplicados entre mosaicos y descarte de cajas cortadas por un
  borde interno de mosaico (el mosaico vecino las ve completas).

La parte que depende de torch/transformers está aislada en `_infer`, de modo
que la lógica se prueba sin el modelo.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

DEFAULT_MODEL = 'google/owlv2-base-patch16-ensemble'


@dataclass
class RawBox:
    """Caja de OWLv2 ya en píxeles de la imagen completa."""

    query: str
    cls: Optional[str]          # None = consulta negativa
    score: float
    xyxy: tuple

    @property
    def negative(self):
        return self.cls is None


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def make_tiles(width, height, mode='auto', overlap=0.2):
    """[(x0, y0, lado)] de mosaicos cuadrados.

    'none': una sola entrada con la imagen completa (OWLv2 la rellena a cuadrado).
    'auto': mosaicos de lado min(ancho, alto) a lo largo del eje largo, con
    solape `overlap`, si la imagen es apaisada (relación > 1.5: la Astra de
    1280x720 sí, la cámara del Dofbot de 640x480 no); si no, 'none'.
    """
    long_, short = max(width, height), min(width, height)
    if mode == 'none' or (mode == 'auto' and long_ / short <= 1.5):
        return [(0, 0, long_)]
    step = short * (1.0 - overlap)
    n = max(2, int(math.ceil((long_ - short) / step)) + 1)
    starts = np.linspace(0, long_ - short, n).round().astype(int).tolist()
    if width >= height:
        return [(s, 0, short) for s in starts]
    return [(0, s, short) for s in starts]


class Owlv2Detector:

    def __init__(self, class_prompts: Dict[str, Sequence[str]],
                 negative_prompts: Sequence[str] = (), threshold=0.1,
                 class_thresholds: Optional[Dict[str, float]] = None, tiles='auto',
                 tile_overlap=0.2, nms_iou=0.5, negative_iou=0.5, min_box_px=4,
                 model_name=DEFAULT_MODEL, device=None):
        self.queries, self.query_class = [], []
        for cls, prompts in class_prompts.items():
            for p in prompts:
                self.queries.append(p)
                self.query_class.append(cls)
        for p in negative_prompts:
            self.queries.append(p)
            self.query_class.append(None)
        if not any(c is not None for c in self.query_class):
            raise ValueError('hace falta al menos una consulta positiva')
        self.threshold = float(threshold)
        self.class_thresholds = dict(class_thresholds or {})
        self.tiles_mode = tiles
        self.tile_overlap = float(tile_overlap)
        self.nms_iou = float(nms_iou)
        self.negative_iou = float(negative_iou)
        self.min_box_px = float(min_box_px)
        self.model_name = model_name
        self.device = device
        self._model = self._processor = None

    # ── Modelo (torch / transformers) ────────────────────────────────────────
    def load(self):
        if self._model is None:
            import torch
            from transformers import Owlv2ForObjectDetection, Owlv2Processor
            self._torch = torch
            self.device = self.device or ('cuda' if torch.cuda.is_available() else 'cpu')
            self._processor = Owlv2Processor.from_pretrained(self.model_name)
            self._model = Owlv2ForObjectDetection.from_pretrained(
                self.model_name).to(self.device).eval()
        return self

    def _infer(self, images, sides):
        """[(cajas Nx4, puntajes N, índices de consulta N)] por imagen, en píxeles
        de esa imagen (igual que el nodo original: target = lado mayor)."""
        from PIL import Image as PILImage
        self.load()
        torch = self._torch
        pils = [PILImage.fromarray(np.ascontiguousarray(im)) for im in images]
        inputs = self._processor(text=[self.queries] * len(pils), images=pils,
                                 return_tensors='pt')
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with torch.no_grad():
            outputs = self._model(**inputs)
        # Umbral bajo aquí; el umbral por clase se aplica después.
        low = min([self.threshold] + list(self.class_thresholds.values()))
        target = torch.tensor([[s, s] for s in sides], device=self.device)
        res = self._processor.post_process_object_detection(
            outputs=outputs, threshold=low, target_sizes=target)
        return [(r['boxes'].cpu().numpy(), r['scores'].cpu().numpy(),
                 r['labels'].cpu().numpy()) for r in res]

    # ── Lógica (sin modelo) ──────────────────────────────────────────────────
    def detect_raw(self, rgb) -> List[RawBox]:
        """Todas las cajas (positivas y negativas) tras unir mosaicos y suprimir."""
        h, w = rgb.shape[:2]
        tiles = make_tiles(w, h, self.tiles_mode, self.tile_overlap)
        if len(tiles) == 1:
            images, sides = [rgb], [tiles[0][2]]
        else:
            images = [rgb[y0:y0 + s, x0:x0 + s] for x0, y0, s in tiles]
            sides = [s for _, _, s in tiles]
        boxes = []
        for (x0, y0, s), (bx, sc, lb) in zip(tiles, self._infer(images, sides)):
            for b, score, label in zip(bx, sc, lb):
                b = [float(b[0]) + x0, float(b[1]) + y0, float(b[2]) + x0, float(b[3]) + y0]
                if len(tiles) > 1 and self._cut_by_tile(b, (x0, y0, s), w, h):
                    continue
                b = [min(max(b[0], 0.0), w - 1.0), min(max(b[1], 0.0), h - 1.0),
                     min(max(b[2], 0.0), w - 1.0), min(max(b[3], 0.0), h - 1.0)]
                if b[2] - b[0] < self.min_box_px or b[3] - b[1] < self.min_box_px:
                    continue
                q = int(label)
                boxes.append(RawBox(self.queries[q], self.query_class[q], float(score),
                                    tuple(b)))
        return self._suppress(boxes)

    @staticmethod
    def _cut_by_tile(b, tile, w, h, tol=2.0):
        """¿La caja toca un borde del mosaico que NO es borde de la imagen?"""
        x0, y0, s = tile
        return ((x0 > 0 and b[0] <= x0 + tol) or (x0 + s < w and b[2] >= x0 + s - tol)
                or (y0 > 0 and b[1] <= y0 + tol) or (y0 + s < h and b[3] >= y0 + s - tol))

    def _suppress(self, boxes):
        # Umbral por clase (las negativas usan el general).
        boxes = [b for b in boxes
                 if b.score >= self.class_thresholds.get(b.cls, self.threshold)]
        # NMS sin distinguir clase: entre mosaicos se repiten cajas con otra consulta.
        kept = []
        for b in sorted(boxes, key=lambda b: -b.score):
            if all(iou(b.xyxy, k.xyxy) <= self.nms_iou for k in kept):
                kept.append(b)
        # Positiva solapada con una negativa más segura -> fuera (p. ej. el pedestal).
        negs = [b for b in kept if b.negative]
        return [b for b in kept if b.negative or not any(
            n.score >= b.score and iou(b.xyxy, n.xyxy) > self.negative_iou for n in negs)]

    def detect(self, rgb):
        """[(clase, puntaje, (x0, y0, x1, y1))] solo positivas."""
        return [(b.cls, b.score, b.xyxy) for b in self.detect_raw(rgb) if not b.negative]


def prompts_from_config(objects_cfg):
    """(consultas por clase, consultas negativas, opciones del detector) de objects.yaml."""
    det = dict((objects_cfg or {}).get('detector') or {})
    classes = {}
    for cls, spec in (objects_cfg or {}).get('objects', {}).items():
        classes[cls] = list(spec.get('prompts') or ['a small %s' % cls])
    negatives = list(det.pop('negative_prompts', []) or [])
    return classes, negatives, det


def draw(rgb, raw_boxes, rejected=()):
    """Imagen anotada: positivas en verde, negativas en gris, rechazadas en rojo."""
    try:
        import cv2
    except ImportError:
        return None
    out = np.ascontiguousarray(rgb).copy()
    rej = [tuple(r) for r in rejected]
    for b in raw_boxes:
        x0, y0, x1, y1 = (int(round(v)) for v in b.xyxy)
        if b.negative:
            color, text = (150, 150, 150), 'NO %s %.2f' % (b.query, b.score)
        elif tuple(b.xyxy) in rej:
            color, text = (255, 40, 40), '%s %.2f (tamaño)' % (b.cls, b.score)
        else:
            color, text = (0, 220, 0), '%s %.2f' % (b.cls, b.score)
        cv2.rectangle(out, (x0, y0), (x1, y1), color, 2)
        cv2.putText(out, text, (x0, max(y0 - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return out
