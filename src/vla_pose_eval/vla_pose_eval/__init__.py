"""Medición del error de estimación de pose de objetos (sin Nav2).

El núcleo (geometry, scene, views, metrics, evaluate, report, estimators) es
Python puro (numpy + PyYAML) y no depende de ROS: se puede usar en la máquina
con GPU. Los nodos ROS 2 viven en ``vla_pose_eval.nodes``.
"""

__version__ = '0.1.0'
