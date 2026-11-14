# X3 Plus Simulation — rama `VLA_models`

Simulación del robot X3 Plus + Dofbot (ROS 2 Jazzy, Gazebo Harmonic) y trabajo con modelos VLA.

## Error de estimación de pose de objetos (sin Nav2)

El trabajo actual de la rama mide qué tan lejos está la pose estimada de cada objeto de su pose real en Gazebo.

- [Guía del experimento](docs/vla_pose_estimation/GUIA.md)
- [Estado de la implementación, uso y pendientes](docs/vla_pose_estimation/IMPLEMENTACION.md)
- [Paquete `vla_pose_eval`](src/vla_pose_eval/README.md)

Prueba rápida sin ROS ni Gazebo:

```bash
cd src/vla_pose_eval && python3 -m pytest test -q && python3 -m vla_pose_eval.cli dry-run --out /tmp/vla_dry --seeds 0 5
```

## Resto del repositorio

Los paquetes de descripción y arranque del robot están documentados en [`src/README.md`](src/README.md). La sintonización de ganancias por algoritmo genético está en `Instrucciones SintonizacionGanancias.txt`.
