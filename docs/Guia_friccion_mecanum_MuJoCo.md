# Guía: fricción de rodillos Mecanum y puesta a punto de MuJoCo para correr el AG

Para Diego y Aldo · Corte al 13 de noviembre de 2026 · Proyecto X3 Plus + Dofbot

## 1. Resumen

- **No necesitan fricción anisotrópica.** En MuJoCo la fricción de un geom se define con tres coeficientes (deslizamiento, torsional y rodadura) y el deslizamiento es isotrópico en el plano de contacto. No hay un equivalente directo del fdir1 de Gazebo. Es muy probable que ese sea el origen del bloqueo: la literatura que se estaba siguiendo (rodillos fijos con fricción anisotrópica) asume un motor de física que sí la soporta.

- **Su modelo ya tiene lo más difícil:** rodillos individuales con junta de revolución. Con esa topología la anisotropía aparece sola: el rodillo gira libre sobre su eje y el contacto solo transmite fuerza en la dirección ortogonal.

- **Lo que falta es poco:** fricción de contacto en rodillos y piso, amortiguamiento en la junta de cada rodillo, parámetros del solver, el mundo (arena) y un puente ROS 2 que dé al AG las mismas interfaces que Gazebo.

- **Prueba de concepto:** un robot simplificado (4 ruedas × 9 rodillos, 4 kg) en MuJoCo 4.0.1 reproduce la cinemática mecanum con errores de 1 a 4 % en 2 s (sección 3). Es un esqueleto de referencia, no su modelo: las dimensiones de los rodillos son supuestos y hay que sustituirlas por las reales.

## 2. Por qué no se usa fricción anisotrópica en MuJoCo

El artículo de Okada et al. (ICAR 2023) reemplaza las juntas de los rodillos por uniones fijas con fricción anisotrópica y aproxima el barril con esferas. Eso acelera la simulación en motores que permiten definir una dirección de baja fricción por geom, como Gazebo con fdir1. En MuJoCo:

- El atributo friction de un geom es deslizamiento torsional rodadura, con deslizamiento igual en las dos direcciones tangentes.

- Cuando dos geoms entran en contacto, MuJoCo toma por defecto el **máximo** de cada coeficiente de ambos geoms (salvo que se use priority). Consecuencia práctica: la fricción del rodillo no puede bajar el valor que ya impone el piso. Verifíquenlo en la documentación (Computation → Contact).

- Existen \<contact\>\<pair\> con coeficientes explícitos, pero los ejes tangentes del contacto no están alineados con el eje de cada rodillo, así que no sirven para esto. No lo probé; si quieren descartarlo, revisen ese punto en la referencia XML.

**Decisión:** mantener los rodillos como cuerpos con junta de revolución libre (ya la tienen) y calibrar fricción de contacto y amortiguamiento de la junta.

## 3. Parámetros que sí hay que fijar

| Parámetro                   | Dónde                                                   | Valor de partida                                                            | Comentario                                                           |
|-----------------------------|---------------------------------------------------------|-----------------------------------------------------------------------------|----------------------------------------------------------------------|
| Fricción de deslizamiento   | friction="0.68 0.005 0.0001" en geoms de rodillo y piso | 0.68                                                                        | Es el μ del piso (MDF) usado en la tesis. Calibrar con datos reales. |
| Amortiguamiento del rodillo | damping en la junta de cada rodillo                     | 1e-4                                                                        | Es la palanca principal del deslizamiento lateral (tabla abajo).     |
| Armadura del rodillo        | armature en la junta del rodillo                        | 1e-6                                                                        | Estabiliza la integración con rodillos muy ligeros.                  |
| Junta de la rueda           | damping, armature                                       | 0.002, 5e-4                                                                 | Valores del esqueleto; ajustar con la planta del motor.              |
| Opciones globales           | \<option\>                                              | timestep="0.001", integrator="implicitfast", cone="elliptic", impratio="10" | El paso de 1 ms es el mismo que usa el mundo de Gazebo.              |
| Colisiones                  | contype / conaffinity                                   | Ver esqueleto                                                               | Evita colisiones rodillo-rodillo y rodillo-chasis.                   |

**Resultados del esqueleto** (comando de 0.2 m/s en y durante 2 s; ideal: 0.40 m):

| Fricción μ | damping 1e-5 | 1e-4  | 1e-3  | 1e-2  |
|------------|--------------|-------|-------|-------|
| 0.3        | 0.380        | 0.379 | 0.287 | 0.175 |
| 0.68       | 0.385        | 0.384 | 0.376 | 0.279 |
| 1.0        | 0.389        | 0.388 | 0.379 | 0.309 |

Con μ = 0.68 y damping 1e-4, el esqueleto avanza 0.394 m en x (ideal 0.40), 0.384 m en y y gira 116.4° en 2 s con wz = 1 rad/s (ideal 114.6°). En esta prueba, de aceleración baja, el amortiguamiento de la junta del rodillo afecta mucho más al movimiento lateral que μ. Estos valores **no están identificados** con el robot real: son punto de partida.

## 4. Esqueleto MJCF probado

Cada rueda es un cuerpo con junta de eje y; cada rodillo es un cuerpo hijo con un elipsoide y una junta cuyo eje está a 45° del eje de la rueda. El signo del ángulo se invierte entre FL/BR y FR/BL. En la prueba, el patrón correcto fue sgn = -1 en este generador; si en su modelo el robot se mueve al revés en y, inviertan el signo.

import math  
R=0.040; RC=0.032; RR=0.0085 \# radio rueda, radio al centro del rodillo, radio máx. del rodillo  
N=9  
  
def wheel(name, x, y, sgn, mu, damp):  
s = f'\<body name="{name}" pos="{x} {y} 0"\>\n'  
s += f'\<joint name="{name}\_j" type="hinge" axis="0 1 0" damping="0.002" armature="0.0005"/\>\n'  
s += '\<geom type="cylinder" size="0.022 0.012" euler="90 0 0" mass="0.08" contype="0" conaffinity="0"/\>\n'  
for i in range(N):  
th = 2\*math.pi\*i/N  
cx, cz = RC\*math.sin(th), RC\*math.cos(th)  
tx, tz = math.cos(th), -math.sin(th) \# tangente en el plano de la rueda  
a = (math.cos(math.pi/4)\*tx, sgn\*math.sin(math.pi/4), math.cos(math.pi/4)\*tz)  
s += (f'\<body name="{name}\_r{i}" pos="{cx:.5f} 0 {cz:.5f}"\>\n'  
f'\<joint name="{name}\_r{i}\_j" type="hinge" axis="{a\[0\]:.5f} {a\[1\]:.5f} {a\[2\]:.5f}" damping="{damp}" armature="1e-6"/\>\n'  
f'\<geom type="ellipsoid" size="{RR} {RR} 0.0155" zaxis="{a\[0\]:.5f} {a\[1\]:.5f} {a\[2\]:.5f}" mass="0.006" '  
f'friction="{mu} 0.005 0.0001" contype="2" conaffinity="1"/\>\n\</body\>\n')  
return s + '\</body\>\n'

Ajustes complementarios del XML (piso, chasis y actuadores):

\<option timestep="0.001" integrator="implicitfast" cone="elliptic" impratio="10"/\>  
\<geom name="floor" type="plane" size="1.2 1.2 0.1" friction="0.68 0.005 0.0001" contype="1" conaffinity="6"/\>  
\<!-- chasis: contype="4" conaffinity="0" (choca con el piso, no con los rodillos) --\>  
\<velocity name="a_fl" joint="fl_j" kv="2" ctrlrange="-25 25"/\>

Las posiciones de las ruedas usan lx = 0.110 m y ly = 0.102 m, igual que el nodo cinemático. Las dimensiones de rodillo (radio máximo 8.5 mm, semieje 15.5 mm, centro a 32 mm del eje) son supuestos para la prueba; tomen las del modelo real.

## 5. El mundo (arena)

Para que la comparación con Gazebo sea válida, la arena debe ser equivalente a arena_pid_tuning.world:

| Elemento           | Valor en Gazebo                                                 | Qué hacer en MuJoCo                                                                            |
|--------------------|-----------------------------------------------------------------|------------------------------------------------------------------------------------------------|
| Gravedad           | 9.81 m/s²                                                       | Igual                                                                                          |
| Paso de simulación | 0.001 s                                                         | timestep="0.001"                                                                               |
| Piso               | caja de 2.42 × 2.42 m, μ = 0.68 (con μ2 = 0.4 en una dirección) | Caja con μ = 0.68; MuJoCo no permite la dirección con μ2 distinto, se calibra con datos reales |
| Paredes            | altura 0.30 m, espesor 0.03 m, μ = 0.5                          | Cuatro cajas estáticas                                                                         |
| Pose de reinicio   | (0, 0, 0.05), yaw = π/2                                         | Misma pose en el servicio de teletransporte                                                    |
| Marcadores         | cilindros en (0,0), (0,0.8), (0.35,0.8)                         | Solo visuales (contype="0")                                                                    |

Las dimensiones de la arena están por definir (2.42 × 2.42 m en los mundos y la tesis; 2.42 × 2.42 m en el plan de Fase 2). Usen las de Gazebo hasta que el equipo decida.

## 6. Actuadores, brazo y masa

- **Ruedas:** actuadores de velocidad (\<velocity\>), rango ±25 rad/s como max_wheel_vel. Gazebo recibe comandos de velocidad por gz_ros2_control; en MuJoCo la ganancia kv debe ser lo bastante alta para que el servo siga el comando sin añadir una dinámica propia. Comparen el escalón de velocidad de rueda en ambos simuladores y ajusten kv.

- **Planta del motor:** la zona muerta, el PID incremental, la ganancia K y el filtro de primer orden (τ ≈ 0.58 s) viven en mecanum_kinematic_node.py, que es ROS 2 puro y no debe cambiar.

- **Brazo Dofbot:** un actuador de posición por junta (\<position\>), con límites del URDF (±1.5708 rad en las juntas 1 a 4; −1.5708 a 3.14159 en la 5). El gripper tiene un único joint comandable (grip_joint); los dedos se acoplan con \<equality\>\<joint\> o se fijan.

- **Masa:** el objetivo es 4.8 kg totales, igual que en el Gazebo actual. Verifiquen con mujoco.mj_getTotalmass(model) y revisen centro de masa e inercias. Ojo con boundmass, boundinertia y balanceinertia: arreglan inercias inválidas en silencio.

## 7. Puente ROS 2 para correr el AG sin modificarlo

El AG (ag_motion_tests.py) solo habla con ROS 2. Si un nodo de MuJoCo ofrece estas interfaces, el AG, ZN, el nodo cinemático y el de odometría funcionan igual:

| Interfaz                                                                                     | Qué espera el AG                                                                                              | Qué debe hacer el nodo de MuJoCo                                                                        |
|----------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------|
| /clock                                                                                       | Reloj de simulación                                                                                           | Publicar el tiempo de simulación                                                                        |
| /joint_states                                                                                | Ruedas front_left_joint, front_right_joint, back_right_joint, back_left_joint con velocidad; juntas del brazo | Publicar qpos y qvel con esos nombres                                                                   |
| /wheel_velocity_controller/commands                                                          | Float64MultiArray en orden fl, fr, br, bl (rad/s)                                                             | Escribir en data.ctrl de las ruedas                                                                     |
| /dofbot_trajectory_controller/joint_trajectory y /dofbot_gripper_controller/joint_trajectory | JointTrajectory con time_from_start                                                                           | Interpolar hacia el objetivo y escribir en los actuadores de posición                                   |
| /world/arena_pid_tuning/set_entity_pose                                                      | Servicio SetEntityPose para teletransportar a omni_dofbot                                                     | Ofrecer el mismo servicio o parametrizar el AG. Reiniciar qpos y qvel del freejoint y llamar mj_forward |
| /ground_truth/odom (nuevo)                                                                   | No existe hoy                                                                                                 | Publicar la pose real del chasis desde qpos del freejoint                                               |

Puntos importantes:

1.  **Opción de implementación.** Pueden usar mujoco_ros2_control o un nodo Python propio con los bindings de MuJoCo. Para diciembre recomiendo el nodo propio: las interfaces de la tabla son pocas, y evita compilar dependencias en Jazzy (verifiquen la compatibilidad de mujoco_ros2_control antes de elegir).

2.  **Reloj.** El AG mide timeouts y tiempos con time.time(), es decir, reloj de pared. Hasta que los nodos migren al reloj de ROS (pendiente en el Frente 1), MuJoCo debe correr a velocidad de tiempo real. Después de esa migración podrán acelerar la simulación.

3.  **Odometría.** mecanum_odometry_node.py integra las velocidades de las ruedas, así que no detecta deslizamiento. Si las ruedas giran pero el robot no se traslada, la odometría igual reporta movimiento. Para medir el error de posición hay que usar /ground_truth/odom. Coordinen con Marino cómo incorporarlo al AG.

4.  **Signos.** El nodo cinemático aplica wheel_polarity = \[-1, -1, -1, -1\]. Los ejes de las juntas de rueda del modelo migrado deben coincidir con los del URDF; si el robot va al revés, el signo está ahí.

## 8. Pruebas de aceptación antes de correr el AG

| Prueba                       | Qué hacer                                          | Criterio                                                  |
|------------------------------|----------------------------------------------------|-----------------------------------------------------------|
| T1. Reposo                   | Soltar el robot sobre el piso                      | Sin vibración ni deriva (velocidad \< 1e-3), z estable    |
| T2. Avance en x              | 0.2 m/s durante 2 s                                | Error de distancia \< 5 %                                 |
| T3. Avance lateral en y      | 0.2 m/s durante 2 s                                | Error \< 5 %                                              |
| T4. Rotación                 | wz = 1 rad/s durante 2 s                           | Error \< 5 %                                              |
| T5. Diagonal                 | vx = vy = 0.2 m/s                                  | Dirección a 45° ± 3°                                      |
| T6. Escalón de rueda         | Comparar con Gazebo                                | Tiempo de subida y régimen similares                      |
| T7. Con brazo                | Coreografía pick and place                         | No vuelca; la perturbación se nota en yaw o deslizamiento |
| T8. Tiempo real              | Medir factor de tiempo real con el modelo completo | ≥ 1×                                                      |
| T9. Determinismo             | Repetir con la misma entrada                       | Misma trayectoria                                         |
| T10. Equivalencia con Gazebo | P1, P2 y P3 con las mismas ganancias               | Curvas vx(t), x(t) y yaw(t) comparables (RMSE reportado)  |

Para referencia, el esqueleto simplificado (41 juntas) corrió unas 33 veces más rápido que el tiempo real en un solo hilo. Con el brazo, el mundo y ROS 2 será mucho menos: midan T8 con su modelo.

Un AG corto de humo (población 8, 3 generaciones) debe pasar antes del AG completo (población 35, 15 generaciones). Estimación propia a partir de los timeouts del código: unas 2 a 4 horas en tiempo real para el AG completo.

## 9. Calibrar la fricción con el robot real

El documento original proponía celdas de carga, captura de movimiento y LuGre. No es necesario ni viable aquí. Para este proyecto, lo útil es:

1.  **Plano inclinado con el material de la arena.** Medir el ángulo crítico de deslizamiento con los rodillos paralelos y perpendiculares a la pendiente; su tangente da los coeficientes de tracción longitudinal y transversal (método de Williams). Repetir la misma prueba en MuJoCo inclinando la gravedad o el piso.

2.  **Rampas de PWM en hardware** en x, y y yaw (ya planeadas): velocidad en régimen frente a PWM y zona muerta. En simulación, ajustar μ y el amortiguamiento de los rodillos hasta igualar esas curvas.

3.  **Parámetros a calibrar:** μ del piso/rodillo, damping del rodillo, kv de las ruedas y, si hace falta, frictionloss de la junta de rueda.

4.  No comparar costos brutos entre simulación y hardware: comparar métricas físicas y ranking de configuraciones, como define el proyecto.

## 10. Plan sugerido (propuesta, a confirmar con Marino)

| Ventana         | Entregable                                                                                                               |
|-----------------|--------------------------------------------------------------------------------------------------------------------------|
| Hasta el 17 nov | Fricción y amortiguamiento de rodillos en su modelo; T1 a T5 en el mundo vacío                                           |
| 18 a 22 nov     | Arena equivalente a Gazebo y nodo puente mínimo (/clock, /joint_states, comandos de rueda, teletransporte, ground truth) |
| 23 a 28 nov     | Brazo, masa total, T6 a T10 y AG corto de humo                                                                           |
| 29 nov a 6 dic  | Calibración con datos de hardware y AG completo                                                                          |

Si para el 25 de noviembre T2 a T4 no pasan, el plan B es reportar MuJoCo con ruedas simplificadas y declararlo como limitación.

## 11. Fuentes

**Implementación en MuJoCo**

- Documentación de MuJoCo (mujoco.readthedocs.io): *XML Reference* (geom, joint, option, contact, actuator) y *Computation → Contact* (fricción, solref, solimp, conos elípticos).

**Fricción de rodillos y modelado (útiles para calibrar)**

- Okada et al., *Fast and Accurate Simulation of Mecanum Wheels with Passive Rollers Emulated by Fixed Joints and Anisotropic Friction*, ICAR 2023. https://ieeexplore.ieee.org/document/10406382/ — Origen de la idea de rodillos fijos; útil para entender el compromiso entre fidelidad y costo.

- Williams et al., *Dynamic Model with Slip for Wheeled Omni-Directional Robots*, IEEE TRA 2002. https://people.ohio.edu/williams/html/PDF/IEEETRA02.pdf — Medición experimental de coeficientes de fricción y efecto de las discontinuidades entre rodillos.

- *Influence of Dissipative Forces and the Design of Mecanum-Wheels on the Omnidirectional Platform Dynamics*, IEEE. https://ieeexplore.ieee.org/document/9666053/ — Fricción viscosa más Coulomb en la junta del rodillo, que corresponde a damping y frictionloss en MuJoCo.

- *Experimental Evaluation of Rolling Resistance in Omnidirectional Wheels Under Quasi-Static Conditions*, Sensors 25(16):5026. https://www.mdpi.com/1424-8220/25/16/5026 — Resistencia a la rodadura.

- *Experimental and Analytical Investigation of Roller Material Influence on Motion Accuracy of Mecanum-Wheeled Mobile Robots*, 2025. https://www.researchgate.net/publication/393664459_Experimental_and_Analytical_Investigation_of_Roller_Material_Influence_on_Motion_Accuracy_of_Mecanum-Wheeled_Mobile_Robots — Efecto del material del rodillo.

- Ko, *Practical Navigation of a Mecanum-Wheel Robot on High-Friction Carpet*, preprint (engrXiv, mayo de 2026). https://engrxiv.org/preprint/download/7347/11996/10311 — Misma familia de robot (ROSMASTER X3); compensación de zona muerta. Es un preprint, sin revisión por pares.

**Opcional (no se necesita en MuJoCo)**

- Colantonio et al., *3D projection of the LuGre friction model adapted to varying normal forces*, 2022 (Université de Mons). Solo como referencia teórica: MuJoCo no implementa LuGre de forma nativa.
