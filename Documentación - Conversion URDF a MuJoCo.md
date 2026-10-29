# Conversión de modelos robóticos de URDF/Xacro a MuJoCo (MJCF)

**Proyecto:** documentación reutilizable para la integración de brazos robóticos en MuJoCo  
**Caso de estudio:** brazo Dofbot del repositorio [`Roberto-Garcia-1/robot-x`](https://github.com/Roberto-Garcia-1/robot-x)  
**Entorno de referencia:** Ubuntu 24.04, ROS 2 Jazzy, Python y biblioteca `mujoco`    
**Estado:** procedimiento documentado a partir del código fuente y de una ejecución previa del Dofbot.

> **Propósito principal.** Establecer un procedimiento que pueda repetirse para distintos brazos robóticos, separando los **archivos originales del robot**, la **conversión URDF → MJCF** y la **configuración específica de la simulación**. Los nombres `robotx_*` son del caso Dofbot y deben sustituirse al migrar otro modelo.

## Índice

1. [Alcance y resultado esperado](#1-alcance-y-resultado-esperado)
2. [Formatos y conceptos](#2-formatos-y-conceptos)
3. [Organización recomendada de archivos y paquetes](#3-organización-recomendada-de-archivos-y-paquetes)
4. [Requisitos y comprobaciones](#4-requisitos-y-comprobaciones)
5. [Preparación del modelo original](#5-preparación-del-modelo-original)
6. [Conversión con el repositorio robot-x](#6-conversión-con-el-repositorio-robot-x)
7. [Cómo funciona internamente la conversión del repositorio](#7-cómo-funciona-internamente-la-conversión-del-repositorio)
8. [Conversión independiente y reutilizable mediante Python](#8-conversión-independiente-y-reutilizable-mediante-python)
9. [Estructura y configuración de un archivo MJCF](#9-estructura-y-configuración-de-un-archivo-mjcf)
10. [Validación del modelo obtenido](#10-validación-del-modelo-obtenido)
11. [Adaptación a otros brazos robóticos](#11-adaptación-a-otros-brazos-robóticos)
12. [Problemas habituales y soluciones](#12-problemas-habituales-y-soluciones)
13. [Criterios de aceptación y registro de conversión](#13-criterios-de-aceptación-y-registro-de-conversión)
14. [Limitaciones y mejoras sugeridas](#14-limitaciones-y-mejoras-sugeridas)
15. [Referencias](#15-referencias)

---

## 1. Alcance y resultado esperado

El punto de partida es la descripción de un robot mediante **URDF** (o archivos **Xacro** que producen URDF), así como sus archivos de geometría, por ejemplo `.stl`, `.dae` u otros formatos compatibles. El objetivo es generar un archivo **MJCF/XML**, que es el formato nativo de modelos de MuJoCo, y comprobar que se carga sin errores.

```text
Archivos del robot
  ├── URDF / Xacro
  ├── Mallas visuales y de colisión
  └── Parámetros físicos (masa, inercia, límites)
             │
             ▼
      Expandir Xacro (si existe)
             │
             ▼
      Resolver rutas de recursos
             │
             ▼
    MuJoCo: leer y compilar URDF
             │
             ▼
       Guardar MJCF base (.xml)
             │
             ▼
  Añadir escena, actuadores y ajustes
             │
             ▼
      Validar y simular en MuJoCo
```

**Es importante distinguir tres metas:**

- **Carga:** MuJoCo puede abrir el URDF; esto no significa necesariamente que se haya generado un archivo MJCF persistente.
- **Conversión:** se escribe un nuevo archivo `.xml` con la estructura MJCF y se comprueba que se puede volver a abrir.
- **Simulación funcional:** además de cargar, el robot se visualiza correctamente, sus articulaciones responden y su comportamiento físico es razonable. Esto requiere configurar y validar otros elementos.

La guía contempla dos rutas: **A)** reproducir la conversión ya implementada en `robot-x`; **B)** usar un procedimiento de Python independiente para distintos paquetes.

## 2. Formatos y conceptos

| Concepto | Función | Observación |
|---|---|---|
| **URDF** | Describe enlaces (`link`), articulaciones (`joint`), geometrías y propiedades físicas. | Es común en ROS; representa solo parte de lo que ofrece MuJoCo. |
| **Xacro** | Permite variables, macros e inclusiones para generar URDF. | Debe expandirse antes de que MuJoCo reciba el modelo. Un archivo con extensión `.urdf` también puede contener Xacro. |
| **MJCF** | Lenguaje XML nativo de MuJoCo. | Puede añadir actuadores, contactos, sensores, opciones físicas, escenas, etc. |
| **Malla (mesh)** | Geometría 3D visual o de colisión. | Debe conservar una ruta válida desde el modelo final. |
| **`MjModel`** | Modelo compilado de MuJoCo que utiliza el simulador. | No equivale a un archivo MJCF en disco. |
| **`MjData`** | Estado dinámico de una instancia de simulación. | Se usa con `mj_step` para avanzar la física. |

**Diferencia estructural:** en URDF las articulaciones unen un `parent` y un `child` definidos por separado; en MJCF, los `body` están **anidados** y las articulaciones se declaran dentro del cuerpo que se mueve respecto a su padre. Por tanto, no se realiza una simple sustitución de etiquetas XML: el **parser y compilador de MuJoCo** interpretan y transforman la descripción [1].

La documentación oficial indica que MuJoCo puede leer URDF directamente y recomienda convertirlo a MJCF cuando se necesiten capacidades de modelado adicionales [1, 2].

## 3. Organización recomendada de archivos y paquetes

### 3.1 Estructura observada en `robot-x`

Los archivos clave del repositorio están organizados, de forma simplificada, así:

```text
robot-x/
└── mujoco_ws/
    └── src/
        ├── robotx_description/
        │   ├── urdf/
        │   │   ├── robotx_position.urdf       # Entrada principal: incluye Xacro
        │   │   ├── robotx.urdf                # Enlaces y articulaciones
        │   │   ├── robotx_macros.urdf         # Macros de links y joints
        │   │   └── robotx_properties.urdf     # Propiedades físicas
        │   ├── meshes/                       # Geometrías STL
        │   ├── mujoco_model/
        │   │   ├── robotx_mujoco_config.xml   # Actuadores y configuración
        │   │   ├── robotx_mujoco_scene.xml    # Piso, luces, mesa y cubo
        │   │   └── robotx_mujoco.xml          # Archivo de salida generado
        │   ├── robotx_description/
        │   │   └── urdf_file_compiler.py
        │   └── setup.py
        ├── robotx_mujoco/
        │   ├── robotx_mujoco/
        │   │   └── mujoco_file_transformer.py
        │   └── setup.py
        └── robotx_bringup/
            └── launch/
                └── robotx_file_compiler.launch.py
```

Este árbol presenta los **archivos relevantes al procedimiento**, no el contenido completo del repositorio. Los archivos de configuración se inspeccionaron directamente en su ubicación remota [3–8].

**Funciones de cada paquete:**

- **`robotx_description`:** URDF/Xacro, mallas, configuraciones y salida MJCF; contiene el script que expande el URDF.
- **`robotx_mujoco`:** incluye el conversor Python que utiliza MuJoCo y agrega las configuraciones.
- **`robotx_bringup`:** contiene el `launch` que pone en marcha ambos ejecutables ROS 2.

### 3.2 Estructura propuesta para cualquier brazo

Para evitar sobreescribir modelos originales y reutilizar el flujo:

```text
robots_ws/src/
├── mi_brazo_description/
│   ├── urdf/
│   │   ├── mi_brazo.urdf.xacro  # O .urdf
│   │   └── componentes.xacro
│   ├── meshes/
│   │   ├── visual/
│   │   └── collision/
│   ├── config/
│   └── package.xml             # Si se administra con ROS 2
├── mi_brazo_mujoco/
│   ├── models/
│   │   ├── mi_brazo_base.xml    # Conversión inicial
│   │   └── mi_brazo_final.xml   # Modelo enriquecido
│   ├── scenes/
│   │   └── escena.xml
│   └── scripts/
│       └── convertir_urdf_a_mjcf.py
└── otro_brazo_description/
    ├── urdf/
    └── meshes/
```

Se recomienda mantener las **mallas**, la **descripción original**, el **MJCF base** y la **escena/configuración** como elementos diferenciados. Las rutas en el XML de salida deben permitir encontrar las mallas incluso si el XML cambia de carpeta o se copia a otra computadora. Documentar si se usan rutas absolutas o relativas.

## 4. Requisitos y comprobaciones

### 4.1 Entorno

En el caso de estudio se trabajó con Ubuntu 24.04 y ROS 2 Jazzy. Para reproducir el flujo ROS 2 se requieren, como mínimo:

- ROS 2 Jazzy y su entorno cargado (`/opt/ros/jazzy/setup.bash`).
- `colcon` y los paquetes ROS 2 del repositorio.
- `xacro`, utilizado por ambos scripts.
- Python con `mujoco`; `numpy` también es importado por los scripts originales.
- Las mallas y archivos XML/URDF del robot.
- Sesión gráfica para abrir `mujoco.viewer` (la validación programática no requiere abrir una ventana).

Comprobaciones iniciales:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help >/dev/null && echo 'ROS 2 disponible'
ros2 pkg executables xacro
python3 --version
```

Si `xacro` no aparece:

```bash
sudo apt update
sudo apt install ros-jazzy-xacro
```

Si se utiliza un entorno virtual existente (por ejemplo `~/mujoco_venv`):

```bash
source ~/mujoco_venv/bin/activate
python -c 'import mujoco; print("MuJoCo:", mujoco.__version__)'
python -c 'import numpy; import ament_index_python; print("Dependencias Python: OK")'
```

Si se necesita crear un entorno nuevo en ROS 2, conviene que utilice la **misma versión de Python que ROS 2** y que permita acceder a los paquetes Python instalados por ROS:

```bash
python3 -m venv --system-site-packages ~/mujoco_venv
source ~/mujoco_venv/bin/activate
python -m pip install mujoco numpy
```

> **Precaución:** no mezclar intérpretes/entornos de Python sin verificar. Un `mujoco` instalado con `pip` en un entorno virtual no será visible automáticamente para un ejecutable ROS 2 lanzado con otro intérprete. Verificar con `which python`, `python -c 'import mujoco'` y `python -c 'import ament_index_python'`.

## 5. Preparación del modelo original

Antes de convertir cualquier brazo, realizar este inventario:

1. **URDF/Xacro principal.** Localizar cuál archivo reúne todos los enlaces y articulaciones. Revisar sus instrucciones `<xacro:include>` y propiedades.
2. **Enlaces y articulaciones.** Registrar nombres y tipos de `joint` (revoluta, prismática, fija, continua), ejes y límites.
3. **Geometrías.** Verificar que cada archivo de malla exista, en especial los referenciados mediante `package://nombre_paquete/...`.
4. **Parámetros físicos.** Revisar masa, centro de masa y matriz de inercia de los cuerpos móviles; evitar masas nulas e inercias físicamente inválidas.
5. **Unidades y marcos de referencia.** Verificar dimensiones, orientación de ejes, ángulos y escala de las mallas.
6. **Dependencias adicionales.** Anotar paquetes ROS, plugins, controladores y macros que pudieran estar presentes en el URDF, sin asumir que se trasladarán a MuJoCo.

Ejemplo del caso estudiado: `robotx_position.urdf` incluye `robotx.urdf`; a su vez se incluyen `robotx_properties.urdf` y `robotx_macros.urdf`. Sus macros incorporan mallas desde `package://robotx_description/meshes/...` [7, 8].

Expansión de Xacro (ejemplo usando el árbol **fuente**, sin modificarlo):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot-x/mujoco_ws/src/robotx_description/urdf
ros2 run xacro xacro robotx_position.urdf -o /tmp/robotx_expandido.urdf
head -n 5 /tmp/robotx_expandido.urdf
```

**Resultado esperado:** `/tmp/robotx_expandido.urdf` contiene un `<robot ...>` con enlaces y articulaciones definidos; ya no deberían aparecer macros Xacro pendientes de expansión.

> **Importante:** la expansión Xacro **no es todavía** la conversión a MJCF. Tampoco convierte por sí sola las rutas ROS `package://` a rutas comprensibles por MuJoCo.

## 6. Conversión con el repositorio `robot-x`

Esta es la ruta **específica** del proyecto Dofbot. Ejecutarla para reproducir el caso de referencia antes de generalizar.

### Paso 1. Obtener el código y preparar ROS 2

Si el repositorio no existe en la computadora:

```bash
git clone https://github.com/Roberto-Garcia-1/robot-x.git ~/robot-x
```

Con el repositorio ya disponible:

```bash
source /opt/ros/jazzy/setup.bash
source ~/mujoco_venv/bin/activate
cd ~/robot-x/mujoco_ws
```

Comprobar que ambos importaciones funcionan en ese **mismo** entorno:

```bash
python -c 'import mujoco, ament_index_python; print("OK")'
```

### Paso 2. Revisar un detalle del `launch`

En la versión examinada, `robotx_file_compiler.launch.py` contiene:

```python
from moveit_configs_utils import MoveItConfigsBuilder
```

El símbolo **no se usa** en el resto del archivo [6]. Si no está instalado `moveit_configs_utils` y el lanzamiento falla por esa importación, se puede **eliminar únicamente esa línea** (conservar una copia del archivo antes de editarlo). No es necesario instalar MoveIt solo para ejecutar esta conversión. Después de editar el código, recompilar el paquete.

### Paso 3. Compilar los paquetes ROS 2

```bash
cd ~/robot-x/mujoco_ws
colcon build --symlink-install \
  --packages-select robotx_description robotx_mujoco robotx_bringup
source install/setup.bash
```

Comprobar los ejecutables registrados por `setup.py`:

```bash
ros2 pkg executables robotx_description
ros2 pkg executables robotx_mujoco
```

Se deben reconocer, entre otros, `urdf_file_compiler` y `mujoco_file_transformer` [5, 9].

### Paso 4. Ejecutar la conversión

**Opción A — dos fases explícitas (preferida para depuración y documentación):**

```bash
ros2 run robotx_description urdf_file_compiler
ros2 run robotx_mujoco mujoco_file_transformer
```

**Opción B — lanzador original:**

```bash
ros2 launch robotx_bringup robotx_file_compiler.launch.py
```

**Advertencia técnica:** el `launch` original usa `OnProcessStart` para iniciar el conversor MJCF cuando **arranca** el procesador URDF, no cuando **termina** [6]. Además, en la versión revisada el transformador **vuelve a ejecutar Xacro** sobre el URDF original y no consume el archivo `_compiled.urdf` del otro script [4, 6]. Por eso las dos etapas son parcialmente redundantes; para registrar resultados ordenados es más claro ejecutar los dos comandos de forma secuencial.

### Paso 5. Localizar la salida

El transformador escribe `robotx_mujoco.xml` en el **directorio compartido de instalación** de `robotx_description`, no necesariamente junto al archivo fuente [4]:

```bash
DESCRIPCION=$(ros2 pkg prefix robotx_description)/share/robotx_description
XML="$DESCRIPCION/mujoco_model/robotx_mujoco.xml"
echo "$XML"
ls -lh "$XML"
```

En el entorno de referencia la ruta correspondió aproximadamente a:

```text
~/robot-x/mujoco_ws/install/robotx_description/share/robotx_description/mujoco_model/robotx_mujoco.xml
```

Si se va a conservar el resultado dentro del repositorio, **copiarlo deliberadamente al directorio fuente**, después de validarlo:

```bash
cp "$XML" ~/robot-x/mujoco_ws/src/robotx_description/mujoco_model/robotx_mujoco.xml
```

Antes de copiar, revisar las rutas de recursos: no asumir que un XML generado en `install/` seguirá encontrando sus mallas si se desplaza o se ejecuta en otra máquina.

### Paso 6. Comprobar en MuJoCo

```bash
python -m mujoco.viewer --mjcf="$XML"
```

Debería visualizarse el robot con los elementos agregados por la escena: piso, mesa, luces y un cubo. Esta comprobación **no sustituye** pruebas de articulaciones y dinámica. El resultado de Dofbot se logró ejecutar previamente; los demás robots requieren validación individual.

## 7. Cómo funciona internamente la conversión del repositorio

Los siguientes puntos proceden de la revisión del código fuente (y no de suposiciones sobre los nombres de archivos).

### 7.1 `urdf_file_compiler.py`

Archivo: [`robotx_description/robotx_description/urdf_file_compiler.py`](https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/robotx_description/urdf_file_compiler.py).

- Obtiene el directorio compartido de `robotx_description` con `get_package_share_directory`.
- Toma como entrada `urdf/robotx_position.urdf`.
- Ejecuta `ros2 run xacro xacro <entrada> -o <salida>` mediante `subprocess`.
- Produce `urdf/robotx_position_compiled.urdf`.

**Su función real es expandir Xacro y generar otro URDF, no convertir a MJCF.**

### 7.2 `mujoco_file_transformer.py`

Archivo: [`robotx_mujoco/robotx_mujoco/mujoco_file_transformer.py`](https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_mujoco/robotx_mujoco/mujoco_file_transformer.py).

El procedimiento real sigue estas operaciones:

```text
1. Lee robotx_position.urdf mediante Xacro (de nuevo).
2. Sustituye las referencias "package://..." por rutas locales.
3. Carga el texto URDF: mujoco.MjModel.from_xml_string(...).
4. Escribe MJCF provisional con mujoco.mj_saveLastXML(...).
5. Inserta robotx_mujoco_config.xml antes de </mujoco>.
6. Inserta robotx_mujoco_scene.xml antes de </worldbody>.
7. Compila nuevamente el XML final para verificar que sea válido.
8. Guarda robotx_mujoco.xml usando mujoco.mj_saveLastXML(...).
```

Fragmento ilustrativo de las funciones fundamentales (simplificado, **no es una transcripción completa**):

```python
import mujoco

modelo = mujoco.MjModel.from_xml_string(urdf_expandido_y_con_rutas_resueltas)
mujoco.mj_saveLastXML("modelo_base.xml", modelo)

# El repositorio añade después la escena y la configuración,
# vuelve a compilar y finalmente guarda el MJCF resultante.
```

**Papel de los XML auxiliares** [4, 10, 11]:

| Archivo | Contenido y función |
|---|---|
| `robotx_mujoco_config.xml` | Actuadores de posición, relaciones entre articulaciones de la pinza, fricción, amortiguamiento, opciones físicas y elementos visuales. |
| `robotx_mujoco_scene.xml` | Plano, luces, mesa y cubo manipulable. |
| `robotx_mujoco.xml` | Modelo MJCF compilable resultante de fusionar el robot con esos componentes. |

**Limitaciones del código actual que afectan la reutilización:**

1. Las rutas y el nombre `robotx_description` están **escritos directamente** en el código: para otro brazo hay que parametrizarlos.
2. La sustitución de `package://` supone que todos los recursos pertenecen a `robotx_description`; **no resuelve correctamente varios paquetes ROS diferentes** sin modificar el algoritmo.
3. El ensamblado de fragmentos utiliza `str.replace()` sobre etiquetas de cierre; funciona para esta estructura conocida, pero es más frágil que un parser XML o una composición MJCF con `<include>`.
4. El modelo URDF expandido y la configuración MJCF **no deben confundirse** con el código de control ROS: no todo plugin o elemento de `ros2_control` de URDF se traduce automáticamente a un controlador MuJoCo.

## 8. Conversión independiente y reutilizable mediante Python

Esta segunda ruta permite generar un **MJCF base** sin depender de los nombres `robotx_*`. Sirve como procedimiento general y como prueba aislada cuando falla el `launch` de ROS 2.

### 8.1 Secuencia mínima para un URDF sin rutas `package://`

Si el archivo ya es un URDF real (sin macros Xacro pendientes), con rutas a mallas que MuJoCo pueda resolver:

```python
import mujoco

modelo = mujoco.MjModel.from_xml_path("/ruta/al/robot.urdf")
mujoco.mj_saveLastXML("/ruta/de/salida/robot_base.xml", modelo)

# Comprobación de la salida
verificacion = mujoco.MjModel.from_xml_path("/ruta/de/salida/robot_base.xml")
print("Cuerpos:", verificacion.nbody, "Articulaciones:", verificacion.njnt)
```

Si las mallas usan rutas relativas, debe respetarse su carpeta base. Esta receta **no** resuelve automáticamente recursos `package://` ni expande Xacro.

### 8.2 Conversión general con Xacro y referencias a paquetes

El siguiente ejemplo de script parametriza el nombre del archivo de entrada, la salida y la ubicación de uno o varios paquetes. Se puede copiar como `convertir_urdf_a_mjcf.py`. Convierte **solo el modelo base**; los actuadores y la escena se agregan después.

```python
#!/usr/bin/env python3
"""Genera un MJCF base desde URDF/Xacro; requiere mujoco y opcionalmente ROS 2."""
import argparse
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


def resolver_paquete(nombre, rutas):
    if nombre in rutas:
        return rutas[nombre]
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(nombre))
    except Exception as exc:
        raise FileNotFoundError(
            f"No se pudo localizar el paquete '{nombre}'. "
            f"Utiliza --paquete {nombre}=/ruta/al/paquete"
        ) from exc


def main():
    p = argparse.ArgumentParser(description="Conversión URDF/Xacro -> MJCF base")
    p.add_argument("--entrada", required=True, type=Path)
    p.add_argument("--salida", required=True, type=Path)
    p.add_argument("--xacro", action="store_true", help="Expandir Xacro")
    p.add_argument("--paquete", action="append", default=[],
                   help="Correspondencia nombre=/ruta/a/paquete; repetible")
    args = p.parse_args()

    entrada = args.entrada.expanduser().resolve()
    salida = args.salida.expanduser().resolve()
    if not entrada.is_file():
        raise FileNotFoundError(f"No existe el archivo de entrada: {entrada}")

    rutas = {}
    for asignacion in args.paquete:
        nombre, separador, ruta = asignacion.partition("=")
        if not separador or not nombre or not ruta:
            p.error("--paquete debe ser nombre=/ruta/a/paquete")
        rutas[nombre] = Path(ruta).expanduser().resolve()

    if args.xacro:
        resultado = subprocess.run(
            ["xacro", str(entrada)], capture_output=True, text=True, check=True
        )
        contenido = resultado.stdout
    else:
        contenido = entrada.read_text(encoding="utf-8")

    raiz = ET.fromstring(contenido)
    if raiz.tag != "robot":
        raise ValueError("La entrada no es un URDF expandido con raíz <robot>")

    # Resolver rutas de mallas URDF, tanto package:// como relativas.
    for mesh in raiz.iter("mesh"):
        referencia = mesh.get("filename")
        if not referencia:
            continue
        if referencia.startswith("package://"):
            nombre_y_ruta = referencia.removeprefix("package://")
            nombre, separador, subruta = nombre_y_ruta.partition("/")
            if not separador:
                raise ValueError(f"Ruta package:// incompleta: {referencia}")
            archivo = resolver_paquete(nombre, rutas) / subruta
        else:
            archivo = Path(referencia).expanduser()
            if not archivo.is_absolute():
                archivo = entrada.parent / archivo
        archivo = archivo.resolve()
        if not archivo.is_file():
            raise FileNotFoundError(f"No existe la malla: {archivo}")
        mesh.set("filename", str(archivo))

    urdf_resuelto = ET.tostring(raiz, encoding="unicode")
    modelo = mujoco.MjModel.from_xml_string(urdf_resuelto)
    salida.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(salida), modelo)

    # No considerar exitosa la conversión sin reabrir el XML generado.
    prueba = mujoco.MjModel.from_xml_path(str(salida))
    print(f"MJCF guardado: {salida}")
    print(f"Cuerpos={prueba.nbody}, articulaciones={prueba.njnt}, "
          f"geometrías={prueba.ngeom}, actuadores={prueba.nu}")


if __name__ == "__main__":
    main()
```

**Ejemplo de uso con los archivos del Dofbot:**

```bash
source /opt/ros/jazzy/setup.bash
source ~/mujoco_venv/bin/activate

python convertir_urdf_a_mjcf.py \
  --entrada "$HOME/robot-x/mujoco_ws/src/robotx_description/urdf/robotx_position.urdf" \
  --salida "$HOME/robot-x/mujoco_ws/src/robotx_description/mujoco_model/robotx_base.xml" \
  --xacro \
  --paquete "robotx_description=$HOME/robot-x/mujoco_ws/src/robotx_description"
```

**Interpretación:** el archivo `robotx_base.xml` es la conversión del robot **sin** la escena ni los actuadores añadidos por el `mujoco_file_transformer.py`. No esperes que sea idéntico al `robotx_mujoco.xml` del repositorio. Para otro brazo cambia las rutas y repite `--paquete` si las mallas pertenecen a más de un paquete.

**Alcance del script de ejemplo:** resuelve los atributos `filename` de elementos URDF `<mesh>`; no implementa todas las posibles variantes de recursos, formatos externos, extensiones de ROS ni una exportación portátil de mallas. Si la salida usa rutas absolutas, funcionará en esa máquina mientras esos archivos existan, pero para compartir el modelo conviene copiar recursos y ajustar las referencias.

### 8.3 Incorporar la configuración de simulación

Una conversión básica puede no tener actuadores. Para pasar a una simulación manipulable:

1. Incorporar actuadores de posición, velocidad o torque, según el robot y la estrategia de control.
2. Establecer límites, ganancias, amortiguamiento y fricción con valores físicamente razonables.
3. Incorporar, cuando corresponda, restricciones de pinza y mecanismos acoplados.
4. Definir el plano y otros cuerpos de la escena.
5. Volver a compilar y verificar el MJCF completo.

En el caso Dofbot **esto ya lo realiza** `mujoco_file_transformer.py` leyendo sus dos XML auxiliares. Para una solución reutilizable conviene mantener esos ajustes separados y utilizar el mecanismo `<include>` de MJCF o un ensamblador XML que valide la jerarquía [1]. Las inclusiones deben conservar una estructura XML compatible y no introducir nombres duplicados.

## 9. Estructura y configuración de un archivo MJCF

Un MJCF mínimo tiene un elemento raíz `<mujoco>` y una sección `<worldbody>`; otros bloques son opcionales y dependen de la funcionalidad requerida:

```xml
<mujoco model="ejemplo_brazo">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.005"/>

  <worldbody>
    <geom name="suelo" type="plane" size="2 2 0.1"/>
    <body name="base" pos="0 0 0.1">
      <geom type="box" size="0.1 0.1 0.1" mass="1"/>
      <body name="eslabon_1" pos="0 0 0.15">
        <joint name="articulacion_1" type="hinge"
               axis="0 0 1" range="-1.57 1.57"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.25"
              size="0.03" mass="0.3"/>
      </body>
    </body>
  </worldbody>

  <actuator>
    <position name="control_1" joint="articulacion_1"
              kp="30" ctrlrange="-1.57 1.57"/>
  </actuator>
</mujoco>
```

Este fragmento es **didáctico**, no corresponde a las dimensiones del Dofbot. Muestra la organización general:

- `<worldbody>`: jerarquía de cuerpos, juntas, geometrías y mundo.
- `<asset>`: mallas, materiales y texturas externas.
- `<actuator>`: entradas de control del robot.
- `<equality>` y `<contact>`: restricciones y tratamiento especial de colisiones.
- `<option>` / `<compiler>` / `<default>`: configuración física y valores predeterminados.

**Aspectos que requieren revisión después de la conversión:** la representación visual y la de colisión pueden ser diferentes; los cuerpos fijos pueden fusionarse durante la compilación; las matrices de inercia deben ser válidas; y la cantidad de actuadores (`nu`) puede ser cero aunque haya articulaciones (`njnt`). La presencia de un `<joint>` no implica que haya un motor asociado.

## 10. Validación del modelo obtenido

### 10.1 Prueba automática de carga y simulación

Definir la ruta al archivo MJCF final en la terminal:

```bash
XML="$(ros2 pkg prefix robotx_description)/share/robotx_description/mujoco_model/robotx_mujoco.xml"
export XML
```

Ejecutar:

```bash
python - <<'PY'
import os
import numpy as np
import mujoco

ruta = os.environ["XML"]
m = mujoco.MjModel.from_xml_path(ruta)
d = mujoco.MjData(m)

print("Modelo:", m.name)
print("nbody:", m.nbody, "njnt:", m.njnt,
      "ngeom:", m.ngeom, "nu:", m.nu)
print("Articulaciones:")
for i in range(m.njnt):
    print(" -", m.joint(i).name, "rango:", m.jnt_range[i])

for _ in range(200):
    mujoco.mj_step(m, d)

if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all():
    raise RuntimeError("La simulación generó un estado no finito")
print("Prueba básica: OK | tiempo simulado:", d.time)
PY
```

**Qué comprueba:** lectura del XML, conteos, nombres y rangos; ejecución de 200 pasos sin estados numéricos no finitos. **Qué no comprueba:** exactitud de masas, ausencia de todas las penetraciones, calidad del control, alcance real del brazo o correspondencia con hardware.

### 10.2 Inspección visual y física

Abrir:

```bash
python -m mujoco.viewer --mjcf="$XML"
```

Verificar al menos:

- Que están presentes **todas las partes visibles** del brazo y la pinza.
- Que sus escalas y orientaciones son razonables; ninguna malla aparece desplazada o sobredimensionada.
- Que las articulaciones están correctamente ubicadas y tienen ejes coherentes.
- Que existen colisiones relevantes, sin bloquear el movimiento por autocolisiones incorrectas.
- Que el robot no atraviesa el piso ni presenta vibraciones o aceleraciones claramente no físicas al iniciar.
- Que los actuadores responden a valores de control dentro de sus límites; verificar el sentido real de movimiento.

## 11. Adaptación a otros brazos robóticos

La conversión no debería exigir copiar manualmente todos los archivos en una sola carpeta. El objetivo es mantener paquetes **independientes** y reutilizar una herramienta de conversión **parametrizada**.

| Elemento | Caso Dofbot | Qué cambiar para otro robot |
|---|---|---|
| Paquete de descripción | `robotx_description` | Nombre del paquete con el URDF/Xacro. |
| Archivo principal | `robotx_position.urdf` | Punto de entrada real del nuevo modelo. |
| Mallas | `meshes/*.STL` | Carpetas visuales y de colisión y formatos. |
| Convertidor | `mujoco_file_transformer.py` | Preferentemente parametrizar entrada, salida y paquetes. |
| XML base | Modelo generado por MuJoCo | Crear uno por robot. |
| Escena | `robotx_mujoco_scene.xml` | Reutilizar una escena común o definir una específica. |
| Actuadores | `robotx_mujoco_config.xml` | Ajustar número, nombres, tipo y parámetros. |
| Validación | Dofbot previamente ejecutado | Repetir validación para **cada** brazo. |

### Protocolo repetible por robot

1. **Registrar** nombre, origen, versión y licencia del paquete del robot.
2. **Localizar** URDF/Xacro principal, inclusiones, mallas y parámetros de inercia.
3. **Expandir** Xacro, si corresponde, y revisar que la salida sea URDF válido.
4. **Resolver** todas las rutas de recursos, incluso las que provengan de otros paquetes.
5. **Convertir** URDF expandido a MJCF base y guardar una copia con nombre estable.
6. **Añadir** escena y actuadores propios del robot; no copiar indiscriminadamente los controles de Dofbot.
7. **Verificar** recompilación del MJCF, geometría, movimiento y condiciones físicas.
8. **Documentar** comandos exactos, versiones, cambios, problemas y resultado.

Para un robot nuevo, **no** es recomendable modificar `robotx_description` hasta convertirlo en una mezcla de varios robots. Mantener `*_description` para cada modelo y una herramienta común de conversión facilita automatizar el proceso y comparar resultados.

## 12. Problemas habituales y soluciones

| Síntoma o mensaje | Causa frecuente | Comprobación o corrección |
|---|---|---|
| `ModuleNotFoundError: No module named 'mujoco'` | El ejecutable ROS y `pip` usan distintos Python. | Verificar `which python`, activar entorno correcto y comprobar importación antes de lanzar ROS. |
| `ModuleNotFoundError: moveit_configs_utils` | Importación no utilizada en el `launch` original. | Retirar esa importación de manera controlada y recompilar `robotx_bringup`; no es necesario MoveIt para el conversor. |
| `Package 'xacro' not found` o error al ejecutar Xacro | Dependencia ausente o ROS no inicializado. | `source /opt/ros/jazzy/setup.bash`; instalar `ros-jazzy-xacro`. |
| `Package ... not found` | Workspace sin compilar o sin `source install/setup.bash`. | Ejecutar `colcon build`, cargar entorno y comprobar `ros2 pkg prefix`. |
| `mesh file ... not found` | Rutas `package://` sin resolver o referencias relativas rotas. | Revisar `filename`, directorio de mallas y rutas resultantes. |
| Error de masa/inercia | Datos físicos nulos, inconsistentes o no válidos. | Revisar `<inertial>`; corregir datos en el URDF de origen. No ocultar sistemáticamente el problema con ajustes del compilador. |
| El robot aparece incompleto | Mallas visuales descartadas o modelos de geometría faltantes. | Revisar `discardvisual="false"` en extensiones URDF de MuJoCo, existencia de las mallas y geometrías. |
| El robot se ve, pero no se mueve con controles | Falta de actuadores o articulación asociada incorrecta. | Revisar `<actuator>`, nombres de `joint` y `model.nu`. |
| Cambios en `src/` no se reflejan | Se está ejecutando una copia instalada desactualizada. | Volver a compilar, cargar `install/setup.bash` y comprobar la ruta del XML; usar `--symlink-install` cuando corresponda. |
| El visor no abre en Ubuntu | Problema de entorno gráfico, controlador o acceso a pantalla. | Comprobar primero la carga programática; revisar sesión gráfica y dependencias OpenGL. |
| El robot vibra, penetra superficies o se desestabiliza | Parámetros dinámicos, contactos o ganancias inadecuados. | Revisar masas, inercias, fricción, límites, `kp` y paso temporal; no dar por buena una conversión solo porque cargó. |

**Nota sobre la extensión URDF:** MuJoCo permite un bloque `<mujoco>` dentro de `<robot>`, por ejemplo:

```xml
<mujoco>
  <compiler discardvisual="false" balanceinertia="true"/>
</mujoco>
```

Es una capacidad documentada oficialmente [1], pero debe usarse con cuidado: `balanceinertia` altera cómo el compilador trata algunas inercias y no sustituye medirlas o modelarlas correctamente. Las extensiones URDF no tienen la misma validación estricta de atributos que MJCF; errores tipográficos pueden pasar inadvertidos [1].

## 13. Criterios de aceptación y registro de conversión

### Lista de verificación

- [ ] Se identificó el archivo URDF/Xacro de entrada y su paquete.
- [ ] Se registraron las dependencias (ROS/Python/MuJoCo/Xacro) con versiones.
- [ ] Xacro se expandió sin errores, si aplica.
- [ ] Todas las mallas referenciadas existen y están accesibles.
- [ ] MuJoCo cargó el URDF expandido.
- [ ] Se generó un archivo MJCF/XML persistente.
- [ ] Se volvió a cargar **el archivo MJCF generado**, no solo el URDF original.
- [ ] Se compararon número y nombres de cuerpos/articulaciones con el modelo de origen.
- [ ] La geometría visual y las colisiones son coherentes.
- [ ] Las masas, inercias, límites y ejes fueron revisados.
- [ ] Se configuraron los actuadores correspondientes, si se requiere control.
- [ ] Se verificó una simulación básica sin inestabilidad manifiesta.
- [ ] Se guardaron comandos, incidencias y rutas del archivo final.

### Ficha para documentar cada modelo

```text
Robot / modelo:
Repositorio o procedencia:
Versión / commit:
Sistema operativo:
ROS 2:
Python:
MuJoCo:
URDF/Xacro de entrada:
Paquetes y mallas necesarios:
Herramienta o script empleado:
Comandos ejecutados:
XML MJCF generado:
Número de links/joints esperado:
Número de bodies/joints/actuators obtenido:
Ajustes manuales del MJCF:
Errores encontrados y solución:
Resultado visual y dinámico:
Fecha y responsable de prueba:
```

La ficha es especialmente útil al pasar del Dofbot al siguiente brazo, porque registra **qué cambia y qué permanece igual** en el procedimiento.

## 14. Limitaciones y mejoras sugeridas

**Resultados ya sustentados:** la revisión de `robot-x` confirma que contiene un flujo de expansión Xacro, compilación mediante `mujoco`, guardado MJCF y añadido de configuración/escena. En una sesión previa se logró ejecutar el caso Dofbot; esta documentación no equivale a haber probado los demás brazos.

**Mejoras propuestas para una versión de producción:**

1. **Parámetros de línea de comandos o archivo YAML** para entrada, salida, nombre del paquete y escenas, en lugar de rutas fijas a `robotx_description`.
2. **Resolución general de `package://`**, con un mapa por paquete o `ament_index_python`, y revisión explícita de cada recurso.
3. **Separación en etapas**: procesar URDF/Xacro, convertir MJCF base, ensamblar escena y validar.
4. **Lanzamiento secuencial real**: evitar usar `OnProcessStart` como sustituto de «el paso anterior terminó». El flujo actual vuelve a expandir el mismo Xacro.
5. **Composición XML robusta**, preferentemente mediante MJCF `<include>` cuando aplique o un parser, en lugar de sustituciones textuales de cierres XML.
6. **Pruebas automáticas** por robot con un archivo de resultados y comparaciones de articulaciones, mallas y actuadores.
7. **Portabilidad:** reemplazar rutas absolutas por rutas relativas comprobadas y distribuir las mallas junto con el MJCF o un paquete de recursos compatible.

**Conclusión:** el repositorio permite estudiar un caso real de URDF/Xacro a MuJoCo y ofrece una base reutilizable. Lo que debe generalizarse **no es el nombre de las carpetas del Dofbot**, sino el proceso: identificar archivos del robot → expandir Xacro → resolver dependencias → compilar a MuJoCo → guardar MJCF → configurar física y escena → validar. Con esa separación se puede aplicar a otros brazos robóticos sin rehacer toda la documentación.

## 15. Referencias

Documentación y código consultados el **28 de octubre de 2026**:

1. MuJoCo, **Modeling**, sección *URDF extensions* y mecanismos MJCF. https://mujoco.readthedocs.io/en/stable/modeling.html
2. MuJoCo, **Python API** y **Model Editing** (carga, compilación y guardado de modelos). https://mujoco.readthedocs.io/en/stable/python.html ; https://mujoco.readthedocs.io/en/stable/programming/modeledit.html
3. Roberto García, **robot-x**, repositorio de GitHub. https://github.com/Roberto-Garcia-1/robot-x
4. `mujoco_file_transformer.py`, código fuente de la conversión. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_mujoco/robotx_mujoco/mujoco_file_transformer.py
5. `urdf_file_compiler.py`, código fuente del procesamiento Xacro. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/robotx_description/urdf_file_compiler.py
6. `robotx_file_compiler.launch.py`, definición del lanzamiento ROS 2. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_bringup/launch/robotx_file_compiler.launch.py
7. `robotx_position.urdf` y `robotx.urdf`, archivos URDF/Xacro de referencia. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/urdf/robotx_position.urdf ; https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/urdf/robotx.urdf
8. `robotx_macros.urdf`, definición de las macros para enlaces y articulaciones. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/urdf/robotx_macros.urdf
9. `setup.py` de `robotx_description` y `robotx_mujoco`, registro de ejecutables y archivos de instalación. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/setup.py ; https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_mujoco/setup.py
10. `robotx_mujoco_config.xml`, configuración de actuadores y física. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/mujoco_model/robotx_mujoco_config.xml
11. `robotx_mujoco_scene.xml`, elementos de la escena. https://github.com/Roberto-Garcia-1/robot-x/blob/main/mujoco_ws/src/robotx_description/mujoco_model/robotx_mujoco_scene.xml

---

*Documento inicial de referencia técnica. Antes de considerar un robot «migrado», ejecutar y registrar sus pruebas específicas de geometría, articulaciones, actuadores y estabilidad física.*
