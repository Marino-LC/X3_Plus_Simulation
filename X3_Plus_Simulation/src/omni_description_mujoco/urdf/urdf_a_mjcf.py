#!/usr/bin/env python3
"""Genera un MJCF completo desde URDF/Xacro, con configuración y escena opcionales.

Requiere mujoco y, opcionalmente, ROS 2 (xacro, ament_index_python).
"""
import argparse
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco

# Secciones de MJCF que solo pueden aparecer una vez dentro de <mujoco>.
# Si el MJCF generado ya tiene una, el contenido de --config se fusiona en ella.
SECCIONES_UNICAS = {
    "compiler", "option", "size", "visual", "statistic", "default",
    "asset", "actuator", "sensor", "contact", "equality", "tendon",
    "keyframe", "custom", "extension",
}


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


def leer_fragmento(ruta):
    """Lee un archivo con uno o varios elementos XML sin raíz única
    y devuelve la lista de elementos."""
    texto = ruta.read_text(encoding="utf-8")
    try:
        envoltorio = ET.fromstring(f"<fragmento>{texto}</fragmento>")
    except ET.ParseError as exc:
        raise ValueError(f"XML inválido en {ruta}: {exc}") from exc
    return list(envoltorio)


def fusionar(mjcf, elemento):
    """Añade un elemento a <mujoco>; si es una sección única que ya existe, la fusiona."""
    existente = mjcf.find(elemento.tag) if elemento.tag in SECCIONES_UNICAS else None
    if existente is None:
        mjcf.append(elemento)
    else:
        existente.attrib.update(elemento.attrib)
        existente.extend(list(elemento))


def hacer_flotante(mjcf, altura):
    """Añade una articulación libre al cuerpo raíz del robot y lo sube a 'altura'."""
    worldbody = mjcf.find("worldbody")
    raiz = worldbody.find("body") if worldbody is not None else None
    if raiz is None:
        raise ValueError("No se encontró el cuerpo raíz del robot en <worldbody>")
    if raiz.find("freejoint") is not None or raiz.find("joint[@type='free']") is not None:
        print("El cuerpo raíz ya es flotante; no se modifica.")
        return
    raiz.insert(0, ET.Element("freejoint", {"name": "base_libre"}))
    x, y, _ = (raiz.get("pos") or "0 0 0").split()
    raiz.set("pos", f"{x} {y} {altura}")
    print(f"Base flotante: cuerpo '{raiz.get('name')}' con freejoint, z inicial={altura}")


def mostrar_articulaciones(modelo, titulo):
    print(titulo)
    for i in range(modelo.njnt):
        print(f"  {modelo.joint(i).name}: {modelo.jnt_range[i]}")


def main():
    p = argparse.ArgumentParser(
        description="Conversión URDF/Xacro -> MJCF con configuración y escena")
    p.add_argument("--entrada", required=True, type=Path)
    p.add_argument("--salida", required=True, type=Path)
    p.add_argument("--xacro", action="store_true", help="Expandir Xacro")
    p.add_argument("--paquete", action="append", default=[],
                   help="Correspondencia nombre=/ruta/a/paquete; repetible")
    p.add_argument("--config", type=Path,
                   help="XML con actuadores, sensores, opciones. Se añade dentro de <mujoco>")
    p.add_argument("--escena", type=Path,
                   help="XML con suelo, luces, cámaras. Se añade dentro de <worldbody>")
    p.add_argument("--flotante", action="store_true",
                   help="Añade un freejoint al cuerpo raíz (robot móvil, no fijo al mundo)")
    p.add_argument("--altura", type=float, default=0.01,
                   help="Altura inicial (z) del cuerpo raíz con --flotante (por defecto 0.01)")
    args = p.parse_args()

    entrada = args.entrada.expanduser().resolve()
    salida = args.salida.expanduser().resolve()
    if not entrada.is_file():
        raise FileNotFoundError(f"No existe el archivo de entrada: {entrada}")

    config = args.config.expanduser().resolve() if args.config else None
    escena = args.escena.expanduser().resolve() if args.escena else None
    for etiqueta, ruta in (("configuración", config), ("escena", escena)):
        if ruta is not None and not ruta.is_file():
            raise FileNotFoundError(f"No existe el archivo de {etiqueta}: {ruta}")

    rutas = {}
    for asignacion in args.paquete:
        nombre, separador, ruta = asignacion.partition("=")
        if not separador or not nombre or not ruta:
            p.error("--paquete debe ser nombre=/ruta/a/paquete")
        rutas[nombre] = Path(ruta).expanduser().resolve()

    if args.xacro:
        resultado = subprocess.run(
            ["xacro", str(entrada)], capture_output=True, text=True
        )
        if resultado.returncode != 0:
            raise RuntimeError(f"Error al procesar xacro:\n{resultado.stderr}")
        contenido = resultado.stdout
    else:
        contenido = entrada.read_text(encoding="utf-8")

    raiz = ET.fromstring(contenido)
    if raiz.tag != "robot":
        raise ValueError("La entrada no es un URDF expandido con raíz <robot>")

    # Resolver rutas de mallas URDF: package://, file:// y relativas.
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
        elif referencia.startswith("file://"):
            archivo = Path(referencia.removeprefix("file://"))
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
    mostrar_articulaciones(modelo, "Articulaciones tras importar el URDF:")

    # MJCF base en un archivo temporal para poder manipularlo como XML.
    with tempfile.TemporaryDirectory() as tmp:
        temporal = Path(tmp) / "base.xml"
        mujoco.mj_saveLastXML(str(temporal), modelo)
        arbol = ET.parse(temporal)
    mjcf = arbol.getroot()

    if args.flotante:
        hacer_flotante(mjcf, args.altura)

    # Inyectar escena dentro de <worldbody>.
    if escena is not None:
        worldbody = mjcf.find("worldbody")
        if worldbody is None:
            raise ValueError("El MJCF generado no tiene <worldbody>")
        for elemento in leer_fragmento(escena):
            worldbody.append(elemento)

    # Inyectar configuración en <mujoco> (fusionando secciones que ya existan).
    if config is not None:
        for elemento in leer_fragmento(config):
            fusionar(mjcf, elemento)

    if args.flotante or escena is not None or config is not None:
        mjcf_final = ET.tostring(mjcf, encoding="unicode")
        modelo = mujoco.MjModel.from_xml_string(mjcf_final)
        mostrar_articulaciones(modelo, "Articulaciones del modelo final:")

    salida.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(salida), modelo)

    # No considerar exitosa la conversión sin reabrir el XML generado.
    prueba = mujoco.MjModel.from_xml_path(str(salida))
    print(f"MJCF guardado: {salida}")
    print(f"Cuerpos={prueba.nbody}, articulaciones={prueba.njnt}, "
          f"geometrías={prueba.ngeom}, actuadores={prueba.nu}")


if __name__ == "__main__":
    main()
