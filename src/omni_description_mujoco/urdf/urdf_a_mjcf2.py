#!/usr/bin/env python3
"""Genera un MJCF completo desde URDF/Xacro, con configuración y escena opcionales.

Requiere mujoco y numpy y, opcionalmente, ROS 2 (xacro, ament_index_python).

Correcciones automáticas (solo se aplican cuando detectan el problema):
  * --flotante con raíz estática: MuJoCo funde el enlace raíz (y los enlaces
    unidos a él con joints fixed) en el mundo, así que el chasis queda como un
    geom estático, sin masa, y las ruedas cuelgan del mundo. Aquí se crea un
    cuerpo con freejoint que contiene todo y se le calcula la inercia combinada
    a partir de los <inertial> del URDF.
  * Filtros de colisión para robots con ruedas: por defecto todos los geoms
    chocan con todos (chasis contra rodillos, rodillos entre sí...), lo que
    produce contactos falsos y fuerzas enormes. Solo las ruedas/rodillos que
    tocan el suelo quedan con colisión.
  * Altura inicial: si el robot arranca hundido en el suelo se sube lo justo.
"""
import argparse
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import mujoco

# Secciones de MJCF que solo pueden aparecer una vez dentro de <mujoco>.
# Si el MJCF generado ya tiene una, el contenido de --config se fusiona en ella.
SECCIONES_UNICAS = {
    "compiler", "option", "size", "visual", "statistic", "default",
    "asset", "actuator", "sensor", "contact", "equality", "tendon",
    "keyframe", "custom", "extension",
}

# Nombres de cuerpos que se consideran ruedas/rodillos (sin distinguir mayúsculas).
PATRON_RUEDAS = r"wheel|rueda|roller|rodillo|llanta|tire|tyre"

# Orientaciones alternativas a pos/quat que puede llevar un <frame>.
ORIENTACIONES_RARAS = ("euler", "axisangle", "xyaxes", "zaxis")


# --------------------------------------------------------------------------
# Utilidades generales
# --------------------------------------------------------------------------
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


def compilar(mjcf):
    return mujoco.MjModel.from_xml_string(ET.tostring(mjcf, encoding="unicode"))


def mostrar_articulaciones(modelo, titulo):
    print(titulo)
    for i in range(modelo.njnt):
        print(f"  {modelo.joint(i).name}: {modelo.jnt_range[i]}")


# --------------------------------------------------------------------------
# Álgebra mínima: rotaciones y tensores de inercia
# --------------------------------------------------------------------------
def _numeros(texto, n):
    if not texto:
        return np.zeros(n)
    valores = [float(v) for v in texto.split()]
    if len(valores) != n:
        raise ValueError(f"Se esperaban {n} números y llegó '{texto}'")
    return np.array(valores)


def _texto(vector):
    return " ".join(f"{v:.9g}" for v in vector)


def _rot_rpy(rpy):
    """Rotación URDF: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _rot_quat(q):
    """Rotación desde un cuaternión MuJoCo (w x y z)."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _origen(elemento):
    """Devuelve (R, t) del <origin xyz rpy> de un elemento URDF."""
    origen = elemento.find("origin") if elemento is not None else None
    if origen is None:
        return np.eye(3), np.zeros(3)
    return _rot_rpy(_numeros(origen.get("rpy"), 3)), _numeros(origen.get("xyz"), 3)


def inercia_de_enlaces_fijos(urdf):
    """Masa, centro de masa e inercia combinados del enlace raíz y de todos los
    enlaces unidos a él solo por joints 'fixed'.

    Son justo los enlaces que MuJoCo funde con el mundo cuando importa el URDF,
    y cuya masa/inercia se pierde. Todo se expresa en el sistema del enlace raíz.
    Devuelve None si ninguno tiene masa."""
    enlaces = {e.get("name"): e for e in urdf.findall("link")}
    joints = urdf.findall("joint")
    hijos = {j.find("child").get("link") for j in joints}
    raices = [n for n in enlaces if n not in hijos]
    if not raices:
        return None
    raiz = raices[0]

    poses = {raiz: (np.eye(3), np.zeros(3))}
    pendientes = [raiz]
    while pendientes:
        actual = pendientes.pop()
        for j in joints:
            if j.get("type") != "fixed" or j.find("parent").get("link") != actual:
                continue
            hijo = j.find("child").get("link")
            if hijo in poses:
                continue
            rj, tj = _origen(j)
            ra, ta = poses[actual]
            poses[hijo] = (ra @ rj, ra @ tj + ta)
            pendientes.append(hijo)

    piezas = []  # (nombre, masa, centro de masa, tensor respecto a su centro, en ejes raíz)
    for nombre, (rl, tl) in poses.items():
        inercial = enlaces[nombre].find("inertial")
        if inercial is None:
            continue
        masa_el = inercial.find("mass")
        masa = float(masa_el.get("value")) if masa_el is not None else 0.0
        if masa <= 0:
            continue
        ri, ti = _origen(inercial)
        tensor = np.zeros((3, 3))
        i = inercial.find("inertia")
        if i is not None:
            ixx, iyy, izz = (float(i.get(k, 0)) for k in ("ixx", "iyy", "izz"))
            ixy, ixz, iyz = (float(i.get(k, 0)) for k in ("ixy", "ixz", "iyz"))
            tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
        r = rl @ ri
        piezas.append((nombre, masa, rl @ ti + tl, r @ tensor @ r.T))
    if not piezas:
        return None

    masa_total = sum(p[1] for p in piezas)
    com = sum(p[1] * p[2] for p in piezas) / masa_total
    inercia = np.zeros((3, 3))
    for _, masa, centro, tensor in piezas:
        d = centro - com
        inercia += tensor + masa * ((d @ d) * np.eye(3) - np.outer(d, d))  # Steiner
    principal = max(piezas, key=lambda p: p[1])[0]
    return {"masa": masa_total, "com": com, "inercia": inercia,
            "nombre": principal, "raiz": raiz}


# --------------------------------------------------------------------------
# Base flotante
# --------------------------------------------------------------------------
def _encapsular_raiz_estatica(wb, elementos, urdf, masa_base):
    """El enlace raíz quedó fundido con el mundo (geoms/frames/cuerpos sueltos en
    <worldbody>). Se mete todo en un cuerpo nuevo y se le devuelve su inercia."""
    inercia = inercia_de_enlaces_fijos(urdf) if urdf is not None else None
    nombre = inercia["nombre"] if inercia else "base_link"
    existentes = {b.get("name") for b in wb.iter("body")}
    while nombre in existentes:
        nombre += "_raiz"

    atributos, contenido = {}, elementos
    rot, trans = np.eye(3), np.zeros(3)
    if (len(elementos) == 1 and elementos[0].tag == "frame"
            and not any(k in elementos[0].attrib for k in ORIENTACIONES_RARAS)):
        # El <frame> único es la pose del enlace: la hereda el cuerpo nuevo.
        marco = elementos[0]
        for k in ("pos", "quat"):
            if marco.get(k):
                atributos[k] = marco.get(k)
        trans = _numeros(marco.get("pos"), 3)
        if marco.get("quat"):
            rot = _rot_quat(_numeros(marco.get("quat"), 4))
        contenido = list(marco)

    cuerpo = ET.Element("body", {"name": nombre, **atributos})
    if inercia:
        com = rot.T @ (inercia["com"] - trans)
        tensor = rot.T @ inercia["inercia"] @ rot
        ET.SubElement(cuerpo, "inertial", {
            "pos": _texto(com),
            "mass": f"{inercia['masa']:.9g}",
            "fullinertia": _texto([tensor[0, 0], tensor[1, 1], tensor[2, 2],
                                   tensor[0, 1], tensor[0, 2], tensor[1, 2]]),
        })
        print(f"Raíz estática detectada: se encapsula en el cuerpo '{nombre}' "
              f"(masa {inercia['masa']:.4f} kg e inercia tomadas del URDF).")
    else:
        # Sin <inertial> en el URDF: valores de partida, hay que revisarlos.
        ET.SubElement(cuerpo, "inertial", {
            "pos": "0 0 0", "mass": f"{masa_base:g}",
            "diaginertia": _texto([0.004 * masa_base] * 3),
        })
        print(f"AVISO: el URDF no tiene masa en el enlace raíz. Se usan {masa_base:g} kg "
              f"y una inercia de partida; ajústalo con --masa-base o añade <inertial> al xacro.")
    for elemento in contenido:
        cuerpo.append(elemento)

    posicion = list(wb).index(elementos[0])
    for elemento in elementos:
        wb.remove(elemento)
    wb.insert(posicion, cuerpo)
    return cuerpo


def hacer_flotante(mjcf, altura, urdf=None, masa_base=1.0):
    """Añade una articulación libre al robot. Devuelve el cuerpo raíz (o None
    si ya era flotante). Si 'altura' es None se conserva la z del modelo."""
    worldbody = mjcf.find("worldbody")
    if worldbody is None:
        raise ValueError("El MJCF generado no tiene <worldbody>")
    if (worldbody.find(".//freejoint") is not None
            or worldbody.find(".//joint[@type='free']") is not None):
        print("El robot ya es flotante; no se modifica.")
        return None

    elementos = [e for e in worldbody if e.tag in ("body", "geom", "frame", "site")]
    if not elementos:
        raise ValueError("No se encontró el robot en <worldbody>")

    if len(elementos) == 1 and elementos[0].tag == "body":
        raiz = elementos[0]            # caso normal: un único cuerpo raíz
    else:
        raiz = _encapsular_raiz_estatica(worldbody, elementos, urdf, masa_base)

    raiz.insert(0, ET.Element("freejoint", {"name": "base_libre"}))
    if altura is not None:
        x, y, _ = (raiz.get("pos") or "0 0 0").split()
        raiz.set("pos", f"{x} {y} {altura}")
    print(f"Base flotante: cuerpo '{raiz.get('name')}' con freejoint, "
          f"z inicial={raiz.get('pos', '0 0 0').split()[2]}")
    return raiz


def asentar_en_suelo(mjcf, raiz, modelo, margen=1e-4, iteraciones=4):
    """Si el robot arranca penetrando el suelo, sube su cuerpo raíz lo justo."""
    for _ in range(iteraciones):
        datos = mujoco.MjData(modelo)
        mujoco.mj_forward(modelo, datos)
        penetraciones = []
        for i in range(datos.ncon):
            c = datos.contact[i]
            en_mundo = (modelo.geom_bodyid[c.geom1] == 0, modelo.geom_bodyid[c.geom2] == 0)
            if en_mundo[0] != en_mundo[1]:      # un geom del mundo y otro del robot
                penetraciones.append(c.dist)
        if not penetraciones or min(penetraciones) >= 0:
            break
        x, y, z = (float(v) for v in (raiz.get("pos") or "0 0 0").split())
        z += -min(penetraciones) + margen
        raiz.set("pos", f"{x:.9g} {y:.9g} {z:.9g}")
        print(f"El robot arrancaba {-min(penetraciones) * 1000:.2f} mm dentro del suelo: z={z:.5f}")
        modelo = compilar(mjcf)
    return modelo


# --------------------------------------------------------------------------
# Filtros de colisión para robots con ruedas
# --------------------------------------------------------------------------
def filtrar_colisiones(mjcf, patron, colision_chasis, forzar):
    """Deja con colisión solo las ruedas/rodillos que tocan el suelo.

    contype/conaffinity (colisiona si contype1 & conaffinity2 o contype2 & conaffinity1):
      ruedas/rodillos hoja : contype=1 conaffinity=0 -> chocan con el suelo (1/1),
                             no entre sí ni con el chasis
      cubo de rueda        : 0/0 (solo visual; sus rodillos hacen el contacto)
      resto (chasis...)    : 0/0, o 2/2 con --colision-chasis (solo choca con
                             objetos que tengan contype=3 conaffinity=3)
    Devuelve True si modificó el modelo."""
    regex = re.compile(patron, re.IGNORECASE)
    cuerpos = list(mjcf.iter("body"))
    ruedas = [b for b in cuerpos if regex.search(b.get("name", ""))]
    if not ruedas:
        print("Filtros de colisión: no detecté ruedas "
              f"(patrón '{patron}'); no se modifica. Usa --patron-ruedas.")
        return False
    geoms = list(mjcf.iter("geom"))
    if not forzar and any(g.get("contype") is not None or g.get("conaffinity") is not None
                          for g in geoms):
        print("Filtros de colisión: el modelo ya define contype/conaffinity; no se modifica "
              "(usa --colisiones si para forzarlo).")
        return False

    padre = {hijo: p for p in mjcf.iter() for hijo in p}

    def cuerpo_de(elemento):
        p = padre.get(elemento)
        while p is not None and p.tag != "body":
            p = padre.get(p)
        return p

    contacto = {b for b in ruedas
                if not any(regex.search(d.get("name", "")) for d in b.iter("body") if d is not b)}
    cubos = set(ruedas) - contacto
    n = {"contacto": 0, "cubo": 0, "resto": 0}
    for g in geoms:
        b = cuerpo_de(g)
        if b in contacto:
            g.set("contype", "1")
            g.set("conaffinity", "0")
            n["contacto"] += 1
        elif b in cubos:
            g.set("contype", "0")
            g.set("conaffinity", "0")
            g.set("group", "1")
            n["cubo"] += 1
        else:
            g.set("contype", "2" if colision_chasis else "0")
            g.set("conaffinity", "2" if colision_chasis else "0")
            g.set("group", "1")
            n["resto"] += 1
    print(f"Filtros de colisión: {n['contacto']} geoms de rueda/rodillo con contacto, "
          f"{n['cubo']} de cubo de rueda y {n['resto']} del resto "
          f"({'2/2: solo contra objetos con contype/conaffinity=3' if colision_chasis else 'sin colisión'}).")
    return True


# --------------------------------------------------------------------------
# Salida
# --------------------------------------------------------------------------
def hacer_mallas_relativas(salida):
    """Copia las mallas a <carpeta del XML>/meshes y deja rutas relativas."""
    arbol = ET.parse(salida)
    mjcf = arbol.getroot()
    compilador = mjcf.find("compiler")
    meshdir = (compilador.get("meshdir") if compilador is not None else None) or ""
    destino = salida.parent / "meshes"
    destino.mkdir(exist_ok=True)
    copiadas = {}
    for mesh in mjcf.iter("mesh"):
        archivo = mesh.get("file")
        if not archivo:
            continue
        origen = Path(archivo)
        if not origen.is_absolute():
            origen = salida.parent / meshdir / archivo
        origen = origen.resolve()
        if not origen.is_file():
            raise FileNotFoundError(f"No existe la malla: {origen}")
        if copiadas.setdefault(origen.name, origen) != origen:
            raise ValueError(f"Dos mallas distintas se llaman '{origen.name}'")
        shutil.copy2(origen, destino / origen.name)
        mesh.set("file", origen.name)
    if compilador is None:
        compilador = ET.Element("compiler")
        mjcf.insert(0, compilador)
    compilador.set("meshdir", "meshes")
    ET.indent(mjcf, space="  ")
    arbol.write(salida, encoding="utf-8")
    print(f"Mallas copiadas a {destino} ({len(copiadas)} archivos); el XML usa rutas relativas.")


def verificar(modelo):
    """Informe rápido: contactos entre piezas del robot y fuerza normal frente al peso."""
    datos = mujoco.MjData(modelo)
    mujoco.mj_forward(modelo, datos)

    def cuerpo(g):
        return mujoco.mj_id2name(modelo, mujoco.mjtObj.mjOBJ_BODY, int(modelo.geom_bodyid[g])) or "world"

    pares = {}
    for i in range(datos.ncon):
        c = datos.contact[i]
        clave = tuple(sorted((cuerpo(c.geom1), cuerpo(c.geom2))))
        pares.setdefault(clave, []).append(float(c.dist))
    internos = {k: v for k, v in pares.items() if "world" not in k}
    print(f"\nVerificación: {datos.ncon} contactos iniciales, "
          f"{len(internos)} pares robot-robot (deberían ser 0)")
    for k, v in list(internos.items())[:10]:
        print(f"   {k}  dist mín = {min(v):.4f}")

    for _ in range(int(1.0 / modelo.opt.timestep)):     # 1 s con ctrl = 0
        mujoco.mj_step(modelo, datos)
    f = np.zeros(6)
    normal = 0.0
    for i in range(datos.ncon):
        mujoco.mj_contactForce(modelo, datos, i, f)
        normal += f[0]
    peso = float(modelo.body_mass.sum()) * abs(float(modelo.opt.gravity[2]))
    print(f"Tras 1 s: {datos.ncon} contactos | fuerza normal total {normal:.2f} N "
          f"| peso del robot {peso:.2f} N")
    print("Correcto si ambos valores son parecidos y no hay pares robot-robot "
          "(sin suelo en la escena la fuerza normal será 0).")


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
                   help="Añade un freejoint al robot (móvil, no fijo al mundo). Si el enlace "
                        "raíz quedó fundido con el mundo, crea el cuerpo raíz con su inercia")
    p.add_argument("--altura", type=float, default=None,
                   help="Altura inicial (z) del cuerpo raíz con --flotante. Si se omite se "
                        "conserva la z del modelo y se sube lo justo si arranca dentro del suelo")
    p.add_argument("--masa-base", type=float, default=1.0,
                   help="Masa (kg) de la base solo si el URDF no define ninguna (por defecto 1.0)")
    p.add_argument("--colisiones", choices=("auto", "si", "no"), default="auto",
                   help="Filtros de colisión para robots con ruedas. auto: se aplican si detecta "
                        "ruedas y el modelo no define contype/conaffinity; si: forzar; no: omitir")
    p.add_argument("--patron-ruedas", default=PATRON_RUEDAS,
                   help=f"Regex con los nombres de cuerpos de ruedas/rodillos (por defecto '{PATRON_RUEDAS}')")
    p.add_argument("--colision-chasis", action="store_true",
                   help="El chasis y demás geoms no-rueda colisionan (2/2) solo con objetos "
                        "creados con contype=3 conaffinity=3")
    p.add_argument("--mallas-relativas", action="store_true",
                   help="Copia las mallas junto al XML (carpeta meshes) y usa rutas relativas")
    p.add_argument("--verificar", action="store_true",
                   help="Imprime contactos entre piezas del robot y fuerza normal frente al peso")
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

    modificado = False
    raiz_flotante = None
    if args.flotante:
        raiz_flotante = hacer_flotante(mjcf, args.altura, raiz, args.masa_base)
        modificado = raiz_flotante is not None

    # Filtros de colisión: antes de inyectar la escena para no tocar su suelo.
    if args.colisiones != "no":
        modificado |= filtrar_colisiones(mjcf, args.patron_ruedas, args.colision_chasis,
                                         forzar=args.colisiones == "si")

    # Inyectar escena dentro de <worldbody>.
    if escena is not None:
        worldbody = mjcf.find("worldbody")
        if worldbody is None:
            raise ValueError("El MJCF generado no tiene <worldbody>")
        for elemento in leer_fragmento(escena):
            worldbody.append(elemento)
        modificado = True

    # Inyectar configuración en <mujoco> (fusionando secciones que ya existan).
    if config is not None:
        for elemento in leer_fragmento(config):
            fusionar(mjcf, elemento)
        modificado = True

    if modificado:
        modelo = compilar(mjcf)
        if raiz_flotante is not None and args.altura is None:
            modelo = asentar_en_suelo(mjcf, raiz_flotante, modelo)
        mostrar_articulaciones(modelo, "Articulaciones del modelo final:")

    salida.parent.mkdir(parents=True, exist_ok=True)
    mujoco.mj_saveLastXML(str(salida), modelo)
    if args.mallas_relativas:
        hacer_mallas_relativas(salida)

    # No considerar exitosa la conversión sin reabrir el XML generado.
    prueba = mujoco.MjModel.from_xml_path(str(salida))
    print(f"MJCF guardado: {salida}")
    print(f"Cuerpos={prueba.nbody}, articulaciones={prueba.njnt}, "
          f"geometrías={prueba.ngeom}, actuadores={prueba.nu}")
    if args.verificar:
        verificar(prueba)


if __name__ == "__main__":
    main()
