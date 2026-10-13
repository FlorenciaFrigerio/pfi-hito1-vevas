#!/usr/bin/env python3
"""
Geometria visual del ABB IRB 120 mediante primitivas.

En vez de importar mallas .stl de terceros, la forma de cada eslabon se
construye con cilindros y cajas cuyas dimensiones salen de la MISMA
tabla D-H que define la cinematica. La geometria queda derivada, no
importada: si se corrige un parametro de la tabla, el modelo visual se
actualiza solo.

Criterio de dimensionado

    Cada eslabon i se dibuja como un cilindro que recorre el segmento
    entre el origen del marco i y el del marco i+1, mas una esfera en
    la articulacion. La longitud del segmento se toma de a_i o d_i
    segun cual sea no nulo; el radio es una fraccion del alcance, para
    que la proporcion sea realista sin inventar cotas que no estan en
    el manual.

    Como la junta 5 no traslada (a_5 = d_5 = 0, muneca esferica), su
    eslabon se representa solo con la esfera de la articulacion.

Uso
    python3 add_visuals.py            escribe urdf/irb120.urdf con visuales
    python3 add_visuals.py --plain    vuelve a la version sin geometria

Grupo VEVAS
"""

import sys
import numpy as np
from pathlib import Path

from fk_irb120 import DH_TABLE
import generate_urdf as gen

# Radios en metros. Decrecen hacia la muneca, como el robot real.
RADIOS = [0.075, 0.060, 0.055, 0.045, 0.038, 0.032, 0.028]

COLOR_CUERPO = "0.95 0.42 0.07 1.0"      # naranja ABB
COLOR_JUNTA = "0.18 0.20 0.22 1.0"       # gris oscuro


def material(nombre, rgba):
    return (f'      <material name="{nombre}">\n'
            f'        <color rgba="{rgba}"/>\n'
            f'      </material>\n')


def cilindro(largo, radio, eje, color, nombre_mat):
    """
    Cilindro que arranca en el origen del marco y se extiende 'largo'
    metros a lo largo de 'eje' ('x' o 'z').

    En URDF el cilindro se dibuja centrado en su origen y alineado con
    z. Para que recorra el segmento hay que desplazarlo medio largo, y
    si el eje es x, rotarlo +pi/2 en y.
    """
    if largo <= 1e-9:
        return ""
    if eje == "z":
        origen = f'<origin xyz="0 0 {largo/2:.6f}" rpy="0 0 0"/>'
    else:
        origen = f'<origin xyz="{largo/2:.6f} 0 0" rpy="0 1.5707963 0"/>'
    return (f'    <visual>\n'
            f'      {origen}\n'
            f'      <geometry>\n'
            f'        <cylinder length="{largo:.6f}" radius="{radio:.4f}"/>\n'
            f'      </geometry>\n'
            + material(nombre_mat, color) +
            f'    </visual>\n')


def esfera(radio, color, nombre_mat):
    return (f'    <visual>\n'
            f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
            f'      <geometry>\n'
            f'        <sphere radius="{radio:.4f}"/>\n'
            f'      </geometry>\n'
            + material(nombre_mat, color) +
            f'    </visual>\n')


def colision(largo, radio, eje):
    """Capsula simplificada: un solo cilindro, sin detalle visual."""
    if largo <= 1e-9:
        return ""
    if eje == "z":
        origen = f'<origin xyz="0 0 {largo/2:.6f}" rpy="0 0 0"/>'
    else:
        origen = f'<origin xyz="{largo/2:.6f} 0 0" rpy="0 1.5707963 0"/>'
    return (f'    <collision>\n'
            f'      {origen}\n'
            f'      <geometry>\n'
            f'        <cylinder length="{largo:.6f}" radius="{radio*0.95:.4f}"/>\n'
            f'      </geometry>\n'
            f'    </collision>\n')


def geometria_eslabon(i):
    """
    Devuelve el bloque de <visual>/<collision> del eslabon i.

    i = 0  base_link, dibuja el pedestal (d_1 de la fila 1)
    i>= 1  link_i, dibuja el segmento correspondiente a la fila i+1
    """
    r = RADIOS[min(i, len(RADIOS) - 1)]
    bloque = ""

    if i == 0:
        # Pedestal: disco ancho y bajo. No sale de la tabla, es soporte.
        bloque += (f'    <visual>\n'
                   f'      <origin xyz="0 0 0.02" rpy="0 0 0"/>\n'
                   f'      <geometry>\n'
                   f'        <cylinder length="0.04" radius="0.11"/>\n'
                   f'      </geometry>\n'
                   + material("base_gris", COLOR_JUNTA) +
                   f'    </visual>\n')
        return bloque

    if i > 6:
        return bloque + esfera(r * 1.15, COLOR_JUNTA, "junta")

    _, d, a, _ = DH_TABLE[i - 1]

    # El eslabon recorre a_i en x local, o d_i en z local.
    if abs(a) > 1e-9:
        bloque += cilindro(abs(a), r, "x", COLOR_CUERPO, "cuerpo")
        bloque += colision(abs(a), r, "x")
    elif abs(d) > 1e-9:
        bloque += cilindro(abs(d), r, "z", COLOR_CUERPO, "cuerpo")
        bloque += colision(abs(d), r, "z")

    # La esfera va DESPUES del cilindro: RViz aplica a todo el link el
    # primer <material> que encuentra, asi que el cuerpo debe ir primero
    # para que el eslabon salga naranja y no gris.
    bloque += esfera(r * 1.15, COLOR_JUNTA, "junta")
    return bloque


def construir(con_geometria=True):
    origins = gen.joint_origins()
    names = ["base_link"] + [f"link_{i}" for i in range(1, 7)] + ["tool0"]

    out = ['<?xml version="1.0"?>\n',
           '<!-- Generado por scripts/add_visuals.py  NO EDITAR A MANO -->\n',
           '<!-- Geometria por primitivas dimensionadas desde la tabla D-H -->\n',
           '<!-- Grupo VEVAS -->\n',
           '<robot name="irb120">\n\n']

    for i, n in enumerate(names):
        if not con_geometria:
            out.append(f'  <link name="{n}"/>\n')
            continue
        cuerpo = geometria_eslabon(i)
        if n == "tool0":
            # La brida se marca con un disco fino, sin cilindro largo.
            cuerpo = (f'    <visual>\n'
                      f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
                      f'      <geometry>\n'
                      f'        <cylinder length="0.012" radius="0.031"/>\n'
                      f'      </geometry>\n'
                      + material("brida", COLOR_JUNTA) +
                      f'    </visual>\n')
        out.append(f'  <link name="{n}">\n{cuerpo}  </link>\n')
    out.append("\n")

    for i in range(6):
        xyz, rpy = origins[i]
        lo, hi = gen.JOINT_LIMITS[i]
        out.append(
            f'  <joint name="joint_{i+1}" type="revolute">\n'
            f'    <parent link="{names[i]}"/>\n'
            f'    <child  link="{names[i+1]}"/>\n'
            f'    <origin xyz="{gen.fmt(xyz[0])} {gen.fmt(xyz[1])} '
            f'{gen.fmt(xyz[2])}" rpy="{gen.fmt(rpy[0])} {gen.fmt(rpy[1])} '
            f'{gen.fmt(rpy[2])}"/>\n'
            f'    <axis xyz="0 0 1"/>\n'
            f'    <limit lower="{gen.fmt(lo, 6)}" upper="{gen.fmt(hi, 6)}"'
            f' effort="{gen.EFFORT[i]}" velocity="{gen.VELOCITY[i]}"/>\n'
            f'  </joint>\n\n'
        )

    xyz, rpy = origins[6]
    out.append(
        f'  <joint name="joint_6_tool0" type="fixed">\n'
        f'    <parent link="link_6"/>\n'
        f'    <child  link="tool0"/>\n'
        f'    <origin xyz="{gen.fmt(xyz[0])} {gen.fmt(xyz[1])} '
        f'{gen.fmt(xyz[2])}" rpy="{gen.fmt(rpy[0])} {gen.fmt(rpy[1])} '
        f'{gen.fmt(rpy[2])}"/>\n'
        f'  </joint>\n\n'
        '</robot>\n'
    )
    return "".join(out)


if __name__ == "__main__":
    con_geo = "--plain" not in sys.argv

    urdf = construir(con_geometria=con_geo)
    destino = Path(__file__).resolve().parent.parent / "urdf" / "irb120.urdf"
    destino.write_text(urdf)

    print(f"\n   URDF escrito en  {destino}")
    print(f"   Geometria: {'primitivas derivadas de D-H' if con_geo else 'sin geometria'}\n")

    if con_geo:
        print("Dimensiones tomadas de la tabla D-H\n")
        for i in range(1, 7):
            _, d, a, _ = DH_TABLE[i - 1]
            r = RADIOS[min(i, len(RADIOS) - 1)]
            if abs(a) > 1e-9:
                print(f"  link_{i}   cilindro en x   largo a_{i} = {abs(a):.3f} m"
                      f"   radio {r:.3f} m")
            elif abs(d) > 1e-9:
                print(f"  link_{i}   cilindro en z   largo d_{i} = {abs(d):.3f} m"
                      f"   radio {r:.3f} m")
            else:
                print(f"  link_{i}   sin traslacion (muneca esferica), "
                      f"solo articulacion")
        print()

    # La cinematica no cambia: se vuelve a verificar por las dudas.
    err = gen.verify_chain()
    print(f"Verificacion de la cadena tras anadir geometria: {err:.3e} m")
    print("RESULTADO:", "OK" if err < 1e-6 else "FALLA")
    print()
