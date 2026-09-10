#!/usr/bin/env python3
"""
Compara el URDF escrito a mano contra el generado desde la tabla D-H.

Parsea los <origin> de ambos archivos y verifica que coincidan numerica
y cinematicamente. El URDF manual documenta el razonamiento; el derivado
lo demuestra. Si difieren, hay un error de transcripcion.

Grupo VEVAS
"""

import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_DIR = Path(__file__).resolve().parent.parent / "urdf"


def leer_origenes(path):
    root = ET.parse(path).getroot()
    datos = {}
    for j in root.findall("joint"):
        o = j.find("origin")
        if o is None:
            datos[j.get("name")] = (np.zeros(3), np.zeros(3))
            continue
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
        rpy = np.array([float(v) for v in o.get("rpy", "0 0 0").split()])
        datos[j.get("name")] = (xyz, rpy)
    return datos


def ang_dif(a, b):
    """Diferencia angular envuelta a (-pi, pi]."""
    return abs((a - b + np.pi) % (2 * np.pi) - np.pi)


if __name__ == "__main__":
    man = leer_origenes(URDF_DIR / "irb120_manual.urdf")
    gen = leer_origenes(URDF_DIR / "irb120.urdf")

    print("\n   Manual vs derivado   comparacion de <origin>\n")

    peor_p, peor_a = 0.0, 0.0
    for nombre in man:
        if nombre not in gen:
            print(f"  {nombre:16s} ausente en el generado")
            continue
        xm, rm = man[nombre]
        xg, rg = gen[nombre]
        ep = float(np.linalg.norm(xm - xg))
        ea = max(ang_dif(rm[k], rg[k]) for k in range(3))
        peor_p, peor_a = max(peor_p, ep), max(peor_a, ea)
        estado = "OK" if ep < 1e-6 and ea < 1e-6 else "DIFIERE"
        print(f"  {nombre:16s} d_pos = {ep:.2e} m   "
              f"d_ang = {ea:.2e} rad   {estado}")

    print(f"\n  Peor discrepancia posicional: {peor_p:.3e} m")
    print(f"  Peor discrepancia angular:    {peor_a:.3e} rad")
    print("  RESULTADO:",
          "IDENTICOS" if peor_p < 1e-6 and peor_a < 1e-6
          else "REVISAR TRANSCRIPCION")
    print()
