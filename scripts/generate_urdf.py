#!/usr/bin/env python3
"""
Generador de la descripcion URDF del ABB IRB 120 a partir de la tabla D-H.

El URDF NO se escribe a mano. Cada tag <origin> se deriva algebraicamente,
de modo que la cadena TF publicada por ROS es identica al producto
0_T_6 de la cinematica directa analitica, salvo redondeo de punto flotante.

Fundamento

    A_i = Rz(q_i + off_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
        = Rz(off_i) . Rz(q_i) . B_i     con  B_i = Tz(d_i).Tx(a_i).Rx(alpha_i)

    porque Rz(a).Rz(b) = Rz(a+b): las rotaciones en torno al mismo eje
    conmutan. URDF impone T_padre_hijo = Origin . Rot(axis, q) con Origin
    constante, asi que reordenando la cadena cada junta recibe

        Origin_i = B_{i-1} . Rz(off_i)          con  B_0 = I

    y una junta fija final con B_6 cierra en la brida.

Grupo VEVAS
"""

import numpy as np
from pathlib import Path
from fk_irb120 import DH_TABLE, JOINT_LIMITS, forward_kinematics

# Limites de esfuerzo y velocidad (datasheet ABB IRB 120)
EFFORT = [42.0, 42.0, 22.0, 5.0, 5.0, 3.0]           # N.m
VELOCITY = [4.36, 4.36, 5.24, 5.58, 5.58, 7.33]      # rad/s

MESH_PKG = "package://irb120_description/meshes"


def B_matrix(d, a, alpha):
    """
    B_i = Tz(d) . Tx(a) . Rx(alpha)

    Las traslaciones puras SI conmutan entre si, por eso Tz(d).Tx(a)
    colapsa al vector (a, 0, d) expresado en el marco padre.
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [1.0, 0.0, 0.0,   a],
        [0.0,  ca, -sa, 0.0],
        [0.0,  sa,  ca,   d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def Rz(theta):
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st, 0.0, 0.0],
        [st,  ct, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) . Ry(pitch) . Rx(roll), convencion de URDF."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rzz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rzz @ Ry @ Rx


def matrix_to_rpy(R):
    """
    Extrae los angulos roll-pitch-yaw de una matriz de rotacion.

    URDF define rpy como composicion extrinseca en orden x-y-z, es decir
    R = Rz(yaw) . Ry(pitch) . Rx(roll).

    De R[2][0] = -sin(pitch) se despeja pitch. Cuando |R[2][0]| -> 1 el
    pitch vale +-pi/2 y la representacion pierde un grado de libertad
    (gimbal lock): roll y yaw dejan de ser independientes. En ese caso se
    fija yaw = 0 y se absorbe todo el giro en roll. Esta rama degenerada
    ocurre realmente en la junta 2 del IRB 120.
    """
    if abs(R[2, 0]) > 1.0 - 1e-10:
        pitch = -np.sign(R[2, 0]) * np.pi / 2
        yaw = 0.0
        if R[2, 0] > 0:
            roll = np.arctan2(-R[0, 1], -R[0, 2])
        else:
            roll = np.arctan2(R[0, 1], R[0, 2])
    else:
        pitch = -np.arcsin(R[2, 0])
        cp = np.cos(pitch)
        roll = np.arctan2(R[2, 1] / cp, R[2, 2] / cp)
        yaw = np.arctan2(R[1, 0] / cp, R[0, 0] / cp)
    return roll, pitch, yaw


def fmt(x, nd=9):
    """Formatea evitando -0.0 y notacion cientifica en el XML."""
    v = 0.0 if abs(x) < 1e-12 else x
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return s if s else "0"


def joint_origins():
    """Lista de (xyz, rpy) de las 6 juntas mas la brida fija."""
    origins = []
    B_prev = np.eye(4)
    for i, (off, d, a, alpha) in enumerate(DH_TABLE):
        O = B_prev @ Rz(off)
        origins.append((O[:3, 3].copy(), matrix_to_rpy(O[:3, :3])))
        B_prev = B_matrix(d, a, alpha)
    O = B_prev
    origins.append((O[:3, 3].copy(), matrix_to_rpy(O[:3, :3])))
    return origins


def link_block(name, mesh=None):
    if mesh is None:
        return f'  <link name="{name}"/>\n'
    return (
        f'  <link name="{name}">\n'
        f'    <visual>\n'
        f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        f'      <geometry>\n'
        f'        <mesh filename="{MESH_PKG}/visual/{mesh}.stl"'
        f' scale="0.001 0.001 0.001"/>\n'
        f'      </geometry>\n'
        f'      <material name="abb_orange">\n'
        f'        <color rgba="0.95 0.42 0.07 1.0"/>\n'
        f'      </material>\n'
        f'    </visual>\n'
        f'    <collision>\n'
        f'      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        f'      <geometry>\n'
        f'        <mesh filename="{MESH_PKG}/collision/{mesh}.stl"'
        f' scale="0.001 0.001 0.001"/>\n'
        f'      </geometry>\n'
        f'    </collision>\n'
        f'  </link>\n'
    )


def build_urdf(with_meshes=False):
    origins = joint_origins()
    out = ['<?xml version="1.0"?>\n',
           '<!-- Generado por scripts/generate_urdf.py  NO EDITAR A MANO -->\n',
           '<!-- Grupo VEVAS -->\n',
           '<robot name="irb120">\n\n']

    names = ["base_link"] + [f"link_{i}" for i in range(1, 7)] + ["tool0"]
    meshes = ["base"] + [f"link_{i}" for i in range(1, 7)] + [None]

    for n, m in zip(names, meshes):
        out.append(link_block(n, m if with_meshes else None))
    out.append("\n")

    for i in range(6):
        xyz, rpy = origins[i]
        lo, hi = JOINT_LIMITS[i]
        out.append(
            f'  <joint name="joint_{i+1}" type="revolute">\n'
            f'    <parent link="{names[i]}"/>\n'
            f'    <child  link="{names[i+1]}"/>\n'
            f'    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}"'
            f' rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}"/>\n'
            f'    <axis xyz="0 0 1"/>\n'
            f'    <limit lower="{fmt(lo, 6)}" upper="{fmt(hi, 6)}"'
            f' effort="{EFFORT[i]}" velocity="{VELOCITY[i]}"/>\n'
            f'  </joint>\n\n'
        )

    xyz, rpy = origins[6]
    out.append(
        f'  <joint name="joint_6_tool0" type="fixed">\n'
        f'    <parent link="link_6"/>\n'
        f'    <child  link="tool0"/>\n'
        f'    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}"'
        f' rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}"/>\n'
        f'  </joint>\n\n'
        '</robot>\n'
    )
    return "".join(out)


def verify_chain(n=1000, seed=42):
    """
    Comprueba offline que la cadena URDF reproduce 0_T_6 analitica.

    Recorre el arbol tal como lo hara robot_state_publisher
    (Origin_i . Rz(q_i)) y compara con forward_kinematics().
    """
    origins = joint_origins()
    rng = np.random.default_rng(seed)
    peor = 0.0
    for _ in range(n):
        q = rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])
        T = np.eye(4)
        for i in range(6):
            xyz, rpy = origins[i]
            O = np.eye(4)
            O[:3, :3] = rpy_to_matrix(*rpy)
            O[:3, 3] = xyz
            T = T @ O @ Rz(q[i])
        xyz, rpy = origins[6]
        O = np.eye(4)
        O[:3, :3] = rpy_to_matrix(*rpy)
        O[:3, 3] = xyz
        T = T @ O
        peor = max(peor, np.linalg.norm(T[:3, 3] - forward_kinematics(q)[:3, 3]))
    return peor


if __name__ == "__main__":
    import sys
    con_mallas = "--with-meshes" in sys.argv

    urdf = build_urdf(with_meshes=con_mallas)
    destino = Path(__file__).resolve().parent.parent / "urdf" / "irb120.urdf"
    destino.parent.mkdir(exist_ok=True)
    destino.write_text(urdf)

    print(f"\n   URDF generado en  {destino}\n")

    print("Procedencia de cada origen\n")
    origins = joint_origins()
    for i, (xyz, rpy) in enumerate(origins):
        etq = f"joint_{i+1}" if i < 6 else "tool0 (fija)"
        if i == 0:
            src = "B_0 . Rz(0) = I"
        else:
            _, d, a, al = DH_TABLE[i - 1]
            offi = DH_TABLE[i][0] if i < 6 else 0.0
            src = (f"B_{i} . Rz({offi:+.4f})    "
                   f"B_{i} = Tz({d:.3f}).Tx({a:.3f}).Rx({al:+.4f})")
        print(f"  {etq:14s} xyz = [{xyz[0]:7.4f} {xyz[1]:7.4f} {xyz[2]:7.4f}]"
              f"   rpy = [{rpy[0]:8.5f} {rpy[1]:8.5f} {rpy[2]:8.5f}]")
        print(f"                 <- {src}")

    err = verify_chain()
    print(f"\nVerificacion offline sobre 1000 configuraciones aleatorias")
    print(f"  Error posicional maximo cadena URDF vs analitica: {err:.3e} m")
    print("  RESULTADO:", "OK" if err < 1e-6 else "FALLA")
    print()
