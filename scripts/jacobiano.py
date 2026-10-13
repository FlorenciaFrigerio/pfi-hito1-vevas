#!/usr/bin/env python3
"""
Jacobiano geometrico y analisis de singularidades del ABB IRB 120.

Grupo VEVAS
"""

import numpy as np
from fk_irb120 import DH_TABLE, dh_matrix, forward_kinematics


def marcos(q):
    """Devuelve la lista de T_0^i para i = 0..6."""
    Ts = [np.eye(4)]
    T = np.eye(4)
    for i in range(6):
        off, d, a, al = DH_TABLE[i]
        T = T @ dh_matrix(q[i] + off, d, a, al)
        Ts.append(T.copy())
    return Ts


def jacobiano(q):
    """
    Jacobiano geometrico 6x6 respecto de la base.

    Para una junta de revolucion i:
        Jv_i = z_{i-1} x (p_e - p_{i-1})
        Jw_i = z_{i-1}
    """
    Ts = marcos(q)
    p_e = Ts[6][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = Ts[i][:3, 2]
        p = Ts[i][:3, 3]
        J[:3, i] = np.cross(z, p_e - p)
        J[3:, i] = z
    return J


def jacobiano_numerico(q, h=1e-7):
    """Jacobiano por diferencias finitas, para validar el analitico."""
    J = np.zeros((6, 6))
    T0 = forward_kinematics(q)
    for i in range(6):
        dq = np.zeros(6); dq[i] = h
        Tp = forward_kinematics(np.asarray(q) + dq)
        J[:3, i] = (Tp[:3, 3] - T0[:3, 3]) / h
        dR = (Tp[:3, :3] - T0[:3, :3]) / h @ T0[:3, :3].T
        J[3:, i] = [dR[2, 1], dR[0, 2], dR[1, 0]]
    return J


def validacion_manual_q0():
    """
    Caso de validacion MANUAL exigido por la rubrica.

    En q = 0 la geometria se conoce sin calcular nada: la brida esta en
    p_e = [0.374, 0, 0.630] m y el primer eje es z_0 = [0, 0, 1] con
    origen en O_0 = [0, 0, 0]. Por la definicion de columna:

        Jv_1 = z_0 x (p_e - O_0)
             = [0,0,1] x [0.374, 0, 0.630]
             = [ 0*0.630 - 1*0 ,  1*0.374 - 0*0.630 ,  0 ]
             = [ 0 , 0.374 , 0 ]

        Jw_1 = z_0 = [0, 0, 1]

    Interpretacion fisica: girar la base a velocidad unitaria mueve la
    brida 0.374 m/s en direccion +y, que es exactamente su distancia al
    eje de giro. La columna del Jacobiano ES el brazo de palanca.

    Devuelve (columna_calculada, columna_a_mano, error).
    """
    q0 = np.zeros(6)
    col_codigo = jacobiano(q0)[:, 0]
    col_mano = np.array([0.0, 0.374, 0.0, 0.0, 0.0, 1.0])
    return col_codigo, col_mano, float(np.abs(col_codigo - col_mano).max())


def jacobiano_2r(q1, q2, a1=1.0, a2=1.0):
    """
    Sub-Jacobiano de traslacion de un 2R planar (ejemplo de clase).

    Permite contrastar el codigo general contra el resultado analitico
    conocido:   det(J_p) = a1 a2 sin(q2)
    que se anula en q2 = 0 (brazo estirado) y q2 = pi (plegado).
    """
    s1, c1 = np.sin(q1), np.cos(q1)
    s12, c12 = np.sin(q1 + q2), np.cos(q1 + q2)
    return np.array([[-a1*s1 - a2*s12, -a2*s12],
                     [a1*c1 + a2*c12,  a2*c12]])


def medida_manipulabilidad(q):
    """w = sqrt(det(J J^T)) = producto de valores singulares."""
    J = jacobiano(q)
    return float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))


def diagnostico(q, tol=1e-6):
    J = jacobiano(q)
    sv = np.linalg.svd(J, compute_uv=False)
    return {
        "det": float(np.linalg.det(J)),
        "rango": int(np.linalg.matrix_rank(J, tol=tol)),
        "sigma_min": float(sv[-1]),
        "cond": float(sv[0] / sv[-1]) if sv[-1] > 1e-15 else np.inf,
        "w": float(np.prod(sv)),
    }


if __name__ == "__main__":
    rng = np.random.default_rng(11)
    peor = 0.0
    for _ in range(300):
        q = rng.uniform(-1.5, 1.5, 6)
        peor = max(peor, np.abs(jacobiano(q) - jacobiano_numerico(q)).max())
    print(f"\n   Jacobiano: analitico vs diferencias finitas\n")
    print(f"  discrepancia maxima  {peor:.3e}")
    print("  RESULTADO:", "OK" if peor < 1e-5 else "REVISAR")

    cc, cm, err = validacion_manual_q0()
    print("\n   Validacion manual: primera columna en q = 0\n")
    print(f"  calculada por codigo  {np.round(cc, 6)}")
    print(f"  calculada a mano      {np.round(cm, 6)}")
    print(f"  discrepancia          {err:.3e}")

    print("\n   Contraste con el 2R planar analitico  det = a1 a2 sin(q2)\n")
    for q2 in [np.pi/2, np.pi/4, 0.0]:
        J2 = jacobiano_2r(0.0, q2)
        print(f"  q2 = {q2:.4f}   det codigo = {np.linalg.det(J2):+.6f}   "
              f"det formula = {np.sin(q2):+.6f}   "
              f"rango = {np.linalg.matrix_rank(J2)}")

    print("\n   Singularidad de muneca (q5 -> 0)\n")
    for q5 in [0.5, 0.2, 0.05, 0.01, 0.001, 0.0]:
        q = np.array([0.3, 0.4, -0.2, 0.1, q5, 0.2])
        d = diagnostico(q)
        print(f"  q5 = {q5:6.3f}   rango {d['rango']}   "
              f"det {d['det']:+.3e}   sigma_min {d['sigma_min']:.3e}   "
              f"w {d['w']:.3e}")
    print()
