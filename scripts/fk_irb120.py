#!/usr/bin/env python3
"""
Cinematica directa del ABB IRB 120 por convencion Denavit-Hartenberg estandar.

Implementacion desde primeros principios: solo NumPy, sin librerias de
robotica. Esta es la FUENTE ANALITICA DE VERDAD contra la que se valida
la descripcion URDF publicada por ROS.

Tabla D-H: parametrizacion de Truc & Lam (2020), contrastada con las
cotas del manual tecnico oficial del ABB IRB 120.

Grupo VEVAS
"""

import numpy as np

# Tabla Denavit-Hartenberg estandar (Siciliano / Spong)
#   (theta_offset [rad], d [m], a [m], alpha [rad])
# El offset es constante aditiva sobre la variable de junta q_i.
DH_TABLE = [
    (0.0,       0.290, 0.000, -np.pi / 2),   # J1  columna base
    (-np.pi / 2, 0.000, 0.270,  0.0),        # J2  brazo (ejes paralelos)
    (0.0,       0.000, 0.070, -np.pi / 2),   # J3  codo
    (0.0,       0.302, 0.000,  np.pi / 2),   # J4  antebrazo
    (0.0,       0.000, 0.000, -np.pi / 2),   # J5  muneca (ejes concurrentes)
    (0.0,       0.072, 0.000,  0.0),         # J6  brida tool0
]

# Limites articulares del manual ABB, convertidos a radianes.
# J3 es la unica junta asimetrica.
JOINT_LIMITS_DEG = [
    (-165.0, 165.0),
    (-110.0, 110.0),
    (-110.0,  70.0),
    (-160.0, 160.0),
    (-120.0, 120.0),
    (-400.0, 400.0),
]
JOINT_LIMITS = np.radians(np.array(JOINT_LIMITS_DEG))


def dh_matrix(theta, d, a, alpha):
    """
    Matriz homogenea elemental de un eslabon D-H.

        i-1_T_i = Rot(z, theta) . Tras(z, d) . Tras(x, a) . Rot(x, alpha)

    Los cuatro factores estan pre-multiplicados analiticamente para
    evitar cuatro productos matriciales por junta en cada evaluacion.
    El orden es estricto: SE(3) no es conmutativo.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def forward_kinematics(q, upto=6):
    """
    Cinematica directa: 0_T_6(q) = prod_{i=1..6} i-1_T_i

    q     : vector de 6 angulos de junta REALES [rad] (como el FlexPendant)
    upto  : permite truncar la cadena para inspeccionar marcos intermedios

    La post-multiplicacion (T @ A) implementa la composicion en el marco
    movil: cada transformacion se expresa respecto al eslabon anterior.
    """
    q = np.asarray(q, dtype=float).ravel()
    if q.size != 6:
        raise ValueError(f"Se esperaban 6 angulos de junta, llegaron {q.size}")

    T = np.eye(4)
    for i in range(upto):
        offset, d, a, alpha = DH_TABLE[i]
        T = T @ dh_matrix(q[i] + offset, d, a, alpha)
    return T


def position(q):
    """Vector de posicion p = [x, y, z] de la brida respecto a base_link."""
    return forward_kinematics(q)[:3, 3]


def orientation(q):
    """Matriz de rotacion R (3x3) de la brida respecto a base_link."""
    return forward_kinematics(q)[:3, :3]


def inverse_transform(T):
    """
    Inversa analitica rapida de una transformacion homogenea:

        T^-1 = [ R^T  |  -R^T . p ]

    Evita la inversion numerica generica de 4x4: es exacta y ~10x mas
    rapida, porque explota la ortogonalidad de R (R^-1 = R^T).
    """
    R = T[:3, :3]
    p = T[:3, 3]
    Tinv = np.eye(4)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ p
    return Tinv


def random_configuration(rng=None):
    """Configuracion aleatoria uniforme dentro de los limites articulares."""
    rng = rng or np.random.default_rng()
    return rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)

    q0 = np.zeros(6)
    T = forward_kinematics(q0)

    print("\n   CINEMATICA DIRECTA   ABB IRB 120   configuracion cero\n\n")
    print("0_T_6(q=0) =")
    print(T)

    p = T[:3, 3]
    print(f"\nPosicion de la brida: [{p[0]:.6f}, {p[1]:.6f}, {p[2]:.6f}] m")

    # Verificacion de cierre: la pose en q=0 debe ser deducible sumando
    # cotas del manual, sin multiplicar una sola matriz.
    x_esp = 0.302 + 0.072          # d4 + d6
    z_esp = 0.290 + 0.270 + 0.070  # d1 + a2 + a3
    esperado = np.array([x_esp, 0.0, z_esp])
    err = np.linalg.norm(p - esperado)

    print(f"Esperado por suma de cotas: [{x_esp:.6f}, 0.000000, {z_esp:.6f}] m")
    print(f"Error vectorial: {err:.3e} m")
    print("RESULTADO:", "OK" if err < 1e-9 else "FALLA")

    # Marcos intermedios: util para depurar la asignacion de ejes.
    print("\nOrigenes de los marcos intermedios en q=0:")
    for i in range(1, 7):
        pi = forward_kinematics(q0, upto=i)[:3, 3]
        print(f"  O{i}: [{pi[0]:8.5f}, {pi[1]:8.5f}, {pi[2]:8.5f}]")
