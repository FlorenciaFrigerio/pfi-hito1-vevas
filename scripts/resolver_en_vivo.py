#!/usr/bin/env python3
"""
Resolvedor interactivo para la defensa del Hito 2.

Para que sirve

    El docente dicta una pose objetivo el dia de la evaluacion. Esta
    herramienta la acepta en cualquiera de los formatos en que podria
    darla, resuelve las ocho ramas de Pieper, verifica cada una por
    cinematica directa y muestra la tabla completa en pantalla.

Formatos de entrada aceptados

    1. Matriz homogenea T_d  (4x4)
    2. Posicion + cuaternion (x, y, z, qx, qy, qz, qw)
    3. Posicion + matriz de rotacion R  (3x3)
    4. Posicion + angulos RPX/RPY en grados  (convencion declarada)
    5. Posicion + eje z6 deseado  (el formato de la practica W09)

Uso en la defensa

    python3 resolver_en_vivo.py            -> menu interactivo
    python3 resolver_en_vivo.py --demo     -> casos borde preparados

Grupo VEVAS
"""

import sys
import numpy as np

from fk_irb120 import forward_kinematics, JOINT_LIMITS
from pieper import pieper, filtrar, elegir, etiquetar, error_pose
from jacobiano import jacobiano, medida_manipulabilidad

TOL_POSE = 1e-6
UMBRAL_SINGULAR = 1e-3


# ---------------------------------------------------------------------
#  Constructores de T_d
# ---------------------------------------------------------------------

def T_desde_cuaternion(x, y, z, qx, qy, qz, qw):
    """Cuaternion en convencion (qx, qy, qz, qw), parte real al final."""
    n = np.linalg.norm([qx, qy, qz, qw])
    if n < 1e-12:
        raise ValueError("cuaternion nulo")
    qx, qy, qz, qw = np.array([qx, qy, qz, qw]) / n
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])
    return T_desde_R_p(R, [x, y, z])


def T_desde_rpy(x, y, z, roll, pitch, yaw, grados=True):
    """
    Angulos fijos XYZ (roll-pitch-yaw), R = Rz(yaw) Ry(pitch) Rx(roll).

    La convencion se declara explicitamente porque "RPY" sin mas es
    ambiguo: hay doce convenciones de Euler y dos interpretaciones
    (ejes fijos frente a ejes moviles) que dan resultados distintos.
    """
    if grados:
        roll, pitch, yaw = np.radians([roll, pitch, yaw])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return T_desde_R_p(Rz @ Ry @ Rx, [x, y, z])


def T_desde_eje_z(x, y, z, zx, zy, zz):
    """
    Posicion mas el eje z6 deseado, completando una orientacion valida.

    Es el formato de la practica W09-S01. Define solo dos grados de
    libertad de orientacion: el giro alrededor de z6 queda libre. Se
    completa con el eje x6 mas estable posible, y se avisa que la
    orientacion no es unica.
    """
    z6 = np.array([zx, zy, zz], float)
    n = np.linalg.norm(z6)
    if n < 1e-12:
        raise ValueError("eje z6 nulo")
    z6 /= n
    # Referencia auxiliar lo mas perpendicular posible a z6.
    aux = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(aux, z6)) > 0.9:
        aux = np.array([1.0, 0.0, 0.0])
    x6 = np.cross(aux, z6); x6 /= np.linalg.norm(x6)
    y6 = np.cross(z6, x6)
    return T_desde_R_p(np.column_stack([x6, y6, z6]), [x, y, z])


def T_desde_R_p(R, p):
    """Ensambla T y verifica que R sea realmente una rotacion."""
    R = np.asarray(R, float).reshape(3, 3)
    orto = np.linalg.norm(R.T @ R - np.eye(3), "fro")
    det = np.linalg.det(R)
    if orto > 1e-6 or abs(det - 1.0) > 1e-6:
        print(f"  [!] AVISO: R no es ortonormal "
              f"(||R^T R - I||_F = {orto:.2e}, det = {det:.6f}).")
        print("      Se reortonormaliza por SVD antes de resolver.")
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, float)
    return T


# ---------------------------------------------------------------------
#  Informe de una pose
# ---------------------------------------------------------------------

def resolver_y_mostrar(T_d, q_actual=None, titulo=""):
    """Tabla completa de las 8 ramas con verificacion FK."""
    if q_actual is None:
        q_actual = np.zeros(6)

    print()
    if titulo:
        print(f"  {titulo}")
    p = T_d[:3, 3]
    print(f"  p_d = [{p[0]:+.4f}, {p[1]:+.4f}, {p[2]:+.4f}] m "
          f"  ||p_d|| = {np.linalg.norm(p):.4f} m")

    # Chequeo de alcance antes de resolver: evita presentar como
    # "fallo del algoritmo" lo que es una pose fuera del espacio.
    d6 = 0.072
    p_W = p - d6 * T_d[:3, 2]
    r_W = np.linalg.norm(p_W - np.array([0.0, 0.0, 0.290]))
    print(f"  centro de muneca p_W = [{p_W[0]:+.4f}, {p_W[1]:+.4f}, "
          f"{p_W[2]:+.4f}]   r_W = {r_W:.4f} m  (max 0.5800 m)")
    # Tolerancia generosa a proposito: una pose exactamente sobre la
    # cascara esferica del alcance es resoluble, y anunciarla como
    # inalcanzable para despues resolverla seria contradecirse.
    holgura = 1e-4      # 0.1 mm: por debajo de eso es ruido numerico
    if r_W > 0.580 + holgura:
        exceso = r_W - 0.580
        print(f"  -> FUERA DE ALCANCE: r_W excede a2 + k = 0.5800 m "
              f"por {exceso*1000:.1f} mm.")
    elif r_W > 0.580 - 1e-3:
        print("  -> EN EL LIMITE del alcance: el brazo queda extendido, "
              "cerca de la singularidad de codo.")

    try:
        crudas = pieper(T_d)
    except Exception as exc:
        print(f"  -> Pieper no produjo ramas: {exc}")
        return None, [], []

    admisibles, rechazadas = filtrar(crudas, T_d, tol_pose=TOL_POSE)

    print(f"\n  ramas calculadas {len(crudas)}    "
          f"admisibles {len(admisibles)}    rechazadas {len(rechazadas)}\n")

    cab = (f"  {'#':>2}  {'q1':>8} {'q2':>8} {'q3':>8} {'q4':>8} "
           f"{'q5':>8} {'q6':>8}  {'e_p [m]':>10} {'e_R':>10}  "
           f"{'w':>9}  etiqueta")
    print(cab)
    print("  " + "-" * (len(cab) - 2))
    for i, q in enumerate(crudas, 1):
        ep, eR = error_pose(q, T_d)
        w = medida_manipulabilidad(q)
        gq = np.degrees(q)
        marca = "" if any(np.allclose(q, a) for a in admisibles) else "  (rechazada)"
        alerta = "  [!] cerca de singularidad" if w < UMBRAL_SINGULAR else ""
        print(f"  {i:>2}  {gq[0]:>8.2f} {gq[1]:>8.2f} {gq[2]:>8.2f} "
              f"{gq[3]:>8.2f} {gq[4]:>8.2f} {gq[5]:>8.2f}  "
              f"{ep:>10.2e} {eR:>10.2e}  {w:>9.2e}  "
              f"{etiquetar(q)}{marca}{alerta}")
    print("\n  Angulos en grados. Criterio de aceptacion: "
          f"e_p < {TOL_POSE:.0e} m y e_R < {TOL_POSE:.0e}.")

    # Ramas que colapsan entre si: es la firma de una singularidad, no
    # un fallo. En la singularidad de codo las ramas "codo arriba" y
    # "codo abajo" se vuelven la misma configuracion.
    pares = [(i + 1, j + 1)
             for i in range(len(crudas))
             for j in range(i + 1, len(crudas))
             if np.allclose(crudas[i], crudas[j], atol=1e-4)]
    if pares:
        print(f"\n  [!] {len(pares)} par(es) de ramas coinciden: "
              + ", ".join(f"{a}={b}" for a, b in pares) + ".")
        print("      No es un error: en una singularidad las ramas que")
        print("      difieren por ese signo convergen a la misma")
        print("      configuracion. Las soluciones distintas son menos")
        print("      de ocho, y eso es justamente el sintoma.")

    if admisibles:
        q_sel, costo = elegir(admisibles, q_actual)
        ep, eR = error_pose(q_sel, T_d)
        print(f"\n  SELECCIONADA: {etiquetar(q_sel)}   "
              f"costo ponderado {costo:.4f}")
        print(f"  q* = {np.degrees(q_sel).round(2)} grados")
        print(f"  verificacion FK: e_p = {ep:.3e} m, e_R = {eR:.3e}")
        w = medida_manipulabilidad(q_sel)
        print(f"  manipulabilidad w = {w:.4e}, rango(J) = "
              f"{np.linalg.matrix_rank(jacobiano(q_sel), tol=1e-6)}")
        if w < UMBRAL_SINGULAR:
            print("  [!] La configuracion elegida esta cerca de una "
                  "singularidad.")
        return q_sel, admisibles, rechazadas

    print("\n  NINGUNA rama es admisible. El sistema no debe moverse.")
    motivos = {}
    for _, m in rechazadas:
        motivos[m] = motivos.get(m, 0) + 1
    for m, n in sorted(motivos.items(), key=lambda kv: -kv[1]):
        print(f"     {n} x  {m}")
    return None, [], rechazadas


# ---------------------------------------------------------------------
#  Menu
# ---------------------------------------------------------------------

def _leer(mensaje, n):
    """Lee n numeros de una linea, tolerando comas, corchetes y espacios."""
    crudo = input(mensaje).replace(",", " ").replace("[", " ").replace("]", " ")
    vals = [float(v) for v in crudo.split()]
    if len(vals) != n:
        raise ValueError(f"se esperaban {n} numeros, llegaron {len(vals)}")
    return vals


def menu():
    print("\n   RESOLVEDOR DE POSES EN VIVO   Hito 2 - Grupo VEVAS\n")
    while True:
        print("  1) Matriz homogenea T_d (4x4)")
        print("  2) Posicion + cuaternion (x y z qx qy qz qw)")
        print("  3) Posicion + matriz R (3x3)")
        print("  4) Posicion + RPY en grados (x y z roll pitch yaw)")
        print("  5) Posicion + eje z6 (x y z zx zy zz)")
        print("  6) Casos borde de demostracion")
        print("  0) Salir")
        try:
            op = input("\n  opcion> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return

        try:
            if op == "0":
                return
            elif op == "1":
                print("  Pegue las 16 cifras de T_d (por filas):")
                T = np.array(_leer("  T_d> ", 16)).reshape(4, 4)
                T = T_desde_R_p(T[:3, :3], T[:3, 3])
                resolver_y_mostrar(T, titulo="Pose por matriz homogenea")
            elif op == "2":
                v = _leer("  x y z qx qy qz qw> ", 7)
                resolver_y_mostrar(T_desde_cuaternion(*v),
                                   titulo="Pose por cuaternion")
            elif op == "3":
                p = _leer("  x y z> ", 3)
                R = np.array(_leer("  R (9 cifras por filas)> ", 9))
                resolver_y_mostrar(T_desde_R_p(R.reshape(3, 3), p),
                                   titulo="Pose por matriz de rotacion")
            elif op == "4":
                v = _leer("  x y z roll pitch yaw[grados]> ", 6)
                print("  Convencion: angulos fijos XYZ, "
                      "R = Rz(yaw) Ry(pitch) Rx(roll).")
                resolver_y_mostrar(T_desde_rpy(*v), titulo="Pose por RPY")
            elif op == "5":
                v = _leer("  x y z zx zy zz> ", 6)
                print("  Nota: el eje z6 fija solo dos grados de libertad de")
                print("  orientacion; el giro alrededor de z6 queda libre y")
                print("  se completa con una eleccion declarada.")
                resolver_y_mostrar(T_desde_eje_z(*v),
                                   titulo="Pose por eje z6")
            elif op == "6":
                casos_borde()
            else:
                print("  opcion no reconocida.")
        except Exception as exc:
            print(f"\n  ERROR de entrada: {exc}")
            print("  Revise el formato y vuelva a intentar.\n")
        print()


# ---------------------------------------------------------------------
#  Casos borde
# ---------------------------------------------------------------------

def casos_borde():
    """
    Poses dificiles a proposito.

    Si el docente dicta una pose incomoda, conviene haberla visto antes.
    """
    print("\n   CASOS BORDE\n")

    # 1. Interior del espacio, lejos de singularidades.
    q = np.radians([25.0, -35.0, 20.0, 40.0, 50.0, -30.0])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="1/6  Interior del espacio de trabajo")

    # 2. Singularidad de muneca: q5 = 0 exacto.
    q = np.radians([20.0, -30.0, 15.0, 35.0, 0.0, -25.0])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="2/6  Singularidad de muneca (q5 = 0)")

    # 3. Singularidad de codo: brazo extendido.
    q = np.array([0.3, 0.0, -1.3430, 0.2, 0.6, 0.1])
    resolver_y_mostrar(forward_kinematics(q),
                       titulo="3/6  Codo extendido (sin(q3 + psi) = 0)")

    # 4. Eje de la base: singularidad de hombro.
    T = np.eye(4)
    T[:3, 3] = [0.0, 0.0, 0.620]
    resolver_y_mostrar(T, titulo="4/6  Sobre el eje de la base (hombro)")

    # 5. Limite del alcance.
    T = np.eye(4)
    T[:3, 3] = [0.5795, 0.0, 0.362]
    resolver_y_mostrar(T, titulo="5/6  Al borde del alcance (r_W ~ 0.580 m)")

    # 6. Fuera de alcance.
    T = np.eye(4)
    T[:3, 3] = [1.5, 1.5, 1.5]
    resolver_y_mostrar(T, titulo="6/6  Fuera de alcance")


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    if "--demo" in sys.argv:
        casos_borde()
    else:
        menu()
